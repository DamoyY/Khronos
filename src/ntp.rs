use std::{
    io::{self, ErrorKind},
    net::{ToSocketAddrs, UdpSocket},
    sync::{Arc, Mutex, mpsc},
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use chrono::{DateTime, Utc};
use rand::Rng;

use crate::program_clock::ProgramClock;
pub const NTP_SERVERS: &[&str] = &[
    "0.cn.pool.ntp.org",
    "1.cn.pool.ntp.org",
    "0.asia.pool.ntp.org",
    "1.asia.pool.ntp.org",
    "ntp.aliyun.com",
    "ntp1.aliyun.com",
    "ntp.tencent.com",
    "ntp1.tencent.com",
    "ntp.ntsc.ac.cn",
    "ntp1.nim.ac.cn",
    "ntp2.nim.ac.cn",
    "time.cloudflare.com",
];
const NTP_PORT: u16 = 123;
const NTP_PACKET_SIZE: usize = 48;
const NTP_UNIX_EPOCH_DIFF: u64 = 2_208_988_800;
const NTP_UNIX_EPOCH_DIFF_U32: u32 = 2_208_988_800;
const RECV_TS_OFFSET: usize = 32;
const TX_TS_OFFSET: usize = 40;
#[derive(Copy, Clone, Debug)]
struct NtpTimestamp {
    seconds: u32,
    fraction: u32,
}
impl NtpTimestamp {
    fn from_chrono_utc(time: DateTime<Utc>) -> io::Result<Self> {
        let systime: SystemTime = time.into();
        let dur = systime
            .duration_since(UNIX_EPOCH)
            .map_err(io::Error::other)?;
        let seconds = dur
            .as_secs()
            .checked_add(NTP_UNIX_EPOCH_DIFF)
            .ok_or_else(|| io::Error::other("NTP seconds overflow"))?;
        let seconds =
            u32::try_from(seconds).map_err(|_| io::Error::other("NTP seconds overflow"))?;
        let nanos = dur.subsec_nanos();
        let fraction =
            u32::try_from((u128::from(nanos) * u128::from(0x1_0000_0000u64)) / 1_000_000_000u128)
                .map_err(|_| io::Error::other("NTP fraction overflow"))?;
        Ok(Self { seconds, fraction })
    }

    fn to_system_time(self) -> io::Result<SystemTime> {
        if self.seconds < NTP_UNIX_EPOCH_DIFF_U32 {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                "NTP time is earlier than Unix epoch",
            ));
        }
        let unix_secs = u64::from(self.seconds) - NTP_UNIX_EPOCH_DIFF;
        let nanos = u32::try_from(
            (u128::from(self.fraction) * 1_000_000_000u128) / u128::from(0x1_0000_0000u64),
        )
        .map_err(|_| io::Error::other("NTP fraction overflow"))?;
        Ok(UNIX_EPOCH + Duration::new(unix_secs, nanos))
    }

    const fn from_bytes(bytes: [u8; 8]) -> Self {
        let seconds = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let fraction = u32::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        Self { seconds, fraction }
    }

    fn to_bytes(self) -> [u8; 8] {
        let mut bytes = [0u8; 8];
        bytes[0..4].copy_from_slice(&self.seconds.to_be_bytes());
        bytes[4..8].copy_from_slice(&self.fraction.to_be_bytes());
        bytes
    }
}
pub enum SyncMessage {
    Syncing(String),
    Success(chrono::Duration, chrono::Duration),
}
pub fn query_ntp(
    server: &str,
    timeout: Duration,
    program_clock: &Arc<Mutex<ProgramClock>>,
) -> io::Result<(chrono::Duration, chrono::Duration)> {
    let addr = (server, NTP_PORT)
        .to_socket_addrs()?
        .next()
        .ok_or_else(|| io::Error::other(format!("Cannot resolve NTP server: {server}")))?;
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    socket.connect(addr)?;
    socket.set_read_timeout(Some(timeout))?;
    socket.set_write_timeout(Some(timeout))?;
    let mut req = [0u8; NTP_PACKET_SIZE];
    req[0] = 0b00_100_011;
    let t1 = { program_clock.lock().unwrap().now() };
    let t1_ntp = NtpTimestamp::from_chrono_utc(t1)
        .map_err(|e| io::Error::other(format!("Cannot convert program time: {e}")))?;
    req[TX_TS_OFFSET..TX_TS_OFFSET + 8].copy_from_slice(&t1_ntp.to_bytes());
    let send_instant = Instant::now();
    socket.send(&req)?;
    let mut buf = [0u8; NTP_PACKET_SIZE];
    let n = socket.recv(&mut buf)?;
    let recv_instant = Instant::now();
    if n < NTP_PACKET_SIZE {
        return Err(io::Error::new(
            ErrorKind::UnexpectedEof,
            "NTP response is too short",
        ));
    }
    let round_trip_duration = recv_instant.duration_since(send_instant);
    let t4 = t1
        + chrono::Duration::from_std(round_trip_duration)
            .map_err(|e| io::Error::other(format!("Round trip duration error: {e}")))?;
    let t2_ntp =
        NtpTimestamp::from_bytes(buf[RECV_TS_OFFSET..RECV_TS_OFFSET + 8].try_into().unwrap());
    let t3_ntp = NtpTimestamp::from_bytes(buf[TX_TS_OFFSET..TX_TS_OFFSET + 8].try_into().unwrap());
    let t2_systime = t2_ntp.to_system_time()?;
    let t3_systime = t3_ntp.to_system_time()?;
    let t2: DateTime<Utc> = t2_systime.into();
    let t3: DateTime<Utc> = t3_systime.into();
    let offset = ((t2 - t1) + (t3 - t4)) / 2;
    let delay = (t4 - t1) - (t3 - t2);
    Ok((offset, delay))
}
fn perform_sync(
    server: &str,
    program_clock: &Arc<Mutex<ProgramClock>>,
) -> io::Result<(chrono::Duration, chrono::Duration)> {
    query_ntp(server, Duration::from_millis(500), program_clock)
}
pub fn start_sync_thread(clock: Arc<Mutex<ProgramClock>>) -> mpsc::Receiver<SyncMessage> {
    let (tx, rx) = mpsc::channel::<SyncMessage>();
    thread::spawn(move || {
        let mut rng = rand::rng();
        loop {
            let next_sync_interval = Duration::from_secs(rng.random_range(0..=2));
            thread::sleep(next_sync_interval);
            let server_index = rng.random_range(0..NTP_SERVERS.len());
            let server = NTP_SERVERS[server_index].to_string();
            if tx.send(SyncMessage::Syncing(server.clone())).is_err() {
                break;
            }
            if let Ok(result) = perform_sync(&server, &clock)
                && tx.send(SyncMessage::Success(result.0, result.1)).is_err()
            {
                break;
            }
        }
    });
    rx
}
