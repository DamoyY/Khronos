use std::io::{self, Write};
use std::time::Duration;
use std::thread;
use chrono::{DateTime, Local};
use rand::Rng;
use crossterm::{cursor, execute, style::Print, terminal};
use std::sync::{mpsc, Arc, Mutex};

mod kalman_filter {
    use std::time::Instant;

    pub struct KalmanFilter {
        x_hat: [f64; 2],
        p_matrix: [[f64; 2]; 2],
        process_noise_q: f64,
        last_timestamp: Instant,
        nis_ema: f64,
        adaptive_q_enabled: bool,
    }

    impl KalmanFilter {
        const NIS_EMA_ALPHA: f64 = 0.05;
        const ADAPTATION_RATE_ETA: f64 = 0.05;

        pub fn new(
            initial_offset: f64,
            initial_uncertainty: f64,
            initial_process_noise_q: f64,
            adaptive_q_enabled: bool,
        ) -> Self {
            KalmanFilter {
                x_hat: [initial_offset, 0.0],
                p_matrix: [[initial_uncertainty, 0.0], [0.0, initial_uncertainty]],
                process_noise_q: initial_process_noise_q,
                last_timestamp: Instant::now(),
                nis_ema: 1.0,
                adaptive_q_enabled,
            }
        }

        fn predict(&self, dt: f64) -> ([f64; 2], [[f64; 2]; 2]) {
            let f_matrix = [[1.0, dt], [0.0, 1.0]];
            let q = self.process_noise_q;
            let dt2 = dt * dt;
            let dt3 = dt2 * dt;
            let q_matrix = [
                [dt3 / 3.0 * q, dt2 / 2.0 * q],
                [dt2 / 2.0 * q, dt * q],
            ];

            let x_hat_predicted = [
                self.x_hat[0] * f_matrix[0][0] + self.x_hat[1] * f_matrix[0][1],
                self.x_hat[0] * f_matrix[1][0] + self.x_hat[1] * f_matrix[1][1],
            ];

            let fp = [
                [
                    f_matrix[0][0] * self.p_matrix[0][0] + f_matrix[0][1] * self.p_matrix[1][0],
                    f_matrix[0][0] * self.p_matrix[0][1] + f_matrix[0][1] * self.p_matrix[1][1],
                ],
                [
                    f_matrix[1][0] * self.p_matrix[0][0] + f_matrix[1][1] * self.p_matrix[1][0],
                    f_matrix[1][0] * self.p_matrix[0][1] + f_matrix[1][1] * self.p_matrix[1][1],
                ],
            ];
            let fpf_t = [
                [
                    fp[0][0] * f_matrix[0][0] + fp[0][1] * f_matrix[0][1],
                    fp[0][0] * f_matrix[1][0] + fp[0][1] * f_matrix[1][1],
                ],
                [
                    fp[1][0] * f_matrix[0][0] + fp[1][1] * f_matrix[0][1],
                    fp[1][0] * f_matrix[1][0] + fp[1][1] * f_matrix[1][1],
                ],
            ];
            let p_predicted = [
                [fpf_t[0][0] + q_matrix[0][0], fpf_t[0][1] + q_matrix[0][1]],
                [fpf_t[1][0] + q_matrix[1][0], fpf_t[1][1] + q_matrix[1][1]],
            ];

            (x_hat_predicted, p_predicted)
        }

        fn correct(
            &mut self,
            measurement: f64,
            measurement_noise_r: f64,
            x_hat_predicted: [f64; 2],
            p_predicted: [[f64; 2]; 2],
        ) {
            let h_matrix = [1.0, 0.0];
            let y = measurement - (h_matrix[0] * x_hat_predicted[0] + h_matrix[1] * x_hat_predicted[1]);

            let hp = [
                h_matrix[0] * p_predicted[0][0] + h_matrix[1] * p_predicted[1][0],
                h_matrix[0] * p_predicted[0][1] + h_matrix[1] * p_predicted[1][1],
            ];
            let hph_t = hp[0] * h_matrix[0] + hp[1] * h_matrix[1];
            let s = hph_t + measurement_noise_r;

            let ph_t = [
                p_predicted[0][0] * h_matrix[0] + p_predicted[0][1] * h_matrix[1],
                p_predicted[1][0] * h_matrix[0] + p_predicted[1][1] * h_matrix[1],
            ];
            let k_gain = [ph_t[0] / s, ph_t[1] / s];

            self.x_hat[0] = x_hat_predicted[0] + k_gain[0] * y;
            self.x_hat[1] = x_hat_predicted[1] + k_gain[1] * y;

            let kh = [
                [k_gain[0] * h_matrix[0], k_gain[0] * h_matrix[1]],
                [k_gain[1] * h_matrix[0], k_gain[1] * h_matrix[1]],
            ];
            let ikh = [[1.0 - kh[0][0], -kh[0][1]], [-kh[1][0], 1.0 - kh[1][1]]];
            self.p_matrix = [
                [
                    ikh[0][0] * p_predicted[0][0] + ikh[0][1] * p_predicted[1][0],
                    ikh[0][0] * p_predicted[0][1] + ikh[0][1] * p_predicted[1][1],
                ],
                [
                    ikh[1][0] * p_predicted[0][0] + ikh[1][1] * p_predicted[1][0],
                    ikh[1][0] * p_predicted[0][1] + ikh[1][1] * p_predicted[1][1],
                ],
            ];

            if self.adaptive_q_enabled {
                let nis = y * y / s;
                self.nis_ema = (1.0 - Self::NIS_EMA_ALPHA) * self.nis_ema + Self::NIS_EMA_ALPHA * nis;
                let factor = (Self::ADAPTATION_RATE_ETA * (self.nis_ema - 1.0)).exp();
                self.process_noise_q *= factor;
            }
        }

        pub fn update(&mut self, measurement: f64, measurement_noise_r: f64) -> f64 {
            let dt = self.last_timestamp.elapsed().as_secs_f64();
            self.last_timestamp = Instant::now();
            let (x_hat_predicted, p_predicted) = self.predict(dt);
            self.correct(measurement, measurement_noise_r, x_hat_predicted, p_predicted);
            self.x_hat[0]
        }

        pub fn get_drift_ppm(&self) -> f64 {
            self.x_hat[1] * 1_000_000.0
        }

        pub fn get_process_noise_q(&self) -> f64 {
            self.process_noise_q
        }
    }
}

mod program_clock {
    use chrono::{DateTime, Utc};
    use std::time::Instant;

    pub struct ProgramClock {
        current_utc: DateTime<Utc>,
        last_updated_at: Instant,
    }

    impl ProgramClock {
        pub fn new() -> Self {
            ProgramClock {
                current_utc: "2000-01-01T00:00:00Z".parse().unwrap(),
                last_updated_at: Instant::now(),
            }
        }

        pub fn now(&self) -> DateTime<Utc> {
            let elapsed = self.last_updated_at.elapsed();
            self.current_utc + chrono::Duration::from_std(elapsed).unwrap()
        }

        pub fn apply_offset(&mut self, offset: chrono::Duration) {
            let current_time = self.now();
            self.current_utc = current_time + offset;
            self.last_updated_at = Instant::now();
        }
    }
}

mod ntp {
    use super::program_clock::ProgramClock;
    use std::io::{self, ErrorKind};
    use std::net::{ToSocketAddrs, UdpSocket};
    use std::time::{Duration, SystemTime, SystemTimeError, UNIX_EPOCH, Instant};
    use chrono::{DateTime, Utc};
    use std::sync::{mpsc, Arc, Mutex};
    use std::thread;
    use rand::Rng;

    pub const NTP_SERVERS: &[&str] = &[
        "0.cn.pool.ntp.org", "1.cn.pool.ntp.org", "0.asia.pool.ntp.org", "1.asia.pool.ntp.org", "ntp.aliyun.com", "ntp1.aliyun.com",
        "ntp.tencent.com", "ntp1.tencent.com", "ntp.ntsc.ac.cn", "ntp1.nim.ac.cn", "ntp2.nim.ac.cn",
        "time.cloudflare.com",
    ];

    const NTP_PORT: u16 = 123;
    const NTP_PACKET_SIZE: usize = 48;
    const NTP_UNIX_EPOCH_DIFF: u64 = 2_208_988_800;
    const RECV_TS_OFFSET: usize = 32;
    const TX_TS_OFFSET: usize = 40;

    #[derive(Copy, Clone, Debug)]
    struct NtpTimestamp {
        seconds: u32,
        fraction: u32,
    }

    impl NtpTimestamp {
        fn from_chrono_utc(time: DateTime<Utc>) -> Result<Self, SystemTimeError> {
            let systime: SystemTime = time.into();
            let dur = systime.duration_since(UNIX_EPOCH)?;
            let seconds = dur.as_secs() + NTP_UNIX_EPOCH_DIFF;
            let nanos = dur.subsec_nanos();
            let fraction = ((nanos as u128 * 0x1_0000_0000) / 1_000_000_000) as u32;
            Ok(NtpTimestamp {
                seconds: seconds as u32,
                fraction,
            })
        }

        fn to_system_time(&self) -> io::Result<SystemTime> {
            if self.seconds < NTP_UNIX_EPOCH_DIFF as u32 {
                return Err(io::Error::new(
                    ErrorKind::InvalidData,
                    "NTP time is earlier than Unix epoch",
                ));
            }
            let unix_secs = self.seconds as u64 - NTP_UNIX_EPOCH_DIFF;
            let nanos = ((self.fraction as u128 * 1_000_000_000) / 0x1_0000_0000) as u32;
            Ok(UNIX_EPOCH + Duration::new(unix_secs, nanos))
        }

        fn from_bytes(bytes: &[u8; 8]) -> Self {
            let seconds = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
            let fraction = u32::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
            NtpTimestamp { seconds, fraction }
        }

        fn to_bytes(&self) -> [u8; 8] {
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

    pub fn query_ntp(server: &str, timeout: Duration, program_clock: &Arc<Mutex<ProgramClock>>) -> io::Result<(chrono::Duration, chrono::Duration)> {
        let addr = (server, NTP_PORT)
            .to_socket_addrs()?
            .next()
            .ok_or_else(|| io::Error::new(ErrorKind::Other, format!("Cannot resolve NTP server: {}", server)))?;
        
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        socket.connect(addr)?;
        socket.set_read_timeout(Some(timeout))?;
        socket.set_write_timeout(Some(timeout))?;
        let mut req = [0u8; NTP_PACKET_SIZE];
        req[0] = 0b00_100_011;
        let t1 = {
            program_clock.lock().unwrap().now()
        };
        let t1_ntp = NtpTimestamp::from_chrono_utc(t1)
            .map_err(|e| io::Error::new(ErrorKind::Other, format!("Cannot convert program time: {}", e)))?;
        req[TX_TS_OFFSET..TX_TS_OFFSET + 8].copy_from_slice(&t1_ntp.to_bytes());
        let send_instant = Instant::now();
        socket.send(&req)?;
        let mut buf = [0u8; NTP_PACKET_SIZE];
        let n = socket.recv(&mut buf)?;
        let recv_instant = Instant::now();
        if n < NTP_PACKET_SIZE {
            return Err(io::Error::new(ErrorKind::UnexpectedEof, "NTP response is too short"));
        }
        let round_trip_duration = recv_instant.duration_since(send_instant);
        let t4 = t1 + chrono::Duration::from_std(round_trip_duration)
            .map_err(|e| io::Error::new(ErrorKind::Other, format!("Round trip duration error: {}", e)))?;
        let t2_ntp = NtpTimestamp::from_bytes(buf[RECV_TS_OFFSET..RECV_TS_OFFSET + 8].try_into().unwrap());
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
        program_clock: &Arc<Mutex<ProgramClock>>
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
                if let Ok(result) = perform_sync(&server, &clock) {
                    if tx.send(SyncMessage::Success(result.0, result.1)).is_err() {
                        break;
                    }
                }
            }
        });
        rx
    }
}
fn initial_sync(clock: &Arc<Mutex<program_clock::ProgramClock>>) -> io::Result<()> {
    let mut rng = rand::rng();
    loop {
        let server_index = rng.random_range(0..ntp::NTP_SERVERS.len());
        let server = ntp::NTP_SERVERS[server_index];
        execute!(
            io::stdout(),
            cursor::MoveToColumn(0),
            terminal::Clear(terminal::ClearType::CurrentLine),
            Print(format!("正在尝试从 {} 进行初始同步...", server))
        )?;
        io::stdout().flush()?;
        if let Ok((initial_offset, _)) = ntp::query_ntp(server, Duration::from_millis(200), clock) {
            clock.lock().unwrap().apply_offset(initial_offset);
            println!();
            break;
        }
        thread::sleep(Duration::from_secs(1));
    }
    Ok(())
}
fn handle_sync_message(
    message: ntp::SyncMessage,
    kalman_filter: &mut kalman_filter::KalmanFilter,
    clock: &Arc<Mutex<program_clock::ProgramClock>>,
    delay_to_r_factor: f64,
) -> io::Result<()> {
    match message {
        ntp::SyncMessage::Syncing(server) => {
            execute!(
                io::stdout(),
                cursor::MoveToColumn(0),
                terminal::Clear(terminal::ClearType::CurrentLine),
                Print(format!("重新同步中 (来自: {})...", server))
            )?;
        }
        ntp::SyncMessage::Success(measured_offset, measured_delay) => {
            let measured_offset_secs = measured_offset.num_microseconds().unwrap_or(0) as f64 / 1_000_000.0;
            let measurement_noise_r = (measured_delay.num_microseconds().unwrap_or(0) as f64 / 1_000_000.0) * delay_to_r_factor;
            
            let smoothed_offset_secs = kalman_filter.update(measured_offset_secs, measurement_noise_r);
            
            let smoothed_offset = if smoothed_offset_secs < 0.0 {
                chrono::Duration::from_std(Duration::from_secs_f64(-smoothed_offset_secs)).map(|d| -d)
            } else {
                chrono::Duration::from_std(Duration::from_secs_f64(smoothed_offset_secs))
            }.unwrap_or(chrono::Duration::zero());
            clock.lock().unwrap().apply_offset(smoothed_offset);
            execute!(
                io::stdout(),
                cursor::MoveToColumn(0),
                terminal::Clear(terminal::ClearType::CurrentLine)
            )?;
            print!(
                "结果：测量偏移: {:.2}ms, 延迟: {}ms | 滤波后偏移: {:.2}ms, 漂移率: {:.2} ppm, 过程噪声: {:.1e}",
                measured_offset_secs * 1000.0,
                measured_delay.num_milliseconds(),
                smoothed_offset_secs * 1000.0,
                kalman_filter.get_drift_ppm(),
                kalman_filter.get_process_noise_q()
            );
        }
    }
    io::stdout().flush()
}
fn run_ui_loop(
    clock: Arc<Mutex<program_clock::ProgramClock>>,
    mut kalman_filter: kalman_filter::KalmanFilter,
    rx: mpsc::Receiver<ntp::SyncMessage>,
    delay_to_r_factor: f64,
) -> io::Result<()> {
    loop {
        let corrected_utc = clock.lock().unwrap().now();
        let corrected_local: DateTime<Local> = corrected_utc.with_timezone(&Local);
        execute!(
            io::stdout(),
            cursor::MoveUp(1),
            cursor::MoveToColumn(0),
            terminal::Clear(terminal::ClearType::CurrentLine),
            Print(corrected_local.format("%Y-%m-%d %H:%M:%S%.3f")),
            cursor::MoveDown(1),
            cursor::MoveToColumn(0),
        )?;
        io::stdout().flush()?;

        if let Ok(message) = rx.try_recv() {
            handle_sync_message(message, &mut kalman_filter, &clock, delay_to_r_factor)?;
        }
        thread::sleep(Duration::from_millis(2));
    }
}
fn main() -> io::Result<()> {
    const ADAPTIVE_Q_ENABLED: bool = true;
    const INITIAL_PROCESS_NOISE_Q: f64 = 5e-10;
    const DELAY_TO_R_FACTOR: f64 = 1.0;
    const INITIAL_UNCERTAINTY: f64 = 10.0;
    println!("按下 Ctrl+C 退出。");
    let clock = Arc::new(Mutex::new(program_clock::ProgramClock::new()));
    initial_sync(&clock)?;
    let kalman_filter = kalman_filter::KalmanFilter::new(
        0.0,
        INITIAL_UNCERTAINTY,
        INITIAL_PROCESS_NOISE_Q,
        ADAPTIVE_Q_ENABLED,
    );
    let rx = ntp::start_sync_thread(Arc::clone(&clock));
    run_ui_loop(clock, kalman_filter, rx, DELAY_TO_R_FACTOR)
}