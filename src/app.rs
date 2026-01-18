use std::{
    io::{self, Write},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    thread,
    time::Duration,
};

use chrono::{DateTime, Local};
use crossterm::{cursor, execute, style::Print, terminal};
use rand::Rng;

use crate::{
    config::{AppConfig, NtpConfig, UiConfig},
    kalman_filter::KalmanFilter,
    ntp,
    program_clock::ProgramClock,
};
pub fn run(config: &AppConfig) -> io::Result<()> {
    println!("按下 Ctrl+C 退出。");
    let running = Arc::new(AtomicBool::new(true));
    let running_flag = Arc::clone(&running);
    ctrlc::set_handler(move || {
        running_flag.store(false, Ordering::SeqCst);
    })
    .map_err(|e| io::Error::other(format!("无法设置 Ctrl+C 处理器: {e}")))?;
    let initial_utc = config.clock.initial_utc()?;
    let clock = Arc::new(Mutex::new(ProgramClock::new(initial_utc)));
    if !initial_sync(&clock, &running, &config.ntp)? {
        println!();
        return Ok(());
    }
    let mut kalman_filter = KalmanFilter::new(
        0.0,
        config.kalman.initial_uncertainty,
        config.kalman.initial_process_noise_q,
        config.kalman.adaptation_rate_eta,
        config.kalman.nis_ema_alpha,
    );
    let rx = ntp::start_sync_thread(Arc::clone(&clock), config.ntp.clone());
    let result = run_ui_loop(
        &clock,
        &mut kalman_filter,
        &rx,
        &config.ui,
        config.kalman.delay_to_r_factor,
        &running,
    );
    println!();
    result
}
fn initial_sync(
    clock: &Arc<Mutex<ProgramClock>>,
    running: &AtomicBool,
    ntp_config: &NtpConfig,
) -> io::Result<bool> {
    let mut rng = rand::rng();
    loop {
        if !running.load(Ordering::SeqCst) {
            return Ok(false);
        }
        let server_index = rng.random_range(0..ntp_config.servers.len());
        let server = &ntp_config.servers[server_index];
        execute!(
            io::stdout(),
            cursor::MoveToColumn(0),
            terminal::Clear(terminal::ClearType::CurrentLine),
            Print(format!("正在尝试从 {server} 进行初始同步..."))
        )?;
        io::stdout().flush()?;
        if let Ok((initial_offset, _)) =
            ntp::query_ntp(server, ntp_config.initial_sync_timeout(), clock, ntp_config)
        {
            clock.lock().unwrap().apply_offset(initial_offset);
            println!();
            return Ok(true);
        }
        thread::sleep(ntp_config.initial_sync_retry_interval());
    }
}
fn handle_sync_message(
    message: ntp::SyncMessage,
    kalman_filter: &mut KalmanFilter,
    clock: &Arc<Mutex<ProgramClock>>,
    delay_to_r_factor: f64,
) -> io::Result<()> {
    match message {
        ntp::SyncMessage::Syncing(server) => {
            execute!(
                io::stdout(),
                cursor::MoveToColumn(0),
                terminal::Clear(terminal::ClearType::CurrentLine),
                Print(format!("重新同步中 (来自: {server})..."))
            )?;
        }
        ntp::SyncMessage::Success(measured_offset, measured_delay) => {
            fn micros_to_secs(micros: i64, what: &'static str) -> io::Result<f64> {
                const MAX_SAFE_INTEGER_IN_F64: u64 = 9_007_199_254_740_992; // 2^53
                const TWO_POW_32: f64 = 4_294_967_296.0;
                let micros_abs = micros.unsigned_abs();
                if micros_abs > MAX_SAFE_INTEGER_IN_F64 {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("{what} 超出 f64 可精确表示的整数范围"),
                    ));
                }
                let high = u32::try_from(micros_abs >> 32).map_err(|_| {
                    io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("{what} 超出 u32 可表示范围"),
                    )
                })?;
                let low = u32::try_from(micros_abs & 0xFFFF_FFFF).map_err(|_| {
                    io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("{what} 超出 u32 可表示范围"),
                    )
                })?;
                let micros_f64 = f64::from(high) * TWO_POW_32 + f64::from(low);
                let micros_secs = micros_f64 / 1_000_000.0;
                Ok(if micros < 0 {
                    -micros_secs
                } else {
                    micros_secs
                })
            }
            let measured_offset_micros = measured_offset.num_microseconds().ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::InvalidData,
                    "NTP measured_offset 超出 microseconds 可表示范围",
                )
            })?;
            let measured_offset_secs = micros_to_secs(measured_offset_micros, "measured_offset")?;
            let measured_delay_micros = measured_delay.num_microseconds().ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::InvalidData,
                    "NTP measured_delay 超出 microseconds 可表示范围",
                )
            })?;
            let measurement_noise_r =
                micros_to_secs(measured_delay_micros, "measured_delay")? * delay_to_r_factor;
            let smoothed_offset_secs =
                kalman_filter.update(measured_offset_secs, measurement_noise_r);
            let smoothed_offset = if smoothed_offset_secs < 0.0 {
                chrono::Duration::from_std(Duration::from_secs_f64(-smoothed_offset_secs))
                    .map(|d| -d)
            } else {
                chrono::Duration::from_std(Duration::from_secs_f64(smoothed_offset_secs))
            }
            .map_err(|e| {
                io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("滤波偏移无法转换为 chrono::Duration: {e}"),
                )
            })?;
            clock.lock().unwrap().apply_offset(smoothed_offset);
            execute!(
                io::stdout(),
                cursor::MoveToColumn(0),
                terminal::Clear(terminal::ClearType::CurrentLine)
            )?;
            print!(
                "结果：测量偏移: {:.2}ms, 延迟: {}ms | 滤波后偏移: {:.2}ms, 漂移率: {:.2} ppm, \
                 过程噪声: {:.1e}",
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
    clock: &Arc<Mutex<ProgramClock>>,
    kalman_filter: &mut KalmanFilter,
    rx: &mpsc::Receiver<ntp::SyncMessage>,
    ui_config: &UiConfig,
    delay_to_r_factor: f64,
    running: &AtomicBool,
) -> io::Result<()> {
    while running.load(Ordering::SeqCst) {
        let corrected_utc = clock.lock().unwrap().now();
        let corrected_local: DateTime<Local> = corrected_utc.with_timezone(&Local);
        execute!(
            io::stdout(),
            cursor::MoveUp(1),
            cursor::MoveToColumn(0),
            terminal::Clear(terminal::ClearType::CurrentLine),
            Print(corrected_local.format(&ui_config.time_format)),
            cursor::MoveDown(1),
            cursor::MoveToColumn(0),
        )?;
        io::stdout().flush()?;
        if let Ok(message) = rx.try_recv() {
            handle_sync_message(message, kalman_filter, clock, delay_to_r_factor)?;
        }
        thread::sleep(ui_config.refresh_interval());
    }
    Ok(())
}
