use std::{
    io::{self, Write},
    sync::{Arc, Mutex, mpsc},
    thread,
    time::Duration,
};

use chrono::{DateTime, Local};
use crossterm::{cursor, execute, style::Print, terminal};
use rand::Rng;

use crate::{config::AppConfig, kalman_filter::KalmanFilter, ntp, program_clock::ProgramClock};
pub fn run(config: AppConfig) -> io::Result<()> {
    println!("按下 Ctrl+C 退出。");
    let clock = Arc::new(Mutex::new(ProgramClock::new()));
    initial_sync(&clock)?;
    let kalman_filter = KalmanFilter::new(
        0.0,
        config.initial_uncertainty,
        config.initial_process_noise_q,
        config.adaptive_q_enabled,
    );
    let rx = ntp::start_sync_thread(Arc::clone(&clock));
    run_ui_loop(clock, kalman_filter, rx, config.delay_to_r_factor)
}
fn initial_sync(clock: &Arc<Mutex<ProgramClock>>) -> io::Result<()> {
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
                Print(format!("重新同步中 (来自: {})...", server))
            )?;
        }
        ntp::SyncMessage::Success(measured_offset, measured_delay) => {
            let measured_offset_secs =
                measured_offset.num_microseconds().unwrap_or(0) as f64 / 1_000_000.0;
            let measurement_noise_r = (measured_delay.num_microseconds().unwrap_or(0) as f64
                / 1_000_000.0)
                * delay_to_r_factor;
            let smoothed_offset_secs =
                kalman_filter.update(measured_offset_secs, measurement_noise_r);
            let smoothed_offset = if smoothed_offset_secs < 0.0 {
                chrono::Duration::from_std(Duration::from_secs_f64(-smoothed_offset_secs))
                    .map(|d| -d)
            } else {
                chrono::Duration::from_std(Duration::from_secs_f64(smoothed_offset_secs))
            }
            .unwrap_or(chrono::Duration::zero());
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
    clock: Arc<Mutex<ProgramClock>>,
    mut kalman_filter: KalmanFilter,
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
