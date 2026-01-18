mod app;
mod config;
mod kalman_filter;
mod ntp;
mod program_clock;
use std::io;
fn main() -> io::Result<()> {
    let config = config::AppConfig::load()?;
    app::run(&config)
}
