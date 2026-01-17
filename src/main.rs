mod app;
mod config;
mod kalman_filter;
mod ntp;
mod program_clock;
use std::io;
fn main() -> io::Result<()> {
    app::run(config::AppConfig::default())
}
