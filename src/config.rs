use std::{fs, io, path::Path, time::Duration};

use chrono::{DateTime, Utc};
use serde::Deserialize;
pub const DEFAULT_CONFIG_PATH: &str = "config.yaml";
#[derive(Debug, Clone, Deserialize)]
pub struct AppConfig {
    pub kalman: KalmanConfig,
    pub ntp: NtpConfig,
    pub ui: UiConfig,
    pub clock: ClockConfig,
}
impl AppConfig {
    pub fn load() -> io::Result<Self> {
        Self::load_from_path(DEFAULT_CONFIG_PATH)
    }

    pub fn load_from_path(path: impl AsRef<Path>) -> io::Result<Self> {
        let path = path.as_ref();
        let content = fs::read_to_string(path).map_err(|e| {
            io::Error::new(
                io::ErrorKind::NotFound,
                format!("无法读取配置文件 {}: {e}", path.display()),
            )
        })?;
        let config: Self = serde_yaml::from_str(&content).map_err(|e| {
            io::Error::new(io::ErrorKind::InvalidData, format!("配置文件解析失败: {e}"))
        })?;
        config.validate()?;
        Ok(config)
    }

    fn validate(&self) -> io::Result<()> {
        self.kalman.validate()?;
        self.ntp.validate()?;
        self.ui.validate()?;
        self.clock.validate()?;
        Ok(())
    }
}
#[derive(Debug, Clone, Deserialize)]
pub struct KalmanConfig {
    pub initial_process_noise_q: f64,
    pub delay_to_r_factor: f64,
    pub initial_uncertainty: f64,
    pub adaptation_rate_eta: f64,
    pub nis_ema_alpha: f64,
}
impl KalmanConfig {
    fn validate(&self) -> io::Result<()> {
        if self.initial_uncertainty <= 0.0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "initial_uncertainty 必须为正值",
            ));
        }
        if self.initial_process_noise_q <= 0.0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "initial_process_noise_q 必须为正值",
            ));
        }
        if self.delay_to_r_factor <= 0.0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "delay_to_r_factor 必须为正值",
            ));
        }
        if !(0.0..=1.0).contains(&self.adaptation_rate_eta) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "adaptation_rate_eta 需要在 0 到 1 之间",
            ));
        }
        if !(0.0..=1.0).contains(&self.nis_ema_alpha) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "nis_ema_alpha 需要在 0 到 1 之间",
            ));
        }
        Ok(())
    }
}
#[derive(Debug, Clone, Deserialize)]
pub struct NtpConfig {
    pub servers: Vec<String>,
    pub port: u16,
    pub packet_size: usize,
    pub unix_epoch_diff_seconds: u64,
    pub recv_timestamp_offset: usize,
    pub transmit_timestamp_offset: usize,
    pub initial_sync_timeout_millis: u64,
    pub initial_sync_retry_interval_secs: u64,
    pub sync_timeout_millis: u64,
    pub sync_interval_min_secs: u64,
    pub sync_interval_max_secs: u64,
}
impl NtpConfig {
    pub const fn initial_sync_timeout(&self) -> Duration {
        Duration::from_millis(self.initial_sync_timeout_millis)
    }

    pub const fn initial_sync_retry_interval(&self) -> Duration {
        Duration::from_secs(self.initial_sync_retry_interval_secs)
    }

    pub const fn sync_timeout(&self) -> Duration {
        Duration::from_millis(self.sync_timeout_millis)
    }

    pub fn unix_epoch_diff_u32(&self) -> io::Result<u32> {
        u32::try_from(self.unix_epoch_diff_seconds).map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                "unix_epoch_diff_seconds 超出 u32 可表示范围",
            )
        })
    }

    fn validate(&self) -> io::Result<()> {
        if self.servers.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "NTP 服务器列表不能为空",
            ));
        }
        if self.packet_size == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "packet_size 必须大于 0",
            ));
        }
        if self.recv_timestamp_offset + 8 > self.packet_size {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "recv_timestamp_offset 超出数据包长度",
            ));
        }
        if self.transmit_timestamp_offset + 8 > self.packet_size {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "transmit_timestamp_offset 超出数据包长度",
            ));
        }
        if self.sync_interval_min_secs > self.sync_interval_max_secs {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "sync_interval_min_secs 不能大于 sync_interval_max_secs",
            ));
        }
        self.unix_epoch_diff_u32()?;
        Ok(())
    }
}
#[derive(Debug, Clone, Deserialize)]
pub struct UiConfig {
    pub refresh_interval_millis: u64,
    pub time_format: String,
}
impl UiConfig {
    pub const fn refresh_interval(&self) -> Duration {
        Duration::from_millis(self.refresh_interval_millis)
    }

    fn validate(&self) -> io::Result<()> {
        if self.refresh_interval_millis == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "refresh_interval_millis 必须大于 0",
            ));
        }
        if self.time_format.trim().is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "time_format 不能为空",
            ));
        }
        Ok(())
    }
}
#[derive(Debug, Clone, Deserialize)]
pub struct ClockConfig {
    pub initial_utc: String,
}
impl ClockConfig {
    pub fn initial_utc(&self) -> io::Result<DateTime<Utc>> {
        let parsed = DateTime::parse_from_rfc3339(&self.initial_utc).map_err(|e| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("initial_utc 格式无效: {e}"),
            )
        })?;
        Ok(parsed.with_timezone(&Utc))
    }

    fn validate(&self) -> io::Result<()> {
        self.initial_utc()?;
        Ok(())
    }
}
