use std::time::Instant;

use chrono::{DateTime, Utc};
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
