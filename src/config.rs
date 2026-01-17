#[derive(Clone, Copy)]
pub struct AppConfig {
    pub initial_process_noise_q: f64,
    pub delay_to_r_factor: f64,
    pub initial_uncertainty: f64,
}
impl Default for AppConfig {
    fn default() -> Self {
        Self {
            initial_process_noise_q: 5e-10,
            delay_to_r_factor: 1.0,
            initial_uncertainty: 10.0,
        }
    }
}
