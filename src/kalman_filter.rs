use quanta::Instant;
pub struct KalmanFilter {
    x_hat: [f64; 2],
    p_matrix: [[f64; 2]; 2],
    process_noise_q: f64,
    last_timestamp: Instant,
    nis_ema: f64,
    adaptation_rate_eta: f64,
    nis_ema_alpha: f64,
}
impl KalmanFilter {
    pub fn new(
        initial_offset: f64,
        initial_uncertainty: f64,
        initial_process_noise_q: f64,
        adaptation_rate_eta: f64,
        nis_ema_alpha: f64,
    ) -> Self {
        Self {
            x_hat: [initial_offset, 0.0],
            p_matrix: [[initial_uncertainty, 0.0], [0.0, initial_uncertainty]],
            process_noise_q: initial_process_noise_q,
            last_timestamp: Instant::now(),
            nis_ema: 1.0,
            adaptation_rate_eta,
            nis_ema_alpha,
        }
    }

    fn predict(&self, dt: f64) -> ([f64; 2], [[f64; 2]; 2]) {
        let f_matrix = [[1.0, dt], [0.0, 1.0]];
        let q = self.process_noise_q;
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;
        let q_matrix = [[dt3 / 3.0 * q, dt2 / 2.0 * q], [dt2 / 2.0 * q, dt * q]];
        let x_hat_predicted = [
            self.x_hat[0].mul_add(f_matrix[0][0], self.x_hat[1] * f_matrix[0][1]),
            self.x_hat[0].mul_add(f_matrix[1][0], self.x_hat[1] * f_matrix[1][1]),
        ];
        let fp = [
            [
                f_matrix[0][0].mul_add(self.p_matrix[0][0], f_matrix[0][1] * self.p_matrix[1][0]),
                f_matrix[0][0].mul_add(self.p_matrix[0][1], f_matrix[0][1] * self.p_matrix[1][1]),
            ],
            [
                f_matrix[1][0].mul_add(self.p_matrix[0][0], f_matrix[1][1] * self.p_matrix[1][0]),
                f_matrix[1][0].mul_add(self.p_matrix[0][1], f_matrix[1][1] * self.p_matrix[1][1]),
            ],
        ];
        let fpf_t = [
            [
                fp[0][0].mul_add(f_matrix[0][0], fp[0][1] * f_matrix[0][1]),
                fp[0][0].mul_add(f_matrix[1][0], fp[0][1] * f_matrix[1][1]),
            ],
            [
                fp[1][0].mul_add(f_matrix[0][0], fp[1][1] * f_matrix[0][1]),
                fp[1][0].mul_add(f_matrix[1][0], fp[1][1] * f_matrix[1][1]),
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
        let h_matrix: [f64; 2] = [1.0, 0.0];
        let y =
            measurement - h_matrix[0].mul_add(x_hat_predicted[0], h_matrix[1] * x_hat_predicted[1]);
        let hp = [
            h_matrix[0].mul_add(p_predicted[0][0], h_matrix[1] * p_predicted[1][0]),
            h_matrix[0].mul_add(p_predicted[0][1], h_matrix[1] * p_predicted[1][1]),
        ];
        let hph_t = hp[0].mul_add(h_matrix[0], hp[1] * h_matrix[1]);
        let s = hph_t + measurement_noise_r;
        let ph_t = [
            p_predicted[0][0].mul_add(h_matrix[0], p_predicted[0][1] * h_matrix[1]),
            p_predicted[1][0].mul_add(h_matrix[0], p_predicted[1][1] * h_matrix[1]),
        ];
        let k_gain = [ph_t[0] / s, ph_t[1] / s];
        self.x_hat[0] = k_gain[0].mul_add(y, x_hat_predicted[0]);
        self.x_hat[1] = k_gain[1].mul_add(y, x_hat_predicted[1]);
        let kh = [
            [k_gain[0] * h_matrix[0], k_gain[0] * h_matrix[1]],
            [k_gain[1] * h_matrix[0], k_gain[1] * h_matrix[1]],
        ];
        let ikh = [[1.0 - kh[0][0], -kh[0][1]], [-kh[1][0], 1.0 - kh[1][1]]];
        self.p_matrix = [
            [
                ikh[0][0].mul_add(p_predicted[0][0], ikh[0][1] * p_predicted[1][0]),
                ikh[0][0].mul_add(p_predicted[0][1], ikh[0][1] * p_predicted[1][1]),
            ],
            [
                ikh[1][0].mul_add(p_predicted[0][0], ikh[1][1] * p_predicted[1][0]),
                ikh[1][0].mul_add(p_predicted[0][1], ikh[1][1] * p_predicted[1][1]),
            ],
        ];
        let nis = y * y / s;
        self.nis_ema = (1.0 - self.nis_ema_alpha).mul_add(self.nis_ema, self.nis_ema_alpha * nis);
        let factor = (self.adaptation_rate_eta * (self.nis_ema - 1.0)).exp();
        self.process_noise_q *= factor;
    }

    pub fn update(&mut self, measurement: f64, measurement_noise_r: f64) -> f64 {
        let dt = self.last_timestamp.elapsed().as_secs_f64();
        self.last_timestamp = Instant::now();
        let (x_hat_predicted, p_predicted) = self.predict(dt);
        self.correct(
            measurement,
            measurement_noise_r,
            x_hat_predicted,
            p_predicted,
        );
        self.x_hat[0]
    }

    pub fn get_drift_ppm(&self) -> f64 {
        self.x_hat[1] * 1_000_000.0
    }

    pub const fn get_process_noise_q(&self) -> f64 {
        self.process_noise_q
    }
}
