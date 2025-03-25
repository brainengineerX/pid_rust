pub struct PIDController {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    prev_error: f64,
    integral: f64,
    integral_min: f64, // 新增：积分项最小值
    integral_max: f64, // 新增：积分项最大值
}

impl PIDController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        PIDController {
            kp,
            ki,
            kd,
            prev_error: 0.0,
            integral: 0.0,
            integral_min: -10.0, // 默认最小值
            integral_max: 10.0,  // 默认最大值
        }
    }

    pub fn update(&mut self, setpoint: f64, measured_value: f64, dt: f64) -> f64 {
        let error = setpoint - measured_value;
        self.integral += error * dt;

        // 限制积分项
        if self.integral > self.integral_max {
            self.integral = self.integral_max;
        } else if self.integral < self.integral_min {
            self.integral = self.integral_min;
        }

        let derivative = (error - self.prev_error) / dt;
        self.prev_error = error;

        self.kp * error + self.ki * self.integral + self.kd * derivative
    }
}
