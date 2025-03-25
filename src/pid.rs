pub struct PIDController {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    prev_error: f64,
    integral: f64,
    integral_min: f64, // 新增：积分项最小值
    integral_max: f64, // 新增：积分项最大值
    deadband: f64,
    output_min: f64,
    output_max: f64,
    derivative_filter: f64,
    filtered_derivative: f64,
    integral_threshold: f64, // 新增：积分分离阈值
}

impl PIDController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        assert!(!kp.is_nan() && !ki.is_nan() && !kd.is_nan(), "PID parameters cannot be NaN");
        PIDController {
            kp,
            ki,
            kd,
            prev_error: 0.0,
            integral: 0.0,
            integral_min: -10.0, // 默认最小值
            integral_max: 10.0,  // 默认最大值
            deadband: 0.0,       // 默认死区
            output_min: f64::NEG_INFINITY,
            output_max: f64::INFINITY,
            derivative_filter: 0.1, // 默认滤波系数
            filtered_derivative: 0.0,
            integral_threshold: 1.0, // 默认积分分离阈值
        }
    }

    pub fn update(&mut self, setpoint: f64, measured_value: f64, dt: f64) -> f64 {
        assert!(dt > 0.0, "Time step must be positive");
        assert!(!setpoint.is_nan() && !measured_value.is_nan(), "Setpoint and measured value cannot be NaN");

        let error = setpoint - measured_value;
        
        // 应用死区
        if error.abs() < self.deadband {
            return 0.0;
        }

        // 积分分离：当误差较大时减小或禁用积分作用
        let integral_factor = if error.abs() > self.integral_threshold {
            0.0
        } else {
            (1.0 - error.abs() / self.integral_threshold).powi(2)
        };
        
        self.integral += integral_factor * error * dt;

        // 限制积分项
        if self.integral > self.integral_max {
            self.integral = self.integral_max;
        } else if self.integral < self.integral_min {
            self.integral = self.integral_min;
        }

        let derivative = (error - self.prev_error) / dt;
        self.filtered_derivative = self.derivative_filter * derivative + 
            (1.0 - self.derivative_filter) * self.filtered_derivative;
        self.prev_error = error;

        let output = self.kp * error + 
                     self.ki * self.integral + 
                     self.kd * self.filtered_derivative;
        output.clamp(self.output_min, self.output_max)
    }

    pub fn reset(&mut self) {
        self.prev_error = 0.0;
        self.integral = 0.0;
    }

    pub fn set_integral_limits(&mut self, min: f64, max: f64) {
        if min <= max {
            self.integral_min = min;
            self.integral_max = max;
        }
    }

    pub fn set_output_limits(&mut self, min: f64, max: f64) {
        if min <= max {
            self.output_min = min;
            self.output_max = max;
        }
    }

    pub fn set_derivative_filter(&mut self, filter: f64) {
        if (0.0..=1.0).contains(&filter) {
            self.derivative_filter = filter;
        }
    }
}

pub struct AdaptivePIDController {
    pub pid: PIDController,
    learning_rate: f64,
    error_threshold: f64,
    last_adaptation_time: f64,
    adaptation_interval: f64,
}

impl AdaptivePIDController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            pid: PIDController::new(kp, ki, kd),
            learning_rate: 0.01,
            error_threshold: 0.1,
            last_adaptation_time: 0.0,
            adaptation_interval: 1.0, // 1秒适应一次
        }
    }

    pub fn update(&mut self, setpoint: f64, measured_value: f64, dt: f64) -> f64 {
        let error = setpoint - measured_value;
        let output = self.pid.update(setpoint, measured_value, dt);
        
        // 自适应参数调整
        if self.should_adapt(dt) {
            self.adapt_parameters(error);
        }
        
        output
    }

    fn should_adapt(&mut self, dt: f64) -> bool {
        self.last_adaptation_time += dt;
        if self.last_adaptation_time >= self.adaptation_interval {
            self.last_adaptation_time = 0.0;
            true
        } else {
            false
        }
    }

    fn adapt_parameters(&mut self, error: f64) {
        if error.abs() > self.error_threshold {
            // 误差较大时增大比例增益
            self.pid.kp += self.learning_rate * error.abs();
            // 减小积分增益防止积分饱和
            self.pid.ki -= self.learning_rate * self.pid.ki;
            // 增大微分增益以加快响应
            self.pid.kd += self.learning_rate * error.abs();
        } else {
            // 误差较小时减小比例和微分增益
            self.pid.kp -= self.learning_rate * self.pid.kp;
            // 增大积分增益以消除稳态误差
            self.pid.ki += self.learning_rate * error.abs();
            self.pid.kd -= self.learning_rate * self.pid.kd;
        }

        // 确保增益不会变得过大或过小
        self.pid.kp = self.pid.kp.clamp(0.1, 10.0);
        self.pid.ki = self.pid.ki.clamp(0.01, 1.0);
        self.pid.kd = self.pid.kd.clamp(0.0, 2.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_output_limits() {
        let mut pid = PIDController::new(1.0, 0.0, 0.0);
        pid.set_output_limits(-1.0, 1.0);
        assert_eq!(pid.update(2.0, 0.0, 0.1), 1.0); // 输出限幅测试
    }

    #[test]
    fn test_derivative_filter() {
        let mut pid = PIDController::new(0.0, 0.0, 1.0);
        pid.set_derivative_filter(0.5);
        // 添加滤波测试用例
    }

    #[test]
    fn test_step_response() {
        let mut pid = PIDController::new(1.0, 0.1, 0.05);
        let setpoint = 100.0;
        let dt = 0.01;
        
        let mut value = 0.0;
        let mut max_overshoot = 0.0;
        let mut settling_time = 0.0;
        let mut rise_time = 0.0;
        let mut found_rise = false;
        
        for i in 0..1000 {  // 模拟10秒
            let output = pid.update(setpoint, value, dt);
            value += output * dt;
            
            // 计算超调量
            if value > max_overshoot {
                max_overshoot = value;
            }
            
            // 计算上升时间 (0.1 到 0.9 的时间)
            if !found_rise && value >= setpoint * 0.9 {
                rise_time = i as f64 * dt;
                found_rise = true;
            }
            
            // 计算稳定时间 (在±2%范围内)
            if (value - setpoint).abs() <= setpoint * 0.02 {
                settling_time = i as f64 * dt;
                break;
            }
        }
        
        println!("Performance Metrics:");
        println!("Max Overshoot: {:.2}%", (max_overshoot - setpoint) / setpoint * 100.0);
        println!("Rise Time: {:.2}s", rise_time);
        println!("Settling Time: {:.2}s", settling_time);
        
        // 性能指标断言
        assert!(max_overshoot <= setpoint * 1.2); // 超调不超过20%
        assert!(rise_time <= 1.0);  // 上升时间不超过1秒
        assert!(settling_time <= 3.0);  // 稳定时间不超过3秒
    }

    #[test]
    fn test_disturbance_rejection() {
        let mut pid = PIDController::new(1.0, 0.1, 0.05);
        let setpoint = 50.0;
        let dt = 0.01;
        let mut value = setpoint;
        
        // 模拟系统达到稳定状态
        for _ in 0..100 {
            pid.update(setpoint, value, dt);
        }
        
        // 添加干扰
        value += 10.0;
        
        let mut recovery_time = 0.0;
        for i in 0..500 {  // 模拟5秒
            let output = pid.update(setpoint, value, dt);
            value += output * dt;
            
            if (value - setpoint).abs() <= setpoint * 0.02 {
                recovery_time = i as f64 * dt;
                break;
            }
        }
        
        println!("Disturbance Recovery Time: {:.2}s", recovery_time);
        assert!(recovery_time <= 2.0);  // 干扰恢复时间不超过2秒
    }

    #[test]
    fn test_adaptive_pid() {
        let mut pid = AdaptivePIDController::new(1.0, 0.1, 0.05);
        let setpoint = 100.0;
        let dt = 0.01;
        
        let mut value = 0.0;
        let mut initial_kp = pid.pid.kp;
        
        // 运行系统一段时间
        for _ in 0..1000 {
            let output = pid.update(setpoint, value, dt);
            value += output * dt;
        }
        
        // 验证参数是否已适应
        assert_ne!(initial_kp, pid.pid.kp, "PID parameters should adapt");
        assert!((value - setpoint).abs() < 2.0, "System should reach setpoint");
        
        println!("Adapted Parameters:");
        println!("Kp: {:.3}", pid.pid.kp);
        println!("Ki: {:.3}", pid.pid.ki);
        println!("Kd: {:.3}", pid.pid.kd);
    }
}
