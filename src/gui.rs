use eframe::egui;
use crate::pid::PIDController;

#[derive(PartialEq)] // 添加此派生宏
pub enum TargetFunction {
    Constant,
    SineWave,
    Step,
}

pub struct PIDApp {
    pid: PIDController,
    setpoint: f64,
    measured_value: f64,
    dt: f64,
    history: Vec<f64>,
    reset_flag: bool,
    target_function: TargetFunction,
    time: f64,
    paused: bool,
    setpoint_history: Vec<f64>,
    sine_period: f64, // 新增：正弦波周期
    sine_phase: f64,  // 新增：正弦波相位
    step_time: f64,   // 新增：阶跃函数切换时间
    step_low: f64,    // 新增：阶跃函数低值
    step_high: f64,   // 新增：阶跃函数高值
}

impl Default for PIDApp {
    fn default() -> Self {
        PIDApp {
            pid: PIDController::new(1.0, 0.1, 0.01),
            setpoint: 100.0,
            measured_value: 90.0,
            dt: 0.1,
            history: vec![],
            reset_flag: false,
            target_function: TargetFunction::Constant,
            time: 0.0,
            paused: false,
            setpoint_history: vec![],
            sine_period: 5.0, // 默认正弦波周期
            sine_phase: 0.0,  // 默认正弦波相位
            step_time: 5.0,   // 默认阶跃切换时间
            step_low: 50.0,   // 默认阶跃低值
            step_high: 100.0, // 默认阶跃高值
        }
    }
}

impl eframe::App for PIDApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("PID Controller Simulation");

            // Add sliders to adjust PID parameters
            ui.horizontal(|ui| {
                ui.label("Kp:");
                if ui.add(egui::Slider::new(&mut self.pid.kp, 0.0..=10.0)).changed() {
                    self.reset_flag = true;
                }
                ui.label("Ki:");
                if ui.add(egui::Slider::new(&mut self.pid.ki, 0.0..=1.0)).changed() {
                    self.reset_flag = true;
                }
                ui.label("Kd:");
                if ui.add(egui::Slider::new(&mut self.pid.kd, 0.0..=1.0)).changed() {
                    self.reset_flag = true;
                }
            });

            // Add dropdown to select target function
            ui.horizontal(|ui| {
                ui.label("Target Function:");
                egui::ComboBox::from_label("")
                    .selected_text(match self.target_function {
                        TargetFunction::Constant => "Constant",
                        TargetFunction::SineWave => "Sine Wave",
                        TargetFunction::Step => "Step",
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.target_function, TargetFunction::Constant, "Constant");
                        ui.selectable_value(&mut self.target_function, TargetFunction::SineWave, "Sine Wave");
                        ui.selectable_value(&mut self.target_function, TargetFunction::Step, "Step");
                    });
            });

            // Add target function parameter controls
            match self.target_function {
                TargetFunction::SineWave => {
                    ui.horizontal(|ui| {
                        ui.label("Sine Period:");
                        ui.add(egui::Slider::new(&mut self.sine_period, 1.0..=10.0));
                        ui.label("Sine Phase:");
                        ui.add(egui::Slider::new(&mut self.sine_phase, 0.0..=(2.0 * std::f64::consts::PI)));
                    });
                }
                TargetFunction::Step => {
                    ui.horizontal(|ui| {
                        ui.label("Step Time:");
                        ui.add(egui::Slider::new(&mut self.step_time, 1.0..=10.0));
                        ui.label("Step Low:");
                        ui.add(egui::Slider::new(&mut self.step_low, 0.0..=100.0));
                        ui.label("Step High:");
                        ui.add(egui::Slider::new(&mut self.step_high, 0.0..=100.0));
                    });
                }
                _ => {}
            }

            // Add pause/resume button
            if ui.button(if self.paused { "Resume" } else { "Pause" }).clicked() {
                self.paused = !self.paused;
            }

            // Reset history and measured value if parameters changed
            if self.reset_flag {
                self.history.clear();
                self.setpoint_history.clear();
                self.measured_value = 90.0;
                self.reset_flag = false;
                self.time = 0.0;
            }

            if !self.paused {
                // Update target function
                self.time += self.dt;
                self.setpoint = match self.target_function {
                    TargetFunction::Constant => 100.0,
                    TargetFunction::SineWave => {
                        100.0 + 20.0 * ((self.time / self.sine_period + self.sine_phase).sin())
                    }
                    TargetFunction::Step => {
                        if self.time < self.step_time {
                            self.step_low
                        } else {
                            self.step_high
                        }
                    }
                };
                self.setpoint_history.push(self.setpoint);

                // Update PID and record history
                let control = self.pid.update(self.setpoint, self.measured_value, self.dt);
                self.measured_value += control * self.dt;
                self.history.push(self.measured_value);
            }

            // Plot the graph
            egui::plot::Plot::new("PID Output")
                .view_aspect(2.0)
                .show(ui, |plot_ui| {
                    // Plot measured values
                    let points: Vec<_> = self.history.iter().enumerate()
                        .map(|(i, &v)| [i as f64 * self.dt, v])
                        .collect();
                    plot_ui.line(egui::plot::Line::new(egui::plot::PlotPoints::from(points)));

                    // Plot setpoint as a thick blue line
                    let setpoint_points: Vec<_> = self.setpoint_history.iter().enumerate()
                        .map(|(i, &v)| [i as f64 * self.dt, v])
                        .collect();
                    plot_ui.line(
                        egui::plot::Line::new(egui::plot::PlotPoints::from(setpoint_points))
                            .color(egui::Color32::BLUE)
                            .width(2.0),
                    );
                });
        });

        // Request continuous repaint to ensure the GUI updates in real-time
        ctx.request_repaint();
    }
}
