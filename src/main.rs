mod pid;
mod gui;

fn main() {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "PID Controller Simulation",
        options,
        Box::new(|_cc| Box::new(gui::PIDApp::default())),
    );
}
