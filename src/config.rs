use serde::Deserialize;
use serde_yaml::Value;
use std::fs::File;
use std::io::Read;

#[derive(Deserialize)]
pub struct Config {
    pub simulation: SimulationConfig,
    pub quadrotor: QuadrotorConfig,
    pub controller: ControllerConfig,
    pub imu: ImuConfig,
    pub maze: MazeConfig,
    pub camera: CameraConfig,
    pub mesh: MeshConfig,
    pub planner_schedule: PlannerScheduleConfig,
}

#[derive(Deserialize)]
pub struct PlannerScheduleConfig {
    pub steps: Vec<PlannerStep>,
}

#[derive(Deserialize)]
pub struct PlannerStep {
    pub step: usize,
    pub planner_type: String,
    pub params: Value,
}
#[derive(Deserialize)]
pub struct SimulationConfig {
    pub control_frequency: f32,
    pub simulation_frequency: f32,
    pub log_frequency: f32,
    pub duration: f32,
}

#[derive(Deserialize)]
pub struct QuadrotorConfig {
    pub mass: f32,
    pub gravity: f32,
    pub drag_coefficient: f32,
    pub inertia_matrix: [f32; 9],
}

#[derive(Deserialize)]
pub struct ControllerConfig {
    pub pos_gains: PIDGains,
    pub att_gains: PIDGains,
    pub pos_max_int: [f32; 3],
    pub att_max_int: [f32; 3],
}

#[derive(Deserialize)]
pub struct PIDGains {
    pub kp: [f32; 3],
    pub ki: [f32; 3],
    pub kd: [f32; 3],
}

#[derive(Deserialize, Default)]
pub struct ImuConfig {
    pub accel_noise_std: f32,
    pub gyro_noise_std: f32,
    pub bias_instability: f32,
}

#[derive(Deserialize)]
pub struct MazeConfig {
    pub upper_bounds: [f32; 3],
    pub lower_bounds: [f32; 3],
    pub num_obstacles: usize,
}

#[derive(Deserialize)]
pub struct CameraConfig {
    pub resolution: (usize, usize),
    pub fov: f32,
    pub near: f32,
    pub far: f32,
}

#[derive(Deserialize)]
pub struct MeshConfig {
    pub division: usize,
    pub spacing: f32,
}
impl Config {
    pub fn from_yaml(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let mut contents = String::new();
        File::open(filename)?.read_to_string(&mut contents)?;
        Ok(serde_yaml::from_str(&contents)?)
    }
}
