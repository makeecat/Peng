//! # Peng - A Minimal Quadrotor Autonomy Framework
//!
//! A high-performance quadrotor autonomy framework written in Rust that provides
//! real-time dynamics simulation, trajectory planning, and control with modern
//! visualization capabilities.
//!
//! # Features
//!
//! ## Real-time Simulation
//! - High-fidelity quadrotor dynamics with configurable parameters
//! - IMU and depth sensor simulation
//! - Optional RK4 integration for accurate dynamics
//!
//! ## Advanced Control
//! - PID control for position and attitude with tunable gains
//! - Integral windup prevention
//! - Support for different control frequencies
//!
//! ## Rich Trajectory Planning
//! - Minimum jerk line trajectory planner
//! - Lissajous curve planner
//! - Circular trajectory planner
//! - Obstacle avoidance planner
//! - Waypoint navigation planner
//! - Landing planner
//!
//! ## Visualization & Debug
//! - Real-time 3D visualization via rerun.io
//! - Depth map rendering
//! - State telemetry logging
//! - Configurable logging frequencies
//!
//! ## Performance
//! - Memory-safe and Efficient Rust implementation
//! - Multi-threaded depth rendering
//!
//! ## Example
//! ```
//! use nalgebra::Vector3;
//! use peng_quad::{Quadrotor, SimulationError};
//! let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
//! let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
//! let quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix);
//! ```
use rand::SeedableRng;
use rayon::prelude::*;
pub mod config;
use nalgebra::{
    DMatrix, DVector, Matrix3, Quaternion, Rotation3, SMatrix, UnitQuaternion, Vector3,
};
use osqp::{CscMatrix, Problem, Settings};
use rand_chacha::ChaCha8Rng;
use rand_distr::{Distribution, Normal};
use std::f32::consts::PI;
use std::ops::AddAssign;
#[derive(thiserror::Error, Debug)]
/// Represents errors that can occur during simulation
/// # Example
/// ```
/// use peng_quad::SimulationError;
/// let error = SimulationError::NalgebraError("Matrix inversion failed".to_string());
/// ```
pub enum SimulationError {
    /// Error related to Rerun visualization
    #[error("Rerun error: {0}")]
    RerunError(#[from] rerun::RecordingStreamError),
    /// Error related to Rerun spawn process
    #[error("Rerun spawn error: {0}")]
    RerunSpawnError(#[from] rerun::SpawnError),
    /// Error related to Rerun logger setup
    #[error("Rerun SetLogger error: {0}")]
    SetLoggerError(#[from] rerun::external::log::SetLoggerError),
    /// Error related to linear algebra operations
    #[error("Nalgebra error: {0}")]
    NalgebraError(String),
    /// Error related to the OSQP solver
    #[error("OSQP error: {0}")]
    OSQPError(String),
    /// Error related to normal distribution calculations
    #[error("Normal error: {0}")]
    NormalError(#[from] rand_distr::NormalError),
    /// Other general errors
    #[error("Other error: {0}")]
    OtherError(String),
}
/// Represents a quadrotor with its physical properties and state
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::Quadrotor;
/// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
/// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
/// let quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix);
/// ```
pub struct Quadrotor {
    /// Current position of the quadrotor in 3D space
    pub position: Vector3<f32>,
    /// Current velocity of the quadrotor
    pub velocity: Vector3<f32>,
    /// Current acceleration of the quadrotor
    pub acceleration: Vector3<f32>,
    /// Current orientation of the quadrotor
    pub orientation: UnitQuaternion<f32>,
    /// Current angular velocity of the quadrotor
    pub angular_velocity: Vector3<f32>,
    /// Mass of the quadrotor in kg
    pub mass: f32,
    /// Gravitational acceleration in m/s^2
    pub gravity: f32,
    /// Simulation time step in seconds
    pub time_step: f32,
    /// Drag coefficient
    pub drag_coefficient: f32,
    /// Inertia matrix of the quadrotor
    pub inertia_matrix: Matrix3<f32>,
    /// Inverse of the inertia matrix
    pub inertia_matrix_inv: Matrix3<f32>,
}
/// Implementation of the Quadrotor struct
impl Quadrotor {
    /// Creates a new Quadrotor with default parameters
    /// # Arguments
    /// * `time_step` - The simulation time step in seconds
    /// * `mass` - The mass of the quadrotor in kg
    /// * `gravity` - The gravitational acceleration in m/s^2
    /// * `drag_coefficient` - The drag coefficient
    /// * `inertia_matrix` - The inertia matrix of the quadrotor
    /// # Returns
    /// * A new Quadrotor instance
    /// # Errors
    /// * Returns a SimulationError if the inertia matrix cannot be inverted
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    ///
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix);
    /// ```
    pub fn new(
        time_step: f32,
        mass: f32,
        gravity: f32,
        drag_coefficient: f32,
        inertia_matrix: [f32; 9],
    ) -> Result<Self, SimulationError> {
        let inertia_matrix = Matrix3::from_row_slice(&inertia_matrix);
        let inertia_matrix_inv =
            inertia_matrix
                .try_inverse()
                .ok_or(SimulationError::NalgebraError(
                    "Failed to invert inertia matrix".to_string(),
                ))?;
        Ok(Self {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            mass,
            gravity,
            time_step,
            drag_coefficient,
            inertia_matrix,
            inertia_matrix_inv,
        })
    }
    /// Updates the quadrotor's dynamics with control inputs
    /// # Arguments
    /// * `control_thrust` - The total thrust force applied to the quadrotor
    /// * `control_torque` - The 3D torque vector applied to the quadrotor
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    ///
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let mut quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
    /// let control_thrust = mass * gravity;
    /// let control_torque = Vector3::new(0.0, 0.0, 0.0);
    /// quadrotor.update_dynamics_with_controls_euler(control_thrust, &control_torque);
    /// ```
    pub fn update_dynamics_with_controls_euler(
        &mut self,
        control_thrust: f32,
        control_torque: &Vector3<f32>,
    ) {
        let gravity_force = Vector3::new(0.0, 0.0, -self.mass * self.gravity);
        let drag_force = -self.drag_coefficient * self.velocity.norm() * self.velocity;
        let thrust_world = self.orientation * Vector3::new(0.0, 0.0, control_thrust);
        self.acceleration = (thrust_world + gravity_force + drag_force) / self.mass;
        self.velocity += self.acceleration * self.time_step;
        self.position += self.velocity * self.time_step;
        let inertia_angular_velocity = self.inertia_matrix * self.angular_velocity;
        let gyroscopic_torque = self.angular_velocity.cross(&inertia_angular_velocity);
        let angular_acceleration = self.inertia_matrix_inv * (control_torque - gyroscopic_torque);
        self.angular_velocity += angular_acceleration * self.time_step;
        self.orientation *=
            UnitQuaternion::from_scaled_axis(self.angular_velocity * self.time_step);
    }
    /// Updates the quadrotor's dynamics with control inputs using the Runge-Kutta 4th order method
    /// # Arguments
    /// * `control_thrust` - The total thrust force applied to the quadrotor
    /// * `control_torque` - The 3D torque vector applied to the quadrotor
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let mut quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
    /// let control_thrust = mass * gravity;
    /// let control_torque = Vector3::new(0.0, 0.0, 0.0);
    /// quadrotor.update_dynamics_with_controls_rk4(control_thrust, &control_torque);
    /// ```
    pub fn update_dynamics_with_controls_rk4(
        &mut self,
        control_thrust: f32,
        control_torque: &Vector3<f32>,
    ) {
        let h = self.time_step;
        let state = self.get_state();

        let k1 = self.state_derivative(&state, control_thrust, control_torque);
        let mut temp_state = [0.0; 13];
        for i in 0..13 {
            temp_state[i] = state[i] + 0.5 * h * k1[i];
        }
        let k2 = self.state_derivative(&temp_state, control_thrust, control_torque);

        for i in 0..13 {
            temp_state[i] = state[i] + 0.5 * h * k2[i];
        }
        let k3 = self.state_derivative(&temp_state, control_thrust, control_torque);

        for i in 0..13 {
            temp_state[i] = state[i] + h * k3[i];
        }
        let k4 = self.state_derivative(&temp_state, control_thrust, control_torque);

        let mut new_state = [0.0; 13];
        for i in 0..13 {
            new_state[i] = state[i] + (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }

        let mut q = Quaternion::new(new_state[9], new_state[6], new_state[7], new_state[8]);
        q = q.normalize();
        new_state[6..10].copy_from_slice(q.coords.as_slice());

        self.set_state(&new_state);
    }
    /// Returns the state derivative of the quadrotor
    /// # Arguments
    /// * `state` - The state of the quadrotor
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    /// use nalgebra::UnitQuaternion;
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
    /// let state = quadrotor.get_state();
    /// ```
    pub fn get_state(&self) -> [f32; 13] {
        let mut state = [0.0; 13];
        state[0..3].copy_from_slice(self.position.as_slice());
        state[3..6].copy_from_slice(self.velocity.as_slice());
        state[6..10].copy_from_slice(self.orientation.coords.as_slice());
        state[10..13].copy_from_slice(self.angular_velocity.as_slice());
        state
    }
    /// Sets the state of the quadrotor
    /// # Arguments
    /// * `state` - The state of the quadrotor
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    /// use nalgebra::UnitQuaternion;
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let mut quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
    /// let state = [
    ///    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    /// ];
    /// quadrotor.set_state(&state);
    /// ```
    pub fn set_state(&mut self, state: &[f32; 13]) {
        self.position = Vector3::from_column_slice(&state[0..3]);
        self.velocity = Vector3::from_column_slice(&state[3..6]);
        self.orientation = UnitQuaternion::from_quaternion(Quaternion::new(
            state[9], state[6], state[7], state[8],
        ));
        self.angular_velocity = Vector3::from_column_slice(&state[10..13]);
    }
    /// Calculates the derivative of the state of the quadrotor
    /// # Arguments
    /// * `state` - The current state of the quadrotor
    /// * `control_thrust` - The thrust applied to the quadrotor
    /// * `control_torque` - The torque applied to the quadrotor
    /// # Returns
    /// The derivative of the state
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    /// use nalgebra::UnitQuaternion;
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let mut quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
    /// let state = [
    ///   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    /// ];
    /// let control_thrust = 0.0;
    /// let control_torque = Vector3::new(0.0, 0.0, 0.0);
    /// let derivative = quadrotor.state_derivative(&state, control_thrust, &control_torque);
    /// ```
    pub fn state_derivative(
        &mut self,
        state: &[f32],
        control_thrust: f32,
        control_torque: &Vector3<f32>,
    ) -> [f32; 13] {
        let velocity = Vector3::from_column_slice(&state[3..6]);
        let orientation = UnitQuaternion::from_quaternion(Quaternion::new(
            state[9], state[6], state[7], state[8],
        ));
        // Quaternion deriviative
        let omega_quat = Quaternion::new(0.0, state[10], state[11], state[12]);
        let q_dot = orientation.into_inner() * omega_quat * 0.5;

        let angular_velocity = Vector3::from_column_slice(&state[10..13]);

        let gravity_force = Vector3::new(0.0, 0.0, -self.mass * self.gravity);
        let drag_force = -self.drag_coefficient * velocity.norm() * velocity;
        let thrust_world = orientation * Vector3::new(0.0, 0.0, control_thrust);
        self.acceleration = (thrust_world + gravity_force + drag_force) / self.mass;

        let inertia_angular_velocity = self.inertia_matrix * angular_velocity;
        let gyroscopic_torque = angular_velocity.cross(&inertia_angular_velocity);
        let angular_acceleration = self.inertia_matrix_inv * (control_torque - gyroscopic_torque);

        let mut derivative = [0.0; 13];
        derivative[0..3].copy_from_slice(velocity.as_slice());
        derivative[3..6].copy_from_slice(self.acceleration.as_slice());
        derivative[6..10].copy_from_slice(q_dot.coords.as_slice());
        derivative[10..13].copy_from_slice(angular_acceleration.as_slice());
        derivative
    }
    /// Simulates IMU readings
    /// # Returns
    /// * A tuple containing the true acceleration and angular velocity of the quadrotor
    /// # Errors
    /// * Returns a SimulationError if the IMU readings cannot be calculated
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::Quadrotor;
    ///
    /// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
    /// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
    /// let quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
    /// let (true_acceleration, true_angular_velocity) = quadrotor.read_imu().unwrap();
    /// ```
    pub fn read_imu(&self) -> Result<(Vector3<f32>, Vector3<f32>), SimulationError> {
        Ok((self.acceleration, self.angular_velocity))
    }
}
/// Represents an Inertial Measurement Unit (IMU) with bias and noise characteristics
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::Imu;
/// let accel_noise_std = 0.0003;
/// let gyro_noise_std = 0.02;
/// let accel_bias_std = 0.0001;
/// let gyro_bias_std = 0.001;
/// let imu = Imu::new(accel_noise_std, gyro_noise_std, accel_bias_std, gyro_bias_std);
/// ```
pub struct Imu {
    /// Accelerometer bias
    pub accel_bias: Vector3<f32>,
    /// Gyroscope bias
    pub gyro_bias: Vector3<f32>,
    /// Standard deviation of accelerometer noise
    pub accel_noise_std: f32,
    /// Standard deviation of gyroscope noise
    pub gyro_noise_std: f32,
    /// Standard deviation of accelerometer bias drift
    pub accel_bias_std: f32,
    /// Standard deviation of gyroscope bias drift
    pub gyro_bias_std: f32,
    /// Accelerometer noise distribution
    accel_noise: Normal<f32>,
    /// Gyroscope noise distribution
    gyro_noise: Normal<f32>,
    /// Accelerometer bias drift distribution
    accel_bias_drift: Normal<f32>,
    /// Gyroscope bias drift distribution
    gyro_bias_drift: Normal<f32>,
    /// Random number generator
    rng: ChaCha8Rng,
}
/// Implements the IMU
impl Imu {
    /// Creates a new IMU with default parameters
    /// # Arguments
    /// * `accel_noise_std` - Standard deviation of accelerometer noise
    /// * `gyro_noise_std` - Standard deviation of gyroscope noise
    /// * `accel_bias_std` - Standard deviation of accelerometer bias drift
    /// * `gyro_bias_std` - Standard deviation of gyroscope bias drift
    /// # Returns
    /// * A new Imu instance
    /// # Example
    /// ```
    /// use peng_quad::Imu;
    ///
    /// let imu = Imu::new(0.01, 0.01, 0.01, 0.01);
    /// ```
    pub fn new(
        accel_noise_std: f32,
        gyro_noise_std: f32,
        accel_bias_std: f32,
        gyro_bias_std: f32,
    ) -> Result<Self, SimulationError> {
        Ok(Self {
            accel_bias: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            accel_noise_std,
            gyro_noise_std,
            accel_bias_std,
            gyro_bias_std,
            accel_noise: Normal::new(0.0, accel_noise_std)?,
            gyro_noise: Normal::new(0.0, gyro_noise_std)?,
            accel_bias_drift: Normal::new(0.0, accel_bias_std)?,
            gyro_bias_drift: Normal::new(0.0, gyro_bias_std)?,
            rng: ChaCha8Rng::from_os_rng(),
        })
    }
    /// Updates the IMU biases over time
    /// # Arguments
    /// * `dt` - Time step for the update
    /// # Errors
    /// * Returns a SimulationError if the bias drift cannot be calculated
    /// # Example
    /// ```
    /// use peng_quad::Imu;
    ///
    /// let mut imu = Imu::new(0.01, 0.01, 0.01, 0.01).unwrap();
    /// imu.update(0.01).unwrap();
    /// ```
    pub fn update(&mut self, dt: f32) -> Result<(), SimulationError> {
        let dt_sqrt = fast_sqrt(dt);
        let accel_drift = self.accel_bias_drift.sample(&mut self.rng) * dt_sqrt;
        let gyro_drift = self.gyro_bias_drift.sample(&mut self.rng) * dt_sqrt;
        self.accel_bias += Vector3::from_iterator((0..3).map(|_| accel_drift));
        self.gyro_bias += Vector3::from_iterator((0..3).map(|_| gyro_drift));
        Ok(())
    }
    /// Simulates IMU readings with added bias and noise
    ///
    /// The added bias and noise are based on normal distributions
    /// # Arguments
    /// * `true_acceleration` - The true acceleration vector
    /// * `true_angular_velocity` - The true angular velocity vector
    /// # Returns
    /// * A tuple containing the measured acceleration and angular velocity
    /// # Errors
    /// * Returns a SimulationError if the IMU readings cannot be calculated
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::Imu;
    ///
    /// let mut imu = Imu::new(0.01, 0.01, 0.01, 0.01).unwrap();
    /// let true_acceleration = Vector3::new(0.0, 0.0, 9.81);
    /// let true_angular_velocity = Vector3::new(0.0, 0.0, 0.0);
    /// let (measured_acceleration, measured_ang_velocity) = imu.read(true_acceleration, true_angular_velocity).unwrap();
    /// ```
    pub fn read(
        &mut self,
        true_acceleration: Vector3<f32>,
        true_angular_velocity: Vector3<f32>,
    ) -> Result<(Vector3<f32>, Vector3<f32>), SimulationError> {
        let accel_noise_sample =
            Vector3::from_iterator((0..3).map(|_| self.accel_noise.sample(&mut self.rng)));
        let gyro_noise_sample =
            Vector3::from_iterator((0..3).map(|_| self.gyro_noise.sample(&mut self.rng)));
        let measured_acceleration = true_acceleration + self.accel_bias + accel_noise_sample;
        let measured_ang_velocity = true_angular_velocity + self.gyro_bias + gyro_noise_sample;
        Ok((measured_acceleration, measured_ang_velocity))
    }
}
/// PID controller for quadrotor position and attitude control
///
/// The kpid_pos and kpid_att gains are following the format of
/// proportional, derivative and integral gains
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::PIDController;
/// let kpid_pos = [
///     [1.0, 1.0, 1.0],
///     [0.1, 0.1, 0.1],
///     [0.01, 0.01, 0.01],
/// ];
/// let kpid_att = [
///     [1.0, 1.0, 1.0],
///     [0.1, 0.1, 0.1],
///     [0.01, 0.01, 0.01],
/// ];
/// let max_integral_pos = [1.0, 1.0, 1.0];
/// let max_integral_att = [1.0, 1.0, 1.0];
/// let mass = 1.0;
/// let gravity = 9.81;
/// let pid_controller = PIDController::new(kpid_pos, kpid_att, max_integral_pos, max_integral_att, mass, gravity);
/// ```
pub struct PIDController {
    /// PID gain for position control including proportional, derivative, and integral gains
    pub kpid_pos: [Vector3<f32>; 3],
    /// PID gain for attitude control including proportional, derivative, and integral gains
    pub kpid_att: [Vector3<f32>; 3],
    /// Accumulated integral error for position
    pub integral_pos_error: Vector3<f32>,
    /// Accumulated integral error for attitude
    pub integral_att_error: Vector3<f32>,
    /// Maximum allowed integral error for position
    pub max_integral_pos: Vector3<f32>,
    /// Maximum allowed integral error for attitude
    pub max_integral_att: Vector3<f32>,
    /// Mass of the quadrotor
    pub mass: f32,
    /// Gravity constant
    pub gravity: f32,
}
/// Implementation of PIDController
impl PIDController {
    /// Creates a new PIDController with default gains
    /// gains are in the order of proportional, derivative, and integral
    /// # Arguments
    /// * `_kpid_pos` - PID gains for position control
    /// * `_kpid_att` - PID gains for attitude control
    /// * `_max_integral_pos` - Maximum allowed integral error for position
    /// * `_max_integral_att` - Maximum allowed integral error for attitude
    /// # Returns
    /// * A new PIDController instance
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::PIDController;
    /// let kpid_pos = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    /// let kpid_att = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    /// let max_integral_pos = [1.0, 1.0, 1.0];
    /// let max_integral_att = [1.0, 1.0, 1.0];
    /// let mass = 1.0;
    /// let gravity = 9.81;
    /// let pid = PIDController::new(kpid_pos, kpid_att, max_integral_pos, max_integral_att, mass, gravity);
    /// ```
    pub fn new(
        _kpid_pos: [[f32; 3]; 3],
        _kpid_att: [[f32; 3]; 3],
        _max_integral_pos: [f32; 3],
        _max_integral_att: [f32; 3],
        _mass: f32,
        _gravity: f32,
    ) -> Self {
        Self {
            kpid_pos: _kpid_pos.map(Vector3::from),
            kpid_att: _kpid_att.map(Vector3::from),
            integral_pos_error: Vector3::zeros(),
            integral_att_error: Vector3::zeros(),
            max_integral_pos: Vector3::from(_max_integral_pos),
            max_integral_att: Vector3::from(_max_integral_att),
            mass: _mass,
            gravity: _gravity,
        }
    }
    /// Computes attitude control torques
    /// # Arguments
    /// * `desired_orientation` - The desired orientation quaternion
    /// * `current_orientation` - The current orientation quaternion
    /// * `current_angular_velocity` - The current angular velocity
    /// * `dt` - Time step
    /// # Returns
    /// * The computed attitude control torques
    /// # Example
    /// ```
    /// use nalgebra::{UnitQuaternion, Vector3};
    /// use peng_quad::PIDController;
    ///
    /// let kpid_pos = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    /// let kpid_att = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    /// let max_integral_pos = [1.0, 1.0, 1.0];
    /// let max_integral_att = [1.0, 1.0, 1.0];
    /// let mass = 1.0;
    /// let gravity = 9.81;
    /// let mut pid = PIDController::new(kpid_pos, kpid_att, max_integral_pos, max_integral_att, mass, gravity);
    /// let desired_orientation = UnitQuaternion::identity();
    /// let current_orientation = UnitQuaternion::identity();
    /// let current_angular_velocity = Vector3::zeros();
    /// let dt = 0.01;
    /// let control_torques = pid.compute_attitude_control(&desired_orientation, &current_orientation, &current_angular_velocity, dt);
    /// ```
    pub fn compute_attitude_control(
        &mut self,
        desired_orientation: &UnitQuaternion<f32>,
        current_orientation: &UnitQuaternion<f32>,
        current_angular_velocity: &Vector3<f32>,
        dt: f32,
    ) -> Vector3<f32> {
        let error_orientation = current_orientation.inverse() * desired_orientation;
        let (roll_error, pitch_error, yaw_error) = error_orientation.euler_angles();
        let error_angles = Vector3::new(roll_error, pitch_error, yaw_error);
        self.integral_att_error += error_angles * dt;
        self.integral_att_error = self
            .integral_att_error
            .zip_map(&self.max_integral_att, |int, max| int.clamp(-max, max));
        let error_angular_velocity = -current_angular_velocity; // TODO: Add desired angular velocity
        self.kpid_att[0].component_mul(&error_angles)
            + self.kpid_att[1].component_mul(&error_angular_velocity)
            + self.kpid_att[2].component_mul(&self.integral_att_error)
    }
    /// Computes position control thrust and desired orientation
    /// # Arguments
    /// * `desired_position` - The desired position
    /// * `desired_velocity` - The desired velocity
    /// * `desired_yaw` - The desired yaw angle
    /// * `current_position` - The current position
    /// * `current_velocity` - The current velocity
    /// * `dt` - Time step
    /// * `mass` - Mass of the quadrotor
    /// * `gravity` - Gravitational acceleration
    /// # Returns
    /// * A tuple containing the computed thrust and desired orientation quaternion
    /// # Example
    /// ```
    /// use nalgebra::{UnitQuaternion, Vector3};
    /// use peng_quad::PIDController;
    ///
    /// let kpid_pos = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    /// let kpid_att = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    /// let max_integral_pos = [1.0, 1.0, 1.0];
    /// let max_integral_att = [1.0, 1.0, 1.0];
    /// let mass = 1.0;
    /// let gravity = 9.81;
    /// let mut pid = PIDController::new(kpid_pos, kpid_att, max_integral_pos, max_integral_att, mass, gravity);
    /// let desired_position = Vector3::new(0.0, 0.0, 1.0);
    /// let desired_velocity = Vector3::zeros();
    /// let desired_yaw = 0.0;
    /// let current_position = Vector3::zeros();
    /// let current_velocity = Vector3::zeros();
    /// let dt = 0.01;
    /// let (thrust, desired_orientation) = pid.compute_position_control(&desired_position, &desired_velocity, desired_yaw, &current_position, &current_velocity, dt);
    /// ```
    pub fn compute_position_control(
        &mut self,
        desired_position: &Vector3<f32>,
        desired_velocity: &Vector3<f32>,
        desired_yaw: f32,
        current_position: &Vector3<f32>,
        current_velocity: &Vector3<f32>,
        dt: f32,
    ) -> (f32, UnitQuaternion<f32>) {
        let error_position = desired_position - current_position;
        let error_velocity = desired_velocity - current_velocity;
        self.integral_pos_error += error_position * dt;
        self.integral_pos_error = self
            .integral_pos_error
            .zip_map(&self.max_integral_pos, |int, max| int.clamp(-max, max));
        let acceleration = self.kpid_pos[0].component_mul(&error_position)
            + self.kpid_pos[1].component_mul(&error_velocity)
            + self.kpid_pos[2].component_mul(&self.integral_pos_error);
        let gravity_compensation = Vector3::new(0.0, 0.0, self.gravity);
        let total_acceleration = acceleration + gravity_compensation;
        let total_acc_norm = total_acceleration.norm();
        let thrust = self.mass * total_acc_norm;
        let desired_orientation = if total_acc_norm > 1e-6 {
            let z_body = total_acceleration / total_acc_norm;
            let yaw_rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, desired_yaw);
            let x_body_horizontal = yaw_rotation * Vector3::new(1.0, 0.0, 0.0);
            let y_body = z_body.cross(&x_body_horizontal).normalize();
            let x_body = y_body.cross(&z_body);
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
                Matrix3::from_columns(&[x_body, y_body, z_body]),
            ))
        } else {
            UnitQuaternion::from_euler_angles(0.0, 0.0, desired_yaw)
        };
        (thrust, desired_orientation)
    }
}
/// Enum representing different types of trajectory planners
/// # Example
/// ```
/// use peng_quad::PlannerType;
/// use peng_quad::HoverPlanner;
/// let hover_planner : PlannerType = PlannerType::Hover(HoverPlanner{
///     target_position: nalgebra::Vector3::new(0.0, 0.0, 1.0),
///     target_yaw: 0.0,
/// });
/// ```
pub enum PlannerType {
    /// Hover planner
    Hover(HoverPlanner),
    /// Minimum jerk line planner
    MinimumJerkLine(MinimumJerkLinePlanner),
    /// Minimum jerk circle planner
    Lissajous(LissajousPlanner),
    /// Minimum jerk circle planner
    Circle(CirclePlanner),
    /// Minimum jerk landing planner
    Landing(LandingPlanner),
    /// Obstacle avoidance planner
    ObstacleAvoidance(ObstacleAvoidancePlanner),
    /// Minimum snap waypoint planner
    MinimumSnapWaypoint(MinimumSnapWaypointPlanner),
    /// Quadratic Polynomial based waypoint planner
    QPpolyTraj(QPpolyTrajPlanner),
}
/// Implementation of the planner type
impl PlannerType {
    /// Plans the trajectory based on the current planner type
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `current_velocity` - The current velocity of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// * A tuple containing the desired position, velocity, and yaw angle
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::PlannerType;
    /// use peng_quad::HoverPlanner;
    /// let hover_planner = HoverPlanner {
    ///     target_position: Vector3::new(0.0, 0.0, 1.0),
    ///     target_yaw: 0.0
    /// };
    /// let hover_planner_type = PlannerType::Hover(hover_planner);
    /// let (desired_position, desired_velocity, desired_yaw) = hover_planner_type.plan(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 0.0);
    /// ```
    pub fn plan(
        &self,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        match self {
            PlannerType::Hover(p) => p.plan(current_position, current_velocity, time),
            PlannerType::MinimumJerkLine(p) => p.plan(current_position, current_velocity, time),
            PlannerType::Lissajous(p) => p.plan(current_position, current_velocity, time),
            PlannerType::Circle(p) => p.plan(current_position, current_velocity, time),
            PlannerType::Landing(p) => p.plan(current_position, current_velocity, time),
            PlannerType::ObstacleAvoidance(p) => p.plan(current_position, current_velocity, time),
            PlannerType::MinimumSnapWaypoint(p) => p.plan(current_position, current_velocity, time),
            PlannerType::QPpolyTraj(p) => p.plan(current_position, current_velocity, time),
        }
    }
    /// Checks if the current trajectory is finished
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// * `true` if the trajectory is finished, `false` otherwise
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::PlannerType;
    /// use peng_quad::HoverPlanner;
    /// use peng_quad::Planner;
    /// let hover_planner = HoverPlanner{
    ///     target_position: Vector3::new(0.0, 0.0, 1.0),
    ///     target_yaw: 0.0,
    /// };
    /// let is_finished = hover_planner.is_finished(Vector3::new(0.0, 0.0, 0.0), 0.0);
    /// ```
    pub fn is_finished(
        &self,
        current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        match self {
            PlannerType::Hover(p) => p.is_finished(current_position, time),
            PlannerType::MinimumJerkLine(p) => p.is_finished(current_position, time),
            PlannerType::Lissajous(p) => p.is_finished(current_position, time),
            PlannerType::Circle(p) => p.is_finished(current_position, time),
            PlannerType::Landing(p) => p.is_finished(current_position, time),
            PlannerType::ObstacleAvoidance(p) => p.is_finished(current_position, time),
            PlannerType::MinimumSnapWaypoint(p) => p.is_finished(current_position, time),
            PlannerType::QPpolyTraj(p) => p.is_finished(current_position, time),
        }
    }
}
/// Trait defining the interface for trajectory planners
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::{Planner, SimulationError};
/// struct TestPlanner;
/// impl Planner for TestPlanner {
///    fn plan(
///         &self,
///         current_position: Vector3<f32>,
///         current_velocity: Vector3<f32>,
///         time: f32,
/// ) -> (Vector3<f32>, Vector3<f32>, f32) {
///         (Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 0.0)
///     }
///     fn is_finished(
///         &self,
///         current_position: Vector3<f32>,
///         time: f32,
///     ) -> Result<bool, SimulationError> {
///         Ok(true)
///     }
/// }
/// ```
pub trait Planner {
    /// Plans the trajectory based on the current state and time
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `current_velocity` - The current velocity of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// * A tuple containing the desired position, velocity, and yaw angle
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::{Planner, SimulationError};
    /// struct TestPlanner;
    /// impl Planner for TestPlanner {
    ///     fn plan(
    ///         &self,
    ///         current_position: Vector3<f32>,
    ///         current_velocity: Vector3<f32>,
    ///         time: f32,
    /// ) -> (Vector3<f32>, Vector3<f32>, f32) {
    ///         (Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 0.0)
    ///     }
    ///     fn is_finished(
    ///         &self,
    ///         current_position: Vector3<f32>,
    ///         time: f32,
    ///     ) -> Result<bool, SimulationError> {
    ///         Ok(true)
    ///     }
    /// }
    /// ```
    fn plan(
        &self,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32);
    /// Checks if the current trajectory is finished
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// * `true` if the trajectory is finished, `false` otherwise
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::{Planner, SimulationError};
    /// struct TestPlanner;
    /// impl Planner for TestPlanner {
    ///     fn plan(
    ///         &self,
    ///         current_position: Vector3<f32>,
    ///         current_velocity: Vector3<f32>,
    ///         time: f32,
    /// ) -> (Vector3<f32>, Vector3<f32>, f32) {
    ///         (Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 0.0)
    ///     }
    ///     fn is_finished(
    ///         &self,
    ///         current_position: Vector3<f32>,
    ///         time: f32,
    ///     ) -> Result<bool, SimulationError> {
    ///         Ok(true)
    ///     }
    /// }
    /// ```
    fn is_finished(
        &self,
        current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError>;
}
/// Planner for hovering at a fixed position
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::HoverPlanner;
/// let hover_planner = HoverPlanner {
///     target_position: Vector3::new(0.0, 0.0, 0.0),
///     target_yaw: 0.0,
/// };
/// ```
pub struct HoverPlanner {
    /// Target position for hovering
    pub target_position: Vector3<f32>,
    /// Target yaw angle for hovering
    pub target_yaw: f32,
}
/// Implementation of the `Planner` trait for the `HoverPlanner`
impl Planner for HoverPlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        _time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        (self.target_position, Vector3::zeros(), self.target_yaw)
    }

    fn is_finished(
        &self,
        _current_position: Vector3<f32>,
        _time: f32,
    ) -> Result<bool, SimulationError> {
        Ok(false) // Hover planner never "finished"
    }
}
/// Planner for minimum jerk trajectories along a straight line
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::MinimumJerkLinePlanner;
/// let minimum_jerk_line_planner = MinimumJerkLinePlanner {
///     start_position: Vector3::new(0.0, 0.0, 0.0),
///     end_position: Vector3::new(1.0, 1.0, 1.0),
///     start_yaw: 0.0,
///     end_yaw: 0.0,
///     start_time: 0.0,
///     duration: 1.0,
/// };
/// ```
pub struct MinimumJerkLinePlanner {
    /// Starting position of the trajectory
    pub start_position: Vector3<f32>,
    /// Ending position of the trajectory
    pub end_position: Vector3<f32>,
    /// Starting yaw angle
    pub start_yaw: f32,
    /// Ending yaw angle
    pub end_yaw: f32,
    /// Start time of the trajectory
    pub start_time: f32,
    /// Duration of the trajectory
    pub duration: f32,
}
/// Implementation of the planner trait for minimum jerk line planner
impl Planner for MinimumJerkLinePlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        let t = ((time - self.start_time) / self.duration).clamp(0.0, 1.0);
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let s = 10.0 * t2 - 15.0 * t3 + 6.0 * t4;
        let s_dot = (30.0 * t.powi(2) - 60.0 * t.powi(3) + 30.0 * t.powi(4)) / self.duration;
        let position = self.start_position + (self.end_position - self.start_position) * s;
        let velocity = (self.end_position - self.start_position) * s_dot;
        let yaw = self.start_yaw + (self.end_yaw - self.start_yaw) * s;
        (position, velocity, yaw)
    }

    fn is_finished(
        &self,
        _current_position: Vector3<f32>,
        _time: f32,
    ) -> Result<bool, SimulationError> {
        Ok((_current_position - self.end_position).norm() < 0.01
            && _time >= self.start_time + self.duration)
    }
}
/// Planner for Lissajous curve trajectories
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::LissajousPlanner;
/// let lissajous_planner = LissajousPlanner {
///     start_position: Vector3::new(0.0, 0.0, 0.0),
///     center: Vector3::new(1.0, 1.0, 1.0),
///     amplitude: Vector3::new(1.0, 1.0, 1.0),
///     frequency: Vector3::new(1.0, 1.0, 1.0),
///     phase: Vector3::new(0.0, 0.0, 0.0),
///     start_time: 0.0,
///     duration: 1.0,
///     start_yaw: 0.0,
///     end_yaw: 0.0,
///     ramp_time: 0.1,
/// };
/// ```
pub struct LissajousPlanner {
    /// Starting position of the trajectory
    pub start_position: Vector3<f32>,
    /// Center of the Lissajous curve
    pub center: Vector3<f32>,
    /// Amplitude of the Lissajous curve
    pub amplitude: Vector3<f32>,
    /// Frequency of the Lissajous curve
    pub frequency: Vector3<f32>,
    /// Phase of the Lissajous curve
    pub phase: Vector3<f32>,
    /// Start time of the trajectory
    pub start_time: f32,
    /// Duration of the trajectory
    pub duration: f32,
    /// Starting yaw angle
    pub start_yaw: f32,
    /// Ending yaw angle
    pub end_yaw: f32,
    /// Ramp-up time for smooth transitions
    pub ramp_time: f32,
}
/// Implementation of the planner trait for Lissajous curve trajectories
impl Planner for LissajousPlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        let t = ((time - self.start_time) / self.duration).clamp(0.0, 1.0);
        let smooth_start = if t < self.ramp_time / self.duration {
            let t_ramp = t / (self.ramp_time / self.duration);
            t_ramp * t_ramp * (3.0 - 2.0 * t_ramp)
        } else {
            1.0
        };
        let velocity_ramp = if t < self.ramp_time / self.duration {
            smooth_start
        } else if t > 1.0 - self.ramp_time / self.duration {
            let t_down = (1.0 - t) / (self.ramp_time / self.duration);
            t_down * t_down * (3.0 - 2.0 * t_down)
        } else {
            1.0
        };
        let ang_pos = self.frequency * t * 2.0 * PI + self.phase;
        let lissajous = self.amplitude.component_mul(&ang_pos.map(f32::sin));
        let position =
            self.start_position + smooth_start * ((self.center + lissajous) - self.start_position);
        let mut velocity = Vector3::new(
            self.amplitude.x * self.frequency.x * 2.0 * PI * ang_pos.x.cos(),
            self.amplitude.y * self.frequency.y * 2.0 * PI * ang_pos.y.cos(),
            self.amplitude.z * self.frequency.z * 2.0 * PI * ang_pos.z.cos(),
        ) * velocity_ramp
            / self.duration;
        if t < self.ramp_time / self.duration {
            let transition_velocity = (self.center - self.start_position)
                * (2.0 * t / self.ramp_time - 2.0 * t * t / (self.ramp_time * self.ramp_time))
                / self.duration;
            velocity += transition_velocity;
        }
        let yaw = self.start_yaw + (self.end_yaw - self.start_yaw) * t;
        (position, velocity, yaw)
    }

    fn is_finished(
        &self,
        _current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        Ok(time >= self.start_time + self.duration)
    }
}
/// Planner for circular trajectories
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::CirclePlanner;
/// let circle_planner = CirclePlanner {
///     center: Vector3::new(1.0, 1.0, 1.0),
///     radius: 1.0,
///     angular_velocity: 1.0,
///     start_position: Vector3::new(0.0, 0.0, 0.0),
///     start_time: 0.0,
///     duration: 1.0,
///     start_yaw: 0.0,
///     end_yaw: 0.0,
///     ramp_time: 0.1,
/// };
/// ```
pub struct CirclePlanner {
    /// Center of the circular trajectory
    pub center: Vector3<f32>,
    /// Radius of the circular trajectory
    pub radius: f32,
    /// Angular velocity of the circular motion
    pub angular_velocity: f32,
    /// Starting position of the trajectory
    pub start_position: Vector3<f32>,
    /// Start time of the trajectory
    pub start_time: f32,
    /// Duration of the trajectory
    pub duration: f32,
    /// Starting yaw angle
    pub start_yaw: f32,
    /// Ending yaw angle
    pub end_yaw: f32,
    /// Ramp-up time for smooth transitions
    pub ramp_time: f32,
}
/// Implementation of the Planner trait for CirclePlanner
impl Planner for CirclePlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        let t = (time - self.start_time) / self.duration;
        let t = t.clamp(0.0, 1.0);
        let smooth_start = if t < self.ramp_time / self.duration {
            let t_ramp = t / (self.ramp_time / self.duration);
            t_ramp * t_ramp * (3.0 - 2.0 * t_ramp)
        } else {
            1.0
        };
        let velocity_ramp = if t < self.ramp_time / self.duration {
            smooth_start
        } else if t > 1.0 - self.ramp_time / self.duration {
            let t_down = (1.0 - t) / (self.ramp_time / self.duration);
            t_down * t_down * (3.0 - 2.0 * t_down)
        } else {
            1.0
        };
        let angle = self.angular_velocity * t * self.duration;
        let circle_offset = Vector3::new(self.radius * angle.cos(), self.radius * angle.sin(), 0.0);
        let position = self.start_position
            + smooth_start * ((self.center + circle_offset) - self.start_position);
        let tangential_velocity = Vector3::new(
            -self.radius * self.angular_velocity * angle.sin(),
            self.radius * self.angular_velocity * angle.cos(),
            0.0,
        );
        let velocity = tangential_velocity * velocity_ramp;
        let yaw = self.start_yaw + (self.end_yaw - self.start_yaw) * t;
        (position, velocity, yaw)
    }

    fn is_finished(
        &self,
        _current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        Ok(time >= self.start_time + self.duration)
    }
}
/// Planner for landing maneuvers
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::LandingPlanner;
/// let landing_planner = LandingPlanner {
///    start_position: Vector3::new(0.0, 0.0, 1.0),
///     start_time: 0.0,
///     duration: 1.0,
///     start_yaw: 0.0,
/// };
/// ```
pub struct LandingPlanner {
    /// Starting position of the landing maneuver
    pub start_position: Vector3<f32>,
    /// Start time of the landing maneuver
    pub start_time: f32,
    /// Duration of the landing maneuver
    pub duration: f32,
    /// Starting yaw angle
    pub start_yaw: f32,
}
/// Implementation of the Planner trait for 'LandingPlanner'
impl Planner for LandingPlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        let t = ((time - self.start_time) / self.duration).clamp(0.0, 1.0);
        let target_z = self.start_position.z * (1.0 - t);
        let target_position = Vector3::new(self.start_position.x, self.start_position.y, target_z);
        let target_velocity = Vector3::new(0.0, 0.0, -self.start_position.z / self.duration);
        (target_position, target_velocity, self.start_yaw)
    }

    fn is_finished(
        &self,
        current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        Ok(current_position.z < 0.05 || time >= self.start_time + self.duration)
    }
}
/// Manages different trajectory planners and switches between them
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::PlannerManager;
/// let initial_position = Vector3::new(0.0, 0.0, 1.0);
/// let initial_yaw = 0.0;
/// let planner_manager = PlannerManager::new(initial_position, initial_yaw);
/// ```
pub struct PlannerManager {
    /// The current planner
    pub current_planner: PlannerType,
}
/// Implementation of the PlannerManager
impl PlannerManager {
    /// Creates a new PlannerManager with an initial hover planner
    /// # Arguments
    /// * `initial_position` - The initial position for hovering
    /// * `initial_yaw` - The initial yaw angle for hovering
    /// # Returns
    /// * A new PlannerManager instance
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::PlannerManager;
    /// let initial_position = Vector3::new(0.0, 0.0, 1.0);
    /// let initial_yaw = 0.0;
    /// let planner_manager = PlannerManager::new(initial_position, initial_yaw);
    /// ```
    pub fn new(initial_position: Vector3<f32>, initial_yaw: f32) -> Self {
        let hover_planner = HoverPlanner {
            target_position: initial_position,
            target_yaw: initial_yaw,
        };
        Self {
            current_planner: PlannerType::Hover(hover_planner),
        }
    }
    /// Sets a new planner
    /// # Arguments
    /// * `new_planner` - The new planner to be set
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::{PlannerManager, CirclePlanner, PlannerType};
    /// let initial_position = Vector3::new(0.0, 0.0, 1.0);
    /// let initial_yaw = 0.0;
    /// let mut planner_manager = PlannerManager::new(initial_position, initial_yaw);
    /// let new_planner = CirclePlanner {
    ///     center: Vector3::new(0.0, 0.0, 1.0),
    ///     radius: 1.0,
    ///     angular_velocity: 1.0,
    ///     start_yaw: 0.0,
    ///     end_yaw: 0.0,
    ///     start_time: 0.0,
    ///     duration: 10.0,
    ///     ramp_time: 1.0,
    ///     start_position: Vector3::new(0.0, 0.0, 1.0),
    /// };
    /// planner_manager.set_planner(PlannerType::Circle(new_planner));
    /// ```
    pub fn set_planner(&mut self, new_planner: PlannerType) {
        self.current_planner = new_planner;
    }
    /// Updates the current planner and returns the desired position, velocity, and yaw
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `current_orientation` - The current orientation of the quadrotor
    /// * `current_velocity` - The current velocity of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// * A tuple containing the desired position, velocity, and yaw angle
    /// # Errors
    /// * Returns a SimulationError if the current planner is not finished
    /// # Example
    /// ```
    /// use nalgebra::{Vector3, UnitQuaternion};
    /// use peng_quad::{PlannerManager, SimulationError};
    /// let initial_position = Vector3::new(0.0, 0.0, 1.0);
    /// let initial_yaw = 0.0;
    /// let mut planner_manager = PlannerManager::new(initial_position, initial_yaw);
    /// let current_position = Vector3::new(0.0, 0.0, 1.0);
    /// let current_orientation = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
    /// let current_velocity = Vector3::new(0.0, 0.0, 0.0);
    /// let obstacles = vec![];
    /// let time = 0.0;
    /// let result = planner_manager.update(current_position, current_orientation, current_velocity, time, &obstacles);
    /// match result {
    ///     Ok((target_position, target_velocity, target_yaw)) => {
    ///         println!("Target Position: {:?}", target_position);
    ///         println!("Target Velocity: {:?}", target_velocity);
    ///         println!("Target Yaw: {:?}", target_yaw);
    ///     }
    ///     Err(SimulationError) => {
    ///         log::error!("Error: Planner is not finished");
    ///     }
    /// }
    /// ```
    pub fn update(
        &mut self,
        current_position: Vector3<f32>,
        current_orientation: UnitQuaternion<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
        obstacles: &[Obstacle],
    ) -> Result<(Vector3<f32>, Vector3<f32>, f32), SimulationError> {
        if self.current_planner.is_finished(current_position, time)? {
            log::info!("Time: {time:.2} s,\tSwitch Hover");
            self.current_planner = PlannerType::Hover(HoverPlanner {
                target_position: current_position,
                target_yaw: current_orientation.euler_angles().2,
            });
        }
        // Update obstacles for ObstacleAvoidancePlanner if needed
        if let PlannerType::ObstacleAvoidance(ref mut planner) = self.current_planner {
            planner.obstacles = obstacles.to_owned();
        }
        Ok(self
            .current_planner
            .plan(current_position, current_velocity, time))
    }
}
/// Obstacle avoidance planner that uses a potential field approach to avoid obstacles
///
/// The planner calculates a repulsive force for each obstacle and an attractive force towards the goal
/// The resulting force is then used to calculate the desired position and velocity
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::{ObstacleAvoidancePlanner, Obstacle};
/// let planner = ObstacleAvoidancePlanner {
///     target_position: Vector3::new(0.0, 0.0, 1.0),
///     start_time: 0.0,
///     duration: 10.0,
///     start_yaw: 0.0,
///     end_yaw: 0.0,
///     obstacles: vec![Obstacle {
///         position: Vector3::new(1.0, 0.0, 1.0),
///         velocity: Vector3::new(0.0, 0.0, 0.0),
///         radius: 0.5,
///     }],
///     k_att: 1.0,
///     k_rep: 1.0,
///     k_vortex: 1.0,
///     d0: 1.0,
///     d_target: 1.0,
///     max_speed: 1.0,
/// };
/// ```
pub struct ObstacleAvoidancePlanner {
    /// Target position of the planner
    pub target_position: Vector3<f32>,
    /// Start time of the planner
    pub start_time: f32,
    /// Duration of the planner
    pub duration: f32,
    /// Starting yaw angle
    pub start_yaw: f32,
    /// Ending yaw angle
    pub end_yaw: f32,
    /// List of obstacles
    pub obstacles: Vec<Obstacle>,
    /// Attractive force gain
    pub k_att: f32,
    /// Repulsive force gain
    pub k_rep: f32,
    /// Vortex force gain
    pub k_vortex: f32,
    /// Influence distance of obstacles
    pub d0: f32,
    /// Influence distance of target
    pub d_target: f32,
    /// Maximum speed of the quadrotor
    pub max_speed: f32,
}
/// Implementation of the Planner trait for ObstacleAvoidancePlanner
impl Planner for ObstacleAvoidancePlanner {
    fn plan(
        &self,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        let t = ((time - self.start_time) / self.duration).clamp(0.0, 1.0);
        let distance_to_target = (self.target_position - current_position).norm();
        let f_att = self.k_att
            * self.smooth_attractive_force(distance_to_target)
            * (self.target_position - current_position).normalize();
        // Repulsive force from obstacles
        let mut f_rep = Vector3::zeros();
        let mut f_vortex = Vector3::zeros();
        for obstacle in &self.obstacles {
            let diff = current_position - obstacle.position;
            let distance = diff.norm();
            if distance < self.d0 {
                f_rep += self.k_rep
                    * (1.0 / distance - 1.0 / self.d0)
                    * (1.0 / distance.powi(2))
                    * diff.normalize();
                f_vortex +=
                    self.k_vortex * current_velocity.cross(&diff).normalize() / distance.powi(2);
            }
        }
        let f_total = f_att + f_rep + f_vortex;
        let desired_velocity = f_total.normalize() * self.max_speed.min(f_total.norm());
        let desired_position = current_position + desired_velocity * self.duration * (1.0 - t);
        let desired_yaw = self.start_yaw + (self.end_yaw - self.start_yaw) * t;
        (desired_position, desired_velocity, desired_yaw)
    }

    fn is_finished(
        &self,
        current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        Ok((current_position - self.target_position).norm() < 0.1
            && time >= self.start_time + self.duration)
    }
}
/// Implementation of the ObstacleAvoidancePlanner
impl ObstacleAvoidancePlanner {
    /// A smooth attractive force function that transitions from linear to exponential decay
    /// When the distance to the target is less than the target distance, the force is linear
    /// When the distance is greater, the force decays exponentially
    /// # Arguments
    /// * `distance` - The distance to the target
    /// # Returns
    /// * The attractive force
    /// # Example
    /// ```
    /// use peng_quad::ObstacleAvoidancePlanner;
    /// let planner = ObstacleAvoidancePlanner {
    ///    target_position: nalgebra::Vector3::new(0.0, 0.0, 1.0),
    ///     start_time: 0.0,
    ///     duration: 10.0,
    ///     start_yaw: 0.0,
    ///     end_yaw: 0.0,
    ///     obstacles: vec![],
    ///     k_att: 1.0,
    ///     k_rep: 1.0,
    ///     k_vortex: 1.0,
    ///     d0: 1.0,
    ///     d_target: 1.0,
    ///     max_speed: 1.0,
    /// };
    /// let distance = 1.0;
    /// let force = planner.smooth_attractive_force(distance);
    /// ```
    #[inline]
    pub fn smooth_attractive_force(&self, distance: f32) -> f32 {
        if distance <= self.d_target {
            distance
        } else {
            self.d_target + (distance - self.d_target).tanh()
        }
    }
}
/// Waypoint planner that generates a minimum snap trajectory between waypoints
/// # Example
/// ```
/// use peng_quad::MinimumSnapWaypointPlanner;
/// use nalgebra::Vector3;
/// let planner = MinimumSnapWaypointPlanner::new(
///     vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)],
///     vec![0.0, 0.0],
///     vec![1.0],
///     0.0,
/// );
/// ```
pub struct MinimumSnapWaypointPlanner {
    /// List of waypoints
    pub waypoints: Vec<Vector3<f32>>,
    /// List of yaw angles
    pub yaws: Vec<f32>,
    /// List of segment times to reach each waypoint
    pub times: Vec<f32>,
    /// Coefficients for the x, y, and z components of the trajectory
    pub coefficients: Vec<Vec<Vector3<f32>>>,
    /// Coefficients for the yaw component of the trajectory
    pub yaw_coefficients: Vec<Vec<f32>>,
    /// Start time of the trajectory
    pub start_time: f32,
}
/// Implementation of the MinimumSnapWaypointPlanner
impl MinimumSnapWaypointPlanner {
    /// Generate a new minimum snap waypoint planner
    /// # Arguments
    /// * `waypoints` - List of waypoints
    /// * `yaws` - List of yaw angles
    /// * `segment_times` - List of segment times to reach each waypoint
    /// * `start_time` - Start time of the trajectory
    /// # Returns
    /// * A new minimum snap waypoint planner
    /// # Errors
    /// * Returns an error if the number of waypoints, yaws, and segment times do not match
    /// # Example
    /// ```
    /// use peng_quad::MinimumSnapWaypointPlanner;
    /// use nalgebra::Vector3;
    /// let waypoints = vec![Vector3::zeros(), Vector3::new(1.0, 0.0, 0.0)];
    /// let yaws = vec![0.0, 0.0];
    /// let segment_times = vec![1.0];
    /// let start_time = 0.0;
    /// let planner = MinimumSnapWaypointPlanner::new(waypoints, yaws, segment_times, start_time);
    /// ```
    pub fn new(
        waypoints: Vec<Vector3<f32>>,
        yaws: Vec<f32>,
        segment_times: Vec<f32>,
        start_time: f32,
    ) -> Result<Self, SimulationError> {
        if waypoints.len() < 2 {
            return Err(SimulationError::OtherError(
                "At least two waypoints are required".to_string(),
            ));
        }
        if waypoints.len() != segment_times.len() + 1 || waypoints.len() != yaws.len() {
            return Err(SimulationError::OtherError("Number of segment times must be one less than number of waypoints, and yaws must match waypoints".to_string()));
        }
        let mut planner = Self {
            waypoints,
            yaws,
            times: segment_times,
            coefficients: Vec::new(),
            yaw_coefficients: Vec::new(),
            start_time,
        };
        planner.compute_minimum_snap_trajectories()?;
        planner.compute_minimum_acceleration_yaw_trajectories()?;
        Ok(planner)
    }
    /// Compute the coefficients for the minimum snap trajectory, calculated for each segment between waypoints
    /// # Errors
    /// * Returns an error if the nalgebra solver fails to solve the linear system
    /// # Example
    /// ```
    /// use peng_quad::MinimumSnapWaypointPlanner;
    /// use nalgebra::Vector3;
    /// let waypoints = vec![Vector3::zeros(), Vector3::new(1.0, 0.0, 0.0)];
    /// let yaws = vec![0.0, 0.0];
    /// let segment_times = vec![1.0];
    /// let start_time = 0.0;
    /// let mut planner = MinimumSnapWaypointPlanner::new(waypoints, yaws, segment_times, start_time).unwrap();
    /// planner.compute_minimum_snap_trajectories();
    /// ```
    pub fn compute_minimum_snap_trajectories(&mut self) -> Result<(), SimulationError> {
        let n = self.waypoints.len() - 1;
        for i in 0..n {
            let duration = self.times[i];
            let (start, end) = (self.waypoints[i], self.waypoints[i + 1]);
            let mut a = SMatrix::<f32, 8, 8>::zeros();
            let mut b = SMatrix::<f32, 8, 3>::zeros();
            a.fixed_view_mut::<4, 4>(0, 0).fill_with_identity();
            b.fixed_view_mut::<1, 3>(0, 0).copy_from(&start.transpose());
            b.fixed_view_mut::<1, 3>(4, 0).copy_from(&end.transpose());
            // End point constraints
            for j in 0..8 {
                a[(4, j)] = duration.powi(j as i32);
                if j > 0 {
                    a[(5, j)] = j as f32 * duration.powi(j as i32 - 1);
                }
                if j > 1 {
                    a[(6, j)] = j as f32 * (j - 1) as f32 * duration.powi(j as i32 - 2);
                }
                if j > 2 {
                    a[(7, j)] =
                        j as f32 * (j - 1) as f32 * (j - 2) as f32 * duration.powi(j as i32 - 3);
                }
            }
            let coeffs = a.lu().solve(&b).ok_or(SimulationError::NalgebraError(
                "Failed to solve for coefficients in MinimumSnapWaypointPlanner".to_string(),
            ))?;
            self.coefficients.push(
                (0..8)
                    .map(|j| Vector3::new(coeffs[(j, 0)], coeffs[(j, 1)], coeffs[(j, 2)]))
                    .collect(),
            );
        }
        Ok(())
    }
    /// Compute the coefficients for yaw trajectories
    /// The yaw trajectory is a cubic polynomial and interpolated between waypoints
    /// # Errors
    /// * Returns an error if nalgebra fails to solve for the coefficients
    /// # Example
    /// ```
    /// use peng_quad::MinimumSnapWaypointPlanner;
    /// use nalgebra::Vector3;
    /// let waypoints = vec![Vector3::zeros(), Vector3::new(1.0, 0.0, 0.0)];
    /// let yaws = vec![0.0, 0.0];
    /// let segment_times = vec![1.0];
    /// let start_time = 0.0;
    /// let mut planner = MinimumSnapWaypointPlanner::new(waypoints, yaws, segment_times, start_time).unwrap();
    /// planner.compute_minimum_snap_trajectories();
    /// planner.compute_minimum_acceleration_yaw_trajectories();
    /// ```
    pub fn compute_minimum_acceleration_yaw_trajectories(&mut self) -> Result<(), SimulationError> {
        let n = self.yaws.len() - 1; // Number of segments
        for i in 0..n {
            let (duration, start_yaw, end_yaw) = (self.times[i], self.yaws[i], self.yaws[i + 1]);
            let mut a = SMatrix::<f32, 4, 4>::zeros();
            let mut b = SMatrix::<f32, 4, 1>::zeros();
            (a[(0, 0)], a[(1, 1)]) = (1.0, 1.0);
            (b[0], b[2]) = (start_yaw, end_yaw);
            for j in 0..4 {
                a[(2, j)] = duration.powi(j as i32);
                if j > 0 {
                    a[(3, j)] = j as f32 * duration.powi(j as i32 - 1);
                }
            }
            let yaw_coeffs = a.lu().solve(&b).ok_or(SimulationError::NalgebraError(
                "Failed to solve for yaw coefficients in MinimumSnapWaypointPlanner".to_string(),
            ))?;
            self.yaw_coefficients.push(yaw_coeffs.as_slice().to_vec());
        }
        Ok(())
    }
    /// Evaluate the trajectory at a given time, returns the position, velocity, yaw, and yaw rate at the given time
    /// # Arguments
    /// * `t` - The time to evaluate the trajectory at
    /// * `coeffs` - The coefficients for the position trajectory
    /// * `yaw_coeffs` - The coefficients for the yaw trajectory
    /// # Returns
    /// * `position` - The position at the given time (meters)
    /// * `velocity` - The velocity at the given time (meters / second)
    /// * `yaw` - The yaw at the given time (radians)
    /// * `yaw_rate` - The yaw rate at the given time (radians / second)
    /// # Example
    /// ```
    /// use nalgebra::Vector3;
    /// use peng_quad::MinimumSnapWaypointPlanner;
    /// let waypoints = vec![Vector3::zeros(), Vector3::new(1.0, 0.0, 0.0)];
    /// let yaws = vec![0.0, 0.0];
    /// let segment_times = vec![1.0];
    /// let start_time = 0.0;
    /// let mut planner = MinimumSnapWaypointPlanner::new(waypoints, yaws, segment_times, start_time).unwrap();
    /// planner.compute_minimum_snap_trajectories();
    /// planner.compute_minimum_acceleration_yaw_trajectories();
    /// let (position, velocity, yaw, yaw_rate) = planner.evaluate_polynomial(0.5, &planner.coefficients[0], &planner.yaw_coefficients[0]);
    /// ```
    pub fn evaluate_polynomial(
        &self,
        t: f32,
        coeffs: &[Vector3<f32>],
        yaw_coeffs: &[f32],
    ) -> (Vector3<f32>, Vector3<f32>, f32, f32) {
        let mut position = Vector3::zeros();
        let mut velocity = Vector3::zeros();
        let mut yaw = 0.0;
        let mut yaw_rate = 0.0;
        for (i, coeff) in coeffs.iter().enumerate() {
            let ti = t.powi(i as i32);
            position += coeff * ti;
            if i > 0 {
                velocity += coeff * (i as f32) * t.powi(i as i32 - 1);
            }
        }
        for (i, &coeff) in yaw_coeffs.iter().enumerate() {
            let ti = t.powi(i as i32);
            yaw += coeff * ti;
            if i > 0 {
                yaw_rate += coeff * (i as f32) * t.powi(i as i32 - 1);
            }
        }
        (position, velocity, yaw, yaw_rate)
    }
}
/// Implement the `Planner` trait for `MinimumSnapWaypointPlanner`
impl Planner for MinimumSnapWaypointPlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        let relative_time = time - self.start_time;
        // Find the current segment
        let mut segment_start_time = 0.0;
        let mut current_segment = 0;
        for (i, &segment_duration) in self.times.iter().enumerate() {
            if relative_time < segment_start_time + segment_duration {
                current_segment = i;
                break;
            }
            segment_start_time += segment_duration;
        }
        // Evaluate the polynomial for the current segment
        let segment_time = relative_time - segment_start_time;
        let (position, velocity, yaw, _yaw_rate) = self.evaluate_polynomial(
            segment_time,
            &self.coefficients[current_segment],
            &self.yaw_coefficients[current_segment],
        );
        (position, velocity, yaw)
    }

    fn is_finished(
        &self,
        current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        let last_waypoint = self.waypoints.last().ok_or(SimulationError::OtherError(
            "No waypoints available".to_string(),
        ))?;
        Ok(time >= self.start_time + self.times.iter().sum::<f32>()
            && (current_position - last_waypoint).norm() < 0.1)
    }
}

/// Waypoint planner that generates a quadratic polynomial trajectory between waypoints
/// # Example
/// ```
/// use peng_quad::QPpolyTrajPlanner;
/// use nalgebra::Vector3;
/// let waypoints: Vec<Vec<f32>> = vec![vec![0.0,0.0,1.0,0.0], vec![1.0,0.0,1.0,0.0]];
/// let segment_times = vec![6.0];
/// let min_deriv = 3;
/// let polyorder = 9;
/// let smooth_upto = 4;
/// let max_velocity = 4.0;
/// let max_acceleration = 3.0;
/// let dt = 0.1;
/// let start_time = 0.0;
/// let mut qp_planner = QPpolyTrajPlanner::new(waypoints,segment_times,polyorder, min_deriv, smooth_upto,max_velocity,max_acceleration, start_time, dt);
/// ```
pub struct QPpolyTrajPlanner {
    /// Matrix of coefficients for each segment and each dimension, organized as nrows: polyorder*segment_times.len(), ncols: 4 (for x, y, z, yaw)
    pub coeff: DMatrix<f64>,
    /// Order of the polynomial to be used in computing trajectory
    pub polyorder: usize,
    /// Minimize which derivative in the QP problem (1->Velocity, 2->Acceleration, 3->Snap, 4->Jerk. Please note that derivative value greater than 4 is not supported)
    pub min_deriv: usize,
    /// Ensure continuity upto which derivative. NOTE: This MUST be <= polynomial_order (1->Velocity, 2->Acceleration, 3->Snap, 4->Jerk. Please note that derivative value greater than 4 is not supported)
    pub smooth_upto: usize,
    /// Vector of time values for each segment, which tells the planner how much time each segment should take to complete. Expressed in seconds.
    pub segment_times: Vec<f32>,
    /// Waypoints for each segment. Note that there should be segment_times.len() + 1 values for waypoints, with the position of the quadrotor being the very first waypoint.
    pub waypoints: Vec<Vec<f32>>,
    /// Maximum velocity constraint. Set to 0.0 to disregard inequality constraints. Please set reasonable values for this as it influences solver convergence or failure.
    pub max_velocity: f32,
    /// Maximum acceleration constraint. Set to 0.0 to disregard inequality constraints. Please set reasonable values for this as it influences solver convergence or failure.
    pub max_acceleration: f32,
    /// Time at which the simulation starts. This value has no bearing to QPpolyTraj itself, but is used during simulation since we use relative time internally when computing values.
    pub start_time: f32,
    /// Step time used while generating inequality constraints. Has no bearing if max_velocity or max_acceleration is set to 0.0. Please set reasonable value for this as it has a huge impact on OSQP solve time.
    pub dt: f32,
}

impl QPpolyTrajPlanner {
    /// Generate a new QPpolyTraj planner
    /// # Arguments
    /// * `waypoints` - The waypoints for the trajectory
    /// * `segment_times` - The times for each segment of the trajectory
    /// * `polyorder` - The order of the polynomial
    /// * `min_deriv` - The minimum derivative to be considered
    /// * `smooth_upto` - The maximum derivative to be considered
    /// * `max_velocity` - The maximum velocity
    /// * `max_acceleration` - The maximum acceleration
    /// * `start_time` - The start time of the trajectory
    /// * `dt` - The time step for the trajectory
    /// # Errors
    /// * Returns an error if the specified waypoints, segment times, smooth_upto or polyorder is invalid
    /// # Examples
    /// ```
    /// use peng_quad::QPpolyTrajPlanner;
    /// use nalgebra::{DMatrix, DVector, Vector3};
    /// let waypoints: Vec<Vec<f32>> = vec![vec![0.0, 0.0, 0.0, 0.0], vec![1.0, 1.0, 1.5, 1.5707963267948966], vec![-1.0, 1.0, 1.75, 3.141592653589793], vec![0.0, -1.0, 1.0, -1.5707963267948966], vec![0.0, 0.0, 0.5, 0.0]];
    /// let segment_times = vec![5.0, 5.0, 5.0, 5.0];
    /// let polyorder = 9;
    /// let min_deriv = 3;
    /// let smooth_upto = 4;
    /// let max_velocity = 4.0;
    /// let max_acceleration = 2.0;
    /// let start_time = 0.0;
    /// let dt = 0.1;
    /// let mut qp_planner = QPpolyTrajPlanner::new(waypoints,segment_times,polyorder, min_deriv, smooth_upto,max_velocity,max_acceleration, start_time, dt).unwrap();
    /// ```
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        waypoints: Vec<Vec<f32>>,
        segment_times: Vec<f32>,
        polyorder: usize,
        min_deriv: usize,
        smooth_upto: usize,
        max_velocity: f32,
        max_acceleration: f32,
        start_time: f32,
        dt: f32,
    ) -> Result<Self, SimulationError> {
        if waypoints.len() < 2 {
            return Err(SimulationError::OtherError(
                "At least two waypoints are required".to_string(),
            ));
        }
        if waypoints.len() != segment_times.len() + 1 || waypoints[0].len() != 4 {
            return Err(SimulationError::OtherError("Number of segment times must be one less than number of waypoints, and waypoints must contain x, y, z, yaw values".to_string()));
        }
        if smooth_upto > polyorder {
            return Err(SimulationError::OtherError(format!(
                "smooth_upto ({smooth_upto}) cannot be greater than polynomial order ({polyorder})"
            )));
        }
        let mut planner = Self {
            coeff: DMatrix::zeros(segment_times.len() * polyorder, 4),
            polyorder,
            min_deriv,
            smooth_upto,
            segment_times,
            waypoints,
            max_velocity,
            max_acceleration,
            start_time,
            dt,
        };
        planner.compute_coefficients()?;
        Ok(planner)
    }

    /// Compute the position, velocity, yaw, and yaw_rate for a given time t in given segment
    /// # Arguments
    /// * `t` - The time at which to evaluate the polynomial
    /// * `segment` - The segment index for which to evaluate the polynomial
    /// # Returns
    /// * A tuple containing the position, velocity, yaw, and yaw_rate at time `t` in segment `segment`
    pub fn evaluate_polynomial(
        &self,
        t: f32,
        segment: usize,
    ) -> (Vector3<f32>, Vector3<f32>, f32, f32) {
        let mut position = Vector3::zeros();
        let mut velocity = Vector3::zeros();

        let pos_basis = self.basis(t, 0);
        let vel_basis = self.basis(t, 1);

        let row_offset = self.polyorder * segment;

        let coeff_segment = &self.coeff.view((row_offset, 0), (self.polyorder, 4));

        let pose_4d = pos_basis.transpose() * coeff_segment;
        let vel_4d = vel_basis.transpose() * coeff_segment;

        //Convert to f32. This is bad for performance but currently no other way as OSQP requires f64 and Peng utilizes f32.
        let pose_4d_f32 = pose_4d.map(|x| x as f32);
        let vel_4d_f32 = vel_4d.map(|x| x as f32);

        // Extract the necessary values
        for i in 0..3 {
            position[i] = pose_4d_f32[(0, i)];
            velocity[i] = vel_4d_f32[(0, i)];
        }
        let yaw = pose_4d_f32[(0, 3)];
        let yaw_rate = vel_4d_f32[(0, 3)];

        (position, velocity, yaw, yaw_rate)
    }

    /// Solve the Quadratic Programming problem and compute the coefficients for the whole trajectory.
    /// The coefficients will be arranged as (poly_order*num_segments, num_dims)
    /// # Errors
    /// * `SimulationError::OSQPError` - Failed to set up or solve OSQP problem
    fn compute_coefficients(&mut self) -> Result<(), SimulationError> {
        let settings: Settings = Settings::default()
            .max_iter(10000000)
            .eps_abs(1e-6)
            .eps_rel(5e-6)
            .verbose(false);

        let (p, q, a, l, u) = self.prepare_qp_problem();

        // Create an OSQP problem
        let mut problem = Problem::new(p, q.as_slice(), a, l.as_slice(), u.as_slice(), &settings)
            .map_err(|_| {
            SimulationError::OSQPError("Failed to set up OSQP problem".to_string())
        })?;

        // Solve problem
        let result = problem.solve();

        // Obtain the solution coefficients
        let coeff_vector = result
            .x()
            .map(|solution| DVector::from_iterator(solution.len(), solution.iter().cloned()))
            .ok_or(SimulationError::OSQPError(
                "Unable to solve OSQP problem".to_string(),
            ))?;

        self.coeff = DMatrix::from_column_slice(
            self.segment_times.len() * self.polyorder,
            4,
            coeff_vector.as_slice(),
        );

        Ok(())
    }

    /// Combine the objective function, the equality and inequality constraints into a format for OSQP
    /// # Returns
    /// * A tuple containing the Q matrix, the linear term vector, the A matrix, the lower bound vector, and the upper bound vector
    fn prepare_qp_problem(
        &self,
    ) -> (
        CscMatrix,
        DVector<f64>,
        CscMatrix,
        DVector<f64>,
        DVector<f64>,
    ) {
        // 1. Generate the Q Matrix for x, y, z, yaw. Since the objective function is the same for all, we simply clone the first matrix instead of recomputing for each dim.
        let num_dims = 4;
        let num_vars_per_dim = self.polyorder * self.segment_times.len();
        let num_vars = num_vars_per_dim * num_dims;

        // Stack Q matrices into a block-diagonal matrix (joint optimization)
        let q_x = self.generate_obj_function();
        let mut q = DMatrix::zeros(num_vars, num_vars);

        for dim in 0..num_dims {
            let row_offset = dim * num_vars_per_dim;
            let col_offset = dim * num_vars_per_dim;
            q.view_mut((row_offset, col_offset), (q_x.nrows(), q_x.ncols()))
                .copy_from(&q_x);
        }

        // 2. Generate the Equality constraints for x, y, z, yaw

        // Construct a block diagonal A_eq matrix
        let num_constraints_per_block = (2 + self.smooth_upto) * (self.segment_times.len() + 1) - 2;
        let mut a_eq = DMatrix::zeros(num_constraints_per_block * num_dims, num_vars);
        let mut b_eq = DVector::zeros(num_constraints_per_block * num_dims);

        for dim in 0..num_dims {
            let row_offset = dim * num_constraints_per_block;
            let col_offset = dim * num_vars_per_dim;
            let (small_a_eq, small_b_eq) = self.generate_equality_constraints(dim);
            a_eq.view_mut(
                (row_offset, col_offset),
                (num_constraints_per_block, num_vars_per_dim),
            )
            .copy_from(&small_a_eq);
            b_eq.rows_mut(row_offset, num_constraints_per_block)
                .copy_from(&small_b_eq);
        }

        // 3. Generate the Inequality constraints, similar to how we generate the Q Matrix.
        // If no inequality constraints, create empty matrices
        let mut a_ineq: DMatrix<f64>;
        let mut d: DVector<f64>;
        let mut f: DVector<f64>;
        if self.max_acceleration > 0.0 && self.max_velocity > 0.0 {
            let (a_ineq_x, d_x, f_x) = self.generate_inequality_constraints();
            a_ineq = DMatrix::zeros(a_ineq_x.nrows() * num_dims, num_vars);
            d = DVector::zeros(d_x.len() * num_dims);
            f = DVector::zeros(f_x.len() * num_dims);

            for dim in 0..num_dims {
                let row_offset = dim * a_ineq_x.nrows();
                let col_offset = dim * num_vars_per_dim;
                a_ineq
                    .view_mut(
                        (row_offset, col_offset),
                        (a_ineq_x.nrows(), a_ineq_x.ncols()),
                    )
                    .copy_from(&a_ineq_x);
                d.rows_mut(row_offset, d_x.len()).copy_from(&d_x);
                f.rows_mut(row_offset, f_x.len()).copy_from(&f_x);
            }
        } else {
            // If no inequality constraints, create empty matrices
            a_ineq = DMatrix::zeros(0, num_vars);
            d = DVector::zeros(0);
            f = DVector::zeros(0);
        }

        // 4. Combine equality and optionally, inequality constraints
        let total_constraints = a_eq.nrows() + a_ineq.nrows();
        let mut a = DMatrix::zeros(total_constraints, num_vars);
        a.view_mut((0, 0), (a_eq.nrows(), a_eq.ncols()))
            .copy_from(&a_eq);

        let mut l = DVector::zeros(total_constraints);
        let mut u = DVector::zeros(total_constraints);

        // Lower and upper bounds
        l.rows_mut(0, b_eq.len()).copy_from(&b_eq);
        u.rows_mut(0, b_eq.len()).copy_from(&b_eq); // Equality constraints: Enforcing l = u = b_eq is the same as A_eq*x = b_eq

        if a_ineq.nrows() > 0 {
            a.view_mut((a_eq.nrows(), 0), (a_ineq.nrows(), a_ineq.ncols()))
                .copy_from(&a_ineq);
            l.rows_mut(b_eq.len(), d.len()).copy_from(&d);
            u.rows_mut(b_eq.len(), f.len()).copy_from(&f);
        }

        // 5. Convert matrices to CSC format for osqp.rs
        let q_csc = self.convert_dense_to_sparse(&q).into_upper_tri();
        let a_csc = self.convert_dense_to_sparse(&a);

        (q_csc, DVector::zeros(num_vars), a_csc, l, u)
    }

    /// Generate the Q Cost Matrix and A Matrix in the required Csc sparse matrix form for osqp by converting the dense DMatrix.
    /// # Arguments
    /// * `a` - The dense matrix to be converted.
    /// # Returns
    /// * The sparse matrix in CSC format.
    fn convert_dense_to_sparse(&self, a: &DMatrix<f64>) -> CscMatrix {
        let (rows, cols) = a.shape();

        let column_major_iter: Vec<f64> = a
            .column_iter()
            .flat_map(|col| col.iter().copied().collect::<Vec<f64>>())
            .collect();

        CscMatrix::from_column_iter(rows, cols, column_major_iter)
    }

    /// Generate the entire objective function for the entire trajectory for a single dimension
    /// # Returns
    /// * The objective function matrix.
    fn generate_obj_function(&self) -> DMatrix<f64> {
        let num_segments = self.segment_times.len();
        let coeff_num = self.polyorder * num_segments;

        let mut q_total: DMatrix<f64> = DMatrix::zeros(coeff_num, coeff_num);

        for (i, &segment_time) in self.segment_times.iter().enumerate() {
            let q_segment = self.generate_q(self.min_deriv, segment_time);
            q_total
                .view_mut(
                    (i * self.polyorder, i * self.polyorder),
                    (self.polyorder, self.polyorder),
                )
                .copy_from(&q_segment);
        }
        q_total
    }

    /// Generate the Q matrix for a single segment of a single dimension
    /// # Arguments
    /// * `min_deriv` - The minimum derivative order to consider.
    /// * `time` - The time at which to evaluate the basis functions.
    /// # Returns
    /// * The Q matrix for the segment.
    fn generate_q(&self, min_deriv: usize, time: f32) -> DMatrix<f64> {
        let mut q: DMatrix<f64> = DMatrix::zeros(self.polyorder, self.polyorder);
        let poly = self.basis(time, min_deriv);

        for i in 0..self.polyorder {
            for j in 0..self.polyorder {
                let power_order = ((i + j) as f64 - (2 * min_deriv) as f64 + 1.0).max(1.0); // Avoid division by zero
                q[(i, j)] = poly[i] * poly[j] * time as f64 / power_order;
            }
        }
        q
    }

    /// Generate the equality matrix for a single dimension
    /// # Arguments
    /// * `dimension` - The dimension for which to generate the equality matrix.
    /// # Returns
    /// * The equality matrix and the vector of constraints.
    fn generate_equality_constraints(&self, dimension: usize) -> (DMatrix<f64>, DVector<f64>) {
        let num_segments = self.segment_times.len();
        let num_coeff = self.polyorder * num_segments;

        // 1 position constraint + 4 derivative constraints (upto snap) for each beginning and ending waypoint = 5*2 +
        // 2 position constraints + 4 derivative constraints (upto snap) for each intermediate waypoint = 6*num__intermediate_waypoints. This is basically 6*(num_segments+1)-2.
        let num_constraints = (2 + self.smooth_upto) * (num_segments + 1) - 2;

        // Initialize the equality matrix a and the vector b. a*x = b, where x is the vector of coefficients we want to compute.

        let mut a: DMatrix<f64> = DMatrix::zeros(num_constraints, num_coeff);
        let mut b: DVector<f64> = DVector::zeros(num_constraints);

        let mut row_index: usize = 0;

        for i in 0..=num_segments {
            let col_offset = i * self.polyorder;

            match i {
                // First part of the first waypoint of the trajectory. NOTE: The first position is assumed to be the position of the robot. Handles position[0] + derivatives = 0
                // This results in 5 constraints (rows) added here.
                0 => {
                    let row = self.basis(0.0, 0);
                    a.row_mut(row_index)
                        .columns_mut(col_offset, self.polyorder)
                        .copy_from(&row.transpose());
                    b[row_index] = self.waypoints[i][dimension] as f64;
                    row_index += 1;

                    //All derivatives from velocity up until snap must be 0
                    for j in 1..=self.smooth_upto {
                        let row = self.basis(0.0, j);
                        a.row_mut(row_index)
                            .columns_mut(col_offset, self.polyorder)
                            .copy_from(&row.transpose());
                        b[row_index] = 0.0;
                        row_index += 1;
                    }
                }
                //Last part of the last waypoint of the trajectory. Handles position[n-1] + derivatives = 0. This results in 5 constraints (rows) added here.
                n if n == num_segments => {
                    let time = self.segment_times[num_segments - 1];
                    let row = self.basis(time, 0);
                    a.row_mut(row_index)
                        .columns_mut(col_offset - self.polyorder, self.polyorder)
                        .copy_from(&row.transpose());
                    b[row_index] = self.waypoints[i][dimension] as f64;
                    row_index += 1;

                    //All derivatives from velocity up until snap must be 0
                    for j in 1..=self.smooth_upto {
                        let row = self.basis(time, j);
                        a.row_mut(row_index)
                            .columns_mut(col_offset - self.polyorder, self.polyorder)
                            .copy_from(&row.transpose());
                        b[row_index] = 0.0;
                        row_index += 1;
                    }
                }
                // Intermediate segments. Handles prev_segment_position[i-1] = waypoint[i] = curr_segment_position[i] + derivative[i-1] = derivative[i]
                // This results in 6 constraints (rows) added here.
                _ => {
                    {
                        // Position constraint. position at segment i-1 must be equal to position at i which is equal to waypoint[i]
                        let segment_prev = self.basis(self.segment_times[i - 1], 0);
                        let segment_next = self.basis(0.0, 0);
                        let prev_col_offset = (i - 1) * self.polyorder;

                        // Offset the col index to i-1 to assign it to the previous segment's coefficients
                        a.row_mut(row_index)
                            .columns_mut(prev_col_offset, self.polyorder)
                            .copy_from(&segment_prev.transpose());
                        b[row_index] = self.waypoints[i][dimension] as f64;
                        row_index += 1;
                        // Offset the col index to i to assign it to this segment's coefficients
                        a.row_mut(row_index)
                            .columns_mut(col_offset, self.polyorder)
                            .copy_from(&segment_next.transpose());
                        b[row_index] = self.waypoints[i][dimension] as f64;
                        row_index += 1;

                        // To ensure velocity upto snap continuity, we equate the derivatives of the previous segment with the derivatives of the next segment.
                        // Since we don't care what the value of the derivatives are, we just formulate the constraint such that deriv[prev_segment] - deriv[next_segment] = 0.
                        // We do this by placing the derivatives of the prev segment at those times in the same row, along with the negative of next segment's derivatives.

                        for j in 1..=self.smooth_upto {
                            let segment_prev = self.basis(self.segment_times[i - 1], j);
                            let segment_next = self.basis(0.0, j);

                            a.row_mut(row_index)
                                .columns_mut(prev_col_offset, self.polyorder)
                                .copy_from(&segment_prev.transpose());
                            a.row_mut(row_index)
                                .columns_mut(col_offset, self.polyorder)
                                .add_assign(-segment_next.transpose());

                            b[row_index] = 0.0;

                            row_index += 1;
                        }
                    }
                }
            }
        }

        (a, b)
    }
    /// Compute the number of constraints
    /// # Returns
    /// * The total number of constraints.
    fn compute_num_constraints(&self) -> usize {
        // Skip if no inequality constraints
        if self.max_velocity == 0.0 || self.max_acceleration == 0.0 {
            return 0;
        }
        // For each segment, compute number of timesteps
        let total_constraints: usize = self
            .segment_times
            .iter()
            .map(|&segment_time| {
                // Calculate number of steps, excluding the end point
                let num_timesteps = (segment_time / self.dt).floor() as usize;
                // 2 constraints (velocity and acceleration) per timestep
                num_timesteps * 2
            })
            .sum();
        total_constraints
    }

    /// Generate inequality constrain for a single dimension. This works by sampling the derivatives using the basis vector across time steps and
    /// enforcing velocity and acceleration constraints for each of those basis vectors.
    /// # Returns
    /// A tuple containing the inequality matrix `c`, the lower bound vector `d`, and the upper bound vector `f`.
    fn generate_inequality_constraints(&self) -> (DMatrix<f64>, DVector<f64>, DVector<f64>) {
        let num_segments = self.segment_times.len();
        let coeff_num = self.polyorder * num_segments;

        let num_constraints = self.compute_num_constraints();

        // Initialize the inequality matrix c, along with the vectors d and f, representing the inequality d <= c*x <= f, where x is the vector of coefficients.

        let mut c: DMatrix<f64> = DMatrix::zeros(num_constraints, coeff_num);
        let mut d = DVector::from_element(num_constraints, 0.0); // We fill this up to -max_velocity or -max_acceleration.
        let mut f = DVector::from_element(num_constraints, 0.0); // We fill this up.

        let mut row_index = 0;

        // Iterate over each segment, then each time step in one segment, compute its basis vector for velocity and acceleration, and set the maximum.
        for (i, &segment_time) in self.segment_times.iter().enumerate() {
            // Col index for each segment
            let col_offset = i * self.polyorder;

            let mut t = self.dt; // To avoid placing equality and inequality constraints at in the beginning.
                                 //Iterate over each time step in one segment
            while t < segment_time {
                let row_velocity = self.basis(t, 1);

                c.row_mut(row_index)
                    .columns_mut(col_offset, self.polyorder)
                    .copy_from(&row_velocity.transpose());
                f[row_index] = self.max_velocity as f64;
                d[row_index] = -self.max_velocity as f64;
                row_index += 1;

                let row_acceleration = self.basis(t, 2);

                c.row_mut(row_index)
                    .columns_mut(col_offset, self.polyorder)
                    .copy_from(&row_acceleration.transpose());
                f[row_index] = self.max_acceleration as f64;
                d[row_index] = -self.max_acceleration as f64;
                row_index += 1;
                t += self.dt; // Update time step to compute basis vector at the next step.
            }
        }
        (c, d, f)
    }

    /// Generate the basis vector of the given derivative order at the given time.
    /// # Arguments
    /// * `time` - The time at which to evaluate the basis vector.
    /// * `derivative` - The derivative order of the basis vector.
    /// # Returns
    /// * The basis vector of the given derivative order at the given time.
    fn basis(&self, time: f32, derivative: usize) -> DVector<f64> {
        assert!(derivative <= 4, "Derivative order must be less than 4");
        let time_f64 = time as f64;
        let t: Vec<f64> = (0..self.polyorder)
            .map(|i| time_f64.powi(i as i32))
            .collect();

        let mut b = DVector::from_element(self.polyorder, 0.0);

        for i in 0..self.polyorder {
            let power = i as isize - derivative as isize;
            let mut coeff = 1.0;

            if power < 0 {
                b[i] = 0.0;
            } else {
                for j in (i - derivative + 1..=i).rev() {
                    coeff *= j as f64;
                }
                b[i] = coeff * t[power as usize];
            }
        }
        b
    }
}

/// Implement the `Planner` trait for `QPpolyTrajPlanner`
impl Planner for QPpolyTrajPlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        let relative_time = time - self.start_time;
        // Find the current segment
        let mut segment_start_time = 0.0;
        let mut current_segment = 0;
        for (i, &segment_duration) in self.segment_times.iter().enumerate() {
            if relative_time < segment_start_time + segment_duration {
                current_segment = i;
                break;
            }
            segment_start_time += segment_duration;
        }
        // Evaluate the polynomial for the current segment
        let segment_time = relative_time - segment_start_time;
        let (position, velocity, yaw, _yaw_rate) =
            self.evaluate_polynomial(segment_time, current_segment);
        (position, velocity, yaw)
    }

    fn is_finished(
        &self,
        current_position: Vector3<f32>,
        time: f32,
    ) -> Result<bool, SimulationError> {
        let last_waypoint = self.waypoints.last().ok_or(SimulationError::OtherError(
            "No waypoints available".to_string(),
        ))?;

        let total_time: f32 = self.segment_times.iter().sum();

        let last_position = Vector3::new(last_waypoint[0], last_waypoint[1], last_waypoint[2]);

        Ok(time >= self.start_time + total_time && (current_position - last_position).norm() < 0.1)
    }
}
/// Represents a step in the planner schedule.
/// # Example
/// ```
/// use peng_quad::PlannerStepConfig;
/// let step = PlannerStepConfig {
///     step: 0,
///     planner_type: "MinimumJerkLocalPlanner".to_string(),
///     params: serde_yaml::Value::Null,
/// };
/// ```
pub struct PlannerStepConfig {
    /// The simulation step at which this planner should be activated (in ms unit).
    pub step: usize,
    /// The type of planner to use for this step.
    pub planner_type: String,
    /// Additional parameters for the planner, stored as a YAML value.
    pub params: serde_yaml::Value,
}
/// Updates the planner based on the current simulation step and configuration
/// # Arguments
/// * `planner_manager` - The PlannerManager instance to update
/// * `step` - The current simulation step in ms unit
/// * `time` - The current simulation time
/// * `simulation_frequency' - The simulation frequency in Hz
/// * `quad` - The Quadrotor instance
/// * `obstacles` - The current obstacles in the simulation
/// * `planner_config` - The planner configuration
/// # Errors
/// * If the planner could not be created
/// # Example
/// ```
/// use peng_quad::{PlannerManager, Quadrotor, Obstacle, PlannerStepConfig, update_planner};
/// use nalgebra::Vector3;
/// let simulation_frequency = 1000;
/// let initial_position = Vector3::new(0.0, 0.0, 0.0);
/// let initial_yaw = 0.0;
/// let mut planner_manager = PlannerManager::new(initial_position, initial_yaw);
/// let step = 0;
/// let time = 0.0;
/// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
/// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
/// let quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
/// let obstacles = vec![Obstacle::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 1.0)];
/// let planner_config = vec![PlannerStepConfig {
///     step: 0,
///     planner_type: "MinimumJerkLine".to_string(),
///     params:
///        serde_yaml::from_str(r#"
///        end_position: [0.0, 0.0, 1.0]
///        end_yaw: 0.0
///        duration: 2.0
///        "#).unwrap(),
/// }];
/// update_planner(&mut planner_manager, step, time, simulation_frequency, &quadrotor, &obstacles, &planner_config).unwrap();
/// ```
pub fn update_planner(
    planner_manager: &mut PlannerManager,
    step: usize,
    time: f32,
    simulation_frequency: usize,
    quad: &Quadrotor,
    obstacles: &[Obstacle],
    planner_config: &[PlannerStepConfig],
) -> Result<(), SimulationError> {
    if let Some(planner_step) = planner_config
        .iter()
        .find(|s| s.step * simulation_frequency == step * 1000)
    {
        log::info!("Time: {:.2} s,\tSwitch {}", time, planner_step.planner_type);
        planner_manager.set_planner(create_planner(planner_step, quad, time, obstacles)?);
    }
    Ok(())
}
/// Creates a planner based on the configuration
/// # Arguments
/// * `step` - The configuration for the planner step in ms unit
/// * `quad` - The Quadrotor instance
/// * `time` - The current simulation time
/// * `obstacles` - The current obstacles in the simulation
/// # Returns
/// * `PlannerType` - The created planner
/// # Errors
/// * If the planner type is not recognized
/// # Example
/// ```
/// use peng_quad::{PlannerType, Quadrotor, Obstacle, PlannerStepConfig, create_planner};
/// use nalgebra::Vector3;
/// let step = PlannerStepConfig {
///    step: 0,
///   planner_type: "MinimumJerkLine".to_string(),
///   params:
///       serde_yaml::from_str(r#"
///       end_position: [0.0, 0.0, 1.0]
///       end_yaw: 0.0
///       duration: 2.0
///       "#).unwrap(),
/// };
/// let time = 0.0;
/// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
/// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
/// let quadrotor = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
/// let obstacles = vec![Obstacle::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 1.0)];
/// let planner = create_planner(&step, &quadrotor, time, &obstacles).unwrap();
/// match planner {
///    PlannerType::MinimumJerkLine(_) => log::info!("Created MinimumJerkLine planner"),
///   _ => log::info!("Created another planner"),
/// }
/// ```
pub fn create_planner(
    step: &PlannerStepConfig,
    quad: &Quadrotor,
    time: f32,
    obstacles: &[Obstacle],
) -> Result<PlannerType, SimulationError> {
    let params = &step.params;
    match step.planner_type.as_str() {
        "MinimumJerkLine" => Ok(PlannerType::MinimumJerkLine(MinimumJerkLinePlanner {
            start_position: quad.position,
            end_position: parse_vector3(params, "end_position")?,
            start_yaw: quad.orientation.euler_angles().2,
            end_yaw: parse_f32(params, "end_yaw")?,
            start_time: time,
            duration: parse_f32(params, "duration")?,
        })),
        "Lissajous" => Ok(PlannerType::Lissajous(LissajousPlanner {
            start_position: quad.position,
            center: parse_vector3(params, "center")?,
            amplitude: parse_vector3(params, "amplitude")?,
            frequency: parse_vector3(params, "frequency")?,
            phase: parse_vector3(params, "phase")?,
            start_time: time,
            duration: parse_f32(params, "duration")?,
            start_yaw: quad.orientation.euler_angles().2,
            end_yaw: parse_f32(params, "end_yaw")?,
            ramp_time: parse_f32(params, "ramp_time")?,
        })),
        "Circle" => Ok(PlannerType::Circle(CirclePlanner {
            center: parse_vector3(params, "center")?,
            radius: parse_f32(params, "radius")?,
            angular_velocity: parse_f32(params, "angular_velocity")?,
            start_position: quad.position,
            start_time: time,
            duration: parse_f32(params, "duration")?,
            start_yaw: quad.orientation.euler_angles().2,
            end_yaw: quad.orientation.euler_angles().2,
            ramp_time: parse_f32(params, "ramp_time")?,
        })),
        "ObstacleAvoidance" => Ok(PlannerType::ObstacleAvoidance(ObstacleAvoidancePlanner {
            target_position: parse_vector3(params, "target_position")?,
            start_time: time,
            duration: parse_f32(params, "duration")?,
            start_yaw: quad.orientation.euler_angles().2,
            end_yaw: parse_f32(params, "end_yaw")?,
            obstacles: obstacles.to_owned(),
            k_att: parse_f32(params, "k_att")?,
            k_rep: parse_f32(params, "k_rep")?,
            k_vortex: parse_f32(params, "k_vortex")?,
            d0: parse_f32(params, "d0")?,
            d_target: parse_f32(params, "d_target")?,
            max_speed: parse_f32(params, "max_speed")?,
        })),
        "MinimumSnapWaypoint" => {
            let mut waypoints = vec![quad.position];
            waypoints.extend(
                params["waypoints"]
                    .as_sequence()
                    .ok_or_else(|| SimulationError::OtherError("Invalid waypoints".to_string()))?
                    .iter()
                    .map(|w| {
                        w.as_sequence()
                            .and_then(|coords| {
                                Some(Vector3::new(
                                    coords[0].as_f64()? as f32,
                                    coords[1].as_f64()? as f32,
                                    coords[2].as_f64()? as f32,
                                ))
                            })
                            .ok_or(SimulationError::OtherError("Invalid waypoint".to_string()))
                    })
                    .collect::<Result<Vec<Vector3<f32>>, SimulationError>>()?,
            );
            let mut yaws = vec![quad.orientation.euler_angles().2];
            yaws.extend(
                params["yaws"]
                    .as_sequence()
                    .ok_or(SimulationError::OtherError("Invalid yaws".to_string()))?
                    .iter()
                    .map(|y| {
                        y.as_f64()
                            .map(|v| v as f32)
                            .ok_or(SimulationError::OtherError("Invalid yaw".to_string()))
                    })
                    .collect::<Result<Vec<f32>, SimulationError>>()?,
            );
            let segment_times = params["segment_times"]
                .as_sequence()
                .ok_or_else(|| SimulationError::OtherError("Invalid segment_times".to_string()))?
                .iter()
                .map(|t| {
                    t.as_f64().map(|v| v as f32).ok_or_else(|| {
                        SimulationError::OtherError("Invalid segment time".to_string())
                    })
                })
                .collect::<Result<Vec<f32>, SimulationError>>()?;
            MinimumSnapWaypointPlanner::new(waypoints, yaws, segment_times, time)
                .map(PlannerType::MinimumSnapWaypoint)
        }
        "QPpolyTraj" => {
            let mut waypoints = vec![vec![
                quad.position.x,
                quad.position.y,
                quad.position.z,
                quad.orientation.euler_angles().2,
            ]];
            waypoints.extend(
                params["waypoints"]
                    .as_sequence()
                    .ok_or_else(|| SimulationError::OtherError("Invalid waypoints".to_string()))?
                    .iter()
                    .map(|w| {
                        w.as_sequence()
                            .and_then(|coords| {
                                Some(vec![
                                    coords[0].as_f64()? as f32,
                                    coords[1].as_f64()? as f32,
                                    coords[2].as_f64()? as f32,
                                    coords[3].as_f64()? as f32,
                                ])
                            })
                            .ok_or(SimulationError::OtherError("Invalid waypoint".to_string()))
                    })
                    .collect::<Result<Vec<Vec<f32>>, SimulationError>>()?,
            );

            let segment_times = params["segment_times"]
                .as_sequence()
                .ok_or_else(|| SimulationError::OtherError("Invalid segment_times".to_string()))?
                .iter()
                .map(|t| {
                    t.as_f64().map(|v| v as f32).ok_or_else(|| {
                        SimulationError::OtherError("Invalid segment time".to_string())
                    })
                })
                .collect::<Result<Vec<f32>, SimulationError>>()?;
            QPpolyTrajPlanner::new(
                waypoints,
                segment_times,
                parse_usize(params, "polynomial_order")?,
                parse_usize(params, "minimize_derivative")?,
                parse_usize(params, "smooth_upto")?,
                parse_f32(params, "max_velocity")?,
                parse_f32(params, "max_acceleration")?,
                time,
                parse_f32(params, "dt")?,
            )
            .map(PlannerType::QPpolyTraj)
        }
        "Landing" => Ok(PlannerType::Landing(LandingPlanner {
            start_position: quad.position,
            start_time: time,
            duration: parse_f32(params, "duration")?,
            start_yaw: quad.orientation.euler_angles().2,
        })),
        _ => Err(SimulationError::OtherError(format!(
            "Unknown planner type: {}",
            step.planner_type
        ))),
    }
}
/// Helper function to parse Vector3 from YAML
/// # Arguments
/// * `value` - YAML value
/// * `key` - key to parse
/// # Returns
/// * `Vector3<f32>` - parsed vector
/// # Errors
/// * `SimulationError` - if the value is not a valid vector
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::{parse_vector3, SimulationError};
/// let value = serde_yaml::from_str("test: [1.0, 2.0, 3.0]").unwrap();
/// assert_eq!(parse_vector3(&value, "test").unwrap(), Vector3::new(1.0, 2.0, 3.0));
/// ```
pub fn parse_vector3(
    value: &serde_yaml::Value,
    key: &str,
) -> Result<Vector3<f32>, SimulationError> {
    value[key]
        .as_sequence()
        .and_then(|seq| {
            if seq.len() == 3 {
                Some(Vector3::new(
                    seq[0].as_f64()? as f32,
                    seq[1].as_f64()? as f32,
                    seq[2].as_f64()? as f32,
                ))
            } else {
                None
            }
        })
        .ok_or_else(|| SimulationError::OtherError(format!("Invalid {key} vector")))
}
/// Helper function to parse f32 from YAML
/// # Arguments
/// * `value` - YAML value
/// * `key` - key to parse
/// # Returns
/// * `f32` - parsed value
/// # Errors
/// * `SimulationError` - if the value is not a valid f32
/// # Example
/// ```
/// use peng_quad::{parse_f32, SimulationError};
/// let value = serde_yaml::from_str("key: 1.0").unwrap();
/// let result = parse_f32(&value, "key").unwrap();
/// assert_eq!(result, 1.0);
/// ```
pub fn parse_f32(value: &serde_yaml::Value, key: &str) -> Result<f32, SimulationError> {
    value[key]
        .as_f64()
        .map(|v| v as f32)
        .ok_or_else(|| SimulationError::OtherError(format!("Invalid {key}")))
}

/// Helper function to parse unsigned integer from YAML
/// # Arguments
/// * `value` - YAML value
/// * `key` - key to parse
/// # Returns
/// * `usize` - parsed value
/// # Errors
/// * `SimulationError` - if the value is not a valid unsigned integer
/// # Example
/// ```
/// use peng_quad::{parse_usize, SimulationError};
/// let value = serde_yaml::from_str("key: 1").unwrap();
/// let result = parse_usize(&value, "key").unwrap();
/// assert_eq!(result, 1);
/// ```
pub fn parse_usize(value: &serde_yaml::Value, key: &str) -> Result<usize, SimulationError> {
    value[key]
        .as_i64()
        .and_then(|v| if v >= 0 { Some(v as usize) } else { None }) // Ensure non-negative
        .ok_or_else(|| SimulationError::OtherError(format!("Invalid {key}")))
}
/// Represents an obstacle in the simulation
/// # Example
/// ```
/// use peng_quad::Obstacle;
/// use nalgebra::Vector3;
/// let obstacle = Obstacle::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 1.0);
/// ```
#[derive(Clone)]
pub struct Obstacle {
    /// The position of the obstacle
    pub position: Vector3<f32>,
    /// The velocity of the obstacle
    pub velocity: Vector3<f32>,
    /// The radius of the obstacle
    pub radius: f32,
}
/// Implementation of the Obstacle
impl Obstacle {
    /// Creates a new obstacle with the given position, velocity, and radius
    /// # Arguments
    /// * `position` - The position of the obstacle
    /// * `velocity` - The velocity of the obstacle
    /// * `radius` - The radius of the obstacle
    /// # Returns
    /// * The new obstacle instance
    /// # Example
    /// ```
    /// use peng_quad::Obstacle;
    /// use nalgebra::Vector3;
    /// let obstacle = Obstacle::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 1.0);
    /// ```
    pub fn new(position: Vector3<f32>, velocity: Vector3<f32>, radius: f32) -> Self {
        Self {
            position,
            velocity,
            radius,
        }
    }
}
/// Represents a maze in the simulation
/// # Example
/// ```
/// use peng_quad::{Maze, Obstacle};
/// use rand_chacha::ChaCha8Rng;
/// use rand::SeedableRng;
/// use nalgebra::Vector3;
/// let maze = Maze {
///     lower_bounds: [0.0, 0.0, 0.0],
///     upper_bounds: [1.0, 1.0, 1.0],
///     obstacles: vec![Obstacle::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0), 1.0)],
///     obstacles_velocity_bounds: [0.0, 0.0, 0.0],
///     obstacles_radius_bounds: [0.0, 0.0],
///     rng: ChaCha8Rng::from_os_rng(),
/// };
/// ```
pub struct Maze {
    /// The lower bounds of the maze in the x, y, and z directions
    pub lower_bounds: [f32; 3],
    /// The upper bounds of the maze in the x, y, and z directions
    pub upper_bounds: [f32; 3],
    /// The obstacles in the maze
    pub obstacles: Vec<Obstacle>,
    /// The bounds of the obstacles' velocity
    pub obstacles_velocity_bounds: [f32; 3],
    /// The bounds of the obstacles' radius
    pub obstacles_radius_bounds: [f32; 2],
    /// Rng for generating random numbers
    pub rng: ChaCha8Rng,
}
/// Implementation of the maze
impl Maze {
    /// Creates a new maze with the given bounds and number of obstacles
    /// # Arguments
    /// * `lower_bounds` - The lower bounds of the maze
    /// * `upper_bounds` - The upper bounds of the maze
    /// * `num_obstacles` - The number of obstacles in the maze
    /// # Returns
    /// * The new maze instance
    /// # Example
    /// ```
    /// use peng_quad::Maze;
    /// let maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
    /// ```
    pub fn new(
        lower_bounds: [f32; 3],
        upper_bounds: [f32; 3],
        num_obstacles: usize,
        obstacles_velocity_bounds: [f32; 3],
        obstacles_radius_bounds: [f32; 2],
    ) -> Self {
        let mut maze = Maze {
            lower_bounds,
            upper_bounds,
            obstacles: Vec::new(),
            obstacles_velocity_bounds,
            obstacles_radius_bounds,
            rng: ChaCha8Rng::from_os_rng(),
        };
        maze.generate_obstacles(num_obstacles);
        maze
    }
    /// Generates the obstacles in the maze
    /// # Arguments
    /// * `num_obstacles` - The number of obstacles to generate
    /// # Example
    /// ```
    /// use peng_quad::Maze;
    /// let mut maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
    /// maze.generate_obstacles(5);
    /// ```
    pub fn generate_obstacles(&mut self, num_obstacles: usize) {
        self.obstacles = (0..num_obstacles)
            .map(|_| {
                let position = Vector3::new(
                    rand::Rng::random_range(
                        &mut self.rng,
                        self.lower_bounds[0]..self.upper_bounds[0],
                    ),
                    rand::Rng::random_range(
                        &mut self.rng,
                        self.lower_bounds[1]..self.upper_bounds[1],
                    ),
                    rand::Rng::random_range(
                        &mut self.rng,
                        self.lower_bounds[2]..self.upper_bounds[2],
                    ),
                );
                let v_bounds = self.obstacles_velocity_bounds;
                let r_bounds = self.obstacles_radius_bounds;
                let velocity = Vector3::new(
                    rand::Rng::random_range(&mut self.rng, -v_bounds[0]..v_bounds[0]),
                    rand::Rng::random_range(&mut self.rng, -v_bounds[1]..v_bounds[1]),
                    rand::Rng::random_range(&mut self.rng, -v_bounds[2]..v_bounds[2]),
                );
                let radius = rand::Rng::random_range(&mut self.rng, r_bounds[0]..r_bounds[1]);
                Obstacle::new(position, velocity, radius)
            })
            .collect();
    }
    /// Updates the obstacles in the maze, if an obstacle hits a boundary, it bounces off
    /// # Arguments
    /// * `dt` - The time step
    /// # Example
    /// ```
    /// use peng_quad::Maze;
    /// let mut maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
    /// maze.update_obstacles(0.1);
    /// ```
    pub fn update_obstacles(&mut self, dt: f32) {
        self.obstacles.iter_mut().for_each(|obstacle| {
            obstacle.position += obstacle.velocity * dt;
            for i in 0..3 {
                if obstacle.position[i] - obstacle.radius < self.lower_bounds[i]
                    || obstacle.position[i] + obstacle.radius > self.upper_bounds[i]
                {
                    obstacle.velocity[i] *= -1.0;
                }
            }
        });
    }
}
/// Represents a camera in the simulation which is used to render the depth of the scene
/// # Example
/// ```
/// use peng_quad::Camera;
/// let camera = Camera::new((800, 600), 60.0, 0.1, 100.0);
/// ```
pub struct Camera {
    /// The resolution of the camera
    pub resolution: (usize, usize),
    /// The vertical field of view of the camera
    pub fov_vertical: f32,
    /// The horizontal field of view of the camera
    pub fov_horizontal: f32,
    /// The vertical focal length of the camera
    pub vertical_focal_length: f32,
    /// The horizontal focal length of the camera
    pub horizontal_focal_length: f32,
    /// The near clipping plane of the camera
    pub near: f32,
    /// The far clipping plane of the camera
    pub far: f32,
    /// The aspect ratio of the camera
    pub aspect_ratio: f32,
    /// The ray directions of each pixel in the camera
    pub ray_directions: Vec<Vector3<f32>>,
    /// Depth buffer
    pub depth_buffer: Vec<f32>,
}
/// Implementation of the camera
impl Camera {
    /// Creates a new camera with the given resolution, field of view, near and far clipping planes
    /// # Arguments
    /// * `resolution` - The resolution of the camera
    /// * `fov_vertical` - The vertical field of view of the camera
    /// * `near` - The near clipping plane of the camera
    /// * `far` - The far clipping plane of the camera
    /// # Returns
    /// * The new camera instance
    /// # Example
    /// ```
    /// use peng_quad::Camera;
    /// let camera = Camera::new((800, 600), 1.0, 5.0, 120.0);
    /// ```
    pub fn new(resolution: (usize, usize), fov_vertical: f32, near: f32, far: f32) -> Self {
        let (width, height) = resolution;
        let (aspect_ratio, tan_half_fov) =
            (width as f32 / height as f32, (fov_vertical / 2.0).tan());
        let mut ray_directions = Vec::with_capacity(width * height);
        for y in 0..height {
            for x in 0..width {
                let x_ndc = (2.0 * x as f32 / width as f32 - 1.0) * aspect_ratio * tan_half_fov;
                let y_ndc = (1.0 - 2.0 * y as f32 / height as f32) * tan_half_fov;
                ray_directions.push(Vector3::new(1.0, x_ndc, y_ndc).normalize());
            }
        }
        let fov_horizontal =
            (width as f32 / height as f32 * (fov_vertical / 2.0).tan()).atan() * 2.0;
        let horizontal_focal_length = (width as f32 / 2.0) / ((fov_horizontal / 2.0).tan());
        let vertical_focal_length = (height as f32 / 2.0) / ((fov_vertical / 2.0).tan());
        let depth_buffer = vec![0.0; width * height];

        Self {
            resolution,
            fov_vertical,
            fov_horizontal,
            vertical_focal_length,
            horizontal_focal_length,
            near,
            far,
            aspect_ratio,
            ray_directions,
            depth_buffer,
        }
    }

    /// Renders the depth of the scene from the perspective of the quadrotor
    ///
    /// When the depth value is out of the near and far clipping planes, it is set to infinity
    /// When the resolution is larger than 32x24, multi-threading can accelerate the rendering
    /// # Arguments
    /// * `quad_position` - The position of the quadrotor
    /// * `quad_orientation` - The orientation of the quadrotor
    /// * `maze` - The maze in the scene
    /// * `use_multi_threading` - Whether to use multi-threading to render the depth
    /// # Errors
    /// * If the depth buffer is not large enough to store the depth values
    /// # Example
    /// ```
    /// use peng_quad::{Camera, Maze};
    /// use nalgebra::{Vector3, UnitQuaternion};
    /// let mut camera = Camera::new((800, 600), 60.0, 0.1, 100.0);
    /// let quad_position = Vector3::new(0.0, 0.0, 0.0);
    /// let quad_orientation = UnitQuaternion::identity();
    /// let mut maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
    /// let use_multi_threading = true;
    /// camera.render_depth(&quad_position, &quad_orientation, &maze, use_multi_threading);
    /// let use_multi_threading = false;
    /// camera.render_depth(&quad_position, &quad_orientation, &maze, use_multi_threading);
    /// ```
    pub fn render_depth(
        &mut self,
        quad_position: &Vector3<f32>,
        quad_orientation: &UnitQuaternion<f32>,
        maze: &Maze,
        use_multi_threading: bool,
    ) -> Result<(), SimulationError> {
        let (width, height) = self.resolution;
        let total_pixels = width * height;
        let rotation_camera_to_world = quad_orientation.to_rotation_matrix().matrix()
            * Matrix3::new(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
        let rotation_world_to_camera = rotation_camera_to_world.transpose();

        const CHUNK_SIZE: usize = 64;
        if use_multi_threading {
            self.depth_buffer
                .par_chunks_mut(CHUNK_SIZE)
                .enumerate()
                .try_for_each(|(chunk_idx, chunk)| {
                    let start_idx = chunk_idx * CHUNK_SIZE;
                    for (i, depth) in chunk.iter_mut().enumerate() {
                        let ray_idx = start_idx + i;
                        if ray_idx >= total_pixels {
                            break;
                        }
                        *depth = ray_cast(
                            quad_position,
                            &rotation_world_to_camera,
                            &(rotation_camera_to_world * self.ray_directions[ray_idx]),
                            maze,
                            self.near,
                            self.far,
                        )?;
                    }
                    Ok::<(), SimulationError>(())
                })?;
        } else {
            for i in 0..total_pixels {
                self.depth_buffer[i] = ray_cast(
                    quad_position,
                    &rotation_world_to_camera,
                    &(rotation_camera_to_world * self.ray_directions[i]),
                    maze,
                    self.near,
                    self.far,
                )?;
            }
        }
        Ok(())
    }
}

/// Casts a ray from the camera origin in the given direction
/// # Arguments
/// * `origin` - The origin of the ray
/// * `rotation_world_to_camera` - The rotation matrix from world to camera coordinates
/// * `direction` - The direction of the ray
/// * `maze` - The maze in the scene
/// * `near` - The minimum distance to consider
/// * `far` - The maximum distance to consider
/// # Returns
/// * The distance to the closest obstacle hit by the ray
/// # Errors
/// * If the ray does not hit any obstacles
/// # Example
/// ```
/// use peng_quad::{ray_cast, Maze};
/// use nalgebra::{Vector3, Matrix3};
/// let origin = Vector3::new(0.0, 0.0, 0.0);
/// let rotation_world_to_camera = Matrix3::identity();
/// let direction = Vector3::new(0.0, 0.0, 1.0);
/// let maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
/// let near = 0.1;
/// let far = 100.0;
/// let distance = ray_cast(&origin, &rotation_world_to_camera, &direction, &maze, near, far);
/// ```
pub fn ray_cast(
    origin: &Vector3<f32>,
    rotation_world_to_camera: &Matrix3<f32>,
    direction: &Vector3<f32>,
    maze: &Maze,
    near: f32,
    far: f32,
) -> Result<f32, SimulationError> {
    let mut closest_hit = far;
    // Inline tube intersection
    for axis in 0..3 {
        let t_near = if direction[axis] > 0.0 {
            (maze.upper_bounds[axis] - origin[axis]) / direction[axis]
        } else {
            (maze.lower_bounds[axis] - origin[axis]) / direction[axis]
        };
        if t_near > near && t_near < closest_hit {
            let intersection_point = origin + direction * t_near;
            let mut valid = true;
            for i in 0..3 {
                if i != axis
                    && (intersection_point[i] < maze.lower_bounds[i]
                        || intersection_point[i] > maze.upper_bounds[i])
                {
                    valid = false;
                    break;
                }
            }
            if valid {
                closest_hit = t_near;
            }
        }
    }
    // Early exit if we've hit a wall closer than any possible obstacle
    if closest_hit <= near {
        return Ok(f32::INFINITY);
    }
    // Inline sphere intersection
    for obstacle in &maze.obstacles {
        let oc = origin - obstacle.position;
        let b = oc.dot(direction);
        let c = oc.dot(&oc) - obstacle.radius * obstacle.radius;
        let discriminant = b * b - c;
        if discriminant >= 0.0 {
            // let t = -b - discriminant.sqrt();
            let t = -b - fast_sqrt(discriminant);
            if t > near && t < closest_hit {
                closest_hit = t;
            }
        }
    }
    if closest_hit < far {
        Ok((rotation_world_to_camera * direction * closest_hit).x)
    } else {
        Ok(f32::INFINITY)
    }
}
/// Logs simulation data to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `quad` - The Quadrotor instance
/// * `desired_position` - The desired position vector
/// * `measured_accel` - The measured acceleration vector
/// * `measured_gyro` - The measured angular velocity vector
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::{Quadrotor, log_data};
/// use nalgebra::Vector3;
/// let rec = rerun::RecordingStreamBuilder::new("peng").spawn().unwrap();
/// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
/// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
/// let quad = Quadrotor::new(time_step, mass, gravity, drag_coefficient, inertia_matrix).unwrap();
/// let desired_position = Vector3::new(0.0, 0.0, 0.0);
/// let desired_velocity = Vector3::new(0.0, 0.0, 0.0);
/// let measured_accel = Vector3::new(0.0, 0.0, 0.0);
/// let measured_gyro = Vector3::new(0.0, 0.0, 0.0);
/// log_data(&rec, &quad, &desired_position, &desired_velocity, &measured_accel, &measured_gyro).unwrap();
/// ```
pub fn log_data(
    rec: &rerun::RecordingStream,
    quad: &Quadrotor,
    desired_position: &Vector3<f32>,
    desired_velocity: &Vector3<f32>,
    measured_accel: &Vector3<f32>,
    measured_gyro: &Vector3<f32>,
) -> Result<(), SimulationError> {
    rec.log(
        "world/quad/desired_position",
        &rerun::Points3D::new([(desired_position.x, desired_position.y, desired_position.z)])
            .with_radii([0.1])
            .with_colors([rerun::Color::from_rgb(255, 255, 255)]),
    )?;
    rec.log(
        "world/quad/base_link",
        &rerun::Transform3D::from_translation_rotation(
            rerun::Vec3D::new(quad.position.x, quad.position.y, quad.position.z),
            rerun::Quaternion::from_xyzw([
                quad.orientation.i,
                quad.orientation.j,
                quad.orientation.k,
                quad.orientation.w,
            ]),
        )
        .with_axis_length(0.7),
    )?;
    let (quad_roll, quad_pitch, quad_yaw) = quad.orientation.euler_angles();
    let quad_euler_angles: Vector3<f32> = Vector3::new(quad_roll, quad_pitch, quad_yaw);
    for (pre, vec) in [
        ("position", &quad.position),
        ("velocity", &quad.velocity),
        ("accel", measured_accel),
        ("orientation", &quad_euler_angles),
        ("gyro", measured_gyro),
        ("desired_position", desired_position),
        ("desired_velocity", desired_velocity),
    ] {
        for (i, a) in ["x", "y", "z"].iter().enumerate() {
            rec.log(format!("{pre}/{a}"), &rerun::Scalars::single(vec[i] as f64))?;
        }
    }
    Ok(())
}
/// Log the maze tube to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `maze` - The maze instance
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::{Maze, log_maze_tube};
/// use rerun::RecordingStreamBuilder;
/// let rec = rerun::RecordingStreamBuilder::new("peng").spawn().unwrap();
/// let mut maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
/// log_maze_tube(&rec, &maze).unwrap();
/// ```
pub fn log_maze_tube(rec: &rerun::RecordingStream, maze: &Maze) -> Result<(), SimulationError> {
    let (lower_bounds, upper_bounds) = (maze.lower_bounds, maze.upper_bounds);
    let center_position = rerun::external::glam::Vec3::new(
        (lower_bounds[0] + upper_bounds[0]) / 2.0,
        (lower_bounds[1] + upper_bounds[1]) / 2.0,
        (lower_bounds[2] + upper_bounds[2]) / 2.0,
    );
    let half_sizes = rerun::external::glam::Vec3::new(
        (upper_bounds[0] - lower_bounds[0]) / 2.0,
        (upper_bounds[1] - lower_bounds[1]) / 2.0,
        (upper_bounds[2] - lower_bounds[2]) / 2.0,
    );
    rec.log(
        "world/maze/tube",
        &rerun::Boxes3D::from_centers_and_half_sizes([center_position], [half_sizes])
            .with_colors([rerun::Color::from_rgb(128, 128, 255)]),
    )?;
    Ok(())
}
/// Log the maze obstacles to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `maze` - The maze instance
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::{Maze, log_maze_obstacles};
/// let rec = rerun::RecordingStreamBuilder::new("peng").spawn().unwrap();
/// let mut maze = Maze::new([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 5, [0.1, 0.1, 0.1], [0.1, 0.5]);
/// log_maze_obstacles(&rec, &maze).unwrap();
/// ```
pub fn log_maze_obstacles(
    rec: &rerun::RecordingStream,
    maze: &Maze,
) -> Result<(), SimulationError> {
    let (positions, radii): (Vec<(f32, f32, f32)>, Vec<f32>) = maze
        .obstacles
        .iter()
        .map(|obstacle| {
            let pos = obstacle.position;
            ((pos.x, pos.y, pos.z), obstacle.radius)
        })
        .unzip();
    rec.log(
        "world/maze/obstacles",
        &rerun::Points3D::new(positions)
            .with_radii(radii)
            .with_colors([rerun::Color::from_rgb(255, 128, 128)]),
    )?;
    Ok(())
}
/// A struct to hold trajectory data
/// # Example
/// ```
/// use peng_quad::Trajectory;
/// let initial_point = nalgebra::Vector3::new(0.0, 0.0, 0.0);
/// let mut trajectory = Trajectory::new(initial_point);
/// ```
pub struct Trajectory {
    /// A vector of 3D points
    pub points: Vec<Vector3<f32>>,
    /// The last point that was logged
    pub last_logged_point: Vector3<f32>,
    /// The minimum distance between points to log
    pub min_distance_threadhold: f32,
}
/// Implement the Trajectory struct
impl Trajectory {
    /// Create a new Trajectory instance
    /// # Arguments
    /// * `initial_point` - The initial point to add to the trajectory
    /// # Returns
    /// * A new Trajectory instance
    /// # Example
    /// ```
    /// use peng_quad::Trajectory;
    /// let initial_point = nalgebra::Vector3::new(0.0, 0.0, 0.0);
    /// let mut trajectory = Trajectory::new(initial_point);
    /// ```
    pub fn new(initial_point: Vector3<f32>) -> Self {
        Self {
            points: vec![initial_point],
            last_logged_point: initial_point,
            min_distance_threadhold: 0.05,
        }
    }
    /// Add a point to the trajectory if it is further than the minimum distance threshold
    /// # Arguments
    /// * `point` - The point to add
    /// # Returns
    /// * `true` if the point was added, `false` otherwise
    /// # Example
    /// ```
    /// use peng_quad::Trajectory;
    /// let mut trajectory = Trajectory::new(nalgebra::Vector3::new(0.0, 0.0, 0.0));
    /// let point = nalgebra::Vector3::new(1.0, 0.0, 0.0);
    /// assert_eq!(trajectory.add_point(point), true);
    /// assert_eq!(trajectory.add_point(point), false);
    /// ```
    pub fn add_point(&mut self, point: Vector3<f32>) -> bool {
        if (point - self.last_logged_point).norm() > self.min_distance_threadhold {
            self.points.push(point);
            self.last_logged_point = point;
            true
        } else {
            false
        }
    }
}
/// log trajectory data to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `trajectory` - The Trajectory instance
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::{Trajectory, log_trajectory};
/// use nalgebra::Vector3;
/// let rec = rerun::RecordingStreamBuilder::new("peng").spawn().unwrap();
/// let mut trajectory = Trajectory::new(nalgebra::Vector3::new(0.0, 0.0, 0.0));
/// trajectory.add_point(nalgebra::Vector3::new(1.0, 0.0, 0.0));
/// log_trajectory(&rec, &trajectory).unwrap();
/// ```
pub fn log_trajectory(
    rec: &rerun::RecordingStream,
    trajectory: &Trajectory,
) -> Result<(), SimulationError> {
    let path = trajectory
        .points
        .iter()
        .map(|p| (p.x, p.y, p.z))
        .collect::<Vec<(f32, f32, f32)>>();
    rec.log(
        "world/quad/path",
        &rerun::LineStrips3D::new([path]).with_colors([rerun::Color::from_rgb(0, 255, 255)]),
    )?;
    Ok(())
}
/// Log depth image data to the rerun recording stream
///
/// When the depth value is `f32::INFINITY`, the pixel is considered invalid and logged as black
/// When the resolution is larger than 32x24, multi-threading can accelerate the rendering
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `cam` - The Camera instance
/// * `use_multi_threading` - Whether to use multithreading to log the depth image
/// # Errors
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::{log_depth_image, Camera};
/// let rec = rerun::RecordingStreamBuilder::new("peng").spawn().unwrap();
/// let camera = Camera::new((640, 480), 0.1, 100.0, 60.0);
/// let use_multi_threading = false;
/// log_depth_image(&rec, &camera, use_multi_threading).unwrap();
/// let use_multi_threading = true;
/// log_depth_image(&rec, &camera, use_multi_threading).unwrap();
/// ```
pub fn log_depth_image(
    rec: &rerun::RecordingStream,
    cam: &Camera,
    use_multi_threading: bool,
) -> Result<(), SimulationError> {
    let (width, height) = (cam.resolution.0, cam.resolution.1);
    let (min_depth, max_depth) = (cam.near, cam.far);
    let depth_image = &cam.depth_buffer;
    let mut image: rerun::external::ndarray::Array<u8, _> =
        rerun::external::ndarray::Array::zeros((height, width, 3));
    let depth_range = max_depth - min_depth;
    let scale_factor = 255.0 / depth_range;
    if use_multi_threading {
        const CHUNK_SIZE: usize = 32;
        image
            .as_slice_mut()
            .expect("Failed to get mutable slice of image")
            .par_chunks_exact_mut(CHUNK_SIZE * 3)
            .enumerate()
            .for_each(|(chunk_index, chunk)| {
                let start_index = chunk_index * 32;
                for (i, pixel) in chunk.chunks_exact_mut(3).enumerate() {
                    let idx = start_index + i;
                    let depth = depth_image[idx];
                    let color = if depth.is_finite() {
                        let normalized_depth =
                            ((depth - min_depth) * scale_factor).clamp(0.0, 255.0);
                        color_map_fn(normalized_depth)
                    } else {
                        (0, 0, 0)
                    };
                    (pixel[0], pixel[1], pixel[2]) = color;
                }
            });
    } else {
        for (index, pixel) in image
            .as_slice_mut()
            .expect("Failed to get mutable slice of image")
            .chunks_exact_mut(3)
            .enumerate()
        {
            let depth = depth_image[index];
            let color = if depth.is_finite() {
                let normalized_depth = ((depth - min_depth) * scale_factor).clamp(0.0, 255.0);
                color_map_fn(normalized_depth)
            } else {
                (0, 0, 0)
            };
            (pixel[0], pixel[1], pixel[2]) = color;
        }
    }
    let rerun_image = rerun::Image::from_color_model_and_tensor(rerun::ColorModel::RGB, image)
        .map_err(|e| SimulationError::OtherError(format!("Failed to create rerun image: {e}")))?;
    rec.log("world/quad/cam/depth", &rerun_image)?;
    Ok(())
}
/// creates pinhole camera
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `cam` - The camera object
/// * `cam_position` - The position vector of the camera (aligns with the quad)
/// * `cam_orientation` - The orientation quaternion of quad
/// * `cam_transform` - The transform matrix between quad and camera alignment
/// # Errors    
/// * If the data cannot be logged to the recording stream
/// # Example
/// ```no_run
/// use peng_quad::{log_pinhole_depth, Camera};
/// use nalgebra::{Vector3, UnitQuaternion};
/// let rec = rerun::RecordingStreamBuilder::new("peng").spawn().unwrap();
/// let depth_image = vec![ 0.0f32 ; 640 * 480];
/// let cam_position = Vector3::new(0.0,0.0,0.0);
/// let cam_orientation = UnitQuaternion::identity();
/// let cam_transform = [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0];
/// let camera = Camera::new((800, 600), 60.0, 0.1, 100.0);
/// log_pinhole_depth(&rec, &camera, cam_position, cam_orientation, cam_transform).unwrap();
/// ```
pub fn log_pinhole_depth(
    rec: &rerun::RecordingStream,
    cam: &Camera,
    cam_position: Vector3<f32>,
    cam_orientation: UnitQuaternion<f32>,
    cam_transform: [f32; 9],
) -> Result<(), SimulationError> {
    let depth_image = &cam.depth_buffer;
    let (width, height) = cam.resolution;
    let pinhole_camera = rerun::Pinhole::from_focal_length_and_resolution(
        (cam.horizontal_focal_length, cam.vertical_focal_length),
        (width as f32, height as f32),
    )
    .with_camera_xyz(rerun::components::ViewCoordinates::RDF)
    .with_resolution((width as f32, height as f32))
    .with_principal_point((width as f32 / 2.0, height as f32 / 2.0));
    let rotated_camera_orientation = UnitQuaternion::from_rotation_matrix(
        &(cam_orientation.to_rotation_matrix()
            * Rotation3::from_matrix_unchecked(Matrix3::from_row_slice(&cam_transform))),
    );
    let cam_transform = rerun::Transform3D::from_translation_rotation(
        rerun::Vec3D::new(cam_position.x, cam_position.y, cam_position.z),
        rerun::Quaternion::from_xyzw([
            rotated_camera_orientation.i,
            rotated_camera_orientation.j,
            rotated_camera_orientation.k,
            rotated_camera_orientation.w,
        ]),
    );
    rec.log("world/quad/cam", &cam_transform)?;
    rec.log("world/quad/cam", &pinhole_camera)?;
    let depth_image_rerun =
        rerun::external::ndarray::Array::from_shape_vec((height, width), depth_image.to_vec())
            .unwrap();
    rec.log(
        "world/quad/cam/rerun_depth",
        &rerun::DepthImage::try_from(depth_image_rerun)
            .unwrap()
            .with_meter(1.0),
    )?;

    Ok(())
}
/// turbo color map function
/// # Arguments
/// * `gray` - The gray value in the range [0, 255]
/// # Returns
/// * The RGB color value in the range [0, 255]
/// # Example
/// ```
/// use peng_quad::color_map_fn;
/// let color = color_map_fn(128.0);
/// ```
#[inline]
pub fn color_map_fn(gray: f32) -> (u8, u8, u8) {
    let x = gray / 255.0;
    let r = (34.61
        + x * (1172.33 - x * (10793.56 - x * (33300.12 - x * (38394.49 - x * 14825.05)))))
        .clamp(0.0, 255.0) as u8;
    let g = (23.31 + x * (557.33 + x * (1225.33 - x * (3574.96 - x * (1073.77 + x * 707.56)))))
        .clamp(0.0, 255.0) as u8;
    let b = (27.2 + x * (3211.1 - x * (15327.97 - x * (27814.0 - x * (22569.18 - x * 6838.66)))))
        .clamp(0.0, 255.0) as u8;
    (r, g, b)
}

/// Fast square root function
/// # Arguments
/// * `x` - The input value
/// # Returns
/// * The square root of the input value
#[inline(always)]
fn fast_sqrt(x: f32) -> f32 {
    let i = x.to_bits();
    let i = 0x1fbd1df5 + (i >> 1);
    f32::from_bits(i)
}
