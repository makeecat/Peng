//! # Quadrotor Simulation
//!
//! This crate provides a comprehensive simulation environment for quadrotor drones.
//! It includes models for quadrotor dynamics, IMU simulation, various trajectory planners,
//! and a PID controller for position and attitude control.
//!
//! ## Features
//!
//! - Realistic quadrotor dynamics simulation
//! - IMU sensor simulation with configurable noise parameters
//! - Multiple trajectory planners including hover, minimum jerk, Lissajous curves, and circular paths
//! - PID controller for position and attitude control
//! - Integration with the `rerun` crate for visualization
//!
//! ## Usage
//!
//! To use this crate in your project, add the following to your `Cargo.toml`:
//!
//! ```toml
//! [dependencies]
//! quadrotor_sim = "0.1.0"
//! ```
//!
//! Then, you can use the crate in your Rust code:
//!
//! ```rust
//! use quadrotor_sim::{Quadrotor, PIDController, PlannerManager};
//!
//! fn main() {
//!     let mut quad = Quadrotor::new();
//!     let controller = PIDController::new();
//!     let planner = PlannerManager::new(quad.position, 0.0);
//!
//!     // Simulation loop
//!     // ...
//! }
//! ```
//!
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use rand_distr::{Distribution, Normal};
/// Represents a quadrotor with its physical properties and state
struct Quadrotor {
    /// Current position of the quadrotor in 3D space
    position: Vector3<f32>,
    /// Current velocity of the quadrotor
    velocity: Vector3<f32>,
    /// Current orientation of the quadrotor
    orientation: UnitQuaternion<f32>,
    /// Current angular velocity of the quadrotor
    angular_velocity: Vector3<f32>,
    /// Mass of the quadrotor in kg
    mass: f32,
    /// Gravitational acceleration in m/s^2
    gravity: f32,
    /// Simulation time step in seconds
    time_step: f32,
    /// Drag coefficient
    drag_coefficient: f32,
    /// Inertia matrix of the quadrotor
    inertia_matrix: Matrix3<f32>,
    /// Inverse of the inertia matrix
    inertia_matrix_inv: Matrix3<f32>,
}

impl Quadrotor {
    /// Creates a new Quadrotor with default parameters
    ///
    /// # Examples
    ///
    /// ```
    /// let quad = Quadrotor::new();
    /// assert_eq!(quad.mass, 1.3);
    /// ```
    pub fn new() -> Self {
        let inertia_matrix = Matrix3::new(
            0.00304475, 0.0, 0.0, 0.0, 0.00454981, 0.0, 0.0, 0.0, 0.00281995,
        );
        Self {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            mass: 1.3,
            gravity: 9.81,
            time_step: 0.01,
            // thrust_coefficient: 0.0,
            drag_coefficient: 0.000,
            inertia_matrix,
            inertia_matrix_inv: inertia_matrix.try_inverse().unwrap(),
        }
    }
    #[allow(dead_code)]
    pub fn update_dynamics(&mut self) {
        // Update position and velocity based on current state and simple physics
        let acceleration = Vector3::new(0.0, 0.0, -self.gravity);
        self.velocity += acceleration * self.time_step;
        self.position += self.velocity * self.time_step;
        self.orientation *=
            UnitQuaternion::from_scaled_axis(self.angular_velocity * self.time_step);
    }

    /// Updates the quadrotor's dynamics with control inputs
    /// # Arguments
    /// * `control_thrust` - The total thrust force applied to the quadrotor
    /// * `control_torque` - The 3D torque vector applied to the quadrotor
    /// # Examples
    /// ```
    /// let mut quad = Quadrotor::new();
    /// let thrust = 10.0;
    /// let torque = Vector3::new(0.1, 0.1, 0.1);
    /// quad.update_dynamics_with_controls(thrust, &torque);
    /// ```
    pub fn update_dynamics_with_controls(
        &mut self,
        control_thrust: f32,
        control_torque: &Vector3<f32>,
    ) {
        let gravity_force = Vector3::new(0.0, 0.0, -self.mass * self.gravity);
        let drag_force = -self.drag_coefficient * self.velocity.norm() * self.velocity;
        let thrust_body = Vector3::new(0.0, 0.0, control_thrust);
        let thrust_world = self.orientation * thrust_body;
        let total_force = thrust_world + gravity_force + drag_force;

        let acceleration = total_force / self.mass;
        self.velocity += acceleration * self.time_step;
        self.position += self.velocity * self.time_step;

        let inertia_angular_velocity = self.inertia_matrix * self.angular_velocity;
        let gyroscopic_torque = self.angular_velocity.cross(&inertia_angular_velocity);
        let angular_acceleration = self.inertia_matrix_inv * (control_torque - gyroscopic_torque);

        self.angular_velocity += angular_acceleration * self.time_step;
        self.orientation *=
            UnitQuaternion::from_scaled_axis(self.angular_velocity * self.time_step);
    }

    /// Simulates IMU readings with added noise
    /// # Returns
    /// A tuple containing the measured acceleration and angular velocity
    pub fn read_imu(&self) -> (Vector3<f32>, Vector3<f32>) {
        let accel_noise = Normal::new(0.0, 0.02).unwrap();
        let gyro_noise = Normal::new(0.0, 0.01).unwrap();
        let mut rng = rand::thread_rng();

        let gravity_world = Vector3::new(0.0, 0.0, self.gravity);
        let specific_force =
            self.orientation.inverse() * (self.velocity / self.time_step - gravity_world);
        let measured_acceleration = specific_force
            + Vector3::new(
                accel_noise.sample(&mut rng),
                accel_noise.sample(&mut rng),
                accel_noise.sample(&mut rng),
            );
        let measured_angular_velocity = self.angular_velocity
            + Vector3::new(
                gyro_noise.sample(&mut rng),
                gyro_noise.sample(&mut rng),
                gyro_noise.sample(&mut rng),
            );
        (measured_acceleration, measured_angular_velocity)
    }
}

/// Represents an Inertial Measurement Unit (IMU) with bias and noise characteristics
struct Imu {
    /// Accelerometer bias
    accel_bias: Vector3<f32>,
    /// Gyroscope bias
    gyro_bias: Vector3<f32>,
    /// Standard deviation of accelerometer noise
    accel_noise_std: f32,
    /// Standard deviation of gyroscope noise
    gyro_noise_std: f32,
    /// Bias instability coefficient
    bias_instability: f32,
}

impl Imu {
    /// Creates a new IMU with default parameters
    pub fn new() -> Self {
        Self {
            accel_bias: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            accel_noise_std: 0.02,
            gyro_noise_std: 0.01,
            bias_instability: 0.0001,
        }
    }

    /// Updates the IMU biases over time
    /// # Arguments
    /// * `dt` - Time step for the update
    pub fn update(&mut self, dt: f32) {
        let bias_drift = Normal::new(0.0, self.bias_instability * dt.sqrt()).unwrap();
        let drift_vector =
            || Vector3::from_iterator((0..3).map(|_| bias_drift.sample(&mut rand::thread_rng())));
        self.accel_bias += drift_vector();
        self.gyro_bias += drift_vector();
    }

    /// Simulates IMU readings with added bias and noise
    /// # Arguments
    /// * `true_acceleration` - The true acceleration vector
    /// * `true_angular_velocity` - The true angular velocity vector
    /// # Returns
    /// A tuple containing the measured acceleration and angular velocity
    pub fn read(
        &self,
        true_acceleration: Vector3<f32>,
        true_angular_velocity: Vector3<f32>,
    ) -> (Vector3<f32>, Vector3<f32>) {
        let mut rng = rand::thread_rng();
        let accel_noise = Normal::new(0.0, self.accel_noise_std).unwrap();
        let gyro_noise = Normal::new(0.0, self.gyro_noise_std).unwrap();

        let measured_acceleration = true_acceleration
            + self.accel_bias
            + Vector3::new(
                accel_noise.sample(&mut rng),
                accel_noise.sample(&mut rng),
                accel_noise.sample(&mut rng),
            );
        let measured_angular_velocity = true_angular_velocity
            + self.gyro_bias
            + Vector3::new(
                gyro_noise.sample(&mut rng),
                gyro_noise.sample(&mut rng),
                gyro_noise.sample(&mut rng),
            );
        (measured_acceleration, measured_angular_velocity)
    }
}

/// PID controller for quadrotor position and attitude control
struct PIDController {
    /// Proportional gain for position control
    kp_pos: Vector3<f32>,
    /// Derivative gain for position control
    kd_pos: Vector3<f32>,
    /// Proportional gain for attitude control
    kp_att: Vector3<f32>,
    /// Derivative gain for attitude control
    kd_att: Vector3<f32>,
    /// Integral gain for position control
    ki_pos: Vector3<f32>,
    /// Integral gain for attitude control
    ki_att: Vector3<f32>,
    /// Accumulated integral error for position
    integral_pos_error: Vector3<f32>,
    /// Accumulated integral error for attitude
    integral_att_error: Vector3<f32>,
    /// Maximum allowed integral error for position
    max_integral_pos: Vector3<f32>,
    /// Maximum allowed integral error for attitude
    max_integral_att: Vector3<f32>,
}

impl PIDController {
    /// Creates a new PIDController with default gains
    /// # Examples
    /// ```
    /// let controller = PIDController::new();
    /// assert_eq!(controller.kp_pos, Vector3::new(7.1, 7.1, 11.9));
    /// ```
    fn new() -> Self {
        Self {
            kp_pos: Vector3::new(7.1, 7.1, 11.9),
            kd_pos: Vector3::new(2.4, 2.4, 6.7),
            kp_att: Vector3::new(1.5, 1.5, 1.0),
            kd_att: Vector3::new(0.13, 0.13, 0.1),
            ki_pos: Vector3::new(0.00, 0.00, 0.00),
            ki_att: Vector3::new(0.00, 0.00, 0.00),
            integral_pos_error: Vector3::zeros(),
            integral_att_error: Vector3::zeros(),
            max_integral_pos: Vector3::new(10.0, 10.0, 10.0),
            max_integral_att: Vector3::new(1.0, 1.0, 1.0),
        }
    }

    /// Computes attitude control torques
    /// # Arguments
    /// * `desired_orientation` - The desired orientation quaternion
    /// * `current_orientation` - The current orientation quaternion
    /// * `current_angular_velocity` - The current angular velocity
    /// * `dt` - Time step
    /// # Returns
    /// The computed control torque vector
    fn compute_attitude_control(
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
        self.integral_att_error
            .component_mul_assign(&self.max_integral_att.map(|x| x.signum()));
        self.integral_att_error = self
            .integral_att_error
            .zip_map(&self.max_integral_att, |int, max| int.clamp(-max, max));

        let error_angular_velocity = -current_angular_velocity;

        self.kp_att.component_mul(&error_angles)
            + self.kd_att.component_mul(&error_angular_velocity)
            + self.ki_att.component_mul(&self.integral_att_error)
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
    /// A tuple containing the computed thrust and desired orientation quaternion
    fn compute_position_control(
        &mut self,
        desired_position: Vector3<f32>,
        desired_velocity: Vector3<f32>,
        desired_yaw: f32,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        dt: f32,
        mass: f32,
        gravity: f32,
    ) -> (f32, UnitQuaternion<f32>) {
        let error_position = desired_position - current_position;
        let error_velocity = desired_velocity - current_velocity;

        self.integral_pos_error += error_position * dt;
        self.integral_pos_error = self
            .integral_pos_error
            .component_mul(&self.max_integral_pos.map(|x| x.signum()));
        self.integral_pos_error = self
            .integral_pos_error
            .zip_map(&self.max_integral_pos, |int, max| int.clamp(-max, max));

        let acceleration = self.kp_pos.component_mul(&error_position)
            + self.kd_pos.component_mul(&error_velocity)
            + self.ki_pos.component_mul(&self.integral_pos_error);

        let gravity_compensation = Vector3::new(0.0, 0.0, gravity);
        let total_acceleration = acceleration + gravity_compensation;

        let thrust = mass * total_acceleration.norm();
        let desired_orientation = if total_acceleration.norm() > 1e-6 {
            let z_body = total_acceleration.normalize();
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
enum PlannerType {
    /// Hover at current position
    Hover(HoverPlanner),
    /// Minimum jerk trajectory along a straight line
    MinimumJerkLine(MinimumJerkLinePlanner),
    /// Lissajous curve trajectory
    Lissajous(LissajousPlanner),
    /// Circular trajectory
    Circle(CirclePlanner),
    /// Landing trajectory
    Landing(LandingPlanner),
    /// Obstacle Avoidance Planner based on Potential field
    ObstacleAvoidance(ObstacleAvoidancePlanner),
}
impl PlannerType {
    /// Plans the trajectory based on the current planner type
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `current_velocity` - The current velocity of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// A tuple containing the desired position, velocity, and yaw angle
    fn plan(
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
        }
    }
    /// Checks if the current trajectory is finished
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// `true` if the trajectory is finished, `false` otherwise
    fn is_finished(&self, current_position: Vector3<f32>, time: f32) -> bool {
        match self {
            PlannerType::Hover(p) => p.is_finished(current_position, time),
            PlannerType::MinimumJerkLine(p) => p.is_finished(current_position, time),
            PlannerType::Lissajous(p) => p.is_finished(current_position, time),
            PlannerType::Circle(p) => p.is_finished(current_position, time),
            PlannerType::Landing(p) => p.is_finished(current_position, time),
            PlannerType::ObstacleAvoidance(p) => p.is_finished(current_position, time),
        }
    }
}

/// Trait defining the interface for trajectory planners
trait Planner {
    /// Plans the trajectory based on the current state and time
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `current_velocity` - The current velocity of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// A tuple containing the desired position, velocity, and yaw angle
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
    /// `true` if the trajectory is finished, `false` otherwise
    fn is_finished(&self, current_position: Vector3<f32>, time: f32) -> bool;
}
/// Planner for hovering at a fixed position
struct HoverPlanner {
    /// Target position for hovering
    target_position: Vector3<f32>,
    /// Target yaw angle for hovering
    target_yaw: f32,
}

impl Planner for HoverPlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        _time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        (self.target_position, Vector3::zeros(), self.target_yaw)
    }

    fn is_finished(&self, _current_position: Vector3<f32>, _time: f32) -> bool {
        true // Hover planner is always "finished" as it's the default state
    }
}
/// Planner for minimum jerk trajectories along a straight line
struct MinimumJerkLinePlanner {
    /// Starting position of the trajectory
    start_position: Vector3<f32>,
    /// Ending position of the trajectory
    end_position: Vector3<f32>,
    /// Starting yaw angle
    start_yaw: f32,
    /// Ending yaw angle
    end_yaw: f32,
    /// Start time of the trajectory
    start_time: f32,
    /// Duration of the trajectory
    duration: f32,
}

impl Planner for MinimumJerkLinePlanner {
    fn plan(
        &self,
        _current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        let t = (time - self.start_time) / self.duration;
        let t = t.clamp(0.0, 1.0);

        let s = 10.0 * t.powi(3) - 15.0 * t.powi(4) + 6.0 * t.powi(5);
        let s_dot = (30.0 * t.powi(2) - 60.0 * t.powi(3) + 30.0 * t.powi(4)) / self.duration;

        let position = self.start_position + (self.end_position - self.start_position) * s;
        let velocity = (self.end_position - self.start_position) * s_dot;
        let yaw = self.start_yaw + (self.end_yaw - self.start_yaw) * s;

        (position, velocity, yaw)
    }

    fn is_finished(&self, _current_position: Vector3<f32>, _time: f32) -> bool {
        (_current_position - self.end_position).norm() < 0.01
            && _time >= self.start_time + self.duration
    }
}
/// Planner for Lissajous curve trajectories
struct LissajousPlanner {
    /// Starting position of the trajectory
    start_position: Vector3<f32>,
    /// Center of the Lissajous curve
    center: Vector3<f32>,
    /// Amplitude of the Lissajous curve
    amplitude: Vector3<f32>,
    /// Frequency of the Lissajous curve
    frequency: Vector3<f32>,
    /// Phase of the Lissajous curve
    phase: Vector3<f32>,
    /// Start time of the trajectory
    start_time: f32,
    /// Duration of the trajectory
    duration: f32,
    /// Starting yaw angle
    start_yaw: f32,
    /// Ending yaw angle
    end_yaw: f32,
    /// Ramp-up time for smooth transitions
    ramp_time: f32,
}

impl Planner for LissajousPlanner {
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

        let lissajous = Vector3::new(
            self.amplitude.x
                * (self.frequency.x * t * 2.0 * std::f32::consts::PI + self.phase.x).sin(),
            self.amplitude.y
                * (self.frequency.y * t * 2.0 * std::f32::consts::PI + self.phase.y).sin(),
            self.amplitude.z
                * (self.frequency.z * t * 2.0 * std::f32::consts::PI + self.phase.z).sin(),
        );

        let position =
            self.start_position + smooth_start * ((self.center + lissajous) - self.start_position);

        let mut velocity = Vector3::new(
            self.amplitude.x
                * self.frequency.x
                * 2.0
                * std::f32::consts::PI
                * (self.frequency.x * t * 2.0 * std::f32::consts::PI + self.phase.x).cos(),
            self.amplitude.y
                * self.frequency.y
                * 2.0
                * std::f32::consts::PI
                * (self.frequency.y * t * 2.0 * std::f32::consts::PI + self.phase.y).cos(),
            self.amplitude.z
                * self.frequency.z
                * 2.0
                * std::f32::consts::PI
                * (self.frequency.z * t * 2.0 * std::f32::consts::PI + self.phase.z).cos(),
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

    fn is_finished(&self, _current_position: Vector3<f32>, time: f32) -> bool {
        time >= self.start_time + self.duration
    }
}
/// Planner for circular trajectories
struct CirclePlanner {
    /// Center of the circular trajectory
    center: Vector3<f32>,
    /// Radius of the circular trajectory
    radius: f32,
    /// Angular velocity of the circular motion
    angular_velocity: f32,
    /// Starting position of the trajectory
    start_position: Vector3<f32>,
    /// Start time of the trajectory
    start_time: f32,
    /// Duration of the trajectory
    duration: f32,
    /// Starting yaw angle
    start_yaw: f32,
    /// Ending yaw angle
    end_yaw: f32,
    /// Ramp-up time for smooth transitions
    ramp_time: f32,
}

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

    fn is_finished(&self, _current_position: Vector3<f32>, time: f32) -> bool {
        time >= self.start_time + self.duration
    }
}

/// Planner for landing maneuvers
struct LandingPlanner {
    /// Starting position of the landing maneuver
    start_position: Vector3<f32>,
    /// Start time of the landing maneuver
    start_time: f32,
    /// Duration of the landing maneuver
    duration: f32,
    /// Starting yaw angle
    start_yaw: f32,
}

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

    fn is_finished(&self, current_position: Vector3<f32>, time: f32) -> bool {
        current_position.z < 0.05 || time >= self.start_time + self.duration
    }
}

/// Manages different trajectory planners and switches between them
struct PlannerManager {
    /// The currently active planner
    current_planner: PlannerType,
}

impl PlannerManager {
    /// Creates a new PlannerManager with an initial hover planner
    /// # Arguments
    /// * `initial_position` - The initial position for hovering
    /// * `initial_yaw` - The initial yaw angle for hovering
    /// # Returns
    /// A new PlannerManager instance
    fn new(initial_position: Vector3<f32>, initial_yaw: f32) -> Self {
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
    fn set_planner(&mut self, new_planner: PlannerType) {
        self.current_planner = new_planner;
    }
    /// Updates the current planner and returns the desired position, velocity, and yaw
    /// # Arguments
    /// * `current_position` - The current position of the quadrotor
    /// * `current_orientation` - The current orientation of the quadrotor
    /// * `current_velocity` - The current velocity of the quadrotor
    /// * `time` - The current simulation time
    /// # Returns
    /// A tuple containing the desired position, velocity, and yaw angle
    fn update(
        &mut self,
        current_position: Vector3<f32>,
        current_orientation: UnitQuaternion<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
        obstacles: &Vec<Obstacle>,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        if self.current_planner.is_finished(current_position, time) {
            self.current_planner = PlannerType::Hover(HoverPlanner {
                target_position: current_position,
                target_yaw: current_orientation.euler_angles().2,
            });
        }
        // Update obstacles for ObstacleAvoidancePlanner if needed
        if let PlannerType::ObstacleAvoidance(ref mut planner) = self.current_planner {
            planner.obstacles = obstacles.clone();
        }
        self.current_planner
            .plan(current_position, current_velocity, time)
    }
}
/// Obstacle avoidance planner
/// This planner uses a potential field approach to avoid obstacles
/// The planner calculates a repulsive force for each obstacle and an attractive force towards the goal
/// The resulting force is then used to calculate the desired position and velocity
struct ObstacleAvoidancePlanner {
    /// Target position of the planner
    target_position: Vector3<f32>,
    /// Start time of the planner
    start_time: f32,
    /// Duration of the planner
    duration: f32,
    /// Starting yaw angle
    start_yaw: f32,
    /// Ending yaw angle
    end_yaw: f32,
    /// List of obstacles
    obstacles: Vec<Obstacle>,
    /// Attractive force gain
    k_att: f32,
    /// Repulsive force gain
    k_rep: f32,
    /// Influence distance of obstacles
    d0: f32,
}

impl Planner for ObstacleAvoidancePlanner {
    fn plan(
        &self,
        current_position: Vector3<f32>,
        _current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        let t = ((time - self.start_time) / self.duration).clamp(0.0, 1.0);

        // Attractive force towards the goal
        let f_att = self.k_att * (self.target_position - current_position);

        // Repulsive force from obstacles
        let mut f_rep = Vector3::zeros();
        for obstacle in &self.obstacles {
            let diff = current_position - obstacle.position;
            let distance = diff.norm();
            if distance < self.d0 {
                f_rep += self.k_rep
                    * (1.0 / distance - 1.0 / self.d0)
                    * (1.0 / distance.powi(2))
                    * diff.normalize();
            }
        }

        // Total force
        let f_total = f_att + f_rep;

        // Desired velocity (capped at a maximum speed)
        let max_speed: f32 = 1.0;
        let desired_velocity = f_total.normalize() * max_speed.min(f_total.norm());

        // Desired position (current position + desired velocity)
        let desired_position = current_position + desired_velocity * self.duration * (1.0 - t);

        // Interpolate yaw
        let desired_yaw = self.start_yaw + (self.end_yaw - self.start_yaw) * t;

        (desired_position, desired_velocity, desired_yaw)
    }

    fn is_finished(&self, current_position: Vector3<f32>, time: f32) -> bool {
        (current_position - self.target_position).norm() < 0.1
            && time >= self.start_time + self.duration
    }
}

/// Updates the planner based on the current simulation step
/// # Arguments
/// * `planner_manager` - The PlannerManager instance to update
/// * `step` - The current simulation step
/// * `time` - The current simulation time
/// * `quad` - The Quadrotor instance
fn update_planner(
    planner_manager: &mut PlannerManager,
    step: usize,
    time: f32,
    quad: &Quadrotor,
    obstacles: &Vec<Obstacle>,
) {
    match step {
        500 => planner_manager.set_planner(PlannerType::MinimumJerkLine(MinimumJerkLinePlanner {
            start_position: quad.position,
            end_position: Vector3::new(0.0, 0.0, 1.0),
            start_yaw: quad.orientation.euler_angles().2,
            end_yaw: 0.0,
            start_time: time,
            duration: 3.0,
        })),
        1000 => planner_manager.set_planner(PlannerType::MinimumJerkLine(MinimumJerkLinePlanner {
            start_position: quad.position,
            end_position: Vector3::new(1.0, 0.0, 1.0),
            start_yaw: quad.orientation.euler_angles().2,
            end_yaw: 0.0,
            start_time: time,
            duration: 3.0,
        })),
        1500 => planner_manager.set_planner(PlannerType::MinimumJerkLine(MinimumJerkLinePlanner {
            start_position: quad.position,
            end_position: Vector3::new(1.0, 1.0, 1.0),
            start_yaw: quad.orientation.euler_angles().2,
            end_yaw: std::f32::consts::PI / 2.0,
            start_time: time,
            duration: 3.0,
        })),
        2000 => planner_manager.set_planner(PlannerType::MinimumJerkLine(MinimumJerkLinePlanner {
            start_position: quad.position,
            end_position: Vector3::new(0.0, 1.0, 1.0),
            start_yaw: quad.orientation.euler_angles().2,
            end_yaw: std::f32::consts::PI / 2.0,
            start_time: time,
            duration: 3.0,
        })),
        2500 => planner_manager.set_planner(PlannerType::MinimumJerkLine(MinimumJerkLinePlanner {
            start_position: quad.position,
            end_position: Vector3::new(0.0, 0.0, 0.5),
            start_yaw: quad.orientation.euler_angles().2,
            end_yaw: 0.0,
            start_time: time,
            duration: 3.0,
        })),
        3000 => planner_manager.set_planner(PlannerType::Lissajous(LissajousPlanner {
            start_position: quad.position,
            center: Vector3::new(0.5, 0.5, 1.0),
            amplitude: Vector3::new(0.5, 0.5, 0.2),
            frequency: Vector3::new(1.0, 2.0, 3.0),
            phase: Vector3::new(0.0, std::f32::consts::PI / 2.0, 0.0),
            start_time: time,
            duration: 20.0,
            start_yaw: quad.orientation.euler_angles().2,
            end_yaw: quad.orientation.euler_angles().2 + 2.0 * std::f32::consts::PI,
            ramp_time: 5.0,
        })),
        5200 => planner_manager.set_planner(PlannerType::Circle(CirclePlanner {
            center: Vector3::new(0.5, 0.5, 1.0),
            radius: 0.5,
            angular_velocity: 1.0,
            start_position: quad.position,
            start_time: time,
            duration: 8.0,
            start_yaw: quad.orientation.euler_angles().2,
            end_yaw: quad.orientation.euler_angles().2,
            ramp_time: 2.0,
        })),
        6200 => planner_manager.set_planner(PlannerType::MinimumJerkLine(MinimumJerkLinePlanner {
            start_position: quad.position,
            end_position: Vector3::new(quad.position.x, quad.position.y, 0.5),
            start_yaw: quad.orientation.euler_angles().2,
            end_yaw: 0.0,
            start_time: time,
            duration: 5.0,
        })),
        7000 => {
            planner_manager.set_planner(PlannerType::ObstacleAvoidance(ObstacleAvoidancePlanner {
                target_position: Vector3::new(1.5, 1.0, 1.0),
                start_time: time,
                duration: 15.0,
                start_yaw: quad.orientation.euler_angles().2,
                end_yaw: 0.0,
                obstacles: obstacles.clone(),
                k_att: 0.03,
                k_rep: 0.02,
                d0: 0.5,
            }))
        }
        8500 => planner_manager.set_planner(PlannerType::Landing(LandingPlanner {
            start_position: quad.position,
            start_time: time,
            duration: 5.0,
            start_yaw: quad.orientation.euler_angles().2,
        })),
        _ => {}
    }
}
/// Represents an obstacle in the simulation
/// # Fields
/// * `position` - The position of the obstacle
/// * `velocity` - The velocity of the obstacle
/// * `radius` - The radius of the obstacle
#[derive(Clone)]
struct Obstacle {
    position: Vector3<f32>,
    velocity: Vector3<f32>,
    radius: f32,
}

impl Obstacle {
    fn new(position: Vector3<f32>, velocity: Vector3<f32>, radius: f32) -> Self {
        Self {
            position,
            velocity,
            radius,
        }
    }
}
/// Represents a maze in the simulation
/// The maze is represented as a set of obstacles
/// that the quadrotor must avoid
/// # Fields
/// * `lower_bounds` - The lower bounds of the maze
/// * `upper_bounds` - The upper bounds of the maze
/// * `obstacles` - The obstacles in the maze

struct Maze {
    lower_bounds: Vector3<f32>,
    upper_bounds: Vector3<f32>,
    obstacles: Vec<Obstacle>,
}
impl Maze {
    /// Creates a new maze with the given bounds and number of obstacles
    /// The obstacles are randomly generated within the bounds
    /// # Arguments
    /// * `lower_bounds` - The lower bounds of the maze
    /// * `upper_bounds` - The upper bounds of the maze
    /// * `num_obstacles` - The number of obstacles in the maze
    fn new(lower_bounds: Vector3<f32>, upper_bounds: Vector3<f32>, num_obstacles: usize) -> Self {
        let mut maze = Maze {
            lower_bounds,
            upper_bounds,
            obstacles: Vec::new(),
        };
        maze.generate_obstacles(num_obstacles);
        maze
    }
    /// Generates the obstacles in the maze
    /// # Arguments
    /// * `num_obstacles` - The number of obstacles to generate
    fn generate_obstacles(&mut self, num_obstacles: usize) {
        let mut rng = rand::thread_rng();
        for _ in 0..num_obstacles {
            let position = Vector3::new(
                rand::Rng::gen_range(&mut rng, self.lower_bounds.x..self.upper_bounds.x),
                rand::Rng::gen_range(&mut rng, self.lower_bounds.y..self.upper_bounds.y),
                rand::Rng::gen_range(&mut rng, self.lower_bounds.z..self.upper_bounds.z),
            );
            let velocity = Vector3::new(
                rand::Rng::gen_range(&mut rng, -0.2..0.2),
                rand::Rng::gen_range(&mut rng, -0.2..0.2),
                rand::Rng::gen_range(&mut rng, -0.1..0.1),
            );
            let radius = rand::Rng::gen_range(&mut rng, 0.05..0.1);
            self.obstacles
                .push(Obstacle::new(position, velocity, radius));
        }
    }
    /// Updates the obstacles in the maze
    /// The obstacles are moved according to their velocities
    /// If an obstacle hits a boundary, it bounces off
    /// # Arguments
    /// * `dt` - The time step
    fn update_obstacles(&mut self, dt: f32) {
        for obstacle in self.obstacles.iter_mut() {
            // Update position
            obstacle.position += obstacle.velocity * dt;
            // Bounce off boundaries
            for i in 0..3 {
                if obstacle.position[i] - obstacle.radius < self.lower_bounds[i]
                    || obstacle.position[i] + obstacle.radius > self.upper_bounds[i]
                {
                    obstacle.velocity[i] = -obstacle.velocity[i];
                }
            }
        }
    }
}

/// Represents a camera in the simulation
/// The camera is used to render the depth of the scene
/// from the perspective of the quadrotor
/// The depth is used by the obstacle avoidance planner
/// # Fields
/// * `resolution` - The resolution of the camera
/// * `fov` - The field of view of the camera
/// * `near` - The near clipping plane of the camera
/// * `far` - The far clipping plane of the camera
/// * `tan_half_fov` - The tangent of half the field of view
/// * `aspect_ratio` - The aspect ratio of the camera
struct Camera {
    resolution: (usize, usize),
    fov: f32,
    near: f32,
    far: f32,
    tan_half_fov: f32,
    aspect_ratio: f32,
}

impl Camera {
    /// Creates a new camera with the given resolution, field of view, near and far clipping planes
    /// # Arguments
    /// * `resolution` - The resolution of the camera
    /// * `fov` - The field of view of the camera
    /// * `near` - The near clipping plane of the camera
    /// * `far` - The far clipping plane of the camera
    fn new(resolution: (usize, usize), fov: f32, near: f32, far: f32) -> Self {
        Camera {
            resolution,
            fov,
            near,
            far,
            tan_half_fov: (fov / 2.0).tan(),
            aspect_ratio: resolution.0 as f32 / resolution.1 as f32,
        }
    }
    /// Renders the depth of the scene from the perspective of the quadrotor
    /// # Arguments
    /// * `quad_position` - The position of the quadrotor
    /// * `quad_orientation` - The orientation of the quadrotor
    /// * `maze` - The maze in the scene
    /// # Returns
    /// A vector of depth values representing the depth of the scene
    fn render_depth(
        &self,
        quad_position: &Vector3<f32>,
        quad_orientation: &UnitQuaternion<f32>,
        maze: &Maze,
    ) -> Vec<f32> {
        let (width, height) = self.resolution;
        let tan_half_fov = (self.fov / 2.0).tan();
        let camera_to_world = quad_orientation.to_rotation_matrix().matrix()
            * Matrix3::new(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
        let mut depth_image = vec![self.far as f32; width * height];
        for (i, depth) in depth_image.iter_mut().enumerate() {
            let x = (2.0 * (i % width) as f32 / width as f32 - 1.0)
                * self.aspect_ratio
                * self.tan_half_fov;
            let y = (1.0 - 2.0 * (i / width) as f32 / height as f32) * tan_half_fov;
            let ray_dir = camera_to_world * Vector3::new(1.0, x, y).normalize();
            if let Some(hit_distance) = self.ray_cast(quad_position, &ray_dir, maze) {
                *depth = hit_distance as f32;
            }
        }
        depth_image
    }
    /// Casts a ray from the camera origin in the given direction
    /// # Arguments
    /// * `origin` - The origin of the ray
    /// * `direction` - The direction of the ray
    /// * `maze` - The maze in the scene
    /// # Returns
    /// The distance to the closest obstacle hit by the ray
    fn ray_cast(
        &self,
        origin: &Vector3<f32>,
        direction: &Vector3<f32>,
        maze: &Maze,
    ) -> Option<f32> {
        let mut closest_hit = self.far;

        // Check obstacle intersections
        for obstacle in &maze.obstacles {
            if let Some(obstacle_hit) =
                self.ray_sphere_intersection(origin, direction, &obstacle.position, obstacle.radius)
            {
                if obstacle_hit < closest_hit {
                    closest_hit = obstacle_hit;
                }
            }
        }

        if closest_hit < self.far {
            Some(closest_hit)
        } else {
            None
        }
    }
    /// Checks if a ray intersects a sphere
    /// The ray is defined by an origin and a direction
    /// The sphere is defined by a center and a radius
    /// The intersection is computed using the quadratic formula
    /// If the discriminant is negative, the ray does not intersect the sphere
    /// If the intersection is outside the near and far clipping planes, it is ignored
    /// # Arguments
    /// * `origin` - The origin of the ray
    /// * `direction` - The direction of the ray
    /// * `center` - The center of the sphere
    /// * `radius` - The radius of the sphere
    /// # Returns
    /// The distance to the intersection point if it exists and is within the clipping planes
    fn ray_sphere_intersection(
        &self,
        origin: &Vector3<f32>,
        direction: &Vector3<f32>,
        center: &Vector3<f32>,
        radius: f32,
    ) -> Option<f32> {
        let oc = origin - center;
        let a = direction.dot(direction);
        let b = 2.0 * oc.dot(direction);
        let c = oc.dot(&oc) - radius * radius;
        let discriminant = b * b - 4.0 * a * c;

        if discriminant < 0.0 {
            None
        } else {
            let t = (-b - discriminant.sqrt()) / (2.0 * a);
            if t > self.near && t < self.far {
                Some(t)
            } else {
                None
            }
        }
    }
}
/// Logs simulation data to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `quad` - The Quadrotor instance
/// * `desired_position` - The desired position vector
/// * `measured_accel` - The measured acceleration vector
/// * `measured_gyro` - The measured angular velocity vector
fn log_data(
    rec: &rerun::RecordingStream,
    quad: &Quadrotor,
    desired_position: &Vector3<f32>,
    measured_accel: &Vector3<f32>,
    measured_gyro: &Vector3<f32>,
) {
    rec.log(
        "quadrotor/desired_position",
        &rerun::Points3D::new([(desired_position.x, desired_position.y, desired_position.z)])
            .with_radii([0.1])
            .with_colors([rerun::Color::from_rgb(255, 255, 255)]),
    )
    .unwrap();
    rec.log(
        "quadrotor/current_position",
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
    )
    .unwrap();
    let (quad_roll, quad_pitch, quad_yaw) = quad.orientation.euler_angles();

    for (name, value) in [
        ("position/x", quad.position.x),
        ("position/y", quad.position.y),
        ("position/z", quad.position.z),
        ("velocity/x", quad.velocity.x),
        ("velocity/y", quad.velocity.y),
        ("velocity/z", quad.velocity.z),
        ("accel/x", measured_accel.x),
        ("accel/y", measured_accel.y),
        ("accel/z", measured_accel.z),
        ("orientation/roll", quad_roll),
        ("orientation/pitch", quad_pitch),
        ("orientation/yaw", quad_yaw),
        ("gyro/x", measured_gyro.x),
        ("gyro/y", measured_gyro.y),
        ("gyro/z", measured_gyro.z),
    ] {
        rec.log(name, &rerun::Scalar::new(value as f64)).unwrap();
    }
}
/// Log the maze tube to the rerun recording stream
/// The tube is defined by the lower and upper bounds of the maze
/// The tube is visualized as a wireframe cube
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `maze` - The maze instance
fn log_maze_tube(rec: &rerun::RecordingStream, maze: &Maze) {
    let (lower_bounds, upper_bounds) = (maze.lower_bounds, maze.upper_bounds);
    // Define the corners of the tube
    let corners = [
        Vector3::new(lower_bounds.x, lower_bounds.y, lower_bounds.z),
        Vector3::new(upper_bounds.x, lower_bounds.y, lower_bounds.z),
        Vector3::new(upper_bounds.x, upper_bounds.y, lower_bounds.z),
        Vector3::new(lower_bounds.x, upper_bounds.y, lower_bounds.z),
        Vector3::new(lower_bounds.x, lower_bounds.y, upper_bounds.z),
        Vector3::new(upper_bounds.x, lower_bounds.y, upper_bounds.z),
        Vector3::new(upper_bounds.x, upper_bounds.y, upper_bounds.z),
        Vector3::new(lower_bounds.x, upper_bounds.y, upper_bounds.z),
    ];

    // Define the edges of the tube
    let edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];

    // Create line strips for the edges
    let line_strips: Vec<Vec<rerun::external::glam::Vec3>> = edges
        .iter()
        .map(|&(start, end)| {
            vec![
                rerun::external::glam::Vec3::new(
                    corners[start].x,
                    corners[start].y,
                    corners[start].z,
                ),
                rerun::external::glam::Vec3::new(corners[end].x, corners[end].y, corners[end].z),
            ]
        })
        .collect();
    let line_radius = 0.05;
    rec.log(
        "maze/tube",
        &rerun::LineStrips3D::new(line_strips)
            .with_colors([rerun::Color::from_rgb(128, 128, 255)])
            .with_radii([line_radius]),
    )
    .unwrap();
}

/// Log the maze obstacles to the rerun recording stream
/// The obstacles are visualized as spheres
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `maze` - The maze instance
fn log_maze_obstacles(rec: &rerun::RecordingStream, maze: &Maze) {
    let positions: Vec<(f32, f32, f32)> = maze
        .obstacles
        .iter()
        .map(|obstacle| {
            (
                obstacle.position.x,
                obstacle.position.y,
                obstacle.position.z,
            )
        })
        .collect();

    let radii: Vec<f32> = maze
        .obstacles
        .iter()
        .map(|obstacle| obstacle.radius)
        .collect();

    rec.log(
        "maze/obstacles",
        &rerun::Points3D::new(positions)
            .with_radii(radii)
            .with_colors([rerun::Color::from_rgb(255, 128, 128)]),
    )
    .unwrap();
}
/// A struct to hold trajectory data
/// # Fields
/// * `points` - A vector of 3D points
/// * `last_logged_point` - The last point that was logged
/// * `min_distance_threadhold` - The minimum distance between points to log
struct Trajectory {
    points: Vec<Vector3<f32>>,
    last_logged_point: Vector3<f32>,
    min_distance_threadhold: f32,
}

impl Trajectory {
    fn new(initial_point: Vector3<f32>) -> Self {
        Self {
            points: vec![initial_point],
            last_logged_point: initial_point,
            min_distance_threadhold: 0.1,
        }
    }
    /// Add a point to the trajectory
    /// The point is only added if it is further than the minimum distance threshold
    /// # Arguments
    /// * `point` - The point to add
    fn add_point(&mut self, point: Vector3<f32>) {
        if (point - self.last_logged_point).norm() > self.min_distance_threadhold {
            self.points.push(point);
            self.last_logged_point = point;
        }
    }
}

/// log trajectory data to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `trajectory` - The Trajectory instance
fn log_trajectory(rec: &rerun::RecordingStream, trajectory: &Trajectory) {
    let path = trajectory
        .points
        .iter()
        .map(|p| (p.x, p.y, p.z))
        .collect::<Vec<_>>();
    rec.log(
        "quadrotor/path",
        &rerun::LineStrips3D::new([path]).with_colors([rerun::Color::from_rgb(0, 255, 255)]),
    )
    .unwrap();
}
/// log mesh data to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `division` - The number of divisions in the mesh
/// * `spacing` - The spacing between divisions
fn log_mesh(rec: &rerun::RecordingStream, division: usize, spacing: f32) {
    let grid_size: usize = division + 1;
    let half_grid_size: f32 = (division as f32 * spacing) / 2.0;
    let points: Vec<rerun::external::glam::Vec3> = (0..grid_size)
        .flat_map(|i| {
            (0..grid_size).map(move |j| {
                rerun::external::glam::Vec3::new(
                    j as f32 * spacing - half_grid_size,
                    i as f32 * spacing - half_grid_size,
                    0.0,
                )
            })
        })
        .collect();
    let horizontal_lines: Vec<Vec<rerun::external::glam::Vec3>> = (0..grid_size)
        .map(|i| points[i * grid_size..(i + 1) * grid_size].to_vec())
        .collect();
    let vertical_lines: Vec<Vec<rerun::external::glam::Vec3>> = (0..grid_size)
        .map(|j| (0..grid_size).map(|i| points[i * grid_size + j]).collect())
        .collect();
    let line_strips: Vec<Vec<rerun::external::glam::Vec3>> =
        horizontal_lines.into_iter().chain(vertical_lines).collect();
    rec.log(
        "maze/mesh",
        &rerun::LineStrips3D::new(line_strips)
            .with_colors([rerun::Color::from_rgb(255, 255, 255)])
            .with_radii([0.02]),
    )
    .unwrap();
}
/// log depth image data to the rerun recording stream
/// # Arguments
/// * `rec` - The rerun::RecordingStream instance
/// * `depth_image` - The depth image data
/// * `width` - The width of the depth image
/// * `height` - The height of the depth image
/// * `min_depth` - The minimum depth value
/// * `max_depth` - The maximum depth value
fn log_depth_image(
    rec: &rerun::RecordingStream,
    depth_image: &Vec<f32>,
    width: usize,
    height: usize,
    min_depth: f32,
    max_depth: f32,
) {
    let mut depth_image_buffer = image::GrayImage::new(width as u32, height as u32);
    for x in 0..width {
        for y in 0..height {
            let depth = depth_image[y * width + x];
            let pixel = if depth.is_infinite() {
                image::Luma([0u8])
            } else {
                let normalized_depth = (depth - min_depth) / (max_depth - min_depth);
                image::Luma([(normalized_depth * 255.0) as u8])
            };
            depth_image_buffer.put_pixel(x as u32, y as u32, pixel);
        }
    }
    let depth_image_rerun = rerun::DepthImage::try_from(depth_image_buffer).unwrap();
    rec.log("depth", &depth_image_rerun).unwrap();
}

/// Main function to run the quadrotor simulation
fn main() {
    let mut quad = Quadrotor::new();
    let mut controller = PIDController::new();
    let mut imu = Imu::new();
    let rec = rerun::RecordingStreamBuilder::new("Peng")
        .connect()
        .unwrap();

    let upper_bounds = Vector3::new(3.0, 2.0, 2.0);
    let lower_bounds = Vector3::new(-3.0, -2.0, 0.0);
    let mut maze = Maze::new(lower_bounds, upper_bounds, 20);
    let camera = Camera::new((128, 96), 90.0_f32.to_radians(), 0.1, 5.0);
    let mut planner_manager = PlannerManager::new(Vector3::zeros(), 0.0);
    let mut trajectory = Trajectory::new(Vector3::new(0.0, 0.0, 0.0));
    rec.set_time_seconds("timestamp", 0);
    log_mesh(&rec, 5, 0.5);
    log_maze_tube(&rec, &maze);
    log_maze_obstacles(&rec, &maze);
    let mut i = 0;
    loop {
        let time = quad.time_step * i as f32;
        rec.set_time_seconds("timestamp", time);
        maze.update_obstacles(quad.time_step);
        update_planner(&mut planner_manager, i, time, &quad, &maze.obstacles);
        let (desired_position, desired_velocity, desired_yaw) = planner_manager.update(
            quad.position,
            quad.orientation,
            quad.velocity,
            time,
            &maze.obstacles,
        );
        let (thrust, calculated_desired_orientation) = controller.compute_position_control(
            desired_position,
            desired_velocity,
            desired_yaw,
            quad.position,
            quad.velocity,
            quad.time_step,
            quad.mass,
            quad.gravity,
        );
        let torque = controller.compute_attitude_control(
            &calculated_desired_orientation,
            &quad.orientation,
            &quad.angular_velocity,
            quad.time_step,
        );
        quad.update_dynamics_with_controls(thrust, &torque);
        imu.update(quad.time_step);
        let (true_accel, true_gyro) = quad.read_imu();
        let (_measured_accel, _measured_gyro) = imu.read(true_accel, true_gyro);
        if i % 5 == 0 {
            trajectory.add_point(quad.position);
            log_trajectory(&rec, &trajectory);
            log_data(
                &rec,
                &quad,
                &desired_position,
                &_measured_accel,
                &_measured_gyro,
            );
            let depth_image = camera.render_depth(&quad.position, &quad.orientation, &maze);
            log_depth_image(
                &rec,
                &depth_image,
                camera.resolution.0,
                camera.resolution.1,
                camera.near,
                camera.far,
            );
            log_maze_obstacles(&rec, &maze);
        }
        i += 1;
        if i >= 9000 {
            break;
        }
    }
}
