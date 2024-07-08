extern crate nalgebra as na;
extern crate rand;

use na::{Vector3, Quaternion, UnitQuaternion, Matrix3, Rotation3};
use rand_distr::{Distribution, Normal};

struct Quadrotor {
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    orientation: Quaternion<f64>,
    angular_velocity: Vector3<f64>,
    mass: f64,
    gravity: f64,
    time_step: f64,
    thrust_coefficient: f64,
    drag_coefficient: f64,
}

impl Quadrotor {
    pub fn new() -> Self {
        Self {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: Quaternion::identity(),
            angular_velocity: Vector3::zeros(),
            mass: 1.0, // Example mass
            gravity: 9.81,
            time_step: 0.01, // Example timestep in seconds
            thrust_coefficient: 1.0,
            drag_coefficient: 0.001,
        }
    }

    pub fn update_dynamics(&mut self) {
        // Update position and velocity based on current state and simple physics
        let acceleration = Vector3::new(0.0, 0.0, -self.gravity);
        self.velocity += acceleration * self.time_step;
        self.position += self.velocity * self.time_step;

        // Update orientation using exponential map
        let half_dt = 0.5 * self.time_step;
        let omega = self.angular_velocity * half_dt;
        let norm_omega = omega.norm();

        if norm_omega > 0.0 {
            let sin_half_omega = norm_omega.sin();
            let cos_half_omega = norm_omega.cos();
            let delta_q = Quaternion::new(
                cos_half_omega,
                sin_half_omega * omega.x / norm_omega,
                sin_half_omega * omega.y / norm_omega,
                sin_half_omega * omega.z / norm_omega
            );
            self.orientation = (self.orientation * delta_q).normalize();  // Normalize to ensure it remains a unit quaternion
        }
    }

    pub fn update_dynamics_with_controls(&mut self, control_thrust: Vector3<f64>, control_torque: Vector3<f64>) {
        let gravity_force = Vector3::new(0.0, 0.0, self.mass * self.gravity);
        let drag_force = -self.drag_coefficient * self.velocity.magnitude() * self.velocity;
        let total_force = control_thrust + gravity_force + drag_force;

        // Updating acceleration including the thrust and drag
        let acceleration = total_force / self.mass;
        self.velocity += acceleration * self.time_step;
        self.position += self.velocity * self.time_step;

        // Torque dynamics
        let inertia_matrix = Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0); // Simplified
        let inertia_inv = inertia_matrix.try_inverse().unwrap();

        // Calculate the product of inertia matrix and angular velocity
        let inertia_angular_velocity = inertia_matrix * self.angular_velocity;
        
        // Correct usage of cross product
        let gyroscopic_torque = self.angular_velocity.cross(&inertia_angular_velocity);

        // Calculate angular acceleration
        let angular_acceleration = inertia_inv * (control_torque - gyroscopic_torque);

        self.angular_velocity += angular_acceleration * self.time_step;
        // Update orientation using exponential map
        let half_dt = 0.5 * self.time_step;
        let omega = self.angular_velocity * half_dt;
        let norm_omega = omega.norm();

        if norm_omega > 0.0 {
            let sin_half_omega = norm_omega.sin();
            let cos_half_omega = norm_omega.cos();
            let delta_q = Quaternion::new(
                cos_half_omega,
                sin_half_omega * omega.x / norm_omega,
                sin_half_omega * omega.y / norm_omega,
                sin_half_omega * omega.z / norm_omega
            );
            self.orientation = (self.orientation * delta_q).normalize();  // Normalize to ensure it remains a unit quaternion
        }

    }

    pub fn read_imu(&self) -> (Vector3<f64>, Vector3<f64>) {
        let accel_noise = Normal::new(0.0, 0.02).unwrap();
        let gyro_noise = Normal::new(0.0, 0.01).unwrap();

        let measured_acceleration = self.velocity / self.time_step + Vector3::new(
            accel_noise.sample(&mut rand::thread_rng()),
            accel_noise.sample(&mut rand::thread_rng()),
            accel_noise.sample(&mut rand::thread_rng()) - self.gravity,
        );

        let measured_angular_velocity = self.angular_velocity + Vector3::new(
            gyro_noise.sample(&mut rand::thread_rng()),
            gyro_noise.sample(&mut rand::thread_rng()),
            gyro_noise.sample(&mut rand::thread_rng()),
        );

        (measured_acceleration, measured_angular_velocity)
    }
}

struct IMU {
    accel_bias: Vector3<f64>,
    gyro_bias: Vector3<f64>,
}

impl IMU {
    pub fn new() -> Self {
        Self {
            accel_bias: Vector3::new(0.0, 0.0, 0.0),
            gyro_bias: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    pub fn update(&mut self) {
        // Simulate bias drift
        let bias_drift = Normal::new(0.0, 0.0001).unwrap();
        self.accel_bias += Vector3::new(bias_drift.sample(&mut rand::thread_rng()), bias_drift.sample(&mut rand::thread_rng()), bias_drift.sample(&mut rand::thread_rng()));
        self.gyro_bias += Vector3::new(bias_drift.sample(&mut rand::thread_rng()), bias_drift.sample(&mut rand::thread_rng()), bias_drift.sample(&mut rand::thread_rng()));
    }

    pub fn read(&self, true_acceleration: Vector3<f64>, true_angular_velocity: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
        let accel_noise = Normal::new(0.0, 0.02).unwrap();
        let gyro_noise = Normal::new(0.0, 0.01).unwrap();

        let measured_acceleration = true_acceleration + self.accel_bias + Vector3::new(
            accel_noise.sample(&mut rand::thread_rng()),
            accel_noise.sample(&mut rand::thread_rng()),
            accel_noise.sample(&mut rand::thread_rng()),
        );

        let measured_angular_velocity = true_angular_velocity + self.gyro_bias + Vector3::new(
            gyro_noise.sample(&mut rand::thread_rng()),
            gyro_noise.sample(&mut rand::thread_rng()),
            gyro_noise.sample(&mut rand::thread_rng()),
        );

        (measured_acceleration, measured_angular_velocity)
    }
}

struct Controller {
    kp_pos: Vector3<f64>,  // Proportional gains for position
    kd_pos: Vector3<f64>,  // Derivative gains for position
    kp_att: Vector3<f64>,  // Proportional gains for attitude
    kd_att: Vector3<f64>,  // Derivative gains for attitude
}

impl Controller {
    fn new() -> Self {
        Self {
            kp_pos: Vector3::new(1.0, 1.0, 1.0),  // Example gains
            kd_pos: Vector3::new(0.1, 0.1, 0.1),
            kp_att: Vector3::new(1.0, 1.0, 1.0),
            kd_att: Vector3::new(0.1, 0.1, 0.1),
        }
    }

    // Compute control torques based on desired and current orientation
    fn compute_attitude_control(&self, desired_orientation: Quaternion<f64>, current_orientation: Quaternion<f64>, current_angular_velocity: Vector3<f64>) -> Vector3<f64> {
        let error_orientation = UnitQuaternion::new_normalize(current_orientation.try_inverse().unwrap() * desired_orientation);
        
        // Convert the orientation error to Euler angles
        let (roll_error, pitch_error, yaw_error) = error_orientation.euler_angles();
        let error_angles = Vector3::new(roll_error, pitch_error, yaw_error);
        // Target angular velocity is zero, hence the error is the negative of current angular velocity
        let error_angular_velocity = -current_angular_velocity;
        self.kp_att.component_mul(&error_angles) + self.kd_att.component_mul(&error_angular_velocity)
    }

    // Compute thrust and desired orientation to reach a target position
    fn compute_position_control(&self, desired_position: Vector3<f64>, current_position: Vector3<f64>, current_velocity: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
        let error_position = desired_position - current_position;
        let error_velocity = -current_velocity;  // Target velocity is zero
        let control_input = self.kp_pos.component_mul(&error_position) + self.kd_pos.component_mul(&error_velocity);

        let desired_orientation = Vector3::new(0.0, 0.0, 0.0);
        (control_input, desired_orientation)
    }
}

fn main() {
    let mut quad = Quadrotor::new();
    let controller = Controller::new();
    let desired_position = Vector3::new(5.0, 5.0, 10.0);
    let desired_orientation = Quaternion::new(1.0, 0.0, 0.0, 0.0);

    for _ in 0..10000000 {
        quad.update_dynamics();
        let (control_thrust, _) = controller.compute_position_control(desired_position, quad.position, quad.velocity);
        let control_torque = controller.compute_attitude_control(desired_orientation, quad.orientation, quad.angular_velocity);
        quad.update_dynamics_with_controls(control_thrust, control_torque);
        println!("Position: {:?}, Orientation: {:?}", quad.position, quad.orientation);
        let (accel, gyro) = quad.read_imu();
        println!("IMU Acceleration: {:?}, Gyro: {:?}", accel, gyro);
    }

}
