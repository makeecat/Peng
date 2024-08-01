extern crate nalgebra as na;
extern crate rand;
extern crate rerun;

use na::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use rand_distr::{Distribution, Normal};
struct Quadrotor {
    position: Vector3<f32>,
    velocity: Vector3<f32>,
    orientation: UnitQuaternion<f32>,
    angular_velocity: Vector3<f32>,
    mass: f32,
    gravity: f32,
    time_step: f32,
    // thrust_coefficient: f32,
    drag_coefficient: f32,
    inertia_matrix: Matrix3<f32>,
    inertia_matrix_inv: Matrix3<f32>,
}

impl Quadrotor {
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

    pub fn update_dynamics_with_controls(
        &mut self,
        control_thrust: f32,
        control_torque: Vector3<f32>,
    ) {
        let gravity_force = Vector3::new(0.0, 0.0, -self.mass * self.gravity);
        let drag_force = -self.drag_coefficient * self.velocity.norm() * self.velocity;
        let thrust_body = Vector3::new(0.0, 0.0, control_thrust);
        let thrust_world = self.orientation * thrust_body;
        let total_force = thrust_world + gravity_force + drag_force;

        let acceleration = total_force / self.mass;
        self.velocity += acceleration * self.time_step;
        self.position += self.velocity * self.time_step;

        let inertia_inv = self.inertia_matrix_inv;
        let inertia_angular_velocity = self.inertia_matrix * self.angular_velocity;
        let gyroscopic_torque = self.angular_velocity.cross(&inertia_angular_velocity);
        let angular_acceleration = inertia_inv * (control_torque - gyroscopic_torque);

        self.angular_velocity += angular_acceleration * self.time_step;
        self.orientation *=
            UnitQuaternion::from_scaled_axis(self.angular_velocity * self.time_step);
    }

    pub fn read_imu(&self) -> (Vector3<f32>, Vector3<f32>) {
        let accel_noise = Normal::new(0.0, 0.02).unwrap();
        let gyro_noise = Normal::new(0.0, 0.01).unwrap();

        let gravity_world = Vector3::new(0.0, 0.0, self.gravity);
        let specific_force =
            self.orientation.inverse() * (self.velocity / self.time_step - gravity_world);

        let measured_acceleration = specific_force
            + Vector3::new(
                accel_noise.sample(&mut rand::thread_rng()),
                accel_noise.sample(&mut rand::thread_rng()),
                accel_noise.sample(&mut rand::thread_rng()),
            );
        let measured_angular_velocity = self.angular_velocity
            + Vector3::new(
                gyro_noise.sample(&mut rand::thread_rng()),
                gyro_noise.sample(&mut rand::thread_rng()),
                gyro_noise.sample(&mut rand::thread_rng()),
            );
        (measured_acceleration, measured_angular_velocity)
    }
}

struct IMU {
    accel_bias: Vector3<f32>,
    gyro_bias: Vector3<f32>,
    accel_noise_std: f32,
    gyro_noise_std: f32,
    bias_instability: f32,
}

impl IMU {
    pub fn new() -> Self {
        Self {
            accel_bias: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            accel_noise_std: 0.02,
            gyro_noise_std: 0.01,
            bias_instability: 0.0001,
        }
    }

    pub fn update(&mut self, dt: f32) {
        let bias_drift = Normal::new(0.0, self.bias_instability * dt.sqrt()).unwrap();
        self.accel_bias += Vector3::new(
            bias_drift.sample(&mut rand::thread_rng()),
            bias_drift.sample(&mut rand::thread_rng()),
            bias_drift.sample(&mut rand::thread_rng()),
        );
        self.gyro_bias += Vector3::new(
            bias_drift.sample(&mut rand::thread_rng()),
            bias_drift.sample(&mut rand::thread_rng()),
            bias_drift.sample(&mut rand::thread_rng()),
        );
    }

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

struct PIDController {
    kp_pos: Vector3<f32>,
    kd_pos: Vector3<f32>,
    kp_att: Vector3<f32>,
    kd_att: Vector3<f32>,
    ki_pos: Vector3<f32>,
    ki_att: Vector3<f32>,
    integral_pos_error: Vector3<f32>,
    integral_att_error: Vector3<f32>,
    max_integral_pos: Vector3<f32>,
    max_integral_att: Vector3<f32>,
}

impl PIDController {
    fn new() -> Self {
        Self {
            kp_pos: Vector3::new(7.1, 7.1, 11.9),
            kd_pos: Vector3::new(2.4, 2.4, 6.7),
            kp_att: Vector3::new(1.5, 1.5, 1.0),
            kd_att: Vector3::new(0.13, 0.13, 0.1),
            ki_pos: Vector3::new(0.00, 0.00, 0.01),
            ki_att: Vector3::new(0.00, 0.00, 0.01),
            integral_pos_error: Vector3::zeros(),
            integral_att_error: Vector3::zeros(),
            max_integral_pos: Vector3::new(10.0, 10.0, 10.0),
            max_integral_att: Vector3::new(1.0, 1.0, 1.0),
        }
    }

    fn compute_attitude_control(
        &mut self,
        desired_orientation: UnitQuaternion<f32>,
        current_orientation: UnitQuaternion<f32>,
        current_angular_velocity: Vector3<f32>,
        dt: f32,
    ) -> Vector3<f32> {
        let error_orientation = current_orientation.inverse() * desired_orientation;
        let (roll_error, pitch_error, yaw_error) = error_orientation.euler_angles();
        let error_angles = Vector3::new(roll_error, pitch_error, yaw_error);

        self.integral_att_error += error_angles * dt;
        self.integral_att_error = self
            .integral_att_error
            .component_mul(&self.max_integral_att.map(|x| x.signum()));
        for i in 0..3 {
            if self.integral_att_error[i].abs() > self.max_integral_att[i] {
                self.integral_att_error[i] =
                    self.integral_att_error[i].signum() * self.max_integral_att[i];
            }
        }
        let error_angular_velocity = -current_angular_velocity;

        self.kp_att.component_mul(&error_angles)
            + self.kd_att.component_mul(&error_angular_velocity)
            + self.ki_att.component_mul(&self.integral_att_error)
    }

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
        for i in 0..3 {
            if self.integral_pos_error[i] > self.max_integral_pos[i] {
                self.integral_pos_error[i] = self.max_integral_pos[i];
            } else if self.integral_pos_error[i] < -self.max_integral_pos[i] {
                self.integral_pos_error[i] = -self.max_integral_pos[i];
            }
        }
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
trait Planner {
    fn plan(
        &self,
        current_position: Vector3<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32);
    fn is_finished(&self, current_position: Vector3<f32>, time: f32) -> bool;
}

struct HoverPlanner {
    target_position: Vector3<f32>,
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
struct MinimumJerkLinePlanner {
    start_position: Vector3<f32>,
    end_position: Vector3<f32>,
    start_yaw: f32,
    end_yaw: f32,
    start_time: f32,
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
struct LissajousPlanner {
    start_position: Vector3<f32>,
    center: Vector3<f32>,
    amplitude: Vector3<f32>,
    frequency: Vector3<f32>,
    phase: Vector3<f32>,
    start_time: f32,
    duration: f32,
    start_yaw: f32,
    end_yaw: f32,
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

        // Smooth start function
        let smooth_start = if t < self.ramp_time / self.duration {
            let t_ramp = t / (self.ramp_time / self.duration);
            t_ramp * t_ramp * (3.0 - 2.0 * t_ramp)
        } else {
            1.0
        };

        // Velocity ramp function
        let velocity_ramp = if t < self.ramp_time / self.duration {
            // Ramp up
            smooth_start
        } else if t > 1.0 - self.ramp_time / self.duration {
            // Ramp down
            let t_down = (1.0 - t) / (self.ramp_time / self.duration);
            t_down * t_down * (3.0 - 2.0 * t_down)
        } else {
            // Full amplitude
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

        // Add the velocity of the transition from start position to center
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
struct CirclePlanner {
    center: Vector3<f32>,
    radius: f32,
    angular_velocity: f32,
    start_position: Vector3<f32>,
    start_time: f32,
    duration: f32,
    start_yaw: f32,
    end_yaw: f32,
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

        // Smooth start function
        let smooth_start = if t < self.ramp_time / self.duration {
            let t_ramp = t / (self.ramp_time / self.duration);
            t_ramp * t_ramp * (3.0 - 2.0 * t_ramp)
        } else {
            1.0
        };

        // Velocity ramp function
        let velocity_ramp = if t < self.ramp_time / self.duration {
            // Ramp up
            smooth_start
        } else if t > 1.0 - self.ramp_time / self.duration {
            // Ramp down
            let t_down = (1.0 - t) / (self.ramp_time / self.duration);
            t_down * t_down * (3.0 - 2.0 * t_down)
        } else {
            // Full amplitude
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
struct PlannerManager {
    current_planner: Box<dyn Planner>,
}

impl PlannerManager {
    fn new(initial_position: Vector3<f32>, initial_yaw: f32) -> Self {
        let hover_planner = HoverPlanner {
            target_position: initial_position,
            target_yaw: initial_yaw,
        };

        Self {
            current_planner: Box::new(hover_planner),
        }
    }

    fn set_planner(&mut self, new_planner: Box<dyn Planner>) {
        self.current_planner = new_planner;
    }

    fn update(
        &mut self,
        current_position: Vector3<f32>,
        current_orientation: UnitQuaternion<f32>,
        current_velocity: Vector3<f32>,
        time: f32,
    ) -> (Vector3<f32>, Vector3<f32>, f32) {
        if self.current_planner.is_finished(current_position, time) {
            self.current_planner = Box::new(HoverPlanner {
                target_position: current_position,
                target_yaw: current_orientation.euler_angles().2, // TODO: fix the yaw angle calculation
            });
        }

        self.current_planner
            .plan(current_position, current_velocity, time)
    }
}

fn log_data(
    rec: &rerun::RecordingStream,
    quad: &Quadrotor,
    desired_position: &Vector3<f32>,
    measured_accel: &Vector3<f32>,
    measured_gyro: &Vector3<f32>,
) {
    rec.log(
        "desired_position",
        &rerun::Points3D::new([(desired_position.x, desired_position.y, desired_position.z)]),
    )
    .unwrap();
    rec.log(
        "quadrotor",
        &rerun::Transform3D::from_translation_rotation(
            rerun::Vec3D::new(quad.position.x, quad.position.y, quad.position.z),
            rerun::Quaternion::from_xyzw([
                quad.orientation.i,
                quad.orientation.j,
                quad.orientation.k,
                quad.orientation.w,
            ]),
        ),
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
fn main() {
    let mut quad = Quadrotor::new();
    let mut controller = PIDController::new();
    let mut imu = IMU::new();
    let rec = rerun::RecordingStreamBuilder::new("quadrotor_simulation")
        .connect()
        .unwrap();
    let mut planner_manager = PlannerManager::new(Vector3::zeros(), 0.0);
    let mut i = 0;
    loop {
        let time = quad.time_step * i as f32;
        rec.set_time_seconds("timestamp", time);
        if i == 500 {
            let new_planner = Box::new(MinimumJerkLinePlanner {
                start_position: quad.position,
                end_position: Vector3::new(0.0, 0.0, 1.0),
                start_yaw: quad.orientation.euler_angles().2,
                end_yaw: 0.0,
                start_time: time,
                duration: 3.0,
            });
            planner_manager.set_planner(new_planner);
        } else if i == 1000 {
            let new_planner = Box::new(MinimumJerkLinePlanner {
                start_position: quad.position,
                end_position: Vector3::new(1.0, 0.0, 1.0),
                start_yaw: quad.orientation.euler_angles().2,
                end_yaw: 0.0,
                start_time: time,
                duration: 3.0,
            });
            planner_manager.set_planner(new_planner);
        } else if i == 1500 {
            let new_planner = Box::new(MinimumJerkLinePlanner {
                start_position: quad.position,
                end_position: Vector3::new(1.0, 1.0, 1.0),
                start_yaw: quad.orientation.euler_angles().2,
                end_yaw: std::f32::consts::PI / 2.0,
                start_time: time,
                duration: 3.0,
            });
            planner_manager.set_planner(new_planner);
        } else if i == 2000 {
            let new_planner = Box::new(MinimumJerkLinePlanner {
                start_position: quad.position,
                end_position: Vector3::new(0.0, 1.0, 1.0),
                start_yaw: quad.orientation.euler_angles().2,
                end_yaw: std::f32::consts::PI / 2.0,
                start_time: time,
                duration: 3.0,
            });
            planner_manager.set_planner(new_planner);
        } else if i == 2500 {
            let new_planner = Box::new(MinimumJerkLinePlanner {
                start_position: quad.position,
                end_position: Vector3::new(0.0, 0.0, 0.5),
                start_yaw: quad.orientation.euler_angles().2,
                end_yaw: 0.0,
                start_time: time,
                duration: 3.0,
            });
            planner_manager.set_planner(new_planner);
        } else if i == 3000 {
            let current_position = quad.position;
            let current_yaw = quad.orientation.euler_angles().2;
            let lissajous_center = Vector3::new(0.5, 0.5, 1.0);

            let new_planner = Box::new(LissajousPlanner {
                start_position: current_position,
                center: lissajous_center,
                amplitude: Vector3::new(0.5, 0.5, 0.2),
                frequency: Vector3::new(1.0, 2.0, 3.0),
                phase: Vector3::new(0.0, std::f32::consts::PI / 2.0, 0.0),
                start_time: time,
                duration: 20.0,
                start_yaw: current_yaw,
                end_yaw: current_yaw + 2.0 * std::f32::consts::PI,
                ramp_time: 5.0,
            });
            planner_manager.set_planner(new_planner);
        } else if i == 5200 {
            let current_position = quad.position;
            let current_yaw = quad.orientation.euler_angles().2;
            let circle_center = Vector3::new(0.5, 0.5, 1.0);

            let new_planner = Box::new(CirclePlanner {
                center: circle_center,
                radius: 0.5,
                angular_velocity: 1.0,
                start_position: current_position,
                start_time: time,
                duration: 8.0,
                start_yaw: current_yaw,
                end_yaw: current_yaw,
                ramp_time: 2.0,
            });
            planner_manager.set_planner(new_planner);
        } else if i == 6200 {
            let new_planner = Box::new(MinimumJerkLinePlanner {
                start_position: quad.position,
                end_position: Vector3::new(quad.position.x, quad.position.y, 0.0),
                start_yaw: quad.orientation.euler_angles().2,
                end_yaw: 0.0,
                start_time: time,
                duration: 3.0,
            });
            planner_manager.set_planner(new_planner);
        }
        let (desired_position, desired_velocity, desired_yaw) =
            planner_manager.update(quad.position, quad.orientation, quad.velocity, time);

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
            calculated_desired_orientation,
            quad.orientation,
            quad.angular_velocity,
            quad.time_step,
        );

        quad.update_dynamics_with_controls(thrust, torque);
        imu.update(quad.time_step);
        let (true_accel, true_gyro) = quad.read_imu();
        let (_measured_accel, _measured_gyro) = imu.read(true_accel, true_gyro);
        log_data(
            &rec,
            &quad,
            &desired_position,
            &_measured_accel,
            &_measured_gyro,
        );
        i += 1;
        if i >= 7000 {
            break;
        }
    }
}
