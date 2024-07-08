extern crate nalgebra as na;
extern crate rand;

use na::{Vector3, UnitQuaternion, Matrix3, Rotation3};
use rand_distr::{Distribution, Normal};
use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::nalgebra::{Translation3, UnitQuaternion as Kiss3dUnitQuaternion};

struct Quadrotor {
    position: Vector3<f32>,
    velocity: Vector3<f32>,
    orientation: UnitQuaternion<f32>,
    angular_velocity: Vector3<f32>,
    mass: f32,
    gravity: f32,
    time_step: f32,
    thrust_coefficient: f32,
    drag_coefficient: f32,
    inertia_matrix: Matrix3<f32>,
}

impl Quadrotor {
    pub fn new() -> Self {
        Self {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            mass: 1.3,
            gravity: 9.81,
            time_step: 0.01,
            thrust_coefficient: 0.0,
            drag_coefficient: 0.000,
            inertia_matrix: Matrix3::new(0.00304475, 0.0, 0.0, 0.0, 0.00454981, 0.0, 0.0, 0.0, 0.00281995),
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
        let delta_q = UnitQuaternion::new(omega);
        self.orientation = self.orientation * delta_q;
    }

    pub fn update_dynamics_with_controls(&mut self, control_thrust: Vector3<f32>, control_torque: Vector3<f32>) {
        let gravity_force = Vector3::new(0.0, 0.0, -self.mass * self.gravity);
        let drag_force = -self.drag_coefficient * self.velocity.norm() * self.velocity;
        let thrust_body = Vector3::new(0.0, 0.0, control_thrust.norm());
        let thrust_world = self.orientation * thrust_body;
        let total_force = thrust_world + gravity_force + drag_force;

        let acceleration = total_force / self.mass;
        self.velocity += acceleration * self.time_step;
        self.position += self.velocity * self.time_step;

        let inertia_inv = self.inertia_matrix.try_inverse().unwrap();
        let inertia_angular_velocity = self.inertia_matrix * self.angular_velocity;
        let gyroscopic_torque = self.angular_velocity.cross(&inertia_angular_velocity);
        let angular_acceleration = inertia_inv * (control_torque - gyroscopic_torque);

        self.angular_velocity += angular_acceleration * self.time_step;
        self.orientation *= UnitQuaternion::from_scaled_axis(self.angular_velocity * self.time_step);
    }

    pub fn read_imu(&self) -> (Vector3<f32>, Vector3<f32>) {
        let accel_noise = Normal::new(0.0, 0.02).unwrap();
        let gyro_noise = Normal::new(0.0, 0.01).unwrap();

        let gravity_world = Vector3::new(0.0, 0.0, self.gravity);
        // let gravity_body = self.orientation.inverse() * gravity_world;
        let specific_force = self.orientation.inverse() * (self.velocity / self.time_step - gravity_world);

        let measured_acceleration = specific_force + Vector3::new(
            accel_noise.sample(&mut rand::thread_rng()),
            accel_noise.sample(&mut rand::thread_rng()),
            accel_noise.sample(&mut rand::thread_rng()),
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

    pub fn read(&self, true_acceleration: Vector3<f32>, true_angular_velocity: Vector3<f32>) -> (Vector3<f32>, Vector3<f32>) {
        let accel_noise = Normal::new(0.0, self.accel_noise_std).unwrap();
        let gyro_noise = Normal::new(0.0, self.gyro_noise_std).unwrap();

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

impl Controller {
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

    fn compute_attitude_control(&mut self, desired_orientation: UnitQuaternion<f32>, current_orientation: UnitQuaternion<f32>, current_angular_velocity: Vector3<f32>, dt: f32) -> Vector3<f32> {
        let error_orientation = current_orientation.inverse() * desired_orientation;
        let (roll_error, pitch_error, yaw_error) = error_orientation.euler_angles();
        let error_angles = Vector3::new(roll_error, pitch_error, yaw_error);
        
        self.integral_att_error += error_angles * dt;
        self.integral_att_error = self.integral_att_error.component_mul(&self.max_integral_att.map(|x| x.signum()));
        let error_angular_velocity = -current_angular_velocity;
        
        self.kp_att.component_mul(&error_angles) + 
        self.kd_att.component_mul(&error_angular_velocity) + 
        self.ki_att.component_mul(&self.integral_att_error)
    }
    fn compute_position_control(&mut self, desired_position: Vector3<f32>, current_position: Vector3<f32>, current_velocity: Vector3<f32>, current_orientation: UnitQuaternion<f32>, dt: f32, mass: f32, gravity: f32) -> (Vector3<f32>, UnitQuaternion<f32>) {
        let error_position = desired_position - current_position;
        let error_velocity = -current_velocity;
        
        self.integral_pos_error += error_position * dt;
        self.integral_pos_error = self.integral_pos_error.component_mul(&self.max_integral_pos.map(|x| x.signum()));
        let acceleration = self.kp_pos.component_mul(&error_position) + 
                           self.kd_pos.component_mul(&error_velocity) + 
                           self.ki_pos.component_mul(&self.integral_pos_error);
        
        let gravity_compensation = Vector3::new(0.0, 0.0, gravity);
        let total_acceleration = acceleration + gravity_compensation;
        
        let thrust = mass * total_acceleration.norm();
        let desired_orientation = if total_acceleration.norm() > 1e-6 {
            let z_body = total_acceleration.normalize();
            let y_body = current_orientation.transform_vector(&Vector3::new(0.0, 1.0, 0.0));
            let x_body = y_body.cross(&z_body).normalize();
            let y_body = z_body.cross(&x_body);
            
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[x_body, y_body, z_body])))
        } else {
            current_orientation
        };
        
        (Vector3::new(0.0, 0.0, thrust), desired_orientation)
    }
}

struct Visualizer {
    window: Window,
    quadrotor: SceneNode,
    desired_position: SceneNode,
}

impl Visualizer {
    fn new() -> Self {
        let mut window = Window::new("Quadrotor Simulation");
        window.set_light(Light::StickToCamera);
        let mut quadrotor = window.add_group();

        let mut quadrotor_body = quadrotor.add_cube(0.2, 0.2, 0.05);
        let mut quadrotor_z_vec = quadrotor.add_cube(0.05, 0.05, 5.0);
        let mut quadrotor_x_vec = quadrotor.add_cube(5.0, 0.05, 0.05);
        let mut quadrotor_y_vec = quadrotor.add_cube(0.05, 5.0, 0.05);
        quadrotor_body.set_color(0.0, 0.0, 0.0);
        quadrotor_x_vec.set_color(1.0, 0.0, 0.0);
        quadrotor_y_vec.set_color(0.0, 1.0, 0.0);
        quadrotor_z_vec.set_color(0.0, 0.0, 1.0);
        quadrotor_x_vec.set_local_translation(Translation3::new(2.5, 0.0, 0.0).into());
        quadrotor_y_vec.set_local_translation(Translation3::new(0.0, 2.5, 0.0).into());
        quadrotor_z_vec.set_local_translation(Translation3::new(0.0, 0.0, 2.5).into());

        let mut desired_position = window.add_sphere(0.1);
        desired_position.set_color(0.0, 1.0, 0.0);

        let linewidth = 0.04;
        let linelength = 2000.0;

        // Floor
        for x_pos in -100..100 {
            let mut cube = window.add_cube(linewidth, linelength, linewidth);
            cube.set_color(0.5, 1.0, 0.5);
            cube.set_local_translation(Translation3::new(5.0 * x_pos as f32, 0.0, 0.0));
        }
        for x_pos in -100..100 {
            let mut cube = window.add_cube(linelength, linewidth, linewidth);
            cube.set_color(0.5, 1.0, 0.5);
            cube.set_local_translation(Translation3::new(0.0, 5.0 * x_pos as f32, 0.0));
        }

        Self {
            window,
            quadrotor,
            desired_position,
        }
    }

    fn update(&mut self, quad_position: Vector3<f32>, quad_orientation: UnitQuaternion<f32>, desired_position: Vector3<f32>) {
        let quadrotor_translation = Translation3::new(quad_position.x, quad_position.y, quad_position.z).into();
        let (quadrotor_roll, quadrotor_pitch, quadrotor_yaw) = quad_orientation.euler_angles();
        let quadrotor_rotation = Kiss3dUnitQuaternion::from_euler_angles(quadrotor_roll, quadrotor_pitch, quadrotor_yaw);
        self.quadrotor.set_local_translation(quadrotor_translation);
        self.quadrotor.set_local_rotation(quadrotor_rotation);

        self.desired_position.set_local_translation(Translation3::new(desired_position.x, desired_position.y, desired_position.z));

    }


    fn render(&mut self) -> bool {
        self.window.render()
    }
}

fn main() {
    let mut quad = Quadrotor::new();
    let mut controller = Controller::new();
    let mut imu = IMU::new();
    let desired_position = Vector3::new(0.1, 0.5, 1.0);
    let mut visualizer = Visualizer::new();

    let mut i = 0;
    while visualizer.render() {
        let (thrust, calculated_desired_orientation) = controller.compute_position_control(
            desired_position,
            quad.position,
            quad.velocity,
            quad.orientation,
            quad.time_step,
            quad.mass,
            quad.gravity
        );
        let torque = controller.compute_attitude_control(
            calculated_desired_orientation,
            quad.orientation,
            quad.angular_velocity,
            quad.time_step
        );

        quad.update_dynamics_with_controls(thrust, torque);
        imu.update(quad.time_step);

        let (true_accel, true_gyro) = quad.read_imu();
        let (measured_accel, measured_gyro) = imu.read(true_accel, true_gyro);

        visualizer.update(quad.position, quad.orientation, desired_position);

        if i % 100 == 0 {
            println!("Position: {:?}, Orientation: {:?}", quad.position, quad.orientation);
            println!("IMU Acceleration: {:?}, Gyro: {:?}", measured_accel, measured_gyro);
        }
        i += 1;
    }
}
