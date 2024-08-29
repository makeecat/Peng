use nalgebra::Vector3;
use std::env;

mod config;
use config::Config;
use peng_quad::*;
/// Main function to run the quadrotor simulation
fn main() -> Result<(), SimulationError> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        return Err(SimulationError::OtherError(format!(
            "Usage: {} <config.yaml>",
            args[0]
        )));
    }
    let config = Config::from_yaml(&args[1]).expect("Failed to load configuration");
    let mut quad = Quadrotor::new(
        1.0 / config.simulation.simulation_frequency,
        config.quadrotor.mass,
        config.quadrotor.gravity,
        config.quadrotor.drag_coefficient,
        config.quadrotor.inertia_matrix,
    )?;
    let _pos_gains = config.controller.pos_gains;
    let _att_gains = config.controller.att_gains;
    let mut controller = PIDController::new(
        [_pos_gains.kp, _pos_gains.kd, _pos_gains.ki],
        [_att_gains.kp, _att_gains.kd, _att_gains.ki],
        config.controller.pos_max_int,
        config.controller.att_max_int,
    );
    let mut imu = Imu::new(
        config.imu.accel_noise_std,
        config.imu.gyro_noise_std,
        config.imu.bias_instability,
    );
    println!("Please start a rerun-cli in another terminal.\n1. cargo install rerun-cli.\n2. rerun\n\n[INFO] Waiting for connection to rerun...");
    let rec = rerun::RecordingStreamBuilder::new("Peng").connect()?;
    let upper_bounds = Vector3::from(config.maze.upper_bounds);
    let lower_bounds = Vector3::from(config.maze.lower_bounds);
    let mut maze = Maze::new(lower_bounds, upper_bounds, config.maze.num_obstacles);
    let camera = Camera::new(
        config.camera.resolution,
        config.camera.fov.to_radians(),
        config.camera.near,
        config.camera.far,
    );
    let mut planner_manager = PlannerManager::new(Vector3::zeros(), 0.0);
    let mut trajectory = Trajectory::new(Vector3::new(0.0, 0.0, 0.0));
    let mut depth_buffer: Vec<f32> = vec![0.0; camera.resolution.0 * camera.resolution.1];
    rec.set_time_seconds("timestamp", 0);
    log_mesh(&rec, config.mesh.division, config.mesh.spacing)?;
    log_maze_tube(&rec, &maze)?;
    log_maze_obstacles(&rec, &maze)?;
    let mut previous_thrust = 0.0;
    let mut previous_torque = Vector3::zeros();
    let mut i = 0;
    let planner_config = config
        .planner_schedule
        .iter()
        .map(|step| PlannerStepConfig {
            step: step.step,
            planner_type: step.planner_type.clone(),
            params: step.params.clone(),
        })
        .collect();
    loop {
        let time = quad.time_step * i as f32;
        rec.set_time_seconds("timestamp", time);
        maze.update_obstacles(quad.time_step);
        update_planner(
            &mut planner_manager,
            i,
            time,
            &quad,
            &maze.obstacles,
            &planner_config,
        )?;
        let (desired_position, desired_velocity, desired_yaw) = planner_manager.update(
            quad.position,
            quad.orientation,
            quad.velocity,
            time,
            &maze.obstacles,
        )?;
        let (thrust, calculated_desired_orientation) = controller.compute_position_control(
            &desired_position,
            &desired_velocity,
            desired_yaw,
            &quad.position,
            &quad.velocity,
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
        if i % (config.simulation.simulation_frequency as usize
            / config.simulation.control_frequency as usize)
            == 0
        {
            quad.update_dynamics_with_controls(thrust, &torque);
            previous_thrust = thrust;
            previous_torque = torque;
        } else {
            quad.update_dynamics_with_controls(previous_thrust, &previous_torque);
        }
        imu.update(quad.time_step)?;
        let (true_accel, true_gyro) = quad.read_imu()?;
        let (_measured_accel, _measured_gyro) = imu.read(true_accel, true_gyro)?;
        if i % (config.simulation.simulation_frequency as usize
            / config.simulation.log_frequency as usize)
            == 0
        {
            if trajectory.add_point(quad.position) {
                log_trajectory(&rec, &trajectory)?;
            }
            log_data(
                &rec,
                &quad,
                &desired_position,
                &desired_velocity,
                &_measured_accel,
                &_measured_gyro,
            )?;
            camera.render_depth(&quad.position, &quad.orientation, &maze, &mut depth_buffer)?;
            log_depth_image(
                &rec,
                &depth_buffer,
                camera.resolution.0,
                camera.resolution.1,
                camera.near,
                camera.far,
            )?;
            log_maze_obstacles(&rec, &maze)?;
        }
        i += 1;
        if time >= config.simulation.duration {
            println!("[INFO] Simulation complete");
            break;
        }
    }
    Ok(())
}
