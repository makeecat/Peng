use nalgebra::Vector3;
use peng_quad::*;
/// Main function for the simulation
fn main() -> Result<(), SimulationError> {
    let mut config_str = "config/quad.yaml";
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 2 {
        println!(
            "[\x1b[33mWARN\x1b[0m peng_quad] Usage: {} <config.yaml>.",
            args[0]
        );
        println!("[\x1b[33mWARN\x1b[0m peng_quad] Loading default configuration: config/quad.yaml");
    } else {
        println!(
            "[\x1b[32mINFO\x1b[0m peng_quad] Loading configuration: {}",
            args[1]
        );
        config_str = &args[1];
    }
    let config = config::Config::from_yaml(config_str).expect("Failed to load configuration.");
    println!(
        "[\x1b[32mINFO\x1b[0m peng_quad]Use rerun.io: {}",
        config.use_rerun
    );
    let mut quad = Quadrotor::new(
        1.0 / config.simulation.simulation_frequency as f32,
        config.quadrotor.mass,
        config.quadrotor.gravity,
        config.quadrotor.drag_coefficient,
        config.quadrotor.inertia_matrix,
    )?;
    let _pos_gains = config.pid_controller.pos_gains;
    let _att_gains = config.pid_controller.att_gains;
    let mut controller = PIDController::new(
        [_pos_gains.kp, _pos_gains.kd, _pos_gains.ki],
        [_att_gains.kp, _att_gains.kd, _att_gains.ki],
        config.pid_controller.pos_max_int,
        config.pid_controller.att_max_int,
        config.quadrotor.mass,
        config.quadrotor.gravity,
    );
    let mut imu = Imu::new(
        config.imu.accel_noise_std,
        config.imu.gyro_noise_std,
        config.imu.accel_bias_std,
        config.imu.gyro_bias_std,
    )?;
    let mut maze = Maze::new(
        config.maze.lower_bounds,
        config.maze.upper_bounds,
        config.maze.num_obstacles,
        config.maze.obstacles_velocity_bounds,
        config.maze.obstacles_radius_bounds,
    );
    let mut camera = Camera::new(
        config.camera.resolution,
        config.camera.fov_vertical.to_radians(),
        config.camera.near,
        config.camera.far,
    );
    let mut planner_manager = PlannerManager::new(Vector3::zeros(), 0.0);
    let mut trajectory = Trajectory::new(Vector3::new(0.0, 0.0, 0.0));
    let mut previous_thrust = 0.0;
    let mut previous_torque = Vector3::zeros();
    let planner_config: Vec<PlannerStepConfig> = config
        .planner_schedule
        .iter()
        .map(|step| PlannerStepConfig {
            step: step.step,
            planner_type: step.planner_type.clone(),
            params: step.params.clone(),
        })
        .collect();
    let rec = if config.use_rerun {
        let _rec = rerun::RecordingStreamBuilder::new("Peng").spawn()?;
        rerun::Logger::new(_rec.clone())
            .with_path_prefix("logs")
            .with_filter(rerun::default_log_filter())
            .init()
            .unwrap();
        Some(_rec)
    } else {
        env_logger::builder()
            .parse_env(env_logger::Env::default().default_filter_or("info"))
            .init();
        None
    };
    log::info!("Use rerun.io: {}", config.use_rerun);
    if let Some(rec) = &rec {
        rec.log_file_from_path(config.rerun_blueprint, None, false)?;
        rec.set_time_seconds("timestamp", 0);
        log_mesh(rec, config.mesh.division, config.mesh.spacing)?;
        log_maze_tube(rec, &maze)?;
        log_maze_obstacles(rec, &maze)?;
    }
    log::info!("Starting simulation...");
    let mut i = 0;
    loop {
        let time = quad.time_step * i as f32;
        maze.update_obstacles(quad.time_step);
        update_planner(
            &mut planner_manager,
            i,
            time,
            config.simulation.simulation_frequency,
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
        );
        let torque = controller.compute_attitude_control(
            &calculated_desired_orientation,
            &quad.orientation,
            &quad.angular_velocity,
            quad.time_step,
        );
        if i % (config.simulation.simulation_frequency / config.simulation.control_frequency) == 0 {
            if config.use_rk4_for_dynamics_control {
                quad.update_dynamics_with_controls_rk4(thrust, &torque);
            } else {
                quad.update_dynamics_with_controls_euler(thrust, &torque);
            }
            previous_thrust = thrust;
            previous_torque = torque;
        } else if config.use_rk4_for_dynamics_update {
            quad.update_dynamics_with_controls_rk4(previous_thrust, &previous_torque);
        } else {
            quad.update_dynamics_with_controls_euler(previous_thrust, &previous_torque);
        }
        imu.update(quad.time_step)?;
        let (true_accel, true_gyro) = quad.read_imu()?;
        let (_measured_accel, _measured_gyro) = imu.read(true_accel, true_gyro)?;
        if i % (config.simulation.simulation_frequency / config.simulation.log_frequency) == 0 {
            if config.render_depth {
                camera.render_depth(
                    &quad.position,
                    &quad.orientation,
                    &maze,
                    config.use_multithreading_depth_rendering,
                )?;
            }
            if let Some(rec) = &rec {
                rec.set_time_seconds("timestamp", time);
                if trajectory.add_point(quad.position) {
                    log_trajectory(rec, &trajectory)?;
                }
                log_data(
                    rec,
                    &quad,
                    &desired_position,
                    &desired_velocity,
                    &_measured_accel,
                    &_measured_gyro,
                )?;
                if config.render_depth {
                    log_depth_image(
                        rec,
                        &camera.depth_buffer,
                        camera.resolution,
                        camera.near,
                        camera.far,
                    )?;
                    pinhole_depth(
                        rec,
                        &camera,
                        quad.position,
                        quad.orientation,
                        config.camera.rotation_transform,
                        &camera.depth_buffer,
                    )?;
                }
                log_maze_obstacles(rec, &maze)?;
            }
        }
        i += 1;
        if time >= config.simulation.duration {
            log::info!("Complete Simulation");
            break;
        }
    }
    log::logger().flush();
    Ok(())
}
