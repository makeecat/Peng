#![feature(thread_sleep_until)]
use nalgebra::Vector3;
use peng_quad::*;
use std::thread;
use std::time::Duration;
use std::time::Instant;

mod liftoff_quad;
mod rc_quad;

/// Main function for the simulation
fn main() -> Result<(), SimulationError> {
    env_logger::builder()
        .parse_env(env_logger::Env::default().default_filter_or("info"))
        .init();
    let mut config_str = "config/quad.yaml";
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 2 {
        log::warn!("Usage: {} <config.yaml>.", args[0]);
        log::warn!("Loading default configuration: config/quad.yaml");
    } else {
        log::info!("Loading configuration: {}", args[1]);
        config_str = &args[1];
    }
    let config = config::Config::from_yaml(config_str).expect("Failed to load configuration");
    let mut quad = SimulatedQuadrotor::new(
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
    let camera = Camera::new(
        config.camera.resolution,
        config.camera.fov_vertical.to_radians(),
        config.camera.near,
        config.camera.far,
    );
    let mut planner_manager = PlannerManager::new(Vector3::zeros(), 0.0);
    let mut trajectory = Trajectory::new(Vector3::new(0.0, 0.0, 0.0));
    let mut depth_buffer: Vec<f32> = vec![0.0; camera.resolution.0 * camera.resolution.1];
    let planner_config: Vec<PlannerStepConfig> = config
        .planner_schedule
        .iter()
        .map(|step| PlannerStepConfig {
            step: step.step,
            planner_type: step.planner_type.clone(),
            params: step.params.clone(),
        })
        .collect();
    log::info!("Use rerun.io: {}", config.use_rerun);
    let rec = if config.use_rerun {
        rerun::spawn(&rerun::SpawnOptions::default())?;
        Some(rerun::RecordingStreamBuilder::new("Peng").connect()?)
    } else {
        None
    };
    if let Some(rec) = &rec {
        rec.set_time_seconds("timestamp", 0);
        log_mesh(rec, config.mesh.division, config.mesh.spacing)?;
        log_maze_tube(rec, &maze)?;
        log_maze_obstacles(rec, &maze)?;
    }
    log::info!("Starting simulation...");
    let mut i = 0;
    let frame_time = Duration::from_secs_f32(1.0 / config.simulation.simulation_frequency as f32);
    let mut next_frame = Instant::now();
    println!("frame_time: {:?}", frame_time);
    loop {
        // If real-time mode is enabled, sleep until the next frame simulation frame
        if config.real_time {
            thread::sleep_until(next_frame);
            next_frame += frame_time;
        }
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
        quad.control(i, &config, thrust, &torque);
        imu.update(quad.time_step)?;
        let (true_accel, true_gyro) = quad.read_imu()?;
        let (_measured_accel, _measured_gyro) = imu.read(true_accel, true_gyro)?;
        if i % (config.simulation.simulation_frequency / config.simulation.log_frequency) == 0 {
            if config.render_depth {
                camera.render_depth(
                    &quad.position,
                    &quad.orientation,
                    &maze,
                    &mut depth_buffer,
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
                        &depth_buffer,
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
                        &depth_buffer,
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
    Ok(())
}
