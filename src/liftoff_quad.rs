use nalgebra::{Matrix3, Quaternion, Rotation3, SMatrix, UnitQuaternion, Vector3};
use peng_quad::{Quadrotor, SimulationError};

/// Represents an RC quadrotor
/// # Example
/// ```
/// use nalgebra::Vector3;
/// use peng_quad::Quadrotor;
/// let (time_step, mass, gravity, drag_coefficient) = (0.01, 1.3, 9.81, 0.01);
/// let inertia_matrix = [0.0347563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.0977];
/// let quadrotor = RCQuad::new(time_step, mass, inertia_matrix);
/// ```
pub struct LiftoffQuad {
    /// Current position of the quadrotor in 3D space
    pub position: Vector3<f32>,
    /// Current velocity of the quadrotor
    pub velocity: Vector3<f32>,
    /// Current orientation of the quadrotor
    pub orientation: UnitQuaternion<f32>,
    /// Current angular velocity of the quadrotor
    pub angular_velocity: Vector3<f32>,
    /// Simulation time step in seconds
    pub time_step: f32,
    /// Mass of the quadrotor in kg
    pub mass: f32,
    /// Inertia matrix of the quadrotor
    pub inertia_matrix: Matrix3<f32>,
    /// Inverse of the inertia matrix
    pub inertia_matrix_inv: Matrix3<f32>,
    /// Previous Thrust
    pub previous_thrust: f32,
    /// Previous Torque
    pub previous_torque: Vector3<f32>,
}

impl Quadrotor for LiftoffQuad {
    fn control(
        &mut self,
        step_number: usize,
        config: &peng_quad::config::Config,
        thrust: f32,
        torque: &Vector3<f32>,
    ) {
        todo!("implement control outputs to CyberRC - gamepad mode")
    }

    fn observe(
        &self,
    ) -> Result<
        (
            Vector3<f32>,
            Vector3<f32>,
            UnitQuaternion<f32>,
            Vector3<f32>,
        ),
        SimulationError,
    > {
        todo!("implement state feedback from Liftoff UDP")
    }
}
