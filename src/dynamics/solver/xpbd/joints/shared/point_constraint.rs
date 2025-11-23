use crate::{
    dynamics::solver::{
        solver_body::{SolverBody, SolverBodyInertia},
        xpbd::*,
    },
    prelude::*,
};
use bevy::prelude::*;

/// Constraint data required by the XPBD constraint solver for a point constraint.
#[derive(Clone, Copy, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct PointConstraintShared {
    /// The world-space anchor point relative to the center of mass of the first body.
    pub world_r1: Vector,
    /// The world-space anchor point relative to the center of mass of the second body.
    pub world_r2: Vector,
    /// The difference in center of mass positions between the two bodies.
    pub center_difference: Vector,
    /// The total Lagrange multiplier across the whole time step.
    pub total_lagrange: Vector,
}

impl XpbdConstraintSolverData for PointConstraintShared {
    fn clear_lagrange_multipliers(&mut self) {
        self.total_lagrange = Vector::ZERO;
    }

    fn total_position_lagrange(&self) -> Vector {
        self.total_lagrange
    }
}

impl PointConstraintShared {
    /// Prepares the constraint with the given bodies and local anchor points.
    pub fn prepare(
        &mut self,
        bodies: [&RigidBodyQueryReadOnlyItem; 2],
        local_anchor1: Vector,
        local_anchor2: Vector,
    ) {
        let [body1, body2] = bodies;

        self.world_r1 = body1.rotation * (local_anchor1 - body1.center_of_mass.0);
        self.world_r2 = body2.rotation * (local_anchor2 - body2.center_of_mass.0);
        self.center_difference = (body2.position.0 - body1.position.0)
            + (body2.rotation * body2.center_of_mass.0 - body1.rotation * body1.center_of_mass.0);
    }

    /// Solves the constraint for the given bodies.
    #[allow(clippy::too_many_arguments)]
    pub fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        compliance: Scalar,
        dt: Scalar,
        conf: &OgcSolverConfig,
        max_displacement: Option<Scalar>,
        max_velocity: Option<Scalar>,
        activation_tau: Option<Scalar>,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        let world_r1 = body1.delta_rotation * self.world_r1;
        let world_r2 = body2.delta_rotation * self.world_r2;

        let separation = (body2.delta_position - body1.delta_position)
            + (world_r2 - world_r1)
            + self.center_difference;

        let magnitude_squared = separation.length_squared();

        if magnitude_squared == 0.0 {
            // No separation, no need to apply a correction.
            return;
        }

        let magnitude = magnitude_squared.sqrt();
        let dir = -separation / magnitude;

        // Compute generalized inverse masses
        let w1 = self.compute_generalized_inverse_mass(
            inv_mass1.max_element(), // TODO: Do this properly.
            inv_angular_inertia1,
            world_r1,
            dir,
        );
        let w2 = self.compute_generalized_inverse_mass(
            inv_mass2.max_element(), // TODO: Do this properly.
            inv_angular_inertia2,
            world_r2,
            dir,
        );

        // Compute Lagrange multiplier update
        let mut delta_lagrange = compute_lagrange_update(0.0, magnitude, &[w1, w2], compliance, dt);

        // OGC: Two-stage activation
        let activation_tau = activation_tau.unwrap_or(conf.activation_tau);
        if activation_tau > 0.0 {
            let activation_factor = 1.0 - (-dt / activation_tau).exp();
            delta_lagrange *= activation_factor;
        }

        let mut impulse = delta_lagrange * dir;

        // OGC: Clamp positional corrections and velocity
        let max_disp = max_displacement.unwrap_or(conf.max_displacement);
        let max_vel = max_velocity.unwrap_or(conf.max_velocity);
        
        // The limit imposed by max_velocity on displacement per substep
        let vel_limit = max_vel * dt;
        let effective_limit = max_disp.min(vel_limit);

        // Find the maximum inverse mass to estimate displacement of the lightest body
        let max_w = w1.max(w2);
        if max_w > Scalar::EPSILON {
             let max_impulse = effective_limit / max_w;
             if impulse.length_squared() > max_impulse * max_impulse {
                 impulse = impulse.normalize() * max_impulse;
             }
        }

        self.total_lagrange += impulse;

        // Apply positional correction to align the positions of the bodies
        self.apply_positional_impulse(
            body1, body2, inertia1, inertia2, impulse, world_r1, world_r2,
        );
    }
}

impl PositionConstraint for PointConstraintShared {}
