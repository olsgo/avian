use crate::prelude::*;
use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use core::time::Duration;

fn create_app() -> App {
    let mut app = App::new();

    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        PhysicsPlugins::default(),
        bevy::asset::AssetPlugin::default(),
    ))
    .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
        1.0 / 60.0,
    )));

    app.finish();

    app
}

fn tick_app(app: &mut App, timestep: f64) {
    let strategy = TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(timestep));
    if let Some(mut update_strategy) = app.world_mut().get_resource_mut::<TimeUpdateStrategy>() {
        *update_strategy = strategy;
    } else {
        app.insert_resource(strategy);
    }
    app.update();
}

#[test]
fn test_ogc_max_displacement() {
    let mut app = create_app();
    app.insert_resource(Gravity::ZERO);
    // Reduce substeps to 1 for easier calculation
    app.insert_resource(SubstepCount(1));

    let max_disp = 0.1;
    
    // Spawn two bodies 2.0 units apart
    let body1 = app.world_mut().spawn((
        RigidBody::Dynamic,
        Position(Vector::X * -1.0),
        Rotation::default(),
        Mass(1.0),
        InverseMass(1.0),
        Inertia::ZERO,
        InverseInertia::ZERO,
    )).id();
    
    let body2 = app.world_mut().spawn((
        RigidBody::Dynamic,
        Position(Vector::X * 1.0),
        Rotation::default(),
        Mass(1.0),
        InverseMass(1.0),
        Inertia::ZERO,
        InverseInertia::ZERO,
    )).id();

    // Connect with distance joint of length 0.0 (should pull them together)
    // Set max_displacement
    app.world_mut().spawn(
        DistanceJoint::new(body1, body2)
            .with_limits(0.0, 0.0)
            .with_compliance(0.0)
            .with_max_displacement(max_disp)
    );

    // Run one step
    let dt = 1.0 / 60.0;
    tick_app(&mut app, dt);

    // Check positions
    let p1 = app.world().get::<Position>(body1).unwrap().0;
    let p2 = app.world().get::<Position>(body2).unwrap().0;

    println!("p1: {}, p2: {}", p1, p2);

    // Bodies start at -1.0 and 1.0. Distance 2.0.
    // They want to be at 0 distance.
    // Without cap, they would move to 0.0.
    // With max_disp = 0.1, total impulse is limited such that impulse * w <= 0.1.
    // w1 = 1.0, w2 = 1.0. max_w = 1.0.
    // max_impulse = 0.1 / 1.0 = 0.1.
    // Applied impulse magnitude clamped to 0.1.
    // Body 1 delta pos = 0.1 * dir (dir is +X).
    // Body 2 delta pos = -0.1 * dir (dir is +X).
    // Wait, impulse logic in code:
    // body1.delta_position += impulse * inv_mass1;
    // body2.delta_position -= impulse * inv_mass2;
    // impulse is vector.
    
    // p1 should be -1.0 + 0.1 = -0.9.
    // p2 should be 1.0 - 0.1 = 0.9.

    assert!((p1.x - -0.9).abs() < 0.001, "Body 1 moved too much or too little: {}", p1.x);
    assert!((p2.x - 0.9).abs() < 0.001, "Body 2 moved too much or too little: {}", p2.x);
}

#[test]
fn test_ogc_max_velocity() {
    let mut app = create_app();
    app.insert_resource(Gravity::ZERO);
    app.insert_resource(SubstepCount(1));

    let max_vel = 1.0; // 1 m/s
    
    // Spawn two bodies 2.0 units apart
    let body1 = app.world_mut().spawn((
        RigidBody::Dynamic,
        Position(Vector::X * -1.0),
        Rotation::default(),
        Mass(1.0),
        InverseMass(1.0),
        Inertia::ZERO,
        InverseInertia::ZERO,
    )).id();
    
    let body2 = app.world_mut().spawn((
        RigidBody::Dynamic,
        Position(Vector::X * 1.0),
        Rotation::default(),
        Mass(1.0),
        InverseMass(1.0),
        Inertia::ZERO,
        InverseInertia::ZERO,
    )).id();

    app.world_mut().spawn(
        DistanceJoint::new(body1, body2)
            .with_limits(0.0, 0.0)
            .with_compliance(0.0)
            .with_max_velocity(max_vel)
    );

    let dt = 1.0 / 60.0;
    tick_app(&mut app, dt);

    let p1 = app.world().get::<Position>(body1).unwrap().0;
    
    // max_vel limit imposes max_disp = max_vel * dt = 1.0 * 1/60 = 0.01666...
    // Each body should move at most 0.01666...
    let expected_disp = max_vel * dt;
    let expected_p1 = -1.0 + expected_disp;

    assert!((p1.x - expected_p1).abs() < 0.001, "Body 1 moved incorrectly with max_velocity: {}", p1.x);
}
