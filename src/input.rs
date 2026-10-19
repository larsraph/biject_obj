use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::{
    grid::{Bijection, ConstGrid, GCell, ObjGrid, WorldGrid},
    projection::solve,
};

const INITIAL_BRUSH_RADIUS: i32 = 4;

#[derive(Resource)]
pub struct Brush {
    radius: i32,
}

impl Default for Brush {
    fn default() -> Self {
        Self {
            radius: INITIAL_BRUSH_RADIUS,
        }
    }
}

#[derive(Resource, Default)]
pub struct Selection(Option<(IVec2, IVec2)>);

/// The most recent world-space mouse position while the right mouse button is held.
#[derive(Resource, Default)]
pub struct MouseTarget(pub Option<Vec3>);

#[derive(Component)]
pub struct MouseFollower;

/// Rapier has no built-in velocity-limit components, so retain Avian's per-body
/// caps with an equivalent clamp before each simulation step.
#[derive(Component)]
pub struct VelocityCap {
    linear: f32,
    angular: f32,
}

/// Creates the right-click physics body and updates the target it follows.
pub fn update_mouse_rigid_body(
    mut commands: Commands,
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform)>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mut target: ResMut<MouseTarget>,
    followers: Query<Entity, With<MouseFollower>>,
) {
    if mouse_buttons.just_released(MouseButton::Right) {
        target.0 = None;
        for entity in &followers {
            commands.entity(entity).despawn();
        }
        return;
    }

    if !mouse_buttons.pressed(MouseButton::Right) {
        target.0 = None;
        return;
    }

    let Some(cursor_position) = window.cursor_position() else {
        return;
    };
    let (camera, camera_transform) = camera.into_inner();
    let Ok(position) = camera.viewport_to_world_2d(camera_transform, cursor_position) else {
        return;
    };
    let position = position.extend(0.0);
    target.0 = Some(position);

    if followers.is_empty() {
        commands.spawn((
            MouseFollower,
            RigidBody::Dynamic,
            Collider::ball(0.5),
            Transform::from_translation(position),
            Velocity::zero(),
            GravityScale(0.0),
            Sleeping::disabled(),
            LockedAxes::TRANSLATION_LOCKED_Z
                | LockedAxes::ROTATION_LOCKED_X
                | LockedAxes::ROTATION_LOCKED_Y,
        ));
    }
}

/// Sets a velocity that reaches the current mouse target in exactly one physics tick.
pub fn move_mouse_rigid_body(
    target: Res<MouseTarget>,
    time: Res<Time>,
    mut followers: Query<(&Transform, &mut Velocity), With<MouseFollower>>,
) {
    let Some(target) = target.0 else {
        return;
    };
    let delta_seconds = time.delta_secs();
    if delta_seconds == 0.0 {
        return;
    }

    for (position, mut velocity) in &mut followers {
        velocity.linear = (target - position.translation) / delta_seconds;
    }
}

pub fn clamp_velocities(mut bodies: Query<(&mut Velocity, &VelocityCap)>) {
    for (mut velocity, cap) in &mut bodies {
        velocity.linear = velocity.linear.clamp_length_max(cap.linear);
        velocity.angular = velocity.angular.clamp_length_max(cap.angular);
    }
}

pub fn edit(
    mut commands: Commands,
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform)>,
    mut world: Single<&mut ConstGrid, With<WorldGrid>>,
    mbi: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut brush: ResMut<Brush>,
    mut last: Local<Option<(GCell, Vec2)>>,
    mut selection: ResMut<Selection>,
) -> Result<(), BevyError> {
    if keyboard.just_pressed(KeyCode::Equal) || keyboard.just_pressed(KeyCode::NumpadAdd) {
        brush.radius += 1;
    }
    if keyboard.just_pressed(KeyCode::Minus) || keyboard.just_pressed(KeyCode::NumpadSubtract) {
        brush.radius = (brush.radius - 1).max(1);
    }

    let lmb = mbi.pressed(MouseButton::Left);
    let selecting = keyboard.any_pressed([KeyCode::ControlLeft, KeyCode::ControlRight]);
    let selection_ended =
        !selecting && keyboard.any_just_released([KeyCode::ControlLeft, KeyCode::ControlRight]);

    if !lmb && !selecting && !selection_ended {
        *last = None;
        return Ok(());
    }

    let (camera, camera_transform) = camera.into_inner();
    if let Some(cursor_position) = window.cursor_position() {
        let pos = camera
            .viewport_to_world_2d(camera_transform, cursor_position)
            .with_severity(Severity::Warning)?;

        if selecting || selection_ended {
            *last = None;
            let current = pos.round().as_ivec2();

            if selecting {
                let (_, end) = selection.0.get_or_insert((current, current));
                *end = current;
            }
            if selection_ended {
                if let Some((start, _)) = selection.0.take() {
                    if let Ok(p0) = start.try_into()
                        && let Ok(p1) = current.try_into()
                    {
                        let rect = URect::from_corners(p0, p1);
                        let objgrid = world.extract_rect(rect);
                        if let Some(objgrid) = objgrid {
                            let mass = objgrid.mass();
                            let Some(collider) = objgrid.collider() else {
                                return Ok(());
                            };

                            let translation = start.min(current).extend(0).as_vec3();
                            let solution =
                                solve(objgrid.shape.clone(), Quat::default(), translation.into());

                            commands.spawn((
                                ObjGrid,
                                objgrid,
                                collider,
                                RigidBody::Dynamic,
                                VelocityCap {
                                    linear: 100.0,
                                    angular: 20.0,
                                },
                                LockedAxes::TRANSLATION_LOCKED_Z
                                    | LockedAxes::ROTATION_LOCKED_X
                                    | LockedAxes::ROTATION_LOCKED_Y,
                                Transform::from_translation(translation),
                                Bijection(solution),
                                ColliderMassProperties::Mass(mass),
                            ));
                        }
                    }
                }
            }
            return Ok(());
        }

        if let Some((_, lpos)) = &mut *last {
            world.set_brush_line(*lpos, pos, brush.radius, GCell::Set);

            *lpos = pos;
        } else {
            *last = Some((GCell::Set, pos));

            world.set_brush(pos, brush.radius, GCell::Set);
        }
    };
    Ok(())
}

pub fn draw_selection(mut gizmos: Gizmos, selection: Res<Selection>) {
    let Some((start, end)) = selection.0 else {
        return;
    };

    let min = start.min(end).as_vec2();
    let max = start.max(end).as_vec2();
    gizmos.rect_2d(
        Isometry2d::from_translation((min + max) * 0.5),
        max - min + Vec2::ONE,
        Color::srgb(0.2, 0.8, 1.0),
    );
}
