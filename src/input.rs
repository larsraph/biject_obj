use bevy::{ecs::system::SystemParam, prelude::*};
use bevy_rapier3d::prelude::*;

use crate::{
    grid::{Bijection, Grid, ObjGrid, WorldGrid},
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
    let position = position.extend(0.5);
    target.0 = Some(position);

    if followers.is_empty() {
        commands.spawn((
            MouseFollower,
            RigidBody::Dynamic,
            Collider::ball(0.5),
            ActiveEvents::COLLISION_EVENTS,
            Ccd::enabled(),
            Transform::from_translation(position),
            Velocity::zero(),
            AdditionalMassProperties::Mass(2.),
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

#[derive(SystemParam)]
pub struct EditorState<'w, 's> {
    brush: ResMut<'w, Brush>,
    selection: ResMut<'w, Selection>,
    last: Local<'s, Option<Vec2>>,
}

pub fn edit(
    mut commands: Commands,
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform)>,
    mut world: Single<&mut Grid, With<WorldGrid>>,
    mbi: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    state: EditorState,
) -> Result<(), BevyError> {
    let EditorState {
        mut brush,
        mut selection,
        mut last,
    } = state;
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
            let current = pos.floor().as_ivec2();

            if selecting {
                let (_, end) = selection.0.get_or_insert((current, current));
                *end = current;
            }
            if selection_ended && let Some((start, _)) = selection.0.take() {
                let max = world.size().as_ivec2();
                let min = start.min(current).clamp(IVec2::ZERO, max).as_uvec2();
                let max = (start.max(current) + IVec2::ONE)
                    .clamp(IVec2::ZERO, max)
                    .as_uvec2();
                let rect = URect::from_corners(min, max);
                if let Some(objgrid) = world.extract_rect(rect)
                    && let Some(collider) = objgrid.build_collider()
                {
                    let translation = rect.min.as_vec2().extend(0.0);
                    let solution = solve(&objgrid, Quat::IDENTITY, translation.into());
                    commands.spawn((
                        ObjGrid,
                        objgrid,
                        collider,
                        RigidBody::Dynamic,
                        // Track the previous step's motion for later fragmentation.
                        Velocity::zero(),
                        LockedAxes::TRANSLATION_LOCKED_Z
                            | LockedAxes::ROTATION_LOCKED_X
                            | LockedAxes::ROTATION_LOCKED_Y,
                        Transform::from_translation(translation),
                        Bijection(solution),
                        ActiveEvents::COLLISION_EVENTS,
                    ));
                }
            }
            return Ok(());
        }

        if let Some(lpos) = &mut *last {
            world.set_brush_line(*lpos, pos, brush.radius, Some(()));

            *lpos = pos;
        } else {
            *last = Some(pos);

            world.set_brush(pos, brush.radius, Some(()));
        }
    } else if selection_ended {
        selection.0 = None;
    };
    Ok(())
}

pub fn draw_selection(mut gizmos: Gizmos, selection: Res<Selection>) {
    let Some((start, end)) = selection.0 else {
        return;
    };

    let min = start.min(end).as_vec2();
    let max = start.max(end).as_vec2() + Vec2::ONE;
    gizmos.rect_2d(
        Isometry2d::from_translation((min + max) * 0.5),
        max - min,
        Color::srgb(0.2, 0.8, 1.0),
    );
}
