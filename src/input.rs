use avian3d::dynamics::rigid_body::{
    AngularVelocity, LinearVelocity, LockedAxes, MaxLinearSpeed, RigidBody,
    mass_properties::components::Mass,
};
use bevy::prelude::*;

use crate::{
    grid::{Bijection, GCell, ObjGrid, WorldGrid},
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

pub fn edit(
    mut commands: Commands,
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform)>,
    mut world: Single<&mut WorldGrid>,
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
    let rmb = mbi.pressed(MouseButton::Right);
    let selecting = keyboard.any_pressed([KeyCode::ControlLeft, KeyCode::ControlRight]);
    let selection_ended =
        !selecting && keyboard.any_just_released([KeyCode::ControlLeft, KeyCode::ControlRight]);

    if !lmb && !rmb && !selecting && !selection_ended {
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
            let cell = pos.round().as_ivec2();

            if selecting {
                let (_, end) = selection.0.get_or_insert((cell, cell));
                *end = cell;
            }
            if selection_ended {
                if let Some((start, _)) = selection.0.take() {
                    let objgrid = world.extract(start, cell);
                    if let Some(objgrid) = objgrid {
                        let collider = objgrid.collider();
                        let mass = objgrid.mass();

                        let translation = start.min(cell).extend(0).as_vec3();
                        let solution =
                            solve(objgrid.shape.clone(), Quat::default(), translation.into());

                        let lin: Vec2 = rand::random();
                        let ang: f32 = rand::random();
                        let lin = (lin % 8.).extend(0.);
                        let ang = Vec3::new(0., 0., ang % 8.);

                        commands.spawn((
                            ObjGrid,
                            objgrid,
                            collider,
                            RigidBody::Dynamic,
                            MaxLinearSpeed(100.),
                            LockedAxes::new()
                                .lock_translation_z()
                                .lock_rotation_x()
                                .lock_rotation_y(),
                            LinearVelocity(lin),
                            AngularVelocity(ang),
                            Transform::from_translation(translation),
                            Bijection(solution),
                            Mass(mass),
                        ));
                    }
                }
            }
            return Ok(());
        }

        if let Some((mode, lpos)) = &mut *last {
            // Swap the mode if neccecary
            match (lmb, rmb, *mode) {
                (true, false, GCell::Unset) => *mode = GCell::Set,
                (false, true, GCell::Set) => *mode = GCell::Unset,
                _ => {}
            }

            world.set_brush_line(*lpos, pos, brush.radius, *mode);

            *lpos = pos;
        } else {
            // prioritize LMB
            // because lmb == false and lmb || rmb == true then rmb == true
            let mode = if lmb { GCell::Set } else { GCell::Unset };
            *last = Some((mode, pos));

            world.set_brush(pos, brush.radius, mode);
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
