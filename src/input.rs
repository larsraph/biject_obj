use bevy::prelude::*;

use crate::grid::{GCell, WorldGrid};

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

pub fn edit(
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform)>,
    mut world: Single<&mut WorldGrid>,
    mbi: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut brush: ResMut<Brush>,
    mut last: Local<Option<(GCell, Vec2)>>,
    mut selection_start: Local<Option<IVec2>>,
) -> Result<(), BevyError> {
    if keyboard.just_pressed(KeyCode::Equal) || keyboard.just_pressed(KeyCode::NumpadAdd) {
        brush.radius += 1;
    }
    if keyboard.just_pressed(KeyCode::Minus) || keyboard.just_pressed(KeyCode::NumpadSubtract) {
        brush.radius = (brush.radius - 1).max(1);
    }

    let lmb = mbi.pressed(MouseButton::Left);
    let rmb = mbi.pressed(MouseButton::Right);
    let mmb = mbi.pressed(MouseButton::Middle);
    let mmb_released = mbi.just_released(MouseButton::Middle);

    if !lmb && !rmb && !mmb && !mmb_released {
        *last = None;
        return Ok(());
    }

    let (camera, camera_transform) = camera.into_inner();
    if let Some(cursor_position) = window.cursor_position() {
        let pos = camera
            .viewport_to_world_2d(camera_transform, cursor_position)
            .with_severity(Severity::Warning)?;

        if mmb || mmb_released {
            *last = None;

            if mmb && selection_start.is_none() {
                *selection_start = Some(pos.round().as_ivec2());
            }
            if mmb_released {
                if let Some(start) = selection_start.take() {
                    let _ = world.extract(start, pos.round().as_ivec2());
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
