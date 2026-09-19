use bevy::prelude::*;

mod connectivity;
mod grid;
mod octree;
mod projection;
mod render;

use grid::*;

// this is a DX thing b/c it's annoying to have dead code warnings
pub use octree::Octree;
pub use projection::solve;

use crate::render::{render, render_setup};

const INITIAL_BRUSH_RADIUS: i32 = 4;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(render::square_window()))
        .init_resource::<WorldGrid>()
        .init_resource::<Brush>()
        .add_systems(Startup, render_setup)
        .add_systems(Update, (input, render).chain())
        .run();
}

#[derive(Resource)]
struct Brush {
    radius: i32,
}

impl Default for Brush {
    fn default() -> Self {
        Self {
            radius: INITIAL_BRUSH_RADIUS,
        }
    }
}

fn input(
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform)>,
    mut w_grid: Single<&mut WorldGrid>,
    mbi: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut brush: ResMut<Brush>,
    mut last: Local<Option<(GCell, Vec2)>>,
) -> Result<(), BevyError> {
    if keyboard.just_pressed(KeyCode::Equal) || keyboard.just_pressed(KeyCode::NumpadAdd) {
        brush.radius += 1;
    }
    if keyboard.just_pressed(KeyCode::Minus) || keyboard.just_pressed(KeyCode::NumpadSubtract) {
        brush.radius = (brush.radius - 1).max(1);
    }

    let lmb = mbi.pressed(MouseButton::Left);
    let rmb = mbi.pressed(MouseButton::Right);

    if !lmb && !rmb {
        *last = None;
        return Ok(());
    }

    let (camera, camera_transform) = camera.into_inner();
    if let Some(cursor_position) = window.cursor_position() {
        let pos = camera
            .viewport_to_world_2d(camera_transform, cursor_position)
            .with_severity(Severity::Warning)?;

        if let Some((mode, lpos)) = &mut *last {
            // Swap the mode if neccecary
            match (lmb, rmb, *mode) {
                (true, false, GCell::Unset) => *mode = GCell::Set,
                (false, true, GCell::Set) => *mode = GCell::Unset,
                _ => {}
            }

            w_grid.set_brush_line(*lpos, pos, brush.radius, *mode);

            *lpos = pos;
        } else {
            // prioritize LMB
            // because lmb == false and lmb || rmb == true then rmb == true
            let mode = if lmb { GCell::Set } else { GCell::Unset };
            *last = Some((mode, pos));

            w_grid.set_brush(pos, brush.radius, mode);
        }
    };
    Ok(())
}

fn game_of_life(mut w_grid: ResMut<WorldGrid>) {
    let previous = w_grid.clone();

    for y in 0..SIZE {
        for x in 0..SIZE {
            let pos = IVec2::new(x as i32, y as i32);
            let cell = previous.get(pos);
            let mut neighbor_count = 0;

            for offset in [
                IVec2::new(-1, -1),
                IVec2::new(0, -1),
                IVec2::new(1, -1),
                IVec2::new(-1, 0),
                IVec2::new(1, 0),
                IVec2::new(-1, 1),
                IVec2::new(0, 1),
                IVec2::new(1, 1),
            ] {
                let neighbor_pos = pos + offset;
                if neighbor_pos.cmplt(IVec2::ZERO).any()
                    || neighbor_pos.cmpge(IVec2::splat(SIZE as i32)).any()
                {
                    continue;
                }
                if previous.get(neighbor_pos) == GCell::Set {
                    neighbor_count += 1;
                }
            }

            let new_cell = match cell {
                GCell::Unset => {
                    if neighbor_count == 3 {
                        GCell::Set
                    } else {
                        GCell::Unset
                    }
                }
                GCell::Set => {
                    if neighbor_count == 2 || neighbor_count == 3 {
                        GCell::Set
                    } else {
                        GCell::Unset
                    }
                }
            };

            w_grid.set(pos, new_cell);
        }
    }
}
