use bevy::{camera::ScalingMode, prelude::*};

mod grid;
mod octree;
mod projection;

use grid::*;
use projection::*;

const PX_PER_CELL: u32 = 4;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                resolution: (SIZE as u32 * PX_PER_CELL, SIZE as u32 * PX_PER_CELL).into(),
                ..default()
            }),
            ..default()
        }))
        .init_resource::<QuadTemplate>()
        .init_resource::<WorldGrid>()
        .add_systems(Startup, setup)
        .add_systems(Update, (input, game_of_life, render_grid).chain())
        .run();
}

#[derive(Bundle, Clone)]
struct QuadBundle {
    material: MeshMaterial2d<ColorMaterial>,
    mesh: Mesh2d,
}

impl FromWorld for QuadBundle {
    fn from_world(world: &mut World) -> Self {
        let asset_server = world.resource::<AssetServer>();
        QuadBundle {
            material: MeshMaterial2d(asset_server.add(Color::WHITE.into())),
            mesh: Mesh2d(asset_server.add(Rectangle::from_length(1.).into())),
        }
    }
}

#[derive(Resource, FromWorld)]
struct QuadTemplate(QuadBundle);

#[derive(Component)]
struct Quad;

fn setup(mut commands: Commands) {
    commands.spawn((
        Camera2d,
        Projection::Orthographic(OrthographicProjection {
            scaling_mode: ScalingMode::Fixed {
                width: SIZE as f32,
                height: SIZE as f32,
            },
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(SIZE as f32 / 2.0 - 0.5, SIZE as f32 / 2.0 - 0.5, 0.0),
    ));
}

fn render_grid(
    mut commands: Commands,
    mut quads: Query<(Entity, &mut Transform), With<Quad>>,
    w_grid: Res<WorldGrid>,
    template: Res<QuadTemplate>,
) {
    let mut recycle = quads.iter_mut();
    for pos in w_grid.iter_set_positions() {
        if let Some((_, mut re_trgt)) = recycle.next() {
            re_trgt.translation = pos.extend(0).as_vec3();
        } else {
            commands.spawn((
                Quad,
                Transform::from_translation(pos.extend(0).as_vec3()),
                template.0.clone(),
            ));
        }
    }

    // We either despawn or set `Disabled`.
    // Despawning is better unless you have non-copy or persistent data
    // because it doesn't perform an archetype move.
    for (e, _) in recycle {
        commands.entity(e).despawn();
    }
}

fn input(
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform)>,
    mut w_grid: ResMut<WorldGrid>,
    mbi: Res<ButtonInput<MouseButton>>,
    mut last: Local<Option<(GCell, Vec2)>>,
) -> Result<(), BevyError> {
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

            w_grid.set_line(*lpos, pos, *mode);

            *lpos = pos;
        } else {
            // prioritize LMB
            // because lmb == false and lmb || rmb == true then rmb == true
            let mode = if lmb { GCell::Set } else { GCell::Unset };
            *last = Some((mode, pos));

            w_grid.set(pos.round().as_ivec2(), mode)
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
