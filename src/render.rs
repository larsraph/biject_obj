use bevy::{camera::ScalingMode, prelude::*};

use crate::grid::{SIZE, WorldGrid};

const PX_PER_CELL: u32 = 4;

pub fn square_window() -> WindowPlugin {
    WindowPlugin {
        primary_window: Some(Window {
            resolution: (SIZE as u32 * PX_PER_CELL + 2, SIZE as u32 * PX_PER_CELL + 2).into(),
            ..default()
        }),
        ..default()
    }
}

#[derive(Resource, FromWorld)]
pub struct QuadTemplate(QuadBundle);

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

#[derive(Component)]
pub struct Quad;

pub fn render_setup(mut commands: Commands) {
    commands.spawn((
        Camera2d,
        Projection::Orthographic(OrthographicProjection {
            scaling_mode: ScalingMode::Fixed {
                width: SIZE as f32 + 1.,
                height: SIZE as f32 + 1.,
            },
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(SIZE as f32 / 2.0 - 0.5, SIZE as f32 / 2.0 - 0.5, 0.0),
    ));
}

pub fn render(
    mut commands: Commands,
    mut quads: Query<(Entity, &mut Transform), With<Quad>>,
    world: Res<WorldGrid>,
    template: Res<QuadTemplate>,
) {
    if !world.is_changed() {
        return;
    }

    let mut recycle = quads.iter_mut();
    for pos in world.iter_set_positions() {
        if let Some((_, mut recycle)) = recycle.next() {
            recycle.translation = pos.extend(0).as_vec3();
        } else {
            commands.spawn((
                Quad,
                Transform::from_translation(pos.extend(0).as_vec3()),
                template.0.clone(),
            ));
        }
    }

    for (e, _) in recycle {
        commands.entity(e).despawn();
    }
}
