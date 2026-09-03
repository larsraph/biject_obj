use bevy::{camera::ScalingMode, math::USizeVec2, prelude::*};
use ndshape::{ConstPow2Shape2usize, ConstShape as _};

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
        .add_systems(Startup, setup)
        .add_systems(Update, sync_grid)
        .run();
}

const SIZE_LOG2: usize = 8;
const SIZE: usize = 1 << SIZE_LOG2;
const SIZE_POW2: usize = SIZE * SIZE;
type Shape = ConstPow2Shape2usize<SIZE_LOG2, SIZE_LOG2>;

#[derive(Component)]
struct Grid {
    exists: [bool; SIZE_POW2],
}

impl Grid {
    fn iter_set_positions(&self) -> impl Iterator<Item = IVec2> + '_ {
        (0..SIZE)
            .flat_map(|y| (0..SIZE).map(move |x| USizeVec2::new(x, y)))
            .filter(|pos| self.exists[Shape::linearize(pos.to_array())])
            .map(|pos| pos.as_ivec2())
    }
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
    let mut grid = Grid {
        exists: [false; SIZE_POW2],
    };
    // debug positions for now
    grid.exists[25] = true;
    grid.exists[25 + SIZE] = true;
    grid.exists[14298] = true;
    grid.exists[14299] = true;
    grid.exists[14300] = true;
    grid.exists[14301] = true;
    commands.spawn(grid);

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

fn sync_grid(
    mut commands: Commands,
    mut quads: Query<(Entity, &mut Transform), With<Quad>>,
    grid: Single<&Grid>,
    template: Res<QuadTemplate>,
) {
    let mut new = grid.iter_set_positions();
    let mut recycle = quads.iter_mut();

    for (pos, (_, mut re_trgt)) in (&mut new).zip(&mut recycle) {
        re_trgt.translation = pos.extend(0).as_vec3();
    }

    // We either despawn to set inactive. Despawning is better unless you
    // have non-copy or persistent data.
    for (e, _) in recycle {
        commands.entity(e).despawn();
    }

    for pos in new {
        commands.spawn((
            Quad,
            Transform::from_translation(pos.extend(0).as_vec3()),
            template.0.clone(),
        ));
    }
}
