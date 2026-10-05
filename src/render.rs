use bevy::{camera::ScalingMode, math::CompassQuadrant, prelude::*};

use crate::grid::{Bijection, ConstGrid, DynGrid, GCell, Grid, ObjGrid, SIZE, WorldGrid};
use ndshape::Shape;

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

pub fn render_world(
    mut commands: Commands,
    mut quads: Query<(Entity, &mut Transform), With<Quad>>,
    world: Single<&ConstGrid, With<WorldGrid>>,
    obj: Query<(&DynGrid, &Bijection, &GlobalTransform), With<ObjGrid>>,
    template: Res<QuadTemplate>,
    mut gizmos: Gizmos,
) {
    let mut recycle = quads.iter_mut();
    let mut quad = |pos: IVec2| {
        if let Some((_, mut recycle)) = recycle.next() {
            recycle.translation = pos.extend(0).as_vec3();
        } else {
            commands.spawn((
                Quad,
                Transform::from_translation(pos.extend(0).as_vec3()),
                template.0.clone(),
            ));
        }
    };
    for pos in world.iter_set_positions() {
        quad(pos)
    }

    draw_grid_seams(world.into_inner(), &mut gizmos, |point| point);

    for (grid, bijection, transform) in obj.iter() {
        for cell in grid.iter_set_positions() {
            if let Some(dst) = bijection.0.src_to_dst(cell.as_uvec2()) {
                quad(dst)
            }
        }
        draw_object_seams(grid, bijection, transform, &mut gizmos);
    }

    for (e, _) in recycle {
        commands.entity(e).despawn();
    }
}

const FACES: [(CompassQuadrant, IVec2); 4] = [
    (CompassQuadrant::North, IVec2::NEG_Y),
    (CompassQuadrant::East, IVec2::X),
    (CompassQuadrant::South, IVec2::Y),
    (CompassQuadrant::West, IVec2::NEG_X),
];

fn should_draw_seam<S: Shape<2, Coord = u32>>(
    grid: &Grid<S>,
    pos: UVec2,
    direction: CompassQuadrant,
    offset: IVec2,
) -> bool {
    let neighbor_is_set = pos
        .checked_add_signed(offset)
        .and_then(|neighbor| grid.get(neighbor))
        == Some(&GCell::Set);

    grid.is_connected(pos, direction) != Some(true)
        && !(neighbor_is_set && matches!(direction, CompassQuadrant::North | CompassQuadrant::West))
}

fn draw_grid_seams<S: Shape<2, Coord = u32>>(
    grid: &Grid<S>,
    gizmos: &mut Gizmos,
    map_point: impl Fn(Vec2) -> Vec2,
) {
    for cell in grid.iter_set_positions() {
        let pos = cell.as_uvec2();
        let center = cell.as_vec2();
        for (direction, offset) in FACES {
            if !should_draw_seam(grid, pos, direction, offset) {
                continue;
            }

            let normal = offset.as_vec2();
            let tangent = Vec2::new(-normal.y, normal.x) * 0.5;
            let midpoint = center + normal * 0.5;
            gizmos.line_2d(
                map_point(midpoint - tangent),
                map_point(midpoint + tangent),
                Color::BLACK,
            );
        }
    }
}

fn draw_object_seams(
    grid: &DynGrid,
    bijection: &Bijection,
    transform: &GlobalTransform,
    gizmos: &mut Gizmos,
) {
    for cell in grid.iter_set_positions() {
        let pos = cell.as_uvec2();
        let Some(center) = bijection.0.src_to_dst(pos).map(|point| point.as_vec2()) else {
            continue;
        };

        for (direction, offset) in FACES {
            if !should_draw_seam(grid, pos, direction, offset) {
                continue;
            }

            let normal = pos
                .checked_add_signed(offset)
                .and_then(|neighbor| bijection.0.src_to_dst(neighbor))
                .map(|neighbor| (neighbor.as_vec2() - center).normalize_or_zero())
                .filter(|normal| *normal != Vec2::ZERO)
                .unwrap_or_else(|| {
                    (transform.rotation() * offset.as_vec2().extend(0.0)).truncate()
                });
            let tangent = Vec2::new(-normal.y, normal.x) * 0.5;
            let midpoint = center + normal * 0.5;
            gizmos.line_2d(midpoint - tangent, midpoint + tangent, Color::BLACK);
        }
    }
}
