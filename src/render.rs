use bevy::{camera::ScalingMode, math::CompassQuadrant, prelude::*};
use bevy_rapier3d::prelude::{Collider, RigidBody};

use crate::{
    All as _, ToIVec2 as _,
    grid::{Bijection, Grid, ObjGrid, SIZE, WorldGrid},
    input::MouseFollower,
};

const PX_PER_CELL: u32 = 4;

pub fn square_window() -> WindowPlugin {
    WindowPlugin {
        primary_window: Some(Window {
            resolution: (SIZE * PX_PER_CELL + 2, SIZE * PX_PER_CELL + 2).into(),
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

type DisplayQuadFilter = (With<Quad>, Without<ObjGrid>);

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
        Transform::from_xyz(SIZE as f32 / 2.0, SIZE as f32 / 2.0, 0.0),
    ));
}

#[derive(Component)]
pub struct DynamicGridCount;

pub fn setup_grid_counter(mut commands: Commands) {
    commands.spawn((
        DynamicGridCount,
        Text::new("Dynamic grids: 0"),
        TextFont {
            font_size: bevy::text::FontSize::Px(20.0),
            ..default()
        },
        TextColor(Color::BLACK),
        Node {
            position_type: PositionType::Absolute,
            left: px(12.0),
            top: px(12.0),
            ..default()
        },
    ));
}

pub fn update_grid_counter(
    grids: Query<&RigidBody, With<Grid>>,
    mut labels: Query<&mut Text, With<DynamicGridCount>>,
) {
    let count = grids
        .iter()
        .filter(|body| **body == RigidBody::Dynamic)
        .count();
    let label = format!("Dynamic grids: {count}");
    for mut text in &mut labels {
        if text.0 != label {
            text.0.clone_from(&label);
        }
    }
}

pub fn render_world(
    mut commands: Commands,
    mut quads: Query<(Entity, &mut Transform), DisplayQuadFilter>,
    world: Single<&Grid, With<WorldGrid>>,
    obj: Query<(&Grid, &Bijection, &Transform), With<ObjGrid>>,
    template: Res<QuadTemplate>,
    mut gizmos: Gizmos,
) {
    let mut recycle = quads.iter_mut();
    let mut draw_quad = |pos: IVec2| {
        let center = (pos.as_vec2() + Vec2::splat(0.5)).extend(0.0);
        if let Some((_, mut recycle)) = recycle.next() {
            recycle.translation = center;
        } else {
            commands.spawn((
                Quad,
                Transform::from_translation(center),
                template.0.clone(),
            ));
        }
    };

    for pos in world.iter_set_positions() {
        draw_quad(pos.as_ivec2())
    }

    draw_grid_seams(world.into_inner(), &mut gizmos);

    for (grid, bijection, transform) in obj.iter() {
        for cell in grid.iter_set_positions() {
            if let Some(dst) = bijection.0.src_to_dst(cell) {
                draw_quad(dst)
            }
        }
        draw_object_seams(grid, &mut gizmos, bijection, transform);
    }

    for (e, _) in recycle {
        commands.entity(e).despawn();
    }
}

/// Shows the actual physics body, including when collisions keep it off the cursor.
pub fn render_mouse_follower(
    followers: Query<(&Transform, &Collider), With<MouseFollower>>,
    mut gizmos: Gizmos,
) {
    for (transform, collider) in &followers {
        if let Some(ball) = collider.as_ball() {
            gizmos.circle_2d(
                Isometry2d::from_translation(transform.translation.truncate()),
                ball.radius(),
                Color::srgb(1.0, 0.2, 0.1),
            );
        }
    }
}

fn should_draw_seam(grid: &Grid, pos: UVec2, direction: CompassQuadrant) -> bool {
    let primary_render = !matches!(direction, CompassQuadrant::North | CompassQuadrant::East);
    let neighbor = pos.wrapping_add_signed(direction.ivec2());
    let is_set = grid.contains(neighbor) && grid[neighbor].is_some();
    if !primary_render && is_set {
        return false;
    }
    let is_connected = grid.get_edge(pos, direction);
    !is_connected
}

fn draw_grid_seams(grid: &Grid, gizmos: &mut Gizmos) {
    for pos in grid.iter_set_positions() {
        let center = pos.as_vec2() + Vec2::splat(0.5);
        for direction in CompassQuadrant::ALL {
            if !should_draw_seam(grid, pos, *direction) {
                continue;
            }

            let half_normal = direction.ivec2().as_vec2() * 0.5;
            let midpoint = center + half_normal;
            let tangent = half_normal.perp();
            gizmos.line_2d(midpoint - tangent, midpoint + tangent, Color::BLACK);
        }
    }
}

fn draw_object_seams(
    grid: &Grid,
    gizmos: &mut Gizmos,
    bijection: &Bijection,
    transform: &Transform,
) {
    for pos in grid.iter_set_positions() {
        let Some(center) = bijection
            .0
            .src_to_dst(pos)
            .map(|point| point.as_vec2() + Vec2::splat(0.5))
        else {
            continue;
        };

        for direction in CompassQuadrant::ALL {
            if !should_draw_seam(grid, pos, *direction) {
                continue;
            }

            let neighbor = pos.wrapping_add_signed(direction.ivec2());
            let neighbor_dst = match bijection.0.src_to_dst(neighbor) {
                Some(neighbor) => {
                    let normal =
                        (neighbor.as_vec2() + Vec2::splat(0.5) - center).normalize_or_zero();
                    (normal != Vec2::ZERO).then_some(normal)
                }
                None => None,
            }
            .unwrap_or_else(|| {
                (transform.rotation * direction.ivec2().as_vec2().extend(0.0)).truncate()
            });
            let tangent = neighbor_dst.perp() * 0.5;
            let midpoint = center + neighbor_dst * 0.5;
            gizmos.line_2d(midpoint - tangent, midpoint + tangent, Color::BLACK);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndshape::RuntimeShape;

    #[test]
    fn seam_checks_handle_outer_edges_and_disconnected_cells() {
        let mut grid = Grid::new(RuntimeShape::<u32, 2>::new([2, 2]));
        grid.set_brush(Vec2::ZERO, 1, Some(()));
        assert!(!should_draw_seam(
            &grid,
            UVec2::ZERO,
            CompassQuadrant::North
        ));
        assert!(!should_draw_seam(&grid, UVec2::ZERO, CompassQuadrant::East));
        for &direction in CompassQuadrant::ALL {
            assert_eq!(
                should_draw_seam(&grid, UVec2::ONE, direction),
                matches!(direction, CompassQuadrant::North | CompassQuadrant::East),
            );
        }
        grid.set_edge(UVec2::ZERO, CompassQuadrant::North, false);
        assert!(should_draw_seam(&grid, UVec2::Y, CompassQuadrant::South));
    }
}
