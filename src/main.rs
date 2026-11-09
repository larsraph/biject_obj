use bevy::{math::CompassQuadrant, prelude::*, transform::TransformSystems};
use bevy_rapier3d::prelude::*;

mod grid;
mod input;
mod projection;
mod render;
mod simulation;

use crate::{
    grid::{
        Bijection, Collisions, Grid, WorldGrid, handle_grid_collisions, on_world_grid_collision,
        recalculate_bijection, spawn_edge_colliders, spawn_world_grid, update_colliders,
    },
    input::{
        Brush, MouseTarget, Selection, draw_selection, edit, move_mouse_rigid_body,
        update_mouse_rigid_body,
    },
    render::{
        QuadTemplate, render_mouse_follower, render_setup, render_world, setup_grid_counter,
        update_grid_counter,
    },
};

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.55, 0.65, 0.8)))
        .add_plugins((
            DefaultPlugins.set(render::square_window()),
            RapierPhysicsPlugin::<NoUserData>::default(),
        ))
        .init_resource::<QuadTemplate>()
        .init_resource::<Brush>()
        .init_resource::<Selection>()
        .init_resource::<MouseTarget>()
        .init_resource::<Collisions>()
        .add_systems(
            Startup,
            (
                spawn_world_grid,
                spawn_edge_colliders,
                render_setup,
                setup_grid_counter,
            ),
        )
        // Apply edits and rebuild colliders before Rapier synchronizes in PostUpdate.
        .add_systems(
            Update,
            (update_mouse_rigid_body, (edit, update_colliders).chain()),
        )
        .add_systems(
            PostUpdate,
            move_mouse_rigid_body.before(PhysicsSet::SyncBackend),
        )
        .add_systems(
            PostUpdate,
            (on_world_grid_collision, handle_grid_collisions)
                .chain()
                .after(PhysicsSet::StepSimulation)
                .before(PhysicsSet::Writeback),
        )
        .add_systems(
            PostUpdate,
            (
                recalculate_bijection,
                repossess_sleeping,
                (
                    render_world,
                    draw_selection,
                    render_mouse_follower,
                    update_grid_counter,
                ),
            )
                .chain()
                .after(PhysicsSet::Writeback)
                .before(TransformSystems::Propagate),
        )
        .run();
}

pub(crate) trait All: Sized + 'static {
    const ALL: &[Self];
}

impl All for CompassQuadrant {
    const ALL: &[Self] = &[
        CompassQuadrant::North,
        CompassQuadrant::South,
        CompassQuadrant::East,
        CompassQuadrant::West,
    ];
}

pub(crate) trait ToIVec2 {
    fn ivec2(&self) -> IVec2;
}

impl ToIVec2 for CompassQuadrant {
    fn ivec2(&self) -> IVec2 {
        match self {
            CompassQuadrant::North => IVec2::Y,
            CompassQuadrant::East => IVec2::X,
            CompassQuadrant::South => IVec2::NEG_Y,
            CompassQuadrant::West => IVec2::NEG_X,
        }
    }
}

fn repossess_sleeping(
    mut commands: Commands,
    world: Single<&mut Grid, With<WorldGrid>>,
    query: Query<(Entity, &Grid, &Bijection, &Sleeping), Without<WorldGrid>>,
) {
    let mut world = world.into_inner();
    for (entity, grid, bijection, sleeping) in &query {
        if sleeping.sleeping && repossess_grid(&mut world, grid, bijection) {
            commands.entity(entity).despawn();
        }
    }
}

fn repossess_grid(world: &mut Grid, grid: &Grid, bijection: &Bijection) -> bool {
    // Validate the entire projection before modifying the world or despawning.
    let mut cells = bevy::platform::collections::HashMap::<UVec2, UVec2>::default();
    for src in grid.iter_set_positions() {
        let Some(dst) = bijection.0.src_to_dst(src) else {
            return false;
        };
        if dst.cmplt(IVec2::ZERO).any() || !world.contains(dst.as_uvec2()) {
            return false;
        }
        cells.insert(dst.as_uvec2(), src);
    }

    // Paste occupancy without bonding the object to existing terrain.
    for (&dst, &src) in &cells {
        world[dst] = grid[src];
        for &direction in CompassQuadrant::ALL {
            world.set_edge(dst, direction, false);
        }
    }
    // Restore internal bonds, including broken ones, in projected coordinates.
    for (&dst, &src) in &cells {
        for direction in [CompassQuadrant::East, CompassQuadrant::North] {
            let neighbor = dst.wrapping_add_signed(direction.ivec2());
            let Some(&other_src) = cells.get(&neighbor) else {
                continue;
            };
            let source_direction = CompassQuadrant::ALL
                .iter()
                .copied()
                .find(|dir| src.as_ivec2() + dir.ivec2() == other_src.as_ivec2());
            let connected = source_direction.is_some_and(|dir| grid.get_edge(src, dir));
            world.set_edge(dst, direction, connected);
        }
    }
    true
}

#[cfg(test)]
mod repossession_tests {
    use super::*;
    use ndshape::RuntimeShape;

    #[test]
    fn repossession_rotates_bonds_and_preserves_broken_edges() {
        let mut object = Grid::new(RuntimeShape::<u32, 2>::new([2, 2]));
        for y in 0..2 {
            for x in 0..2 {
                object.paint_cell(UVec2::new(x, y), Some(()));
            }
        }
        object.set_edge(UVec2::ZERO, CompassQuadrant::East, false);
        let bijection = Bijection(projection::solve(
            &object,
            Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
            bevy::math::Vec3A::new(4.0, 1.0, 0.0),
        ));
        let mut world = Grid::new(RuntimeShape::<u32, 2>::new([6, 6]));
        // Source (0,0) projects to (3,1); source east becomes world north.
        world.paint_cell(UVec2::new(4, 1), Some(()));
        assert!(repossess_grid(&mut world, &object, &bijection));
        assert!(!world.get_edge(UVec2::new(3, 1), CompassQuadrant::North));
        assert!(world.get_edge(UVec2::new(3, 1), CompassQuadrant::West));
        assert!(!world.get_edge(UVec2::new(3, 1), CompassQuadrant::East));
        assert_eq!(world.iter_set_positions().count(), 5);
    }

    #[test]
    fn repossession_outside_world_does_not_partially_paste() {
        let mut object = Grid::new(RuntimeShape::<u32, 2>::new([2, 1]));
        object.paint_cell(UVec2::ZERO, Some(()));
        object.paint_cell(UVec2::X, Some(()));
        let bijection = Bijection(projection::solve(
            &object,
            Quat::IDENTITY,
            bevy::math::Vec3A::new(-1.0, 0.0, 0.0),
        ));
        let mut world = Grid::new(RuntimeShape::<u32, 2>::new([2, 2]));
        assert!(!repossess_grid(&mut world, &object, &bijection));
        assert_eq!(world.iter_set_positions().count(), 0);
    }
}
