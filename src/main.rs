use bevy::{math::CompassQuadrant, prelude::*};
use bevy_rapier3d::prelude::*;

mod grid;
mod input;
mod projection;
mod render;
mod simulation;

use crate::{
    grid::{
        Collisions, handle_grid_collisions, on_world_grid_collision, recalculate_bijection,
        spawn_edge_colliders, spawn_world_grid, update_colliders,
    },
    input::{
        Brush, MouseTarget, Selection, draw_selection, edit, move_mouse_rigid_body,
        update_mouse_rigid_body,
    },
    render::{QuadTemplate, render_setup, render_world},
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
            (spawn_world_grid, spawn_edge_colliders, render_setup),
        )
        .add_systems(Update, update_mouse_rigid_body)
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
                edit,
                update_colliders,
                render_world,
                draw_selection,
            )
                .chain(),
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
