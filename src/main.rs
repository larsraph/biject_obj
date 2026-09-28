use avian3d::PhysicsPlugins;
use bevy::prelude::*;

mod grid;
mod input;
mod projection;
mod render;

use crate::{
    grid::{WorldGrid, recalculate_bijection, spawn_world_grid, update_colliders},
    input::{Brush, Selection, draw_selection, edit},
    render::{QuadTemplate, render_setup, render_world},
};

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(render::square_window()),
            PhysicsPlugins::default(),
        ))
        .init_resource::<QuadTemplate>()
        .init_resource::<Brush>()
        .init_resource::<Selection>()
        .add_systems(Startup, (spawn_world_grid, render_setup))
        .add_systems(
            Update,
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

fn solve(_grid: &WorldGrid) {}
