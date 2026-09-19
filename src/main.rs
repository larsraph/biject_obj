use avian3d::PhysicsPlugins;
use bevy::prelude::*;

mod grid;
mod input;
mod projection;
mod render;

use crate::{
    grid::{spawn_world_grid, update_world_grid_collider},
    input::{Brush, edit},
    render::{QuadTemplate, render, render_setup},
};

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(render::square_window()),
            PhysicsPlugins::default(),
        ))
        .init_resource::<QuadTemplate>()
        .init_resource::<Brush>()
        .add_systems(Startup, (spawn_world_grid, render_setup))
        .add_systems(Update, (edit, update_world_grid_collider, render).chain())
        .run();
}
