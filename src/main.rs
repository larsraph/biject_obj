use bevy::prelude::*;

mod grid;
mod input;
mod projection;
mod render;

use crate::{
    grid::WorldGrid,
    input::{Brush, edit},
    render::{QuadTemplate, render, render_setup},
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(render::square_window()))
        .init_resource::<QuadTemplate>()
        .init_resource::<WorldGrid>()
        .init_resource::<Brush>()
        .add_systems(Startup, render_setup)
        .add_systems(Update, (edit, render).chain())
        .run();
}
