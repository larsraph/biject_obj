use avian3d::PhysicsPlugins;
use bevy::prelude::*;

mod grid;
mod input;
mod projection;
mod render;

use crate::{
    grid::{GCell, Grid, WorldGrid, recalculate_bijection, spawn_world_grid, update_colliders},
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

use ndshape::Shape;

const MAX_TRANSFER: f32 = 20.;

struct Scratch {
    // each cell stores it's velocity
    cell: Vec<Vec2>,
    // each cell stores how much velocity has been transferred between each of it's faces
    face: Vec<[Vec2; 2]>,
}

fn solve<S: Shape<2, Coord = u32>>(
    grid: &mut Grid<S>,
    scratch: &mut Scratch,
    impulse: Vec2,
    pos: Vec2,
    n: usize,
) {
    // initalize the scratch
    scratch.cell.clear();
    scratch.face.clear();
    scratch.cell.resize(grid.shape.usize(), Vec2::ZERO);
    scratch.face.resize(grid.shape.usize(), [Vec2::ZERO; 2]);

    // add impulse
    let pos = pos.as_ivec2().as_uvec2();
    let idx = grid.linearize(pos).unwrap();
    scratch.cell[idx] = impulse;

    // iterativly solve
    for _ in 0..n {
        // for every constraint try to equalize the velocities
        let [x_range, y_range] = grid.shape.as_array();
        for y in 0..y_range {
            for x in 0..x_range {
                let pos = UVec2::new(x, y);
                if matches!(grid[pos], GCell::Unset) {
                    continue;
                }
                for is_y in [true, false] {
                    // skip the faces facing into the boundary
                    if is_y && y == y_range - 1 || !is_y && x == x_range - 1 {
                        continue;
                    }

                    let step = if is_y { IVec2::Y } else { IVec2::X };
                    let a = pos;
                    let b = pos.wrapping_add_signed(step);

                    if matches!(grid[b], GCell::Unset) {
                        continue;
                    }

                    let a_idx = grid.linearize(a).unwrap();
                    let b_idx = grid.linearize(b).unwrap();

                    let vel_a = scratch.cell[a_idx];
                    let vel_b = scratch.cell[b_idx];

                    // TODO: figure out how to make cohesion and pulling apart work
                    // differently/more realistically. I'm thinking the method below
                    // isn't the best. I probably want to setup a type of ratio funciton?
                    // Or actually part of the reason I did it piecewise in my head was becasue
                    // I wanted all the velocity to transfer at once. if I don't do it piece-wise
                    // then the velocity transfer could just happen next frame? But that feels counter-
                    // intuitive since I would expect a body to feel the same amount of rigid either way...
                    // OR heres a better way of thinking about it:
                    // When bonds break (aka above the maximum), we can only transfer velocity via non-cohesion
                    // therefore velocity transfer is based on the dot product where if the doc product is negative
                    // it's zero.

                    let dot = vel_a.dot(vel_b);
                    if dot > 0. {
                        // completely rigid collision
                        let half_diff = (vel_a - vel_b) / 2.;
                        scratch.cell[a_idx] -= half_diff;
                        scratch.cell[b_idx] += half_diff;
                        scratch.face[a_idx][is_y as usize] += half_diff;
                    } else if grid.is_connected(pos, is_y).unwrap() {
                        // cohesion
                        let transferred = &mut scratch.face[a_idx][is_y as usize];

                        let half_diff = (vel_a - vel_b) / 2.;
                        let maxed_diff = if half_diff == Vec2::ZERO {
                            Vec2::ZERO
                        } else {
                            max_b_len_d(*transferred, half_diff, MAX_TRANSFER) * half_diff
                        };

                        scratch.cell[a_idx] -= maxed_diff;
                        scratch.cell[b_idx] += maxed_diff;
                        scratch.face[a_idx][is_y as usize] += maxed_diff;
                    }
                }
            }
        }
    }

    // detect separations
    //
    // for now we're just taking all that transfered more than MAX_TRANSFER
    // and breaking the bonds... however it may make more sense to have collision
    // based velocity transfer not break bonds.
    for (idx, [x, y]) in scratch.face.iter().enumerate() {
        if x.length() >= MAX_TRANSFER - 1e-6 {
            grid.set_connection(idx * 2, false);
        }
        if y.length() >= MAX_TRANSFER - 1e-6 {
            grid.set_connection(idx * 2 + 1, false);
        }
    }

    // detect all blocks of cells NOT connected to the boundary that have non-zero velocity.
}

// find c_max such that ||a + cb|| <= d
//
// AI made this  and it's an extrapolation of the length squared formula + quadratic formula
fn max_b_len_d(a: Vec2, b: Vec2, d: f32) -> f32 {
    let aa = a.dot(a);
    let ab = a.dot(b);
    let bb = b.dot(b);

    let disc = ab * ab - bb * (aa - d * d);

    let c = ((-ab + disc.sqrt()) / bb).clamp(0.0, 1.0);
    c
}
