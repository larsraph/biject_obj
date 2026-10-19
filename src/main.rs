use bevy::{math::CompassQuadrant, prelude::*};
use bevy_rapier3d::prelude::*;
use bitvec::vec::BitVec;

mod grid;
mod input;
mod projection;
mod render;

use crate::{
    grid::{
        Const, Dyn, DynGrid, GCell, Grid, recalculate_bijection, spawn_world_grid,
        update_colliders, world_grid_collision_start,
    },
    input::{
        Brush, MouseTarget, Selection, draw_selection, edit, move_mouse_rigid_body,
        clamp_velocities, update_mouse_rigid_body,
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
        .add_systems(Startup, (spawn_world_grid, render_setup))
        .add_systems(Update, update_mouse_rigid_body)
        .add_systems(
            PostUpdate,
            (
                move_mouse_rigid_body,
                clamp_velocities,
            )
                .chain()
                .before(PhysicsSet::SyncBackend),
        )
        .add_systems(
            PostUpdate,
            (
                (
                    world_grid_collision_start::<Const>,
                    world_grid_collision_start::<Dyn>,
                ),
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

const MAX_TRANSFER: f32 = 1.;

#[derive(Default)]
struct Scratch {
    // each cell stores it's velocity
    cell: Vec<Vec2>,
    // each cell stores how much velocity has been transferred between each of it's faces
    face: Vec<[Vec2; 2]>,
    visited: BitVec,
}

pub const ALL: [CompassQuadrant; 4] = [
    CompassQuadrant::North,
    CompassQuadrant::East,
    CompassQuadrant::South,
    CompassQuadrant::West,
];

fn initalize_scratch<S: Shape<2, Coord = u32> + Clone>(grid: &Grid<S>, scratch: &mut Scratch) {
    scratch.cell.clear();
    scratch.face.clear();
    scratch.visited.clear();
    scratch.cell.resize(grid.shape.usize(), Vec2::ZERO);
    scratch.face.resize(grid.shape.usize(), [Vec2::ZERO; 2]);
    scratch.visited.resize(grid.shape.usize(), false);
}

fn add_impulse<S: Shape<2, Coord = u32> + Clone>(
    grid: &Grid<S>,
    scratch: &mut Scratch,
    impulse: Vec2,
    pos: Vec2,
) {
    // Contact points lie on a voxel surface, so they can be just outside the
    // grid at its outer faces. Clamp them to the adjacent edge cell instead of
    // turning a negative coordinate into a huge unsigned value.
    let [width, height] = grid.shape.as_array();
    let max = IVec2::new(
        width.saturating_sub(1) as i32,
        height.saturating_sub(1) as i32,
    );
    let pos = pos.floor().as_ivec2().clamp(IVec2::ZERO, max).as_uvec2();
    let Some(idx) = grid.linearize(pos) else {
        return;
    };
    scratch.cell[idx] += impulse;
}

// TODO; on edit re-extract surrounding rigid blobs
fn solve<S: Shape<2, Coord = u32> + Clone>(
    grid: &mut Grid<S>,
    scratch: &mut Scratch,
    n: usize,
    anchors_boundary: bool,
) -> impl Iterator<Item = (DynGrid, Collider, Velocity)> {
    // iterativly solve
    for _ in 0..n {
        // for every constraint try to equalize the velocities
        let [x_range, y_range] = grid.shape.as_array();
        for y in 0..y_range {
            for x in 0..x_range {
                let pos = UVec2::new(x, y);
                if grid[pos] == GCell::Unset {
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

                    if grid[b] == GCell::Unset {
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

                    // example case I want to support
                    //
                    // a a a a
                    // a a a a
                    // _ a a a
                    // _ _ b b
                    //
                    // where cells in a are inter-connected and cells in b are inter-connected and connected to the boundary.
                    //
                    // What I would want to happen is for the constraint between a and b to be non-sticky AND
                    // for the a cells to be pulled up-left under gravity. However with our current system the b cells
                    // would just absorb all of the velocity of their a cells and zero them out because the b cells are connected
                    // to the boundary.
                    //
                    // thats because my current system tries to equalize velocity, which it is equalized across the entier a block.
                    // the key is that the interaction between a and b should be non-sticky AKA it needs to do something else.
                    // I'm just having trouble figuring out what it should do such that when we solve this we get the a blob
                    // as having a CCW rotation.
                    //
                    // Additionally thats in contrast to the system as such:
                    //
                    // a a a a
                    // a a a a
                    // a a a a
                    // b b b b
                    //
                    // where cells in a are inter-connected and cells in b are inter-connected and connected to the boundary.
                    //
                    // In this case I would expect for the a blob to end up with no conglomerate velocity. In order
                    // to end up with no conglomerate velocity then ideally there would be some translation between the a and b cells.
                    //
                    // Also to clarify this is under gravity so all the cells have a equal downward velocity.
                    //
                    //
                    // To summarize the above conflict:
                    // I wanted to support completely rigid simulation via sticky/rigid collisions
                    // at the same time I wanted to support rigid body simulation insdie the grid
                    // because I thought I had to always simiulate rigid body gravity in order for it
                    // to be correct. However the obvious answer is NOT to simulate gravity in the grid
                    // rather extract rigid blobs, run gravity, and then once they stop moving reproject
                    // them onto the grid... after this point we know gravity has no effect. The danger case
                    // is if cells are removed. We can probably handily simulate that by, on removal, re-extracting
                    // any rigid blobs directly adjacent to the cell, simulating them again. Then reprojecting
                    // once they stop moving.
                    //
                    // Another comment. Gravity cannot tear blobs apart because it's only simulated on entire
                    // rigid blobs at a time.

                    let dot = vel_a.dot(vel_b);
                    if dot > 0. {
                        // completely sticky/rigid collision
                        let half_diff = (vel_a - vel_b) / 2.;
                        scratch.cell[a_idx] -= half_diff;
                        scratch.cell[b_idx] += half_diff;
                        scratch.face[a_idx][is_y as usize] += half_diff;
                    } else if grid.is_connected_raw(pos, is_y).unwrap() {
                        // pulled apart has a cap and requires connection
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
    // based velocity transfer not break bonds?
    for (idx, [x, y]) in scratch.face.iter().enumerate() {
        if x.length() >= MAX_TRANSFER - 1e-6 {
            grid.connection.set(idx * 2, false);
        }
        if y.length() >= MAX_TRANSFER - 1e-6 {
            grid.connection.set(idx * 2 + 1, false);
        }
    }

    fn is_boundary(pos: UVec2, size: UVec2) -> bool {
        pos.cmpeq(UVec2::ZERO).any() || pos + UVec2::ONE == size
    }

    // detect all blobs of cells NOT connected to the boundary that have non-zero velocity.
    //
    // okay but theres a slight problem here. What about cells that aren't connected to the boundary? but are sitting still and
    // are stirred into motion by gravity? Well in the end what's happening is that the boundary cells work as velocity sinks and
    // then the simulation can use the constraints again and again to zero out the velocity of all cells... in other words that doesn't
    // make any fucking sense. The situation that arises that we must handle is a non-connected balanced blob that becomes unbalanced.
    // In order to test this we cannot act like the unconnected blob is rigidly connected to it's neighbors (which is what the above assumes)
    // which we used to elide large-scale rotation of a rigid body. I want to avoid large-scale rotation of a (cellular) rigid body in general but I
    // still need to somehow detect when it starts so I can extract it from the main_grid.
    let mut blobs = Vec::new();
    let [x_size, y_size] = grid.shape.as_array();
    let size = UVec2::new(x_size, y_size);
    for y in 0..y_size {
        for x in 0..x_size {
            let pos = UVec2::new(x, y);
            let idx = grid.shape.linearize(pos) as usize;
            if !scratch.visited[idx] {
                if scratch.cell[idx].abs().cmpge(Vec2::splat(1e-5)).any() {
                    fn flood<S: Shape<2, Coord = u32>>(
                        cell: UVec2,
                        grid: &Grid<S>,
                        scratch: &mut Scratch,
                        blob_vec: &mut Vec<IVec3>,
                        anchors_boundary: bool,
                        size: UVec2,
                    ) -> bool {
                        let mut touches_boundary = false;
                        for to in ALL {
                            let offset = match to {
                                CompassQuadrant::East => IVec2::new(1, 0),
                                CompassQuadrant::South => IVec2::new(0, 1),
                                CompassQuadrant::West => IVec2::new(-1, 0),
                                CompassQuadrant::North => IVec2::new(0, -1),
                            };
                            if let Some(target) = cell.checked_add_signed(offset)
                                && let Some(index) = grid.linearize(target)
                                && !scratch.visited[index as usize]
                                && grid.ty[index] == GCell::Set
                                && grid.is_connected(cell, to).unwrap()
                            {
                                scratch.visited.set(index as usize, true);
                                blob_vec.push(target.as_ivec2().extend(0));
                                touches_boundary |= anchors_boundary && is_boundary(target, size);
                                touches_boundary |=
                                    flood(target, grid, scratch, blob_vec, anchors_boundary, size);
                            }
                        }
                        touches_boundary
                    }
                    let mut blob_vec = Vec::new();
                    scratch.visited.set(idx, true);
                    blob_vec.push(pos.as_ivec2().extend(0));
                    // Always flood, including when the seed is on the boundary.
                    // Short-circuiting here leaves the seed's connected neighbors
                    // unvisited; they are then incorrectly reconsidered as a
                    // detached component later in this scan.
                    let mut touches_boundary = anchors_boundary && is_boundary(pos, size);
                    touches_boundary |= flood(
                        UVec2::new(x, y),
                        grid,
                        scratch,
                        &mut blob_vec,
                        anchors_boundary,
                        size,
                    );

                    if !touches_boundary {
                        let mut blob_grid = Grid::new(grid.shape.clone()).to_dyn();

                        // Copy occupancy and every internal bond before removing
                        // the component. The old traversal copied only its spanning
                        // tree, which silently fractured cycles in the new object.
                        for cell in &blob_vec {
                            let pos = cell.truncate().as_uvec2();
                            blob_grid[pos] = grid[pos];
                        }
                        for cell in &blob_vec {
                            let pos = cell.truncate().as_uvec2();
                            for (offset, is_y) in [(UVec2::X, false), (UVec2::Y, true)] {
                                let neighbor = pos + offset;
                                if blob_grid.contains(neighbor)
                                    && blob_grid[neighbor] == GCell::Set
                                    && grid.is_connected_raw(pos, is_y) == Some(true)
                                {
                                    let index = blob_grid.linearize_conn(pos, is_y).unwrap();
                                    blob_grid.connection.set(index, true);
                                }
                            }
                        }
                        for cell in &blob_vec {
                            grid.paint_cell(cell.truncate().as_uvec2(), GCell::Unset);
                        }
                        blobs.push((blob_vec, blob_grid));
                    }
                }
            }
        }
    }

    // accumulate rotation and linear velocity from cellular velocity for each blob
    blobs.into_iter().map(|(cells, blob_grid)| {
        // AI helped out here
        let collider = Collider::voxels(Vec3::ONE, &cells);
        // A unit voxel with density one has unit mass. Its center is at its
        // integer grid coordinate plus (0.5, 0.5, 0.5), which matches Rapier's
        // voxel convention and the former Avian calculation.
        let mass = cells.len() as f32;
        let cell_mass = 1.0;
        let center_of_mass = cells
            .iter()
            .map(|cell| cell.as_vec3() + Vec3::splat(0.5))
            .sum::<Vec3>()
            / mass;
        let mut angular_momentum = Vec3::ZERO;
        let mut linear_momentum = Vec3::ZERO;
        let [x, y] = blob_grid.shape.as_array();
        for y in 0..y {
            for x in 0..x {
                let pos = UVec2::new(x, y);
                if *blob_grid.get(pos).unwrap() == GCell::Set {
                    let idx = blob_grid.linearize(pos).unwrap();
                    let velocity = scratch.cell[idx].extend(0.0);
                    // `Collider::voxels` interprets an integer coordinate as the
                    // voxel's lower corner, so use its center for the moment arm.
                    let cell_center = (pos.as_vec2() + Vec2::splat(0.5)).extend(0.0);
                    let cell_momentum = cell_mass * velocity;

                    linear_momentum += cell_momentum;
                    angular_momentum += (cell_center - center_of_mass).cross(cell_momentum);
                }
            }
        }

        let linear_velocity = linear_momentum / mass;
        // All voxels lie in the XY plane, so only rotation about Z is relevant.
        let principal_angular_inertia_z = cells.iter().fold(0.0, |inertia, cell| {
            let center = cell.as_vec3() + Vec3::splat(0.5);
            // The moment of inertia of a unit-mass unit cube about Z through its
            // center is 1/6; add the parallel-axis contribution for the blob.
            inertia + 1.0 / 6.0 + (center - center_of_mass).truncate().length_squared()
        });
        let angular_velocity =
            Vec3::Z * (angular_momentum.z / principal_angular_inertia_z);

        (blob_grid, collider, Velocity {
            linear: linear_velocity,
            angular: angular_velocity,
        })
    })
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

/// AI made these
#[cfg(test)]
mod tests {
    use super::*;
    use ndshape::RuntimeShape;

    #[test]
    fn a_large_impulse_breaks_a_bond_and_extracts_the_detached_component() {
        let shape = RuntimeShape::<u32, 2>::new([4, 3]);
        let mut grid = Grid::new(shape);
        for pos in [UVec2::new(0, 1), UVec2::new(1, 1), UVec2::new(2, 1)] {
            grid.paint_cell(pos, GCell::Set);
        }

        let mut scratch = Scratch::default();
        initalize_scratch(&grid, &mut scratch);
        scratch.cell[grid.linearize(UVec2::new(2, 1)).unwrap()] = Vec2::X * 100.0;

        let fragments = solve(&mut grid, &mut scratch, 1, true).collect::<Vec<_>>();

        assert_eq!(fragments.len(), 1);
        assert_eq!(grid[UVec2::new(0, 1)], GCell::Set);
        assert_eq!(grid[UVec2::new(1, 1)], GCell::Set);
        assert_eq!(grid[UVec2::new(2, 1)], GCell::Unset);
        assert_eq!(fragments[0].0[UVec2::new(2, 1)], GCell::Set);
    }

    #[test]
    fn a_small_impulse_does_not_break_an_anchored_component() {
        let shape = RuntimeShape::<u32, 2>::new([4, 3]);
        let mut grid = Grid::new(shape);
        for pos in [UVec2::new(0, 1), UVec2::new(1, 1), UVec2::new(2, 1)] {
            grid.paint_cell(pos, GCell::Set);
        }

        let mut scratch = Scratch::default();
        initalize_scratch(&grid, &mut scratch);
        scratch.cell[grid.linearize(UVec2::new(2, 1)).unwrap()] = Vec2::X;

        let fragments = solve(&mut grid, &mut scratch, 10, true).collect::<Vec<_>>();
        assert!(fragments.is_empty());
        assert_eq!(grid[UVec2::new(2, 1)], GCell::Set);
    }
}
