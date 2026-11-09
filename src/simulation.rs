use bevy::{math::CompassQuadrant, prelude::*};
use bevy_rapier3d::prelude::*;
use bitvec::vec::BitVec;
use ndshape::{RuntimeShape, Shape};

use crate::{All as _, ToIVec2 as _, grid::Grid};

const MAX_TRANSFER: f32 = 50.;

#[derive(Default)]
pub(crate) struct Scratch {
    // Velocity per cell.
    cell_velocity: Vec<Vec2>,
    // Accumulated velocity transfer across the east and north faces.
    face: Vec<[Vec2; 2]>,
    cell_visited: BitVec,
}

pub(crate) fn initialize_scratch(size: usize, scratch: &mut Scratch) {
    scratch.cell_velocity.clear();
    scratch.face.clear();
    scratch.cell_visited.clear();
    scratch.cell_velocity.resize(size, Vec2::ZERO);
    scratch.face.resize(size, [Vec2::ZERO; 2]);
    scratch.cell_visited.resize(size, false);
}

pub(crate) fn add_impulse(grid: &Grid, scratch: &mut Scratch, impulse: Vec2, pos: Vec2) {
    // Contact points lie on a voxel surface, so they can be just outside the
    // grid at its outer faces. Clamp them to the adjacent edge cell instead of
    // turning a negative coordinate into a huge unsigned value.
    let [width, height] = grid.shape.as_array();
    let max = IVec2::new(
        width.saturating_sub(1) as i32,
        height.saturating_sub(1) as i32,
    );
    let pos = pos.floor().as_ivec2().clamp(IVec2::ZERO, max).as_uvec2();
    let Some(idx) = grid.linearize_checked(pos) else {
        return;
    };
    scratch.cell_velocity[idx] += impulse;
}

// TODO: Re-extract unsupported components after editing.
pub(crate) fn solve(
    grid: &mut Grid,
    scratch: &mut Scratch,
    iterations: usize,
    anchors_boundary: bool,
) -> impl Iterator<Item = (Grid, Collider, Velocity, Vec3)> {
    equalize_velocity(iterations, grid, scratch);
    break_bonds(scratch, grid);
    let mut blobs = Vec::new();
    extract_blobs(grid, scratch, anchors_boundary, &mut blobs);

    let source_shape = grid.shape;
    blobs.into_iter().map(move |(cells, origin, blob_grid)| {
        build_blob(cells, origin, blob_grid, source_shape, scratch)
    })
}

// Accumulate angular and linear momentum in each blob's local frame.
fn build_blob(
    cells: Vec<IVec3>,
    origin: UVec2,
    blob_grid: Grid,
    source_shape: RuntimeShape<u32, 2>,
    scratch: &Scratch,
) -> (Grid, Collider, Velocity, Vec3) {
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
            if blob_grid.get(pos).is_some() {
                let idx = source_shape.linearize(pos + origin) as usize;
                let velocity = scratch.cell_velocity[idx].extend(0.0);
                // `Collider::voxels` interprets an integer coordinate as the
                // voxel's lower corner, so use its center for the moment arm.
                let cell_center = (pos.as_vec2() + Vec2::splat(0.5)).extend(0.5);
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
    let angular_velocity = Vec3::Z * (angular_momentum.z / principal_angular_inertia_z);

    (
        blob_grid,
        collider,
        Velocity {
            linear: linear_velocity,
            angular: angular_velocity,
        },
        origin.as_vec2().extend(0.),
    )
}

fn equalize_velocity(iterations: usize, grid: &Grid, scratch: &mut Scratch) {
    // Iteratively equalize neighboring velocities.
    for _ in 0..iterations {
        // for every constraint try to equalize the velocities
        let [x_range, y_range] = grid.shape.as_array();
        for y in 0..y_range {
            for x in 0..x_range {
                let pos = UVec2::new(x, y);
                if grid[pos].is_none() {
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

                    if grid[b].is_none() {
                        continue;
                    }

                    let a_idx = grid.linearize(a);
                    let b_idx = grid.linearize(b);

                    let vel_a = scratch.cell_velocity[a_idx];
                    let vel_b = scratch.cell_velocity[b_idx];

                    let quadrant = if is_y {
                        CompassQuadrant::North
                    } else {
                        CompassQuadrant::East
                    };

                    let dot = vel_a.dot(vel_b);
                    if dot > 0. {
                        // completely sticky/rigid collision
                        let half_diff = (vel_a - vel_b) / 2.;
                        scratch.cell_velocity[a_idx] -= half_diff;
                        scratch.cell_velocity[b_idx] += half_diff;
                        scratch.face[a_idx][is_y as usize] += half_diff;
                    } else if grid.get_edge(pos, quadrant) {
                        // pulled apart has a cap and requires connection
                        let transferred = &mut scratch.face[a_idx][is_y as usize];

                        let half_diff = (vel_a - vel_b) / 2.;
                        let maxed_diff = if half_diff == Vec2::ZERO {
                            Vec2::ZERO
                        } else {
                            max_b_len_d(*transferred, half_diff, MAX_TRANSFER) * half_diff
                        };

                        scratch.cell_velocity[a_idx] -= maxed_diff;
                        scratch.cell_velocity[b_idx] += maxed_diff;
                        scratch.face[a_idx][is_y as usize] += maxed_diff;
                    }
                }
            }
        }
    }
}

fn break_bonds(scratch: &mut Scratch, grid: &mut Grid) {
    // Break bonds whose accumulated transfer reaches the limit.
    for (idx, [x, y]) in scratch.face.iter().enumerate() {
        if x.length() >= MAX_TRANSFER - 1e-6 {
            grid.edges.set(idx * 2, false);
        }
        if y.length() >= MAX_TRANSFER - 1e-6 {
            grid.edges.set(idx * 2 + 1, false);
        }
    }
}

fn extract_blobs(
    grid: &mut Grid,
    scratch: &mut Scratch,
    anchors_boundary: bool,
    blobs: &mut Vec<(Vec<IVec3>, UVec2, Grid)>,
) {
    let [x_size, y_size] = grid.shape.as_array();
    let size = UVec2::new(x_size, y_size);
    for y in 0..y_size {
        for x in 0..x_size {
            let pos = UVec2::new(x, y);
            let idx = grid.shape.linearize(pos) as usize;
            if !scratch.cell_visited[idx]
                && grid[pos].is_some()
                && scratch.cell_velocity[idx]
                    .abs()
                    .cmpge(Vec2::splat(1e-5))
                    .any()
            {
                let mut positions = Vec::new();
                let mut pending = vec![pos];
                scratch.cell_visited.set(idx, true);
                let mut touches_boundary = false;
                while let Some(cell) = pending.pop() {
                    positions.push(cell.as_ivec2().extend(0));
                    touches_boundary |= anchors_boundary && is_boundary(cell, size);
                    for &dir in CompassQuadrant::ALL {
                        let target = cell.wrapping_add_signed(dir.ivec2());
                        if let Some(index) = grid.linearize_checked(target)
                            && !scratch.cell_visited[index]
                            && grid[target].is_some()
                            && grid.get_edge(cell, dir)
                        {
                            scratch.cell_visited.set(index, true);
                            pending.push(target);
                        }
                    }
                }

                if !touches_boundary {
                    let mut blob_grid = Grid::new(grid.shape);

                    for cell in &positions {
                        let pos = cell.truncate().as_uvec2();
                        blob_grid[pos] = grid[pos];
                    }

                    for cell in &positions {
                        let pos = cell.truncate().as_uvec2();
                        for &dir in CompassQuadrant::ALL {
                            let neighbor = pos.wrapping_add_signed(dir.ivec2());
                            if blob_grid.contains(neighbor)
                                && blob_grid[neighbor].is_some()
                                && grid.get_edge(pos, dir)
                            {
                                blob_grid.set_edge(pos, dir, true);
                            }
                        }
                    }
                    for cell in &mut positions {
                        let pos = cell.truncate().as_uvec2();
                        grid.paint_cell(pos, None);
                    }
                    let Some((origin, blob_grid)) = blob_grid.shrink_to_fit() else {
                        error!("How did it shrink to nothing? {:?}", positions);
                        continue;
                    };

                    // TODO: see if this can be lazy
                    for cell in &mut positions {
                        *cell = cell.wrapping_sub_unsigned(origin.extend(0));
                    }
                    blobs.push((positions, origin, blob_grid));
                }
            }
        }
    }
}

// find c_max such that ||a + cb|| <= d
//
// Solve the quadratic bound on the accumulated transfer.
fn max_b_len_d(a: Vec2, b: Vec2, d: f32) -> f32 {
    let aa = a.dot(a);
    let ab = a.dot(b);
    let bb = b.dot(b);

    if bb == 0.0 {
        return 1.0;
    }
    let disc = (ab * ab - bb * (aa - d * d)).max(0.0);

    ((-ab + disc.sqrt()) / bb).clamp(0.0, 1.0)
}

fn is_boundary(pos: UVec2, size: UVec2) -> bool {
    pos.cmpeq(UVec2::ZERO).any() || (pos + UVec2::ONE).cmpeq(size).any()
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndshape::RuntimeShape;

    #[test]
    fn detached_blob_retains_velocity_after_cropping() {
        let mut grid = Grid::new(RuntimeShape::<u32, 2>::new([8, 8]));
        for pos in [UVec2::new(3, 4), UVec2::new(4, 4)] {
            grid.paint_cell(pos, Some(()));
        }
        let mut scratch = Scratch::default();
        initialize_scratch(grid.shape.usize(), &mut scratch);
        add_impulse(&grid, &mut scratch, Vec2::Y, Vec2::new(3.0, 4.0));
        add_impulse(&grid, &mut scratch, Vec2::Y, Vec2::new(4.0, 4.0));
        let blobs: Vec<_> = solve(&mut grid, &mut scratch, 0, true).collect();
        assert_eq!(blobs.len(), 1);
        let (blob, _, velocity, origin) = &blobs[0];
        assert_eq!(blob.size(), UVec2::new(2, 1));
        assert_eq!(*origin, Vec3::new(3.0, 4.0, 0.0));
        assert_eq!(velocity.linear, Vec3::Y);
        assert_eq!(velocity.angular, Vec3::ZERO);
        assert_eq!(grid.iter_set_positions().count(), 0);
    }

    #[test]
    fn all_four_boundaries_anchor_connected_components() {
        for pos in [
            UVec2::new(0, 2),
            UVec2::new(2, 0),
            UVec2::new(4, 2),
            UVec2::new(2, 4),
        ] {
            let mut grid = Grid::new(RuntimeShape::<u32, 2>::new([5, 5]));
            grid.paint_cell(pos, Some(()));
            let mut scratch = Scratch::default();
            initialize_scratch(grid.shape.usize(), &mut scratch);
            add_impulse(&grid, &mut scratch, Vec2::Y, pos.as_vec2());
            assert_eq!(solve(&mut grid, &mut scratch, 0, true).count(), 0);
            assert!(grid[pos].is_some());
        }
    }

    #[test]
    fn large_connected_component_is_fully_visited() {
        let mut grid = Grid::new(RuntimeShape::<u32, 2>::new([256, 256]));
        grid.set_brush(Vec2::splat(128.0), 128, Some(()));
        let mut scratch = Scratch::default();
        initialize_scratch(grid.shape.usize(), &mut scratch);
        add_impulse(&grid, &mut scratch, Vec2::Y, Vec2::ZERO);
        assert_eq!(solve(&mut grid, &mut scratch, 0, true).count(), 0);
        assert!(scratch.cell_visited.all());
    }

    #[test]
    fn transfer_limit_handles_zero_and_bounded_steps() {
        assert_eq!(max_b_len_d(Vec2::ZERO, Vec2::ZERO, 1.0), 1.0);
        let factor = max_b_len_d(Vec2::new(0.5, 0.0), Vec2::X, 1.0);
        assert!((factor - 0.5).abs() < 1e-6);
    }
}
