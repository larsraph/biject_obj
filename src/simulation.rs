use bevy::{math::CompassQuadrant, prelude::*};
use bevy_rapier3d::prelude::*;
use bitvec::vec::BitVec;
use ndshape::{RuntimeShape, Shape};

use crate::{
    All as _, ToIVec2 as _,
    grid::{Bijection, Grid, ObjGrid, inherit_fragment_velocity},
};

const MAX_TRANSFER: f32 = 30.;

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

pub(crate) struct ParentBody {
    pub entity: Entity,
    pub transform: GlobalTransform,
    pub velocity: Velocity,
    pub center: Option<Vec3>,
    pub anchors_boundary: bool,
}

struct Blob {
    positions: Vec<IVec3>,
    min: IVec2,
    max: IVec2,
    touches_boundary: bool,
}

// TODO: Re-extract unsupported components after editing.
pub(crate) fn solve(
    commands: &mut Commands,
    grid: &mut Grid,
    scratch: &mut Scratch,
    iterations: usize,
    parent: &ParentBody,
) {
    relax(iterations, grid, scratch);
    if break_bonds(scratch, grid) {
        let mut blobs = Vec::new();
        find_blobs(grid, scratch, &mut blobs, parent.anchors_boundary);
        extract_blobs_to_one(commands, parent, blobs, scratch, grid);
    }
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

fn relax(iterations: usize, grid: &Grid, scratch: &mut Scratch) {
    // Iteratively equalize neighboring velocities.
    for _ in 0..iterations {
        // for every constraint try to equalize the velocities
        let [x_range, y_range] = grid.shape.as_array();
        for direction in [CompassQuadrant::North, CompassQuadrant::East] {
            let is_y = direction == CompassQuadrant::North;
            for y in 0..y_range {
                if is_y && y == y_range - 1 {
                    continue;
                }
                for x in 0..x_range {
                    if !is_y && x == x_range - 1 {
                        continue;
                    }

                    let pos = UVec2::new(x, y);
                    if grid[pos].is_none() {
                        continue;
                    }

                    let a = pos;
                    // guaranteed in bounds by the checks above
                    let b = pos.wrapping_add_signed(direction.ivec2());

                    if grid[b].is_none() {
                        continue;
                    }

                    let a_idx = grid.linearize(a);
                    let b_idx = grid.linearize(b);

                    let vel_a = scratch.cell_velocity[a_idx];
                    let vel_b = scratch.cell_velocity[b_idx];

                    let aligned =
                        aligned(vel_a, vel_b, matches!(direction, CompassQuadrant::North));

                    // completely sticky/rigid collision
                    if aligned {
                        let half_diff = (vel_a - vel_b) / 2.;
                        scratch.cell_velocity[a_idx] -= half_diff;
                        scratch.cell_velocity[b_idx] += half_diff;
                        scratch.face[a_idx][is_y as usize] += half_diff;
                    } else if grid.get_edge(pos, direction) {
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

fn break_bonds(scratch: &mut Scratch, grid: &mut Grid) -> bool {
    let mut any = false;
    // Break bonds whose accumulated transfer reaches the limit AND
    // that are not aligned.
    let [range_x, range_y] = grid.shape.as_array();
    for y in 0..range_y {
        for x in 0..range_x {
            let pos = UVec2::new(x, y);
            let c_idx = grid.linearize(pos);
            let cell = scratch.cell_velocity[c_idx];
            for (face, direction) in [CompassQuadrant::East, CompassQuadrant::North]
                .into_iter()
                .enumerate()
            {
                if !grid.get_edge(pos, direction) {
                    continue;
                }
                let neighbor = pos.wrapping_add_signed(direction.ivec2());
                let Some(other_idx) = grid.linearize_checked(neighbor) else {
                    continue;
                };
                let transferred = scratch.face[c_idx][face];
                let other_cell = scratch.cell_velocity[other_idx];
                if transferred.length() >= MAX_TRANSFER - 1e-6
                    && !aligned(cell, other_cell, direction == CompassQuadrant::North)
                {
                    grid.set_edge(pos, direction, false);
                    any |= true;
                }
            }
        }
    }
    any
}

fn find_blobs(grid: &Grid, scratch: &mut Scratch, blobs: &mut Vec<Blob>, anchors_boundary: bool) {
    let [x_size, y_size] = grid.shape.as_array();
    let size = UVec2::new(x_size, y_size);
    for y in 0..y_size {
        for x in 0..x_size {
            let pos = UVec2::new(x, y);
            let idx = grid.shape.linearize(pos) as usize;
            if !scratch.cell_visited[idx] && grid[pos].is_some() {
                let mut min = IVec2::MAX;
                let mut max = IVec2::MIN;
                let mut positions = Vec::new();
                let mut pending = vec![pos];
                scratch.cell_visited.set(idx, true);
                let mut touches_boundary = false;
                while let Some(cell) = pending.pop() {
                    let pos_2 = cell.as_ivec2();
                    min = min.min(pos_2);
                    max = max.max(pos_2);
                    positions.push(pos_2.extend(0));

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

                blobs.push(Blob {
                    positions,
                    min,
                    max,
                    touches_boundary,
                });
            }
        }
    }
}

fn extract_blobs_to_one(
    commands: &mut Commands,
    parent: &ParentBody,
    blobs: Vec<Blob>,
    scratch: &Scratch,
    grid: &mut Grid,
) {
    // Static terrain retains every anchored component. A dynamic body keeps
    // its largest component, preserving the original entity and physics pose.
    let retained = if parent.anchors_boundary {
        None
    } else {
        blobs
            .iter()
            .enumerate()
            .max_by_key(|(_, blob)| blob.positions.len())
            .map(|(index, _)| index)
    };
    let source_shape = grid.shape;
    let mut extracted = false;
    for (index, blob) in blobs.into_iter().enumerate() {
        if blob.touches_boundary || Some(index) == retained {
            continue;
        }
        let Blob {
            mut positions,
            min,
            max,
            ..
        } = blob;
        let origin = min.as_uvec2();
        let size = (max - min + 1).as_uvec2();
        let mut blob_grid = Grid::new(RuntimeShape::<u32, 2>::new(size));

        // Copy occupancy, then bonds, before clearing anything in the source.
        for cell in &positions {
            let pos = cell.truncate().as_uvec2();
            blob_grid[pos - origin] = grid[pos];
        }
        for cell in &positions {
            let pos = cell.truncate().as_uvec2();
            let local_pos = pos - origin;
            for dir in [CompassQuadrant::East, CompassQuadrant::North] {
                let neighbor = local_pos.wrapping_add_signed(dir.ivec2());
                if blob_grid.contains(neighbor)
                    && blob_grid[neighbor].is_some()
                    && grid.get_edge(pos, dir)
                {
                    blob_grid.set_edge(local_pos, dir, true);
                }
            }
        }
        for cell in &positions {
            grid.paint_cell(cell.truncate().as_uvec2(), None);
        }
        for cell in &mut positions {
            *cell -= min.extend(0);
        }
        let (grid, collider, relative_velocity, offset) =
            build_blob(positions, origin, blob_grid, source_shape, scratch);
        let object_transform = Transform {
            translation: parent.transform.transform_point(offset),
            rotation: parent.transform.rotation(),
            ..default()
        };
        let solution = crate::projection::solve(
            &grid,
            object_transform.rotation,
            object_transform.translation.into(),
        );
        let center_offset = parent.center.map_or(Vec3::ZERO, |parent_center| {
            let fragment_center = collider.raw.mass_properties(1.0).local_com;
            parent
                .transform
                .affine()
                .transform_vector3(offset + fragment_center - parent_center)
        });
        let velocity = inherit_fragment_velocity(
            parent.velocity,
            center_offset,
            object_transform.rotation,
            relative_velocity,
        );
        commands.spawn((
            grid,
            collider,
            velocity,
            object_transform,
            RigidBody::Dynamic,
            ObjGrid,
            LockedAxes::TRANSLATION_LOCKED_Z
                | LockedAxes::ROTATION_LOCKED_X
                | LockedAxes::ROTATION_LOCKED_Y,
            Bijection(solution),
            ActiveEvents::COLLISION_EVENTS,
            Sleeping::default(),
        ));
        extracted = true;
    }
    if extracted {
        // The retained body keeps its pose and Rapier's collision response.
        // Only its occupied geometry needs rebuilding before the next step.
        if let Some(collider) = grid.build_collider() {
            commands.entity(parent.entity).insert(collider);
        } else if !parent.anchors_boundary {
            commands.entity(parent.entity).despawn();
        } else {
            commands.entity(parent.entity).remove::<Collider>();
        }
    }
}

fn aligned(vel_a: Vec2, vel_b: Vec2, is_y: bool) -> bool {
    let vec = match is_y {
        true => Vec2::Y,
        false => Vec2::X,
    };
    (vel_b - vel_a).dot(vec) > 0.0
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

    fn extract_for_test(
        grid: &mut Grid,
        scratch: &mut Scratch,
        anchors_boundary: bool,
    ) -> (World, Entity) {
        let mut world = World::new();
        let entity = world.spawn_empty().id();
        let parent = ParentBody {
            entity,
            transform: GlobalTransform::IDENTITY,
            velocity: Velocity::zero(),
            center: None,
            anchors_boundary,
        };
        let mut blobs = Vec::new();
        find_blobs(grid, scratch, &mut blobs, anchors_boundary);
        let mut queue = bevy::ecs::world::CommandQueue::default();
        extract_blobs_to_one(
            &mut Commands::new(&mut queue, &world),
            &parent,
            blobs,
            scratch,
            grid,
        );
        queue.apply(&mut world);
        (world, entity)
    }

    #[test]
    fn detached_blob_retains_velocity_and_bonds_after_cropping() {
        let mut grid = Grid::new(RuntimeShape::<u32, 2>::new([8, 8]));
        for pos in [UVec2::new(3, 4), UVec2::new(4, 4)] {
            grid.paint_cell(pos, Some(()));
        }
        let mut scratch = Scratch::default();
        initialize_scratch(grid.shape.usize(), &mut scratch);
        add_impulse(&grid, &mut scratch, Vec2::Y, Vec2::new(3.0, 4.0));
        add_impulse(&grid, &mut scratch, Vec2::Y, Vec2::new(4.0, 4.0));
        let (mut world, _) = extract_for_test(&mut grid, &mut scratch, true);
        let (blob, velocity, transform, sleeping) = world
            .query_filtered::<(&Grid, &Velocity, &Transform, &Sleeping), With<ObjGrid>>()
            .single(&world)
            .unwrap();
        assert_eq!(blob.size(), UVec2::new(2, 1));
        assert!(blob.get_edge(UVec2::ZERO, CompassQuadrant::East));
        assert_eq!(transform.translation, Vec3::new(3.0, 4.0, 0.0));
        assert_eq!(velocity.linear, Vec3::Y);
        assert_eq!(velocity.angular, Vec3::ZERO);
        assert!(!sleeping.sleeping);
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
            let (mut world, _) = extract_for_test(&mut grid, &mut scratch, true);
            assert_eq!(
                world
                    .query_filtered::<Entity, With<ObjGrid>>()
                    .iter(&world)
                    .count(),
                0
            );
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
        let (mut world, _) = extract_for_test(&mut grid, &mut scratch, true);
        assert_eq!(
            world
                .query_filtered::<Entity, With<ObjGrid>>()
                .iter(&world)
                .count(),
            0
        );
        assert!(scratch.cell_visited.all());
    }

    #[test]
    fn dynamic_body_retains_largest_component_including_unloaded_cells() {
        let mut grid = Grid::new(RuntimeShape::<u32, 2>::new([5, 1]));
        for x in [0, 1, 4] {
            grid.paint_cell(UVec2::new(x, 0), Some(()));
        }
        let mut scratch = Scratch::default();
        initialize_scratch(grid.shape.usize(), &mut scratch);
        let (mut world, original) = extract_for_test(&mut grid, &mut scratch, false);
        assert!(world.get_entity(original).is_ok());
        assert!(world.get::<Collider>(original).is_some());
        assert_eq!(grid.iter_set_positions().count(), 2);
        assert!(grid.get_edge(UVec2::ZERO, CompassQuadrant::East));
        let (fragment, transform) = world
            .query_filtered::<(&Grid, &Transform), With<ObjGrid>>()
            .single(&world)
            .unwrap();
        assert_eq!(fragment.size(), UVec2::ONE);
        assert_eq!(transform.translation, Vec3::X * 4.0);
    }

    #[test]
    fn intact_dynamic_body_is_not_replaced() {
        let mut grid = Grid::new(RuntimeShape::<u32, 2>::new([2, 1]));
        grid.paint_cell(UVec2::ZERO, Some(()));
        grid.paint_cell(UVec2::X, Some(()));
        let mut scratch = Scratch::default();
        initialize_scratch(grid.shape.usize(), &mut scratch);
        let mut world = World::new();
        let original = world.spawn(Sleeping::default()).id();
        let mut queue = bevy::ecs::world::CommandQueue::default();
        solve(
            &mut Commands::new(&mut queue, &world),
            &mut grid,
            &mut scratch,
            0,
            &ParentBody {
                entity: original,
                transform: GlobalTransform::IDENTITY,
                velocity: Velocity::zero(),
                center: None,
                anchors_boundary: false,
            },
        );
        queue.apply(&mut world);
        assert_eq!(
            world
                .query_filtered::<Entity, With<ObjGrid>>()
                .iter(&world)
                .count(),
            0,
        );
        assert!(world.get::<Sleeping>(original).is_some());
        assert_eq!(grid.iter_set_positions().count(), 2);
    }

    #[test]
    fn breaking_selects_only_the_loaded_face_and_ignores_outer_faces() {
        let shape = RuntimeShape::<u32, 2>::new([3, 2]);
        for loaded_idx in 0..shape.usize() {
            for loaded_face in 0..2 {
                let mut grid = Grid::new(shape);
                for y in 0..2 {
                    for x in 0..3 {
                        grid.paint_cell(UVec2::new(x, y), Some(()));
                    }
                }
                let mut scratch = Scratch::default();
                initialize_scratch(shape.usize(), &mut scratch);
                scratch.face[loaded_idx][loaded_face] = Vec2::X * MAX_TRANSFER;
                break_bonds(&mut scratch, &mut grid);

                for y in 0..2 {
                    for x in 0..3 {
                        let pos = UVec2::new(x, y);
                        for (face, direction) in [CompassQuadrant::East, CompassQuadrant::North]
                            .into_iter()
                            .enumerate()
                        {
                            let neighbor = pos.wrapping_add_signed(direction.ivec2());
                            let loaded = grid.linearize(pos) == loaded_idx && face == loaded_face;
                            assert_eq!(
                                grid.get_edge(pos, direction),
                                grid.contains(neighbor) && !loaded,
                                "loaded cell {loaded_idx}, face {loaded_face}; checked {pos:?} {direction:?}",
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn transfer_limit_handles_zero_and_bounded_steps() {
        assert_eq!(max_b_len_d(Vec2::ZERO, Vec2::ZERO, 1.0), 1.0);
        let factor = max_b_len_d(Vec2::new(0.5, 0.0), Vec2::X, 1.0);
        assert!((factor - 0.5).abs() < 1e-6);
    }
}
