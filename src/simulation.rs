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
    let bonds_broken = break_bonds(scratch, grid);
    // A repossessed component may already be disconnected from the terrain.
    // Its first contact must reactivate it without requiring another fracture.
    if bonds_broken || parent.anchors_boundary {
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

                    let normal = direction.ivec2().as_vec2();
                    let approaching = (vel_b - vel_a).dot(normal) < 0.0;
                    if !approaching && !grid.get_edge(pos, direction) {
                        continue;
                    }

                    let half_diff = (vel_a - vel_b) / 2.0;
                    let transferred = &mut scratch.face[a_idx][is_y as usize];
                    // Compression is unlimited, but still recorded so later
                    // tension first cancels the accumulated compressive load.
                    let fraction = if approaching {
                        1.0
                    } else {
                        max_tensile_fraction(*transferred, half_diff, normal, MAX_TRANSFER)
                    };
                    let transfer = half_diff * fraction;
                    scratch.cell_velocity[a_idx] -= transfer;
                    scratch.cell_velocity[b_idx] += transfer;
                    *transferred += transfer;
                }
            }
        }
    }
}

fn break_bonds(scratch: &mut Scratch, grid: &mut Grid) -> bool {
    let mut any = false;
    let [range_x, range_y] = grid.shape.as_array();
    for y in 0..range_y {
        for x in 0..range_x {
            let pos = UVec2::new(x, y);
            let c_idx = grid.linearize(pos);
            for (face, direction) in [CompassQuadrant::East, CompassQuadrant::North]
                .into_iter()
                .enumerate()
            {
                if !grid.get_edge(pos, direction) {
                    continue;
                }
                // Positive normal transfer is compression; only the negative
                // normal component consumes this bond's tensile strength.
                let normal = direction.ivec2().as_vec2();
                let tension = (-scratch.face[c_idx][face].dot(normal)).max(0.0);
                if tension >= MAX_TRANSFER - 1e-6 {
                    grid.set_edge(pos, direction, false);
                    any = true;
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
            if !scratch.cell_visited[idx]
                && grid[pos].is_some()
                && (!anchors_boundary || scratch.cell_velocity[idx] != Vec2::ZERO)
            {
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
        let (mut grid, collider, relative_velocity, offset) =
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
        grid.mark_collider_clean();
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
            grid.mark_collider_clean();
        } else if !parent.anchors_boundary {
            commands.entity(parent.entity).despawn();
        } else {
            commands.entity(parent.entity).remove::<Collider>();
            grid.mark_collider_clean();
        }
    }
}

// Find the largest fraction of this transfer that keeps accumulated normal
// tension within the limit. Compression and tangential transfer are unrestricted.
fn max_tensile_fraction(accumulated: Vec2, requested: Vec2, normal: Vec2, limit: f32) -> f32 {
    let normal_step = requested.dot(normal);
    if normal_step >= 0.0 {
        return 1.0;
    }
    ((limit + accumulated.dot(normal)) / -normal_step).clamp(0.0, 1.0)
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
    fn first_hit_reactivates_disconnected_world_component_without_breaking_bonds() {
        let mut grid = Grid::new(RuntimeShape::<u32, 2>::new([8, 8]));
        // Hit component, untouched component, and anchored terrain.
        for pos in [
            UVec2::new(3, 4),
            UVec2::new(4, 4),
            UVec2::new(6, 6),
            UVec2::new(0, 1),
        ] {
            grid.paint_cell(pos, Some(()));
        }
        let mut scratch = Scratch::default();
        initialize_scratch(grid.shape.usize(), &mut scratch);
        add_impulse(&grid, &mut scratch, Vec2::Y, Vec2::new(3.0, 4.0));
        add_impulse(&grid, &mut scratch, Vec2::Y, Vec2::new(0.0, 1.0));
        let mut world = World::new();
        let entity = world.spawn_empty().id();
        let mut queue = bevy::ecs::world::CommandQueue::default();
        solve(
            &mut Commands::new(&mut queue, &world),
            &mut grid,
            &mut scratch,
            10,
            &ParentBody {
                entity,
                transform: GlobalTransform::IDENTITY,
                velocity: Velocity::zero(),
                center: None,
                anchors_boundary: true,
            },
        );
        queue.apply(&mut world);
        let fragment = world
            .query_filtered::<&Grid, With<ObjGrid>>()
            .single(&world)
            .unwrap();
        assert_eq!(fragment.size(), UVec2::new(2, 1));
        assert!(fragment.get_edge(UVec2::ZERO, CompassQuadrant::East));
        assert!(grid[UVec2::new(3, 4)].is_none());
        assert!(grid[UVec2::new(6, 6)].is_some());
        assert!(grid[UVec2::new(0, 1)].is_some());
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
                scratch.face[loaded_idx][loaded_face] =
                    -if loaded_face == 0 { Vec2::X } else { Vec2::Y } * MAX_TRANSFER;
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
    fn compression_above_strength_equalizes_without_breaking() {
        for normal in [UVec2::X, UVec2::Y] {
            let mut grid = Grid::new(RuntimeShape::<u32, 2>::new(
                (normal + UVec2::ONE).to_array(),
            ));
            grid.paint_cell(UVec2::ZERO, Some(()));
            grid.paint_cell(normal, Some(()));
            let mut scratch = Scratch::default();
            initialize_scratch(grid.shape.usize(), &mut scratch);
            let drift = Vec2::new(40.0, -15.0);
            let a = grid.linearize(UVec2::ZERO);
            let b = grid.linearize(normal);
            scratch.cell_velocity[a] = drift + normal.as_vec2() * MAX_TRANSFER * 20.0;
            scratch.cell_velocity[b] = drift;
            relax(10, &grid, &mut scratch);
            let expected = drift + normal.as_vec2() * MAX_TRANSFER * 10.0;
            assert!((scratch.cell_velocity[a] - expected).length() < 1e-4);
            assert!((scratch.cell_velocity[b] - expected).length() < 1e-4);
            assert!(!break_bonds(&mut scratch, &mut grid));
        }
    }

    #[test]
    fn bottom_row_impact_does_not_fracture_a_compressed_block() {
        let mut grid = Grid::new(RuntimeShape::<u32, 2>::new([4, 12]));
        for y in 0..12 {
            for x in 0..4 {
                grid.paint_cell(UVec2::new(x, y), Some(()));
            }
        }
        let mut scratch = Scratch::default();
        initialize_scratch(grid.shape.usize(), &mut scratch);
        for x in 0..4 {
            add_impulse(
                &grid,
                &mut scratch,
                Vec2::Y * MAX_TRANSFER * 100.0,
                Vec2::new(x as f32 + 0.5, 0.5),
            );
        }
        let momentum_before: Vec2 = scratch.cell_velocity.iter().sum();
        let energy_before: f32 = scratch
            .cell_velocity
            .iter()
            .map(|v| v.length_squared())
            .sum();
        relax(10, &grid, &mut scratch);
        assert!(!break_bonds(&mut scratch, &mut grid));
        let momentum_after: Vec2 = scratch.cell_velocity.iter().sum();
        let energy_after: f32 = scratch
            .cell_velocity
            .iter()
            .map(|v| v.length_squared())
            .sum();
        assert!((momentum_after - momentum_before).length() < 0.1);
        assert!(energy_after <= energy_before);
    }

    #[test]
    fn accumulated_compression_and_shear_do_not_break_a_bond() {
        let mut grid = Grid::new(RuntimeShape::<u32, 2>::new([2, 1]));
        grid.paint_cell(UVec2::ZERO, Some(()));
        grid.paint_cell(UVec2::X, Some(()));
        let mut scratch = Scratch::default();
        initialize_scratch(grid.shape.usize(), &mut scratch);
        scratch.face[0][0] = Vec2::new(MAX_TRANSFER * 100.0, MAX_TRANSFER * 100.0);
        assert!(!break_bonds(&mut scratch, &mut grid));
    }

    #[test]
    fn tensile_transfer_is_capped_and_breaks_the_bond() {
        for normal in [UVec2::X, UVec2::Y] {
            let mut grid = Grid::new(RuntimeShape::<u32, 2>::new(
                (normal + UVec2::ONE).to_array(),
            ));
            grid.paint_cell(UVec2::ZERO, Some(()));
            grid.paint_cell(normal, Some(()));
            let mut scratch = Scratch::default();
            initialize_scratch(grid.shape.usize(), &mut scratch);
            let a = grid.linearize(UVec2::ZERO);
            let b = grid.linearize(normal);
            scratch.cell_velocity[a] = -normal.as_vec2() * MAX_TRANSFER * 20.0;
            relax(10, &grid, &mut scratch);
            assert_eq!(scratch.cell_velocity[b], -normal.as_vec2() * MAX_TRANSFER);
            assert!(break_bonds(&mut scratch, &mut grid));
        }
    }

    #[test]
    fn bond_fracture_uses_accumulated_tension_even_after_velocities_equalize() {
        let mut grid = Grid::new(RuntimeShape::<u32, 2>::new([2, 1]));
        grid.paint_cell(UVec2::ZERO, Some(()));
        grid.paint_cell(UVec2::X, Some(()));
        let mut scratch = Scratch::default();
        initialize_scratch(grid.shape.usize(), &mut scratch);
        scratch.cell_velocity[0] = -Vec2::X * MAX_TRANSFER * 2.0;
        relax(1, &grid, &mut scratch);
        assert_eq!(scratch.cell_velocity[0], scratch.cell_velocity[1]);
        assert!(break_bonds(&mut scratch, &mut grid));
    }

    #[test]
    fn tensile_limit_accounts_for_prior_compression_and_ignores_shear() {
        let normal = Vec2::X;
        assert_eq!(
            max_tensile_fraction(Vec2::ZERO, Vec2::ZERO, normal, 1.0),
            1.0
        );
        assert_eq!(
            max_tensile_fraction(Vec2::ZERO, Vec2::Y * 1000.0, normal, 1.0),
            1.0
        );
        assert_eq!(
            max_tensile_fraction(Vec2::X * 1000.0, Vec2::X, normal, 1.0),
            1.0
        );
        let fraction = max_tensile_fraction(Vec2::X * 2.0, -Vec2::X * 4.0, normal, 1.0);
        assert_eq!(fraction, 0.75);
        assert_eq!(
            max_tensile_fraction(-Vec2::X * 0.5, -Vec2::X, normal, 1.0),
            0.5
        );
    }
}
