use bevy::{math::CompassQuadrant, platform::collections::HashSet, prelude::*};
use bevy_rapier3d::prelude::*;
use bitvec::{bitbox, boxed::BitBox};
use ndshape::{RuntimeShape, Shape};
use std::ops::{Index, IndexMut};

use crate::{All as _, ToIVec2 as _, projection};

pub const SIZE_LOG2: u32 = 8;
pub const SIZE: u32 = 1 << SIZE_LOG2;

pub type CellInner = ();
pub type Cell = Option<CellInner>;

#[derive(Component)]
pub struct WorldGrid;

#[derive(Component)]
pub struct ObjGrid;

const BOUNDARY_THICKNESS: f32 = 1.0;

#[derive(Component, Debug)]
pub struct Grid {
    pub values: Box<[Cell]>,
    pub edges: BitBox,
    pub shape: RuntimeShape<u32, 2>,
}

impl Grid {
    pub fn new(shape: RuntimeShape<u32, 2>) -> Self {
        Self {
            values: vec![None; shape.usize()].into_boxed_slice(),
            edges: bitbox!(0; 2 * shape.usize()),
            shape,
        }
    }

    pub fn size(&self) -> UVec2 {
        UVec2::from(self.shape.as_array())
    }

    pub fn contains(&self, pos: UVec2) -> bool {
        pos.cmplt(self.size()).all()
    }

    pub fn linearize(&self, pos: UVec2) -> usize {
        self.shape.linearize(pos) as usize
    }

    pub fn linearize_checked(&self, pos: UVec2) -> Option<usize> {
        self.contains(pos).then_some(self.linearize(pos))
    }

    #[track_caller]
    pub fn linearize_expect(&self, pos: UVec2) -> usize {
        assert!(self.contains(pos), "{:?} oob of {:?}", pos, self.size());
        self.linearize(pos)
    }

    pub fn linearize_edge(&self, pos: UVec2, dir: CompassQuadrant) -> (usize, UVec2) {
        let (lin_off, bit_0) = match dir {
            CompassQuadrant::North => (IVec2::ZERO, 0b1),
            CompassQuadrant::East => (IVec2::ZERO, 0b0),
            CompassQuadrant::South => (IVec2::NEG_Y, 0b1),
            CompassQuadrant::West => (IVec2::NEG_X, 0b0),
        };
        let pos = pos.wrapping_add_signed(lin_off);
        let idx = self.linearize(pos);
        ((idx << 1) | bit_0, pos)
    }

    pub fn linearize_edge_checked(&self, pos: UVec2, dir: CompassQuadrant) -> Option<usize> {
        let (idx, pos) = self.linearize_edge(pos, dir);
        self.contains(pos).then_some(idx)
    }

    pub fn linearize_edge_expect(&self, pos: UVec2, dir: CompassQuadrant) -> usize {
        let (idx, pos) = self.linearize_edge(pos, dir);
        let contains = self.contains(pos);
        let size = self.size();
        assert!(contains, "{:?} edge {:?} oob of {:?}", pos, dir, size);
        idx
    }

    #[track_caller]
    pub fn get(&self, pos: UVec2) -> &Cell {
        let index = self.linearize_expect(pos);
        // SAFETY: index is checked to be within bounds by linearize
        unsafe { self.values.get_unchecked(index) }
    }

    #[track_caller]
    pub fn get_mut(&mut self, pos: UVec2) -> &mut Cell {
        let index = self.linearize_expect(pos);
        // SAFETY: index is checked to be within bounds by linearize
        unsafe { self.values.get_unchecked_mut(index) }
    }

    #[track_caller]
    pub fn get_edge(&self, pos: UVec2, dir: CompassQuadrant) -> bool {
        let index = self.linearize_edge_expect(pos, dir);
        // SAFETY: index is checked to be within bounds by linearize
        unsafe { *self.edges.get_unchecked(index) }
    }

    #[track_caller]
    pub fn set_edge(&mut self, pos: UVec2, dir: CompassQuadrant, connected: bool) {
        let index = self.linearize_edge_expect(pos, dir);
        // SAFETY: index is checked to be within bounds by linearize
        unsafe { self.edges.set_unchecked(index, connected) }
    }

    /// Edits one cell and keeps the four incident bonds consistent with occupancy.
    ///
    /// Painting a new cell bonds it to each existing orthogonal equal neighbor.
    pub fn paint_cell(&mut self, pos: UVec2, to: Cell) {
        let Some(index) = self.linearize_checked(pos) else {
            return;
        };

        self.values[index] = to;

        for &quadrant in CompassQuadrant::ALL {
            let Some(edge_index) = self.linearize_edge_checked(pos, quadrant) else {
                continue;
            };
            let adj = pos.wrapping_add_signed(quadrant.ivec2());
            let adj_index = self.linearize(adj);

            let value = to.is_some() && self.values[adj_index] == to;
            self.edges.set(edge_index, value);
        }
    }

    pub fn iter_set_positions(&self) -> impl Iterator<Item = UVec2> + '_ {
        let size = self.size();
        (0..size.y)
            .flat_map(move |y| (0..size.x).map(move |x| UVec2::new(x, y)))
            .filter(|&pos| self.values[self.linearize(pos)].is_some())
    }

    pub fn set_brush(&mut self, center: Vec2, radius: i32, to: Cell) {
        let center = center.round().as_ivec2();
        for y in -radius..=radius {
            for x in -radius..=radius {
                let cell = center + IVec2::new(x, y);
                self.paint_cell(cell.as_uvec2(), to);
            }
        }
    }

    // I understand the theory of how to traverse voxels but not the implementation.
    pub fn set_brush_line(&mut self, start: Vec2, end: Vec2, radius: i32, to: Cell) {
        let mut cell = start.round().as_ivec2();
        let end = end.round().as_ivec2();
        let delta = (end - cell).abs();
        let step = (end - cell).signum();
        let mut error = delta.x - delta.y;

        loop {
            self.set_brush(cell.as_vec2(), radius, to);
            if cell == end {
                break;
            }

            let twice_error = 2 * error;
            if twice_error > -delta.y {
                error -= delta.y;
                cell.x += step.x;
            }
            if twice_error < delta.x {
                error += delta.x;
                cell.y += step.y;
            }
        }
    }

    /// Sets the edges around this rectangle to disconnected.
    pub fn disconnect_rect(&mut self, rect: URect) {
        // top and bottom row
        for y in [rect.min.y - 1, rect.max.y - 1] {
            for x in rect.min.x..rect.max.x {
                let pos = UVec2::new(x, y);
                self.set_edge(pos, CompassQuadrant::North, false);
            }
        }
        // right and left row
        for y in rect.min.y..rect.max.y {
            for x in [rect.min.x - 1, rect.max.x - 1] {
                let pos = UVec2::new(x, y);
                self.set_edge(pos, CompassQuadrant::East, false);
            }
        }
    }

    pub fn copy_rect(&mut self, rect: URect) -> (bool, Self) {
        let size = rect.size();

        let shape = RuntimeShape::<u32, 2>::new(size);
        let mut grid = Grid::new(shape);

        let mut any = false;

        for y in 0..size.y {
            for x in 0..size.x {
                let lpos = UVec2::new(x, y);
                let wpos = lpos + rect.min;

                let value = self[wpos];
                grid[lpos] = value;
                any |= value.is_some();

                for quad in [CompassQuadrant::North, CompassQuadrant::East] {
                    if self.get_edge(wpos, quad) {
                        grid.set_edge(lpos, quad, true);
                    }
                }
            }
        }

        (any, grid)
    }

    pub fn clear_rect(&mut self, rect: URect) {
        for y in rect.min.y..rect.max.y {
            for x in rect.min.x..rect.max.x {
                let pos = UVec2::new(x, y);
                self[pos] = None;
                for quad in [CompassQuadrant::North, CompassQuadrant::East] {
                    self.set_edge(pos, quad, false);
                }
            }
        }
    }

    pub fn extract_rect(&mut self, rect: URect) -> Option<Self> {
        self.disconnect_rect(rect);
        let (any, grid) = self.copy_rect(rect);
        if !any {
            return None;
        }
        self.clear_rect(rect);
        Some(grid)
    }

    pub fn build_collider(&self) -> Option<Collider> {
        let voxels = self
            .iter_set_positions()
            .map(|position| position.as_ivec2().extend(0))
            .collect::<Vec<_>>();

        (!voxels.is_empty()).then(|| Collider::voxels(Vec3::ONE, &voxels))
    }

    pub fn shrink_to_fit(mut self) -> Option<(UVec2, Self)> {
        let mut cells = self.iter_set_positions();
        let first = cells.next()?;
        let (min, max) = cells.fold((first, first), |(min, max), cell| {
            (min.min(cell), max.max(cell))
        });

        // `URect` has an exclusive maximum, while `max` is an occupied cell.
        let rect = URect::from_corners(min, max + UVec2::ONE);
        let (non_empty, shrunk) = self.copy_rect(rect);
        debug_assert!(non_empty, "{:?}", rect);
        Some((min, shrunk))
    }
}

impl Index<UVec2> for Grid {
    type Output = Cell;

    fn index(&self, pos: UVec2) -> &Self::Output {
        self.get(pos)
    }
}

impl IndexMut<UVec2> for Grid {
    fn index_mut(&mut self, pos: UVec2) -> &mut Self::Output {
        self.get_mut(pos)
    }
}

pub fn spawn_world_grid(mut commands: Commands) {
    commands.spawn((
        WorldGrid,
        Grid::new(RuntimeShape::<u32, 2>::new([SIZE, SIZE])),
        RigidBody::Fixed,
        ActiveEvents::COLLISION_EVENTS,
    ));
}

pub fn spawn_edge_colliders(mut commands: Commands) {
    let size = SIZE as f32;
    let center = (size - 1.0) * 0.5;
    let outer_length = size + 2.0 * BOUNDARY_THICKNESS;
    let low_edge = -0.5 - BOUNDARY_THICKNESS * 0.5;
    let high_edge = size - 0.5 + BOUNDARY_THICKNESS * 0.5;

    // The grid occupies the square from -0.5 to SIZE - 0.5 on both axes.
    // These four static colliders meet that square at each edge, retaining
    // dynamic voxel objects within the playable area.
    commands.spawn_batch([
        (
            RigidBody::Fixed,
            Collider::cuboid(BOUNDARY_THICKNESS, outer_length, 1.0),
            ActiveEvents::COLLISION_EVENTS,
            Transform::from_xyz(low_edge, center, 0.0),
        ),
        (
            RigidBody::Fixed,
            Collider::cuboid(BOUNDARY_THICKNESS, outer_length, 1.0),
            ActiveEvents::COLLISION_EVENTS,
            Transform::from_xyz(high_edge, center, 0.0),
        ),
        (
            RigidBody::Fixed,
            Collider::cuboid(outer_length, BOUNDARY_THICKNESS, 1.0),
            ActiveEvents::COLLISION_EVENTS,
            Transform::from_xyz(center, low_edge, 0.0),
        ),
        (
            RigidBody::Fixed,
            Collider::cuboid(outer_length, BOUNDARY_THICKNESS, 1.0),
            ActiveEvents::COLLISION_EVENTS,
            Transform::from_xyz(center, high_edge, 0.0),
        ),
    ]);
}

#[derive(Resource, Default)]
pub struct Collisions {
    set: HashSet<(Entity, Entity)>,
}

pub fn on_world_grid_collision(
    mut events: MessageReader<CollisionEvent>,
    mut collisions: ResMut<Collisions>,
) {
    for event in events.read() {
        match event {
            CollisionEvent::Started(a, b, _) => {
                collisions.set.insert((*a, *b));
            }
            CollisionEvent::Stopped(a, b, _) => {
                collisions.set.remove(&(*a, *b));
            }
        }
    }
}

pub fn handle_grid_collisions(
    mut commands: Commands,
    collisions: Res<Collisions>,
    mut grids: Query<(&mut Grid, &GlobalTransform, Option<&WorldGrid>)>,
    rapier_context: ReadRapierContext,
    mut scratch: Local<super::Scratch>,
) {
    let Ok(rapier_context) = rapier_context.single() else {
        error!("Multiple contexts");
        return;
    };
    for (a, b) in collisions.set.iter().copied() {
        for grid_entity in [a, b] {
            let Ok((mut grid, transform, anchors)) = grids.get_mut(grid_entity) else {
                continue;
            };
            let local_from_world = transform.affine().inverse();

            let Some(contact_pair) = rapier_context.contact_pair(a, b) else {
                error!("Expected contact pair");
                continue;
            };
            let grid_first = contact_pair.collider1() == Some(grid_entity);

            for manifold in contact_pair.manifolds() {
                super::initalize_scratch(&grid, &mut scratch);
                for point in manifold.points() {
                    let impulse_world =
                        manifold.normal() * point.impulse() * if grid_first { 1. } else { -1. };
                    let p = if grid_first {
                        point.local_p1()
                    } else {
                        point.local_p2()
                    };
                    let point_world = transform.transform_point(p);
                    super::add_impulse(
                        &grid,
                        &mut scratch,
                        local_from_world.transform_vector3(impulse_world).truncate(),
                        local_from_world.transform_point3(point_world).truncate(),
                    );
                }
                let result = super::solve(&mut grid, &mut scratch, 10, anchors.is_some());
                for (grid, collider, velocity, offset) in result {
                    let solution = super::projection::solve(
                        grid.shape.clone(),
                        Quat::default(),
                        transform.translation_vec3a(),
                    );
                    commands.spawn((
                        grid,
                        collider,
                        velocity,
                        Transform::from_translation(transform.translation() + offset),
                        RigidBody::Dynamic,
                        ObjGrid,
                        LockedAxes::TRANSLATION_LOCKED_Z
                            | LockedAxes::ROTATION_LOCKED_X
                            | LockedAxes::ROTATION_LOCKED_Y,
                        Bijection(solution),
                        ActiveEvents::COLLISION_EVENTS,
                    ));
                }
            }
        }
    }
}

pub fn update_colliders(
    mut commands: Commands,
    mut query: Query<(Entity, &Grid, Option<&mut Collider>), Changed<Grid>>,
) {
    for (entity, grid, collider) in query.iter_mut() {
        match (collider, grid.build_collider()) {
            (Some(mut old), Some(new)) => *old = new,
            (Some(_), None) => {
                commands.entity(entity).remove::<Collider>();
            }
            (None, Some(new)) => {
                commands.entity(entity).insert(new);
            }
            (None, None) => {}
        }
    }
}

#[derive(Component, Deref, DerefMut)]
pub struct Bijection(pub projection::Solution);

pub fn recalculate_bijection(
    mut query: Query<(&GlobalTransform, &Grid, &mut Bijection), With<ObjGrid>>,
) {
    for (transform, grid, mut bijection) in query.iter_mut() {
        bijection.0 = projection::solve(
            grid.shape.clone(),
            transform.rotation(),
            transform.translation_vec3a(),
        )
    }
}
