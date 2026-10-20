use bevy::{math::CompassQuadrant, platform::collections::HashSet, prelude::*};
use bevy_rapier3d::prelude::*;
use bitvec::{bitbox, boxed::BitBox};
use ndshape::{ConstPow2Shape2u32, RuntimeShape, Shape};
use std::ops::{Index, IndexMut};

use crate::projection::{Solution, solve};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GCell {
    Set,
    Unset,
}

pub const SIZE_LOG2: u32 = 8;
pub const SIZE: u32 = 1 << SIZE_LOG2;
pub const _SIZE_POW2: u32 = SIZE * SIZE;
pub type S = ConstPow2Shape2u32<SIZE_LOG2, SIZE_LOG2>;
pub const SHAPE: S = S {};

#[derive(Component)]
pub struct WorldGrid;
#[derive(Component)]
pub struct ObjGrid;

pub type Const = ConstPow2Shape2u32<SIZE_LOG2, SIZE_LOG2>;
pub type ConstGrid = Grid<Const>;
pub type Dyn = RuntimeShape<u32, 2>;
pub type DynGrid = Grid<Dyn>;

const BOUNDARY_THICKNESS: f32 = 1.0;

#[derive(Component, Debug)]
pub struct Grid<S> {
    pub ty: Box<[GCell]>,
    pub connection: BitBox,
    pub shape: S,
}

impl<S: Shape<2, Coord = u32>> Grid<S> {
    pub fn new(shape: S) -> Self {
        Self {
            ty: vec![GCell::Unset; shape.usize()].into_boxed_slice(),
            connection: bitbox!(0; 2 * shape.usize()),
            shape,
        }
    }

    pub fn _take_from<D: Shape<2, Coord = u32>>(&mut self, other: &mut Grid<D>, pos: UVec2) {
        let index = self.linearize(pos).unwrap();
        self.ty[index] = other.ty[index];
        other.ty[index] = GCell::Unset;
    }

    pub fn contains(&self, pos: UVec2) -> bool {
        let bound = UVec2::from(self.shape.as_array());
        pos.cmplt(bound).all()
    }

    pub fn linearize(&self, pos: UVec2) -> Option<usize> {
        self.contains(pos)
            .then_some(self.shape.linearize(pos) as usize)
    }

    pub fn linearize_conn(&self, pos: UVec2, y: bool) -> Option<usize> {
        self.linearize(pos).map(|idx| (idx << 1) | (y as usize))
    }

    pub fn get(&self, pos: UVec2) -> Option<&GCell> {
        let index = self.linearize(pos)?;
        // SAFETY: index is checked to be within bounds by linearize
        Some(unsafe { self.ty.get_unchecked(index) })
    }

    pub fn get_mut(&mut self, pos: UVec2) -> Option<&mut GCell> {
        let index = self.linearize(pos)?;
        // SAFETY: index is checked to be within bounds by linearize
        Some(unsafe { self.ty.get_unchecked_mut(index) })
    }

    /// Edits one cell and keeps the four incident bonds consistent with occupancy.
    ///
    /// Painting a new cell bonds it to each existing orthogonal neighbor. Erasing a
    /// cell removes every bond that touches it. Repainting an existing cell leaves
    /// its bonds alone, so a brush stroke does not silently repair a fractured bond.
    pub fn paint_cell(&mut self, pos: UVec2, to: GCell) {
        let Some(index) = self.linearize(pos) else {
            return;
        };

        let previous = self.ty[index];
        if previous == to {
            return;
        }
        self.ty[index] = to;

        // Edges are stored at the cell on their negative side: X/Y for right/down,
        // and the neighbor's X/Y edge for left/up.
        for (offset, is_y) in [
            (IVec2::X, false),
            (IVec2::Y, true),
            (IVec2::NEG_X, false),
            (IVec2::NEG_Y, true),
        ] {
            let Some(neighbor) = pos.checked_add_signed(offset) else {
                continue;
            };
            let Some(neighbor_index) = self.linearize(neighbor) else {
                continue;
            };

            let edge_owner = if offset.x < 0 || offset.y < 0 {
                neighbor
            } else {
                pos
            };
            let edge_index = self.linearize_conn(edge_owner, is_y).unwrap();
            let connected = to == GCell::Set && self.ty[neighbor_index] == GCell::Set;
            self.connection.set(edge_index, connected);
        }
    }

    pub fn is_connected_raw(&self, pos: UVec2, y: bool) -> Option<bool> {
        let index = self.linearize_conn(pos, y)?;
        // SAFETY: index is checked to be within bounds by linearize
        Some(unsafe { *self.connection.get_unchecked(index) })
    }

    pub fn is_connected(&self, pos: UVec2, quad: CompassQuadrant) -> Option<bool> {
        match quad {
            CompassQuadrant::North => pos
                .checked_add_signed(IVec2::NEG_Y)
                .and_then(|neighbor| self.is_connected_raw(neighbor, true)),
            CompassQuadrant::South => self.is_connected_raw(pos, true),
            CompassQuadrant::East => self.is_connected_raw(pos, false),
            CompassQuadrant::West => pos
                .checked_add_signed(IVec2::NEG_X)
                .and_then(|neighbor| self.is_connected_raw(neighbor, false)),
        }
    }

    pub fn set_connection(&mut self, pos: UVec2, quad: CompassQuadrant, connected: bool) {
        let (offset, is_y) = match quad {
            CompassQuadrant::North => (IVec2::NEG_Y, true),
            CompassQuadrant::South => (IVec2::ZERO, true),
            CompassQuadrant::East => (IVec2::ZERO, false),
            CompassQuadrant::West => (IVec2::NEG_X, false),
        };
        let owner = pos.checked_add_signed(offset).unwrap();
        let index = self.linearize_conn(owner, is_y).unwrap();
        self.connection.set(index, connected);
    }

    pub fn iter_set_positions(&self) -> impl Iterator<Item = IVec2> + '_ {
        let [x, y] = self.shape.as_array();
        (0..y)
            .flat_map(move |y| (0..x).map(move |x| UVec2::new(x, y)))
            .filter(|&pos| matches!(self.ty[self.shape.linearize(pos) as usize], GCell::Set))
            .map(|pos| pos.as_ivec2())
    }

    pub fn set_brush(&mut self, center: Vec2, radius: i32, to: GCell) {
        let center = center.round().as_ivec2();
        for y in -radius..=radius {
            for x in -radius..=radius {
                let cell = center + IVec2::new(x, y);
                if let Ok(cell) = cell.try_into() {
                    self.paint_cell(cell, to);
                }
            }
        }
    }

    pub fn set_brush_line(&mut self, start: Vec2, end: Vec2, radius: i32, to: GCell) {
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

    pub fn extract_rect(&mut self, rect: URect) -> Option<DynGrid> {
        let size = UVec2::from(self.shape.as_array());
        if rect.min.cmpgt(rect.max).any() || rect.max.cmpgt(size).any() {
            return None;
        }
        let size = rect.size();

        let shape = RuntimeShape::<u32, 2>::new(size);
        let mut grid = DynGrid::new(shape);

        for y in 0..size.y {
            for x in 0..size.x {
                let lpos = UVec2::new(x, y);
                let wpos = lpos + rect.min;

                grid[lpos] = self[wpos];
            }
        }

        // Copy only bonds wholly inside the selection. Every incident source-grid
        // bond is then cleared as its cell is removed, which severs the selection
        // from the world without damaging its internal connectivity.
        for y in 0..size.y {
            for x in 0..size.x {
                // Painting a new cell bonds it to each existing orthogonal neighbor. Erasing a
                let lpos = UVec2::new(x, y);
                let wpos = lpos + rect.min;
                if grid[lpos] != GCell::Set {
                    continue;
                }

                for (offset, is_y, direction) in [
                    (UVec2::X, false, CompassQuadrant::East),
                    (UVec2::Y, true, CompassQuadrant::South),
                ] {
                    let neighbor = lpos + offset;
                    if neighbor.cmplt(size).all()
                        && grid[neighbor] == GCell::Set
                        && self.is_connected_raw(wpos, is_y) == Some(true)
                    {
                        grid.set_connection(lpos, direction, true);
                    }
                }
            }
        }

        for y in 0..size.y {
            for x in 0..size.x {
                self.paint_cell(UVec2::new(x, y) + rect.min, GCell::Unset);
            }
        }

        let has_voxels = grid.iter_set_positions().next().is_some();
        has_voxels.then_some(grid)
    }

    pub fn collider(&self) -> Option<Collider> {
        let voxels = self
            .iter_set_positions()
            .map(|position| position.extend(0))
            .collect::<Vec<_>>();

        (!voxels.is_empty()).then(|| Collider::voxels(Vec3::ONE, &voxels))
    }

    pub fn mass(&self) -> f32 {
        self.iter_set_positions().count() as f32
    }

    pub fn to_dyn(self) -> DynGrid {
        let [x, y] = self.shape.as_array();
        let shape = RuntimeShape::<u32, 2>::new([x, y]);
        DynGrid {
            shape,
            ty: self.ty,
            connection: self.connection,
        }
    }

    pub fn shrink_to_fit(mut self) -> Option<(URect, DynGrid)> {
        let [x, y] = self.shape.as_array();

        // returns the y-coordinate which has a set cell
        let y_fn = |start_incl: u32, stop_excl: u32, step: i32| -> Option<u32> {
            let mut y = start_incl;
            loop {
                for x in 0..x {
                    let pos = UVec2::new(x, y);
                    if matches!(self.get(pos), Some(GCell::Set)) {
                        return Some(y);
                    }
                }
                if y == stop_excl {
                    return None;
                }
                y = y.wrapping_add_signed(step)
            }
        };
        let upper_y = y_fn(y - 1, 0, -1)?;
        let lower_y = y_fn(0, y - 1, 1)?;

        let x_fn = |start_incl: u32, stop_excl: u32, step: i32| -> Option<u32> {
            let mut x = start_incl;
            loop {
                for y in lower_y..=upper_y {
                    let pos = UVec2::new(x, y);
                    if matches!(self.get(pos), Some(GCell::Set)) {
                        return Some(x);
                    }
                }
                if x == stop_excl {
                    return None;
                }
                x = x.wrapping_add_signed(step)
            }
        };
        let upper_x = x_fn(x - 1, 0, -1)?;
        let lower_x = x_fn(0, x - 1, 1)?;

        let rect = URect::new(lower_x, lower_y, upper_x + 1, upper_y + 1);
        Some((rect, self.extract_rect(rect).expect("at least 1 cell")))
    }
}

impl<S: Shape<2, Coord = u32>> Index<UVec2> for Grid<S> {
    type Output = GCell;

    fn index(&self, pos: UVec2) -> &Self::Output {
        self.get(pos).unwrap()
    }
}

impl<S: Shape<2, Coord = u32>> IndexMut<UVec2> for Grid<S> {
    fn index_mut(&mut self, pos: UVec2) -> &mut Self::Output {
        self.get_mut(pos).unwrap()
    }
}

pub fn spawn_world_grid(mut commands: Commands) {
    let grid = ConstGrid::new(SHAPE);
    commands.spawn((
        grid,
        RigidBody::Fixed,
        WorldGrid,
        Anchors,
        ActiveEvents::COLLISION_EVENTS,
    ));

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

#[derive(Clone, Copy, Component)]
pub struct Anchors;

#[derive(Resource, Default)]
pub struct Collisions {
    set: HashSet<(Entity, Entity)>,
}

pub fn handle_grid_collisions<
    S: Shape<2, Coord = u32> + Send + Sync + 'static + Clone + std::fmt::Debug,
>(
    collisions: Res<Collisions>,
    mut commands: Commands,
    mut grids: Query<(Entity, &mut Grid<S>, &GlobalTransform, Option<&Anchors>)>,
    rapier_context: ReadRapierContext,
    mut scratch: Local<super::Scratch>,
) {
    let Ok(rapier_context) = rapier_context.single() else {
        error!("IDK how to handle multiple contexts");
        return;
    };
    for (a, b) in collisions.set.iter().copied() {
        for grid_entity in [a, b] {
            let Ok((_, mut grid, transform, anchors)) = grids.get_mut(grid_entity) else {
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
                    let point_world = if grid_first {
                        transform.transform_point(point.local_p1())
                    } else {
                        transform.transform_point(point.local_p2())
                    };
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

pub fn world_grid_collision_start(
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

pub fn update_colliders(
    mut commands: Commands,
    mut colliders: Query<&mut Collider>,
    consts: Query<(Entity, &ConstGrid), Changed<ConstGrid>>,
    dyns: Query<(Entity, &DynGrid), Changed<DynGrid>>,
) {
    for (entity, grid) in consts.iter() {
        match (colliders.get_mut(entity).ok(), grid.collider()) {
            (Some(mut trgt), Some(src)) => *trgt = src,
            (Some(_), None) => {
                commands.entity(entity).remove::<Collider>();
            }
            (None, Some(src)) => {
                commands.entity(entity).insert(src);
            }
            (None, None) => {}
        }
    }
    for (entity, grid) in dyns.iter() {
        match (colliders.get_mut(entity).ok(), grid.collider()) {
            (Some(mut trgt), Some(src)) => *trgt = src,
            (Some(_), None) => {
                commands.entity(entity).remove::<Collider>();
            }
            (None, Some(src)) => {
                commands.entity(entity).insert(src);
            }
            (None, None) => {}
        }
    }
}

#[derive(Component)]
pub struct Bijection(pub Solution);

pub fn recalculate_bijection(
    mut query: Query<(&GlobalTransform, &DynGrid, &mut Bijection), With<ObjGrid>>,
) {
    for (transform, grid, mut bijection) in query.iter_mut() {
        bijection.0 = solve(
            grid.shape.clone(),
            transform.rotation(),
            transform.translation_vec3a(),
        )
    }
}
