use avian3d::prelude::*;
use bevy::{math::CompassQuadrant, prelude::*};
use bitvec::{bitbox, boxed::BitBox};
use ndshape::{ConstPow2Shape2u32, RuntimeShape, Shape};
use std::ops::{Index, IndexMut};

use crate::ALL;
use crate::projection::{Solution, solve};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GCell {
    Set,
    Unset,
}

pub const SIZE_LOG2: u32 = 8;
pub const SIZE: u32 = 1 << SIZE_LOG2;
pub const SIZE_POW2: u32 = SIZE * SIZE;
pub type S = ConstPow2Shape2u32<SIZE_LOG2, SIZE_LOG2>;
pub const SHAPE: S = S {};

#[derive(Component)]
pub struct WorldGrid;
#[derive(Component)]
pub struct ObjGrid;

pub type ConstGrid = Grid<ConstPow2Shape2u32<SIZE_LOG2, SIZE_LOG2>>;
pub type DynGrid = Grid<RuntimeShape<u32, 2>>;

#[derive(Component)]
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

    pub fn take_from(&mut self, other: &mut Self, pos: UVec2) {
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

    pub fn is_connected_raw(&self, pos: UVec2, y: bool) -> Option<bool> {
        let index = self.linearize_conn(pos, y)?;
        // SAFETY: index is checked to be within bounds by linearize
        Some(unsafe { *self.connection.get_unchecked(index) })
    }

    pub fn is_connected(&self, pos: UVec2, quad: CompassQuadrant) -> Option<bool> {
        match quad {
            CompassQuadrant::North => self.is_connected_raw(pos, true),
            CompassQuadrant::South => self.is_connected_raw(pos - UVec2::new(0, 1), true),
            CompassQuadrant::East => self.is_connected_raw(pos - UVec2::new(1, 0), false),
            CompassQuadrant::West => self.is_connected_raw(pos, false),
        }
    }

    pub fn set_connection(&mut self, pos: UVec2, quad: CompassQuadrant, connected: bool) {
        let (offset, is_y) = match quad {
            CompassQuadrant::North => (UVec2::new(0, 0), true),
            CompassQuadrant::South => (UVec2::new(0, 1), true),
            CompassQuadrant::East => (UVec2::new(1, 0), false),
            CompassQuadrant::West => (UVec2::new(0, 0), false),
        };
        let index = self.linearize_conn(pos + offset, is_y).unwrap();
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
                if let Ok(cell) = cell.try_into()
                    && let Some(ref_mut) = self.get_mut(cell)
                {
                    *ref_mut = to;
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
        if !self.contains(rect.max) {
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
                self[wpos] = GCell::Unset;
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
    commands.spawn((grid, RigidBody::Static, WorldGrid));
}

pub fn world_grid_collision_start(
    mut commands: Commands,
    mut events: MessageReader<CollisionStart>,
    mut grid: Single<&mut ConstGrid, With<WorldGrid>>,
    collisions: Collisions,
    mut scratch: Local<super::Scratch>,
) {
    for event in events.read() {
        info!("{:?}", event);
        let contact_pair = collisions.get(event.collider1, event.collider2).unwrap();
        for manifold in &contact_pair.manifolds {
            for point in &manifold.points {
                let result = super::solve(
                    &mut *grid,
                    &mut *scratch,
                    (point.normal_impulse * manifold.normal).truncate(),
                    point.point.truncate(),
                    20,
                );
                for (grid, collider, lin, ang) in result {
                    commands.spawn((grid, collider, lin, ang));
                }
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
