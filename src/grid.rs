use avian3d::prelude::*;
use bevy::{math::USizeVec2, prelude::*};
use ndshape::{ConstPow2Shape2usize, RuntimeShape, Shape as _};

use crate::projection::{Solution, solve};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GCell {
    Set,
    Unset,
}

pub const SIZE_LOG2: usize = 8;
pub const SIZE: usize = 1 << SIZE_LOG2;
pub const SIZE_POW2: usize = SIZE * SIZE;
pub type S = ConstPow2Shape2usize<SIZE_LOG2, SIZE_LOG2>;
pub const SHAPE: S = S {};

#[derive(Component)]
pub struct ObjGrid;

#[derive(Component, Clone)]
pub struct WorldGrid {
    pub data: Box<[GCell; SIZE_POW2]>,
}

impl Default for WorldGrid {
    fn default() -> Self {
        Self {
            data: vec![GCell::Unset; SIZE_POW2]
                .into_boxed_slice()
                .try_into()
                .unwrap(),
        }
    }
}

impl WorldGrid {
    fn collider(&self) -> Collider {
        let size = SIZE as i32;
        let mut voxels = Vec::with_capacity(4 * SIZE + self.iter_set_positions().count());

        for x in -1..=size {
            voxels.push(IVec3::new(x, -1, 0));
            voxels.push(IVec3::new(x, size, 0));
        }
        for y in 0..size {
            voxels.push(IVec3::new(-1, y, 0));
            voxels.push(IVec3::new(size, y, 0));
        }

        voxels.extend(self.iter_set_positions().map(|position| position.extend(0)));
        Collider::voxels(Vec3::ONE, &voxels)
    }

    pub fn iter_set_positions(&self) -> impl Iterator<Item = IVec2> + '_ {
        (0..SIZE)
            .flat_map(|y| (0..SIZE).map(move |x| USizeVec2::new(x, y)))
            .filter(|&pos| matches!(self.data[SHAPE.linearize(pos)], GCell::Set))
            .map(|pos| pos.as_ivec2())
    }

    pub fn get(&self, pos: IVec2) -> GCell {
        assert!(contains(pos));
        let pos = pos.as_usizevec2();
        let index = SHAPE.linearize(pos);
        self.data[index]
    }

    pub fn set(&mut self, pos: IVec2, to: GCell) {
        assert!(contains(pos));
        let pos = pos.as_usizevec2();
        let index = SHAPE.linearize(pos);
        self.data[index] = to;
    }

    // I had AI do this but it's like a really common algorithm
    pub fn set_line(&mut self, start: Vec2, end: Vec2, to: GCell) {
        let mut cell = start.round().as_ivec2();
        let end = end.round().as_ivec2();
        let delta = (end - cell).abs();
        let step = (end - cell).signum();
        let mut error = delta.x - delta.y;

        loop {
            if contains(cell) {
                self.set(cell, to);
            }
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

    pub fn set_brush(&mut self, center: Vec2, radius: i32, to: GCell) {
        let center = center.round().as_ivec2();
        for y in -radius..=radius {
            for x in -radius..=radius {
                let cell = center + IVec2::new(x, y);
                if contains(cell) {
                    self.set(cell, to);
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

    pub fn extract(&mut self, from: IVec2, to: IVec2) -> Option<Grid> {
        let min = from.min(to);
        let max = to.max(from);
        if !contains(min) || !contains(max) {
            return None;
        }
        let min = clamp(min);
        let max = clamp(max);
        let size = (max - min + IVec2::ONE).as_uvec2();

        let shape = RuntimeShape::<u32, 2>::new(size);
        let mut grid = Grid::new(shape);

        let mut once = false;
        for y in 0..size.y {
            for x in 0..size.x {
                let gpos = IVec2::new(x as i32, y as i32);
                let wpos = gpos + min;

                let value = self.get(wpos);
                grid.set(gpos, value);
                self.set(wpos, GCell::Unset);

                if matches!(value, GCell::Set) {
                    once = true;
                }
            }
        }
        once.then_some(grid)
    }
}

pub fn spawn_world_grid(mut commands: Commands) {
    let grid = WorldGrid::default();
    let collider = grid.collider();
    commands.spawn((grid, RigidBody::Static, collider));
}

pub fn update_world_grid_collider(
    collider: Single<(&mut Collider, &WorldGrid), Changed<WorldGrid>>,
) {
    let (mut collider, grid) = collider.into_inner();
    *collider = grid.collider();
}

fn contains(pos: IVec2) -> bool {
    pos.cmpge(IVec2::ZERO).all() && pos.cmplt(IVec2::splat(SIZE as i32)).all()
}

fn clamp(pos: IVec2) -> IVec2 {
    pos.clamp(IVec2::ZERO, IVec2::splat(SIZE as i32))
}

#[derive(Component)]
pub struct Grid {
    data: Vec<GCell>,
    pub shape: RuntimeShape<u32, 2>,
}

impl Grid {
    pub fn new(shape: RuntimeShape<u32, 2>) -> Self {
        Self {
            data: vec![GCell::Unset; shape.usize()],
            shape,
        }
    }

    pub fn set(&mut self, pos: IVec2, cell: GCell) {
        let index = self.shape.linearize(pos.as_uvec2()) as usize;
        self.data[index] = cell;
    }

    pub fn get(&self, pos: IVec2) -> GCell {
        let index = self.shape.linearize(pos.as_uvec2()) as usize;
        self.data[index]
    }

    pub fn iter_set_positions(&self) -> impl Iterator<Item = IVec2> + '_ {
        let [x, y] = self.shape.as_array();
        (0..y)
            .flat_map(move |y| (0..x).map(move |x| UVec2::new(x, y)))
            .filter(|&pos| matches!(self.data[self.shape.linearize(pos) as usize], GCell::Set))
            .map(|pos| pos.as_ivec2())
    }

    pub fn collider(&self) -> Collider {
        let voxels = self
            .iter_set_positions()
            .map(|position| position.extend(0))
            .collect::<Vec<_>>();

        Collider::voxels(Vec3::ONE, &voxels)
    }

    pub fn mass(&self) -> f32 {
        self.iter_set_positions().count() as f32
    }
}

#[derive(Component)]
pub struct Bijection(pub Solution);

pub fn recalculate_bijection(
    mut query: Query<
        (
            &GlobalTransform,
            &Grid,
            &mut Bijection,
            &MaxLinearSpeed,
            &LinearVelocity,
        ),
        (With<ObjGrid>),
    >,
) {
    for (transform, grid, mut bijection, max, vel) in query.iter_mut() {
        assert!(vel.length() <= max.0);
        bijection.0 = solve(
            grid.shape.clone(),
            transform.rotation(),
            transform.translation_vec3a(),
        )
    }
}
