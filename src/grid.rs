use bevy::{math::USizeVec2, prelude::*};
use ndshape::{ConstPow2Shape2usize, Shape as _};

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

#[derive(Resource, Clone)]
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
    pub fn iter_set_positions(&self) -> impl Iterator<Item = IVec2> + '_ {
        (0..SIZE)
            .flat_map(|y| (0..SIZE).map(move |x| USizeVec2::new(x, y)))
            .filter(|&pos| matches!(self.data[SHAPE.linearize(pos)], GCell::Set))
            .map(|pos| pos.as_ivec2())
    }

    pub fn get(&self, pos: IVec2) -> GCell {
        assert!(pos.cmpge(IVec2::ZERO).all());
        assert!(pos.cmplt(IVec2::splat(SIZE as i32)).all());
        let pos = pos.as_usizevec2();
        let index = SHAPE.linearize(pos);
        self.data[index]
    }

    pub fn set(&mut self, pos: IVec2, to: GCell) {
        assert!(pos.cmpge(IVec2::ZERO).all());
        assert!(pos.cmplt(IVec2::splat(SIZE as i32)).all());
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
            if cell.cmpge(IVec2::ZERO).all() && cell.cmplt(IVec2::splat(SIZE as i32)).all() {
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
}
