/// Anchors. So another possibility is that we have a set of anchors each who keeps track of all cells they are anchoring (and optionally a lookup table).
/// And then when we remove anchors then we recalculate anchorship for all cells it anchored. BUT this doesn't work because we also want to be able to remove
/// non-anchor cells. The other option is we have a tree structure with the anchor sa the root basically we can flood from the anchor assigning each cell to
/// the cell that keeps it in place. Then whenever we remove a cell we can flood all dependent cells as no-longer anchoring and then reflood anchors. This might work
/// but it's inherantly non-parrallel (not only at construction) for every operation (and might also be recursive AKA not very good).
use std::num::NonZero;

use bevy::ecs::component::Component;
use bevy::math::{IVec2, Vec2};
use bevy::platform::collections::{HashMap, HashSet};

use crate::grid::{GCell, SIZE, SIZE_POW2, WorldGrid};

use super::octree::Octree;

use super::grid::SIZE_LOG2;

const DEPTH: u32 = SIZE_LOG2 as u32;

fn linearize(pos: IVec2) -> usize {
    pos.x as usize + pos.y as usize * SIZE
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Left,
    Right,
    Up,
    Down,
}

impl Direction {
    const ALL: [Self; 4] = [Self::Left, Self::Right, Self::Up, Self::Down];

    pub fn inv(self) -> Self {
        match self {
            Self::Left => Self::Right,
            Self::Right => Self::Left,
            Self::Up => Self::Down,
            Self::Down => Self::Up,
        }
    }

    pub fn to_ivec2(self) -> IVec2 {
        match self {
            Self::Left => IVec2::new(-1, 0),
            Self::Right => IVec2::new(1, 0),
            Self::Up => IVec2::new(0, -1),
            Self::Down => IVec2::new(0, 1),
        }
    }
}

#[derive(Component)]
pub struct Connectivity {
    // Theoretically a minimum of 3 bits per cell (5 states. If we can find a way to elide the None state we can save 1 bit per cell)
    pub anchors: Box<[Option<Direction>; SIZE_POW2]>,
}

impl Connectivity {
    #[track_caller]
    pub fn get(&self, pos: IVec2) -> Option<Direction> {
        let linearized = linearize(pos);
        self.anchors[linearized]
    }

    fn _flood(&mut self, pos: IVec2, from: Direction, world: &WorldGrid) {
        let linearized = linearize(pos);
        self.anchors[linearized] = Some(from);

        for dir in Direction::ALL {
            if dir == from.inv() {
                continue;
            }
            let neighbor = pos + dir.to_ivec2();

            let ge = neighbor.cmpge(IVec2::ZERO).all();
            let lt = neighbor.cmplt(IVec2::splat(SIZE as i32)).all();
            if !(ge && lt) {
                continue;
            }

            if self.get(neighbor).is_some() {
                return;
            }
            if world.get(neighbor) == GCell::Unset {
                return;
            }
            self._flood(neighbor, dir, world);
        }
    }

    pub fn detect_detatched(&self, world: &WorldGrid) -> Vec<Vec<IVec2>> {
        let mut groups = Vec::new();
        // wheweee this is inefficient
        let mut all = world.iter_set_positions().collect::<HashSet<IVec2>>();
        for pos in world.iter_set_positions() {
            if !all.remove(&pos) {
                continue;
            }
            if self.get(pos).is_some() {
                continue;
            }
            let mut group = Vec::new();
            // we naturally iterate top to bottom, left to right
            self.flood_detect(world, pos, &mut group, &mut all, Direction::Right);
            groups.push(group);
        }
        groups
    }

    fn flood_detect(
        &self,
        world: &WorldGrid,
        pos: IVec2,
        group: &mut Vec<IVec2>,
        all: &mut HashSet<IVec2>,
        from: Direction,
    ) {
        group.push(pos);
        for direction in Direction::ALL {
            if direction == from.inv() {
                continue;
            }
            let neighbor = pos + direction.to_ivec2();
            if world.get(neighbor) == GCell::Set && self.get(neighbor).is_none() {
                self.flood_detect(world, neighbor, group, all, direction);
                all.remove(&neighbor);
            }
        }
    }

    pub fn flood(&mut self, world: &WorldGrid) {
        self.anchors.fill(None);

        // top row
        for x in 0..SIZE {
            let pos = IVec2::new(x as i32, 0);
            self._flood(pos, Direction::Down, world);
        }

        // sides
        for y in 1..SIZE - 1 {
            for dir in [Direction::Right, Direction::Left] {
                let x = match dir {
                    Direction::Right => SIZE - 1,
                    Direction::Left => 0,
                    _ => unreachable!(),
                };
                let pos = IVec2::new(x as i32, y as i32);
                self._flood(pos, dir, world);
            }
        }

        // bottom row
        for x in 0..SIZE {
            let pos = IVec2::new(x as i32, (SIZE - 1) as i32);
            self._flood(pos, Direction::Up, world);
        }
    }
}

impl Default for Connectivity {
    fn default() -> Self {
        Self {
            anchors: vec![None; SIZE_POW2].into_boxed_slice().try_into().unwrap(),
        }
    }
}

pub fn set(world: &mut WorldGrid, connectivity: &mut Connectivity, pos: IVec2, to: GCell) {
    world.set(pos, to);

    connectivity.flood(world);

    let detatched_groups = connectivity.detect_detatched(world);
    println!("Detatched groups: {}", detatched_groups.len());
}

pub fn set_line(
    world: &mut WorldGrid,
    connectivity: &mut Connectivity,
    start: Vec2,
    end: Vec2,
    to: GCell,
) {
    world.set_line(start, end, to);

    connectivity.flood(world);

    let detatched_groups = connectivity.detect_detatched(world);
    println!("Detatched groups: {}", detatched_groups.len());
}

pub fn set_brush(
    world: &mut WorldGrid,
    connectivity: &mut Connectivity,
    center: Vec2,
    radius: i32,
    to: GCell,
) {
    world.set_brush(center, radius, to);

    connectivity.flood(world);

    let detatched_groups = connectivity.detect_detatched(world);
    println!("Detatched groups: {}", detatched_groups.len());
}

pub fn set_brush_line(
    world: &mut WorldGrid,
    connectivity: &mut Connectivity,
    start: Vec2,
    end: Vec2,
    radius: i32,
    to: GCell,
) {
    world.set_brush_line(start, end, radius, to);

    connectivity.flood(world);

    let detatched_groups = connectivity.detect_detatched(world);
    println!("Detatched groups: {}", detatched_groups.len());
}
