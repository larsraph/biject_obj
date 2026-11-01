use bevy::prelude::*;
use ndshape::{RuntimeShape, Shape as _};
use sparse_linear_assignment::{AuctionSolver as _, KhoslaSolver};

use crate::grid::Grid;

pub fn solve(grid: &Grid, rotation: Quat, translation: Vec3A) -> Solution {
    let src_shape = grid.shape;
    assert!(
        translation.is_finite() && rotation.is_finite(),
        "non-finite transform"
    );
    assert!(
        translation.abs().cmplt(Vec3A::splat(i32::MAX as f32)).all(),
        "translation out of range"
    );
    let [x, y] = src_shape.as_array();
    assert!(x > 0 && y > 0, "cannot project an empty grid");

    let mut min = IVec2::MAX;
    let mut max = IVec2::MIN;
    for y in [0, y - 1] {
        for x in [0, x - 1] {
            let corner = UVec2::new(x, y);
            let point = corner.as_vec2();
            let w_pos = transform_point(rotation, translation, point);
            let floor = w_pos.floor().as_ivec2();
            let ceil = w_pos.ceil().as_ivec2();
            min = min.min(floor);
            max = max.max(ceil);
        }
    }
    let dst_origin = min;
    let dst_size = (max - min).as_uvec2() + UVec2::ONE;
    let dst_shape = RuntimeShape::<u32, 2>::new(dst_size);

    // Only occupied cells participate. The solver rejects empty rows.
    let sources: Vec<_> = grid.iter_set_positions().collect();
    let mut src_to_dst = vec![u32::MAX; src_shape.usize()];
    if sources.is_empty() {
        return Solution {
            src_to_dst,
            src_shape,
            dst_shape,
            dst_origin,
        };
    }
    let num_rows = sources.len() as u32;
    let num_cols = dst_shape.size();
    let (mut solver, mut solution) =
        KhoslaSolver::<u32>::new(sources.len(), num_cols as usize, sources.len() * 4);
    solver.init(num_rows, num_cols).unwrap_or_else(|error| {
        panic!("projection initialization failed at {translation:?}, {rotation:?}: {error:?}")
    });

    for (row, pos) in sources.iter().enumerate() {
        let w_point = transform_point(rotation, translation, pos.as_vec2());
        let candidates = bilinear(w_point);
        let mut columns = [0; 4];
        let mut values = [0.0; 4];
        let mut count = 0;
        for candidate in candidates {
            let column = dst_shape.linearize((candidate - dst_origin).as_uvec2());
            if columns[..count].contains(&column) {
                continue;
            }
            columns[count] = column;
            values[count] = candidate.as_vec2().distance_squared(w_point) as f64;
            count += 1;
        }
        solver
            .extend_from_values(row as u32, &columns[..count], &values[..count])
            .unwrap();
    }

    solver.solve(&mut solution, false, None).unwrap();

    // TODO: unfortunately this invariant is NOT currently upheld by our solver.
    // assert_eq!(solution.num_unassigned, 0);

    for (pos, destination) in sources.iter().zip(solution.person_to_object) {
        src_to_dst[src_shape.linearize(*pos) as usize] = destination;
    }
    Solution {
        src_to_dst,
        src_shape,
        dst_shape,
        dst_origin,
    }
}

pub struct Solution {
    src_to_dst: Vec<u32>,
    src_shape: RuntimeShape<u32, 2>,
    dst_shape: RuntimeShape<u32, 2>,
    dst_origin: IVec2,
}

impl Solution {
    pub fn src_to_dst(&self, pos: UVec2) -> Option<IVec2> {
        if !self.src_shape.contains(pos) {
            return None;
        }
        let src_idx = self.src_shape.linearize(pos) as usize;
        let dst_idx = self.src_to_dst[src_idx];
        // TODO: see invariant in solve() that should be upheld
        if dst_idx == u32::MAX {
            return None;
        }
        let local_pos = UVec2::from_array(self.dst_shape.delinearize(dst_idx));
        let pos = local_pos.as_ivec2() + self.dst_origin;
        Some(pos)
    }
}

/// Transforms a voxel center and expresses it in destination minimum-corner coordinates.
#[inline]
fn transform_point(rotation: Quat, translation: Vec3A, point: Vec2) -> Vec2 {
    // Source and destination integers identify minimum corners. Match the
    // transformed voxel center to destination centers, then return corner space.
    let mut point = Vec3A::new(point.x + 0.5, point.y + 0.5, 0.5);
    point = rotation * point;
    point += translation;
    point.xy() - Vec2::splat(0.5)
}

#[inline]
fn bilinear(vec: Vec2) -> [IVec2; 4] {
    let floor = vec.floor();
    let ceil = vec.ceil();
    // row major ordering
    [
        floor.as_ivec2(),
        Vec2::new(ceil.x, floor.y).as_ivec2(),
        Vec2::new(floor.x, ceil.y).as_ivec2(),
        ceil.as_ivec2(),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn integer_translation_maps_every_source_cell() {
        let shape = RuntimeShape::<u32, 2>::new([3, 2]);
        let mut grid = Grid::new(shape);
        grid.set_brush(Vec2::ZERO, 4, Some(()));
        let solution = solve(&grid, Quat::IDENTITY, Vec3A::new(-4.0, 7.0, 0.0));
        for pos in grid.iter_set_positions() {
            assert_eq!(
                solution.src_to_dst(pos),
                Some(pos.as_ivec2() + IVec2::new(-4, 7))
            );
        }
        assert_eq!(solution.src_to_dst(UVec2::MAX), None);
    }

    #[test]
    #[should_panic(expected = "cannot project an empty grid")]
    fn empty_grid_is_rejected_before_corner_arithmetic() {
        solve(
            &Grid::new(RuntimeShape::<u32, 2>::new([0, 2])),
            Quat::IDENTITY,
            Vec3A::ZERO,
        );
    }
    #[test]
    fn sparse_projection_only_assigns_occupied_cells() {
        let mut grid = Grid::new(RuntimeShape::<u32, 2>::new([4, 3]));
        grid.paint_cell(UVec2::new(2, 1), Some(()));
        grid.paint_cell(UVec2::new(3, 2), Some(()));
        let solution = solve(&grid, Quat::IDENTITY, Vec3A::ZERO);
        assert_eq!(solution.src_to_dst(UVec2::ZERO), None);
        assert_eq!(
            solution.src_to_dst(UVec2::new(2, 1)),
            Some(IVec2::new(2, 1))
        );
        assert_eq!(
            solution.src_to_dst(UVec2::new(3, 2)),
            Some(IVec2::new(3, 2))
        );
    }

    #[test]
    fn quarter_turn_projects_voxel_center_to_correct_minimum_corner() {
        let mut grid = Grid::new(RuntimeShape::<u32, 2>::new([1, 1]));
        grid.paint_cell(UVec2::ZERO, Some(()));
        let solution = solve(
            &grid,
            Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
            Vec3A::ZERO,
        );
        assert_eq!(solution.src_to_dst(UVec2::ZERO), Some(IVec2::new(-1, 0)));
    }

    #[test]
    fn empty_occupancy_does_not_enter_assignment_solver() {
        let grid = Grid::new(RuntimeShape::<u32, 2>::new([2, 2]));
        let solution = solve(&grid, Quat::IDENTITY, Vec3A::ZERO);
        assert_eq!(solution.src_to_dst(UVec2::ZERO), None);
    }
}
