use bevy::prelude::*;
use ndshape::{RuntimeShape, Shape as _};
use sparse_linear_assignment::{AuctionSolver as _, KhoslaSolver};

pub fn solve(src_shape: RuntimeShape<u32, 2>, rotation: Quat, translation: Vec3A) -> Solution {
    let [x, y] = src_shape.as_array();

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

    let num_rows = src_shape.size();
    let num_cols = dst_shape.size();

    let row_capacity = num_rows as usize;
    let column_capacity = num_cols as usize;
    // bilinear sampling returns 4 points per source cell
    let arcs_capacity = row_capacity * 4;

    let (mut solver, mut solution) =
        KhoslaSolver::<u32>::new(row_capacity, column_capacity, arcs_capacity);
    solver.init(num_rows, num_cols).unwrap();

    for (idx, pos) in iter_tuple_idx_pos(&src_shape) {
        let point = pos.as_vec2();
        let w_point = transform_point(rotation, translation, point);
        let bilinear = bilinear(w_point);
        let columns = bilinear.map(|w_pos| dst_shape.linearize((w_pos - dst_origin).as_uvec2()));
        let values = bilinear.map(|w_pos| w_pos.as_vec2().distance_squared(w_point) as f64);
        // this expects row major ordering which is ensured by the `bilinear` function
        solver.extend_from_values(idx, &columns, &values).unwrap();
    }

    solver.solve(&mut solution, false, None).unwrap();
    assert_eq!(solution.num_unassigned, 0);

    Solution {
        src_to_dst: solution.person_to_object,
        dst_to_src: solution.object_to_person,
        src_shape,
        dst_shape,
        dst_origin,
    }
}

pub struct Solution {
    src_to_dst: Vec<u32>,
    dst_to_src: Vec<u32>,
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
        let local_pos = UVec2::from_array(self.dst_shape.delinearize(dst_idx));
        let pos = local_pos.as_ivec2() + self.dst_origin;
        Some(pos)
    }

    pub fn dst_to_src(&self, pos: IVec2) -> Option<UVec2> {
        let local_pos = (pos - self.dst_origin).as_uvec2();
        if !self.dst_shape.contains(local_pos) {
            return None;
        }
        let dst_idx = self.dst_shape.linearize(local_pos) as usize;
        let src_idx = self.dst_to_src[dst_idx];
        // dst_cells are not guaranteed to map to src_cells
        if src_idx == u32::MAX {
            return None;
        }
        let pos = UVec2::from_array(self.src_shape.delinearize(src_idx));
        Some(pos)
    }
}

/// Copy of [`Transform::transform_point`] without scaling
///
/// Using [`Vec3A`] vectors because we have to convert anyway
#[inline]
fn transform_point(rotation: Quat, translation: Vec3A, point: Vec2) -> Vec2 {
    let mut point = Vec3A::new(point.x, point.y, 0.);
    point = rotation * point;
    point += translation;
    point.xy()
}

#[inline]
pub fn iter_tuple_idx_pos(shape: &RuntimeShape<u32, 2>) -> impl Iterator<Item = (u32, UVec2)> + '_ {
    let [x, y] = shape.as_array();
    (0..y).flat_map(move |y| (0..x).map(move |x| (shape.linearize([x, y]), UVec2::new(x, y))))
}

#[inline]
fn bilinear(mut vec: Vec2) -> [IVec2; 4] {
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
