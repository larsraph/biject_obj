//! An algorithm that creates a bijection AKA its a two way map from each source to each target cell.
//!
//! Since the end reuslt of the bijection is too expensive to store on the fly, it makes sense
//! to compute two tables, each the inverse of eachother, mapping from each grid. For now though
//! it will be simpliest to have the maps as HashMap<IVec3, IVec3>. In addition each of these grid
//! are sparse since only non-empty cells need to be included in the bijection.
//!
//! I'm currently using `sparse_linear_assignment` to solve this which has bad documentation and
//! is not idiomatic rust... it should work it's just ugly as hell.

use bevy::prelude::*;
use ndshape::{RuntimeShape, Shape};
use sparse_linear_assignment::{AuctionSolution, AuctionSolver, KhoslaSolver};

pub struct DenseGrid<T> {
    pub data: Vec<T>,
    /// TODO: `data.len` is duped in `shape.size`
    /// TODO: fast division in delinearization
    pub shape: RuntimeShape<u32, 2>,
}

impl<T> DenseGrid<T> {
    pub fn new(dims: UVec2, fill: T) -> Self
    where
        T: Clone,
    {
        let shape = RuntimeShape::<u32, 2>::new(dims.to_array());
        let data = vec![fill; shape.usize()];
        Self { shape, data }
    }
}

/// The `target` must have no Transform OR we project the body into the target space (currently assuming target space == world space)
fn solve(src_shape: RuntimeShape<u32, 2>, rotation: Quat, translation: Vec3) -> Solution {
    let [x, y] = src_shape.as_array();

    let mut min = IVec2::MIN;
    let mut max = IVec2::MAX;
    for y in [0, y] {
        for x in [0, x] {
            let corner = UVec2::new(x, y);
            let point = corner.as_vec2();
            let w_pos = transform_point(rotation, translation, point);
            // bilinear() is pretty cheap but I think there should be a way to use
            // which corner we're looking at + the rotation to quickly figure out
            // which bilinear corner is the one that matters
            for pos in bilinear(w_pos) {
                min = min.min(pos);
                max = max.max(pos);
            }
        }
    }
    let trgt_origin = min;
    let trgt_size = (max - min).as_uvec2();
    let trgt_shape = RuntimeShape::<u32, 2>::new(trgt_size.to_array());

    let (mut solver, mut solution) = KhoslaSolver::<u32>::new(0, 0, 0);

    let num_rows = src_shape.size();
    let num_cols = trgt_shape.size();
    solver
        .init(num_rows, num_cols)
        // this shouldn't be an error in the first place because it just improves alloc perf not correctness
        .expect("the projected AABB-similar shape is always >= the source shape");

    for (idx, pos) in iter_tuple_idx_pos(&src_shape) {
        let point = pos.as_vec2();
        let w_point = transform_point(rotation, translation, point);
        let bilinear = bilinear(w_point);
        // columns should be ordered row major because bilinear samples are ordered row major
        let columns =
            bilinear.map(|w_pos| trgt_shape.linearize((w_pos - trgt_origin).as_uvec2().to_array()));
        // I'm fairly certain correcting by 0.5 is the right thing to do here
        // wait it's not unless I also correct by 0.5 before bilinear sampling, which is only correct
        // if I render cells centered about their "position" which classically is not how it's done
        let values = bilinear
            .map(|w_pos| (w_pos.as_vec2() + Vec2::splat(0.5)).distance_squared(w_point) as f64);
        solver
            .extend_from_values(idx, &columns, &values)
            .expect("this crate keeps giving `anyhow::Error`... anyway this function wont fail");
    }

    solver.solve(&mut solution, false, None).unwrap();

    Solution {
        solution,
        src_shape,
        trgt_shape,
        trgt_origin,
    }
}

struct Solution {
    solution: AuctionSolution<u32>,
    src_shape: RuntimeShape<u32, 2>,
    trgt_shape: RuntimeShape<u32, 2>,
    trgt_origin: IVec2,
}

impl Solution {
    pub fn src_to_dst(&self, pos: UVec2) -> Option<IVec2> {
        let src_idx = self.src_shape.linearize(pos.to_array());
        let dst_idx = *self.solution.person_to_object.get(src_idx as usize)?;
        let local_pos = UVec2::from_array(self.trgt_shape.delinearize(dst_idx));
        Some(local_pos.as_ivec2() + self.trgt_origin)
    }

    pub fn dst_to_src(&self, pos: IVec2) -> Option<UVec2> {
        let local_pos = (pos - self.trgt_origin).as_uvec2();
        let dst_idx = self.trgt_shape.linearize(local_pos.to_array());
        let src_idx = *self.solution.object_to_person.get(dst_idx as usize)?;
        Some(UVec2::from_array(self.src_shape.delinearize(src_idx)))
    }
}

/// Doesn't include shear hence why i have an (almost) copy of a function
#[inline]
fn transform_point(rotation: Quat, translation: Vec3, point: Vec2) -> Vec2 {
    let point = Vec3A::new(point.x, point.y, 0.);
    let point = rotation * point;
    let point = point + translation.to_vec3a();
    point.xy()
}

#[inline]
pub fn iter_tuple_idx_pos(shape: &RuntimeShape<u32, 2>) -> impl Iterator<Item = (u32, UVec2)> + '_ {
    let [x, y] = shape.as_array();
    (0..y).flat_map(move |y| (0..x).map(move |x| (shape.linearize([x, y]), UVec2::new(x, y))))
}

#[inline(always)]
fn bilinear(vec: Vec2) -> [IVec2; 4] {
    let floor = vec.floor();
    let ceil = vec.ceil();
    // Ordered row major asc
    [
        floor.as_ivec2(),
        Vec2::new(ceil.x, floor.y).as_ivec2(),
        Vec2::new(floor.x, ceil.y).as_ivec2(),
        ceil.as_ivec2(),
    ]
}
