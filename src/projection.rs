//! An algorithm that creates a bijection AKA its a two way map from each source to each target cell.
//!
//! Since the end reuslt of the bijection is too expensive to store on the fly, it makes sense
//! to compute two tables, each the inverse of eachother, mapping from each grid. For now though
//! it will be simpliest to have the maps as HashMap<IVec3, IVec3>. In addition each of these grid
//! are sparse since only non-empty cells need to be included in the bijection.
//!
//! I'm currently using `sparse_linear_assignment` to solve this which has bad documentation and
//! is not idiomatic rust... it should work it's just ugly as hell.
use std::iter;

use bevy::{platform::collections::HashMap, prelude::*};
use ndshape::{AbstractShape, RuntimeShape};
use sparse_linear_assignment::{AuctionSolution, AuctionSolver, KhoslaSolver};

use crate::Grid;

struct BodyGrid {}

impl BodyGrid {
    fn iter_lin_delin_pos(&self) -> impl Iterator<Item = (usize, IVec2)> + '_ {
        iter::empty()
    }

    fn num_cells(&self) -> usize {
        0
    }
}

/// The `target` must have no Transform OR we project the body into the target space (currently assuming target space == world space)
fn solve(
    body: &BodyGrid,
    body_transform: &GlobalTransform,
    shape: RuntimeShape<u32, 2>,
) -> Solution {
    let (mut solver, mut solution) = KhoslaSolver::<u32>::new(0, 0, 0);

    // The target is infinite but we will have at least the number of columns
    let num_rows = body.num_cells() as u32;
    solver.init(num_rows, num_rows).unwrap();

    // instead of handling this sparse mapping we could use direct indexing on `target`, although
    // it's infinite we *do* know that only a subset of it is targetable, so we could create
    // a direct addressing sub-grid that delinearizes instead of doing a table lookup.
    let mut col_to_trgt = Vec::new();
    let mut trgt_to_col = HashMap::new();

    for (row, pos) in body.iter_lin_delin_pos() {
        let w_pos = body_transform
            .transform_point(pos.as_vec2().extend(0.))
            .xy();
        let bilinear = bilinear(w_pos);
        let columns = bilinear.map(|bpos| {
            trgt_to_col.get(&bpos).copied().unwrap_or_else(|| {
                let col = col_to_trgt.len() as u32;
                col_to_trgt.push(bpos);
                trgt_to_col.insert(bpos, col);
                col
            })
        });
        let values = bilinear.map(|bpos| bpos.as_vec2().distance_squared(w_pos) as f64);
        solver
            .extend_from_values(row as u32, &columns, &values)
            .unwrap();
    }

    solver.solve(&mut solution, false, None).unwrap();

    Solution {
        shape,
        auct_solution: solution,
        col_to_trgt,
        trgt_to_col,
    }
}

struct Solution {
    shape: RuntimeShape<u32, 2>,
    auct_solution: AuctionSolution<u32>,
    // It will be far fewer lookups, although probably more memory, to have this
    // mapping be direct indexing
    col_to_trgt: Vec<IVec2>,
    trgt_to_col: HashMap<IVec2, u32>,
}

impl Solution {
    /// Panicky RN
    pub fn src_to_dst(&self, pos: UVec2) -> IVec2 {
        let col =
            self.auct_solution.person_to_object[self.shape.linearize(pos.to_array()) as usize];
        self.col_to_trgt[col as usize]
    }

    /// Panicky RN
    pub fn dst_to_src(&self, pos: IVec2) -> UVec2 {
        let col = self.trgt_to_col[&pos];
        let person = self.auct_solution.object_to_person[col as usize];
        self.shape.delinearize(person).into()
    }
}

#[inline(always)]
fn bilinear(vec: Vec2) -> [IVec2; 4] {
    let Vec2 { x, y } = vec;
    let x_floor = x.floor() as i32;
    let y_floor = y.floor() as i32;
    let x_ceil = x.ceil() as i32;
    let y_ceil = y.ceil() as i32;

    [
        IVec2::new(x_floor, y_floor),
        IVec2::new(x_ceil, y_floor),
        IVec2::new(x_floor, y_ceil),
        IVec2::new(x_ceil, y_ceil),
    ]
}
