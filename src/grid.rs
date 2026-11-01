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
        self.contains(pos).then(|| self.linearize(pos))
    }

    #[track_caller]
    pub fn linearize_expect(&self, pos: UVec2) -> usize {
        assert!(self.contains(pos), "{:?} oob of {:?}", pos, self.size());
        self.linearize(pos)
    }

    pub fn linearize_edge_checked(&self, pos: UVec2, dir: CompassQuadrant) -> Option<usize> {
        let neighbor = pos.wrapping_add_signed(dir.ivec2());
        if !self.contains(pos) || !self.contains(neighbor) {
            return None;
        }
        let (owner, axis) = match dir {
            CompassQuadrant::North => (pos, 1),
            CompassQuadrant::East => (pos, 0),
            CompassQuadrant::South => (neighbor, 1),
            CompassQuadrant::West => (neighbor, 0),
        };
        Some(self.linearize(owner) * 2 + axis)
    }

    #[track_caller]
    pub fn get(&self, pos: UVec2) -> &Cell {
        &self.values[self.linearize_expect(pos)]
    }

    #[track_caller]
    pub fn get_mut(&mut self, pos: UVec2) -> &mut Cell {
        let index = self.linearize_expect(pos);
        &mut self.values[index]
    }

    pub fn get_edge(&self, pos: UVec2, dir: CompassQuadrant) -> bool {
        self.linearize_edge_checked(pos, dir)
            .is_some_and(|index| self.edges[index])
    }

    pub fn set_edge(&mut self, pos: UVec2, dir: CompassQuadrant, connected: bool) {
        if let Some(index) = self.linearize_edge_checked(pos, dir) {
            self.edges.set(index, connected);
        }
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

    /// Paints a continuous line using Bresenham's algorithm.
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
        let rect = self.clip_rect(rect);
        if rect.is_empty() {
            return;
        }
        for x in rect.min.x..rect.max.x {
            self.set_edge(UVec2::new(x, rect.min.y), CompassQuadrant::South, false);
            self.set_edge(
                UVec2::new(x, rect.max.y.saturating_sub(1)),
                CompassQuadrant::North,
                false,
            );
        }
        for y in rect.min.y..rect.max.y {
            self.set_edge(UVec2::new(rect.min.x, y), CompassQuadrant::West, false);
            self.set_edge(
                UVec2::new(rect.max.x.saturating_sub(1), y),
                CompassQuadrant::East,
                false,
            );
        }
    }

    fn clip_rect(&self, rect: URect) -> URect {
        URect::from_corners(rect.min.min(self.size()), rect.max.min(self.size()))
    }

    pub fn copy_rect(&self, rect: URect) -> (bool, Self) {
        let rect = self.clip_rect(rect);
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
                    if grid.contains(lpos.wrapping_add_signed(quad.ivec2()))
                        && self.get_edge(wpos, quad)
                    {
                        grid.set_edge(lpos, quad, true);
                    }
                }
            }
        }

        (any, grid)
    }

    pub fn clear_rect(&mut self, rect: URect) {
        let rect = self.clip_rect(rect);
        for y in rect.min.y..rect.max.y {
            for x in rect.min.x..rect.max.x {
                self.paint_cell(UVec2::new(x, y), None);
            }
        }
    }

    pub fn extract_rect(&mut self, rect: URect) -> Option<Self> {
        let rect = self.clip_rect(rect);
        if rect.is_empty() {
            return None;
        }
        let (any, grid) = self.copy_rect(rect);
        if !any {
            return None;
        }
        self.disconnect_rect(rect);
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

    pub fn shrink_to_fit(self) -> Option<(UVec2, Self)> {
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
            Collider::cuboid(BOUNDARY_THICKNESS * 0.5, outer_length * 0.5, 0.5),
            ActiveEvents::COLLISION_EVENTS,
            Transform::from_xyz(low_edge, center, 0.0),
        ),
        (
            RigidBody::Fixed,
            Collider::cuboid(BOUNDARY_THICKNESS * 0.5, outer_length * 0.5, 0.5),
            ActiveEvents::COLLISION_EVENTS,
            Transform::from_xyz(high_edge, center, 0.0),
        ),
        (
            RigidBody::Fixed,
            Collider::cuboid(outer_length * 0.5, BOUNDARY_THICKNESS * 0.5, 0.5),
            ActiveEvents::COLLISION_EVENTS,
            Transform::from_xyz(center, low_edge, 0.0),
        ),
        (
            RigidBody::Fixed,
            Collider::cuboid(outer_length * 0.5, BOUNDARY_THICKNESS * 0.5, 0.5),
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
    mut scratch: Local<crate::simulation::Scratch>,
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
                crate::simulation::initialize_scratch(&grid, &mut scratch);
                for point in manifold.points() {
                    let impulse_world =
                        manifold.normal() * point.impulse() * if grid_first { 1. } else { -1. };
                    let p = if grid_first {
                        point.local_p1()
                    } else {
                        point.local_p2()
                    };
                    let point_world = transform.transform_point(p);
                    crate::simulation::add_impulse(
                        &grid,
                        &mut scratch,
                        local_from_world.transform_vector3(impulse_world).truncate(),
                        local_from_world.transform_point3(point_world).truncate(),
                    );
                }
                let result =
                    crate::simulation::solve(&mut grid, &mut scratch, 10, anchors.is_some());
                for (grid, collider, velocity, offset) in result {
                    let object_transform = Transform {
                        translation: transform.transform_point(offset),
                        rotation: transform.rotation(),
                        ..default()
                    };
                    let solution = projection::solve(
                        grid.shape,
                        object_transform.rotation,
                        object_transform.translation.into(),
                    );
                    let rotation = object_transform.rotation;
                    let velocity = Velocity {
                        linear: rotation * velocity.linear,
                        angular: rotation * velocity.angular,
                    };
                    commands.spawn((
                        grid,
                        collider,
                        velocity,
                        object_transform,
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
            grid.shape,
            transform.rotation(),
            transform.translation_vec3a(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(size: [u32; 2]) -> Grid {
        Grid::new(RuntimeShape::<u32, 2>::new(size))
    }

    #[test]
    fn painting_at_boundaries_preserves_valid_bonds() {
        let mut grid = grid([3, 3]);
        grid.set_brush(Vec2::ZERO, 1, Some(()));
        grid.set_brush(Vec2::splat(2.0), 1, Some(()));
        assert_eq!(grid.iter_set_positions().count(), 7);
        assert!(grid.get_edge(UVec2::ZERO, CompassQuadrant::North));
        assert!(grid.get_edge(UVec2::ZERO, CompassQuadrant::East));
        assert!(!grid.get_edge(UVec2::ZERO, CompassQuadrant::South));
        assert!(!grid.get_edge(UVec2::new(2, 2), CompassQuadrant::North));
        assert_eq!(grid.linearize_checked(UVec2::MAX), None);
        grid.paint_cell(UVec2::MAX, Some(()));
    }

    #[test]
    fn extraction_clips_to_grid_and_preserves_internal_bonds() {
        let mut grid = grid([3, 3]);
        grid.set_brush(Vec2::ONE, 2, Some(()));
        let extracted = grid
            .extract_rect(URect::from_corners(UVec2::ZERO, UVec2::new(2, 5)))
            .unwrap();
        assert_eq!(extracted.size(), UVec2::new(2, 3));
        assert_eq!(extracted.iter_set_positions().count(), 6);
        assert!(extracted.get_edge(UVec2::ZERO, CompassQuadrant::East));
        assert!(extracted.get_edge(UVec2::ZERO, CompassQuadrant::North));
        assert!(!extracted.get_edge(UVec2::X, CompassQuadrant::East));
        assert_eq!(grid.iter_set_positions().count(), 3);
        assert!(!grid.get_edge(UVec2::new(2, 1), CompassQuadrant::West));
        assert!(grid.get_edge(UVec2::new(2, 1), CompassQuadrant::North));
    }

    #[test]
    fn empty_extraction_does_not_change_bonds() {
        let mut grid = grid([3, 3]);
        grid.set_brush(Vec2::ONE, 1, Some(()));
        let before = grid.edges.clone();
        assert!(
            grid.extract_rect(URect::from_corners(UVec2::ONE, UVec2::ONE))
                .is_none()
        );
        assert!(
            grid.extract_rect(URect::from_corners(UVec2::splat(5), UVec2::splat(6)))
                .is_none()
        );
        assert_eq!(grid.edges, before);
    }

    #[test]
    fn clearing_cells_removes_all_incident_bonds() {
        let mut grid = grid([3, 3]);
        grid.set_brush(Vec2::ONE, 1, Some(()));
        grid.clear_rect(URect::from_corners(UVec2::ONE, UVec2::splat(2)));
        assert!(grid[UVec2::ONE].is_none());
        for &direction in CompassQuadrant::ALL {
            assert!(!grid.get_edge(UVec2::ONE, direction));
        }
    }
}
