use bevy::{
    math::CompassQuadrant,
    platform::collections::{HashMap, HashSet},
    prelude::*,
};
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

    pub fn is_1x1(&self) -> bool {
        self.shape.as_array() == [1, 1]
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
        let center = center.floor().as_ivec2();
        for y in -radius..=radius {
            for x in -radius..=radius {
                let cell = center + IVec2::new(x, y);
                self.paint_cell(cell.as_uvec2(), to);
            }
        }
    }

    /// Paints a continuous line using Bresenham's algorithm.
    pub fn set_brush_line(&mut self, start: Vec2, end: Vec2, radius: i32, to: Cell) {
        let mut cell = start.floor().as_ivec2();
        let end = end.floor().as_ivec2();
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
        Transform::default(),
        ActiveEvents::COLLISION_EVENTS,
    ));
}

pub fn spawn_edge_colliders(mut commands: Commands) {
    let size = SIZE as f32;
    let center = size * 0.5;
    let outer_length = size + 2.0 * BOUNDARY_THICKNESS;
    let low_edge = -BOUNDARY_THICKNESS * 0.5;
    let high_edge = size + BOUNDARY_THICKNESS * 0.5;

    // The grid occupies the square from 0 to SIZE on both axes.
    // These four static colliders meet that square at each edge, retaining
    // dynamic voxel objects within the playable area.
    commands.spawn_batch([
        (
            RigidBody::Fixed,
            Collider::cuboid(BOUNDARY_THICKNESS * 0.5, outer_length * 0.5, 0.5),
            ActiveEvents::COLLISION_EVENTS,
            Transform::from_xyz(low_edge, center, 0.5),
        ),
        (
            RigidBody::Fixed,
            Collider::cuboid(BOUNDARY_THICKNESS * 0.5, outer_length * 0.5, 0.5),
            ActiveEvents::COLLISION_EVENTS,
            Transform::from_xyz(high_edge, center, 0.5),
        ),
        (
            RigidBody::Fixed,
            Collider::cuboid(outer_length * 0.5, BOUNDARY_THICKNESS * 0.5, 0.5),
            ActiveEvents::COLLISION_EVENTS,
            Transform::from_xyz(center, low_edge, 0.5),
        ),
        (
            RigidBody::Fixed,
            Collider::cuboid(outer_length * 0.5, BOUNDARY_THICKNESS * 0.5, 0.5),
            ActiveEvents::COLLISION_EVENTS,
            Transform::from_xyz(center, high_edge, 0.5),
        ),
    ]);
}

#[derive(Resource, Default)]
pub struct Collisions {
    map: HashMap<Entity, HashSet<Entity>>,
}

pub fn on_world_grid_collision(
    mut events: MessageReader<CollisionEvent>,
    mut collisions: ResMut<Collisions>,
    query: Query<(), With<Grid>>,
) {
    for event in events.read() {
        match event {
            CollisionEvent::Started(a, b, _) => {
                for (grid, other) in [(*a, *b), (*b, *a)] {
                    if query.contains(grid) {
                        collisions.map.entry(grid).or_default().insert(other);
                    }
                }
            }
            CollisionEvent::Stopped(a, b, _) => {
                for (grid, other) in [(a, b), (b, a)] {
                    if let Some(set) = collisions.map.get_mut(grid) {
                        set.remove(other);
                        if set.is_empty() {
                            collisions.map.remove(grid);
                        }
                    }
                }
            }
        }
    }
}

type CollisionGridData = (
    &'static mut Grid,
    &'static GlobalTransform,
    &'static Collider,
    Option<&'static WorldGrid>,
    Option<&'static Velocity>,
);

pub fn handle_grid_collisions(
    mut commands: Commands,
    collisions: Res<Collisions>,
    mut grids: Query<CollisionGridData>,
    rapier_context: ReadRapierContext,
    mut scratch: Local<crate::simulation::Scratch>,
) {
    let Ok(rapier_context) = rapier_context.single() else {
        error!("Multiple contexts");
        return;
    };
    for (base, others) in collisions.map.iter() {
        let Ok((mut grid, transform, parent_collider, anchors, parent_velocity)) =
            grids.get_mut(*base)
        else {
            warn!("No grid for base entity: {:?}", base);
            continue;
        };
        if grid.is_1x1() {
            continue;
        }

        let local_from_world = transform.affine().inverse();
        // This system runs before Writeback: ECS velocity is the parent's
        // pre-step motion, so adding the cellular impulse does not count it twice.
        let parent_velocity = parent_velocity.copied().unwrap_or_else(Velocity::zero);
        let spinning = parent_velocity.angular != Vec3::ZERO;
        let parent_center = if spinning {
            parent_collider.raw.mass_properties(1.0).local_com
        } else {
            Vec3::ZERO
        };

        crate::simulation::initialize_scratch(grid.shape.usize(), &mut scratch);
        let mut has_impulse = false;
        for other in others {
            let Some(contact_pair) = rapier_context.contact_pair(*base, *other) else {
                // Cached event pairs can disappear when colliders are removed.
                continue;
            };
            let grid_first = contact_pair.collider1() == Some(*base);

            for manifold in contact_pair.manifolds() {
                for point in manifold.points() {
                    let impulse = point.impulse();
                    if impulse == 0.0 {
                        continue;
                    }
                    has_impulse = true;
                    let impulse_world =
                        manifold.normal() * impulse * if grid_first { 1.0 } else { -1.0 };
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
            }
        }
        if !has_impulse {
            continue;
        }
        let result = crate::simulation::solve(&mut grid, &mut scratch, 10, anchors.is_some());
        for (grid, collider, velocity, offset) in result {
            let object_transform = Transform {
                translation: transform.transform_point(offset),
                rotation: transform.rotation(),
                ..default()
            };
            let solution = projection::solve(
                &grid,
                object_transform.rotation,
                object_transform.translation.into(),
            );
            let center_offset = if spinning {
                let fragment_center = collider.raw.mass_properties(1.0).local_com;
                transform
                    .affine()
                    .transform_vector3(offset + fragment_center - parent_center)
            } else {
                Vec3::ZERO
            };
            let velocity = inherit_fragment_velocity(
                parent_velocity,
                center_offset,
                object_transform.rotation,
                velocity,
            );
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

/// Adds the parent's world-space rigid motion to the fragment's local impulse response.
fn inherit_fragment_velocity(
    parent: Velocity,
    center_offset: Vec3,
    rotation: Quat,
    relative: Velocity,
) -> Velocity {
    Velocity {
        linear: parent.linear + parent.angular.cross(center_offset) + rotation * relative.linear,
        angular: parent.angular + rotation * relative.angular,
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

pub fn recalculate_bijection(mut query: Query<(&Transform, &Grid, &mut Bijection), With<ObjGrid>>) {
    for (transform, grid, mut bijection) in query.iter_mut() {
        bijection.0 = projection::solve(grid, transform.rotation, transform.translation.into())
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
    #[test]
    fn brush_uses_minimum_corner_cell_coordinates() {
        let mut grid = grid([3, 2]);
        grid.set_brush(Vec2::new(0.9, 0.9), 0, Some(()));
        grid.set_brush(Vec2::new(-0.1, 0.1), 0, Some(()));
        assert!(grid[UVec2::ZERO].is_some());
        assert_eq!(grid.iter_set_positions().count(), 1);
    }

    #[test]
    fn projection_and_display_transforms_are_current_before_extraction() {
        use bevy::transform::{TransformPlugin, TransformSystems};
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, TransformPlugin));
        let mut grid = grid([1, 1]);
        grid.paint_cell(UVec2::ZERO, Some(()));
        let bijection = Bijection(projection::solve(&grid, Quat::IDENTITY, Vec3A::ZERO));
        let body = app
            .world_mut()
            .spawn((ObjGrid, grid, bijection, Transform::default()))
            .id();
        // Stand in for Rapier's writeback of the current simulation pose.
        app.add_systems(
            PostUpdate,
            (|mut bodies: Query<&mut Transform, With<ObjGrid>>| {
                for mut transform in &mut bodies {
                    transform.translation = Vec3::new(4.0, 2.0, 0.0);
                }
            })
            .in_set(PhysicsSet::Writeback),
        );
        let quad = app
            .world_mut()
            .spawn((crate::render::Quad, Transform::default()))
            .id();
        app.add_systems(
            PostUpdate,
            (
                recalculate_bijection,
                |bodies: Query<&Bijection, With<ObjGrid>>,
                 mut quads: Query<&mut Transform, With<crate::render::Quad>>| {
                    let pos = bodies.single().unwrap().src_to_dst(UVec2::ZERO).unwrap();
                    quads.single_mut().unwrap().translation =
                        (pos.as_vec2() + Vec2::splat(0.5)).extend(0.0);
                },
            )
                .chain()
                .after(PhysicsSet::Writeback)
                .before(TransformSystems::Propagate),
        );
        app.update();
        assert_eq!(
            app.world()
                .get::<Bijection>(body)
                .unwrap()
                .src_to_dst(UVec2::ZERO),
            Some(IVec2::new(4, 2))
        );
        assert_eq!(
            app.world()
                .get::<GlobalTransform>(quad)
                .unwrap()
                .translation(),
            Vec3::new(4.5, 2.5, 0.0)
        );
    }

    #[test]
    fn world_grid_is_visible_to_collision_processing_query() {
        use bevy::transform::TransformPlugin;
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, TransformPlugin));
        app.add_systems(Startup, spawn_world_grid);
        app.update();
        assert_eq!(
            app.world_mut()
                .query_filtered::<(&Grid, &GlobalTransform), With<WorldGrid>>()
                .iter(app.world())
                .count(),
            1
        );
    }
    #[test]
    fn fragments_inherit_translation_and_spin_at_their_centers() {
        let parent = Velocity {
            linear: Vec3::new(3.0, 4.0, 0.0),
            angular: Vec3::Z * 2.0,
        };
        let parent_collider = Collider::voxels(Vec3::ONE, &[IVec3::ZERO, IVec3::X]);
        let fragment_collider = Collider::voxels(Vec3::ONE, &[IVec3::ZERO]);
        let parent_center = parent_collider.raw.mass_properties(1.0).local_com;
        let fragment_center = fragment_collider.raw.mass_properties(1.0).local_com;
        let left = inherit_fragment_velocity(
            parent,
            fragment_center - parent_center,
            Quat::IDENTITY,
            Velocity::zero(),
        );
        let right = inherit_fragment_velocity(
            parent,
            Vec3::X + fragment_center - parent_center,
            Quat::IDENTITY,
            Velocity::zero(),
        );
        assert_eq!(left.linear, Vec3::new(3.0, 3.0, 0.0));
        assert_eq!(right.linear, Vec3::new(3.0, 5.0, 0.0));
        assert_eq!(left.angular, parent.angular);
        assert_eq!(right.angular, parent.angular);
        assert_eq!((left.linear + right.linear) * 0.5, parent.linear);
        let angular_momentum = (fragment_center - parent_center).cross(left.linear - parent.linear)
            + (Vec3::X + fragment_center - parent_center).cross(right.linear - parent.linear)
            + (left.angular + right.angular) / 6.0;
        assert!((angular_momentum.z - (5.0 / 6.0) * parent.angular.z).abs() < 1e-6);
    }

    #[test]
    fn inherited_motion_and_local_impulse_use_their_correct_frames() {
        let rotation = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
        let parent = Velocity {
            linear: Vec3::X * 3.0,
            angular: Vec3::Z * 2.0,
        };
        let relative = Velocity {
            linear: Vec3::X * 4.0,
            angular: Vec3::Z * 0.5,
        };
        let result = inherit_fragment_velocity(parent, rotation * Vec3::X, rotation, relative);
        assert!((result.linear - Vec3::new(1.0, 4.0, 0.0)).length() < 1e-6);
        assert_eq!(result.angular, Vec3::Z * 2.5);
        let stationary = inherit_fragment_velocity(Velocity::zero(), Vec3::ONE, rotation, relative);
        assert!((stationary.linear - Vec3::Y * 4.0).length() < 1e-6);
        assert!((stationary.angular - relative.angular).length() < 1e-6);
    }
    fn collision_app() -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .add_message::<CollisionEvent>()
            .init_resource::<Collisions>()
            .add_systems(Update, on_world_grid_collision);
        app
    }

    #[test]
    fn collision_map_groups_other_colliders_and_removes_only_stopped_pairs() {
        use bevy_rapier3d::rapier::geometry::CollisionEventFlags;
        let mut app = collision_app();
        let a = app.world_mut().spawn(grid([2, 2])).id();
        let b = app.world_mut().spawn(grid([2, 2])).id();
        let wall = app.world_mut().spawn_empty().id();
        let unrelated = app.world_mut().spawn_empty().id();
        for (first, second) in [(a, b), (b, a), (wall, a), (wall, unrelated)] {
            app.world_mut().write_message(CollisionEvent::Started(
                first,
                second,
                CollisionEventFlags::empty(),
            ));
        }
        app.update();
        let collisions = app.world().resource::<Collisions>();
        assert_eq!(collisions.map.len(), 2);
        assert_eq!(collisions.map[&a], HashSet::from_iter([b, wall]));
        assert_eq!(collisions.map[&b], HashSet::from_iter([a]));
        assert!(!collisions.map[&a].contains(&a));

        app.world_mut()
            .write_message(CollisionEvent::Stopped(b, a, CollisionEventFlags::empty()));
        app.update();
        let collisions = app.world().resource::<Collisions>();
        assert_eq!(collisions.map[&a], HashSet::from_iter([wall]));
        assert!(!collisions.map.contains_key(&b));
        app.world_mut().write_message(CollisionEvent::Stopped(
            a,
            wall,
            CollisionEventFlags::empty(),
        ));
        app.update();
        assert!(app.world().resource::<Collisions>().map.is_empty());
    }

    #[test]
    fn stopped_collision_cleans_up_despawned_grid() {
        use bevy_rapier3d::rapier::geometry::CollisionEventFlags;
        let mut app = collision_app();
        let a = app.world_mut().spawn(grid([2, 2])).id();
        let b = app.world_mut().spawn(grid([2, 2])).id();
        app.world_mut()
            .write_message(CollisionEvent::Started(a, b, CollisionEventFlags::empty()));
        app.update();
        app.world_mut().despawn(a);
        app.world_mut()
            .write_message(CollisionEvent::Stopped(a, b, CollisionEventFlags::empty()));
        app.update();
        assert!(app.world().resource::<Collisions>().map.is_empty());
    }
}
