use bevy::{camera::ScalingMode, math::USizeVec2, prelude::*};
use ndshape::{ConstPow2Shape2usize, ConstShape as _};

const PX_PER_CELL: u32 = 4;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                resolution: (SIZE as u32 * PX_PER_CELL, SIZE as u32 * PX_PER_CELL).into(),
                ..default()
            }),
            ..default()
        }))
        .init_resource::<QuadTemplate>()
        .add_systems(Startup, setup)
        .add_systems(Update, (input, sync_grid).chain())
        .run();
}

const SIZE_LOG2: usize = 8;
const SIZE: usize = 1 << SIZE_LOG2;
const SIZE_POW2: usize = SIZE * SIZE;
type Shape = ConstPow2Shape2usize<SIZE_LOG2, SIZE_LOG2>;

#[derive(Component)]
struct Grid {
    state: [State; SIZE_POW2],
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Set,
    Unset,
}

impl Grid {
    fn iter_set_positions(&self) -> impl Iterator<Item = IVec2> + '_ {
        (0..SIZE)
            .flat_map(|y| (0..SIZE).map(move |x| USizeVec2::new(x, y)))
            .filter(|pos| matches!(self.state[Shape::linearize(pos.to_array())], State::Set))
            .map(|pos| pos.as_ivec2())
    }

    fn set(&mut self, pos: IVec2, to: State) {
        assert!(pos.cmpge(IVec2::ZERO).all());
        assert!(pos.cmplt(IVec2::splat(SIZE as i32)).all());
        let pos = pos.as_usizevec2();
        let index = Shape::linearize(pos.to_array());
        self.state[index] = to;
    }

    fn set_line(&mut self, start: Vec2, end: Vec2, to: State) {
        // TODO: actual raycast impl
        self.set(end.round().as_ivec2(), to);
    }
}

#[derive(Bundle, Clone)]
struct QuadBundle {
    material: MeshMaterial2d<ColorMaterial>,
    mesh: Mesh2d,
}

impl FromWorld for QuadBundle {
    fn from_world(world: &mut World) -> Self {
        let asset_server = world.resource::<AssetServer>();
        QuadBundle {
            material: MeshMaterial2d(asset_server.add(Color::WHITE.into())),
            mesh: Mesh2d(asset_server.add(Rectangle::from_length(1.).into())),
        }
    }
}

#[derive(Resource, FromWorld)]
struct QuadTemplate(QuadBundle);

#[derive(Component)]
struct Quad;

fn setup(mut commands: Commands) {
    let mut grid = Grid {
        state: [State::Unset; SIZE_POW2],
    };
    // debug positions for now
    grid.state[25] = State::Set;
    grid.state[25 + SIZE] = State::Set;
    grid.state[14298] = State::Set;
    grid.state[14299] = State::Set;
    grid.state[14300] = State::Set;
    grid.state[14301] = State::Set;
    commands.spawn(grid);

    commands.spawn((
        Camera2d,
        Projection::Orthographic(OrthographicProjection {
            scaling_mode: ScalingMode::Fixed {
                width: SIZE as f32,
                height: SIZE as f32,
            },
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(SIZE as f32 / 2.0 - 0.5, SIZE as f32 / 2.0 - 0.5, 0.0),
    ));
}

fn sync_grid(
    mut commands: Commands,
    mut quads: Query<(Entity, &mut Transform), With<Quad>>,
    grid: Single<&Grid>,
    template: Res<QuadTemplate>,
) {
    let mut recycle = quads.iter_mut();
    for pos in grid.iter_set_positions() {
        if let Some((_, mut re_trgt)) = recycle.next() {
            re_trgt.translation = pos.extend(0).as_vec3();
        } else {
            commands.spawn((
                Quad,
                Transform::from_translation(pos.extend(0).as_vec3()),
                template.0.clone(),
            ));
        }
    }

    // We either despawn or set `Disabled`.
    // Despawning is better unless you have non-copy or persistent data
    // because it doesn't perform an archetype move.
    for (e, _) in recycle {
        commands.entity(e).despawn();
    }
}

fn input(
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform)>,
    mut grid: Single<&mut Grid>,
    mbi: Res<ButtonInput<MouseButton>>,
    mut last: Local<Option<(State, Vec2)>>,
) -> Result<(), BevyError> {
    let lmb = mbi.pressed(MouseButton::Left);
    let rmb = mbi.pressed(MouseButton::Right);

    if !lmb && !rmb {
        *last = None;
        return Ok(());
    }

    let (camera, camera_transform) = camera.into_inner();
    if let Some(cursor_position) = window.cursor_position() {
        let pos = camera
            .viewport_to_world_2d(camera_transform, cursor_position)
            .with_severity(Severity::Warning)?;

        if let Some((mode, lpos)) = &mut *last {
            // Swap the mode if neccecary
            match (lmb, rmb, *mode) {
                (true, false, State::Unset) => *mode = State::Set,
                (false, true, State::Set) => *mode = State::Unset,
                _ => {}
            }

            grid.set_line(*lpos, pos, *mode);

            *lpos = pos;
        } else {
            // prioritize LMB
            // because lmb == false and lmb || rmb == true then rmb == true
            let mode = if lmb { State::Set } else { State::Unset };
            *last = Some((mode, pos));

            grid.set(pos.round().as_ivec2(), mode)
        }
    };
    Ok(())
}
