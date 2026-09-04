use bevy::prelude::*;

const HIT_RADIUS: f32 = 14.0;
const DOT_RADIUS: f32 = 4.0;
const RING_RADIUS: f32 = 12.0;
const CROSS_HALF: f32 = 10.0;

pub struct PointsPlugin;

impl Plugin for PointsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DragState>()
            .add_systems(Update, (handle_pointer, draw_point_overlays).chain());
    }
}

#[derive(Component)]
struct PlacedPoint;

#[derive(Resource, Default, Clone, Copy)]
enum DragState {
    #[default]
    Idle,
    Dragging {
        entity: Entity,
        /// World cursor minus point position at press (avoids snap-to-cursor).
        grab_offset: Vec2,
    },
}

enum PointerPhase {
    Press,
    Drag,
    Release,
    Idle,
}

enum Hit {
    Point { entity: Entity, pos: Vec2 },
    Nothing,
}

fn pointer_phase(mouse: &ButtonInput<MouseButton>) -> PointerPhase {
    if mouse.just_pressed(MouseButton::Left) {
        PointerPhase::Press
    } else if mouse.just_released(MouseButton::Left) {
        PointerPhase::Release
    } else if mouse.pressed(MouseButton::Left) {
        PointerPhase::Drag
    } else {
        PointerPhase::Idle
    }
}

fn hit_test(points: &Query<(Entity, &mut Transform), With<PlacedPoint>>, cursor: Vec2) -> Hit {
    points
        .iter()
        .filter_map(|(entity, tf)| {
            let pos = tf.translation.truncate();
            let dist = pos.distance(cursor);
            (dist <= HIT_RADIUS).then_some((entity, dist, pos))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(entity, _, pos)| Hit::Point { entity, pos })
        .unwrap_or(Hit::Nothing)
}

fn cursor_world_pos(
    window: &Window,
    camera: &Camera,
    camera_transform: &GlobalTransform,
) -> Option<Vec2> {
    let cursor = window.cursor_position()?;
    camera.viewport_to_world_2d(camera_transform, cursor).ok()
}

fn handle_pointer(
    mouse: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window>,
    camera: Query<(&Camera, &GlobalTransform), With<Camera2d>>,
    mut points: Query<(Entity, &mut Transform), With<PlacedPoint>>,
    mut drag: ResMut<DragState>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let Ok(window) = windows.single() else {
        return;
    };
    let Ok((camera, cam_tf)) = camera.single() else {
        return;
    };
    let Some(cursor) = cursor_world_pos(window, camera, cam_tf) else {
        return;
    };

    match pointer_phase(&mouse) {
        PointerPhase::Press => match hit_test(&points, cursor) {
            Hit::Point { entity, pos } => {
                *drag = DragState::Dragging {
                    entity,
                    grab_offset: cursor - pos,
                };
            }
            Hit::Nothing => {
                commands.spawn((
                    PlacedPoint,
                    Mesh2d(meshes.add(Circle::new(DOT_RADIUS))),
                    MeshMaterial2d(materials.add(Color::BLACK)),
                    Transform::from_translation(cursor.extend(1.0)),
                ));
            }
        },
        PointerPhase::Drag => {
            if let DragState::Dragging {
                entity,
                grab_offset,
            } = *drag
            {
                if let Ok((_, mut tf)) = points.get_mut(entity) {
                    let new_pos = cursor - grab_offset;
                    tf.translation.x = new_pos.x;
                    tf.translation.y = new_pos.y;
                }
            }
        }
        PointerPhase::Release | PointerPhase::Idle => {
            *drag = DragState::Idle;
        }
    }
}

fn draw_point_overlays(mut gizmos: Gizmos, points: Query<&Transform, With<PlacedPoint>>) {
    for tf in &points {
        let iso = Isometry2d::from_translation(tf.translation.truncate());
        gizmos.circle_2d(iso, RING_RADIUS, Color::BLACK);
        gizmos.cross_2d(iso, CROSS_HALF, Color::BLACK);
    }
}
