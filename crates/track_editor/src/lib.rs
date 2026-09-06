use bevy::prelude::*;
use track_core::{ActiveSpline, ControlPoint, Spline};

const HIT_RADIUS: f32 = 14.0;
const DOT_RADIUS: f32 = 4.0;

pub struct TrackEditorPlugin;

impl Plugin for TrackEditorPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DragState>()
            .init_resource::<ControlPointOrder>()
            .add_systems(Update, (handle_pointer, sync_spline_from_points).chain());
    }
}

/// Spawn order of control points.
/// Segments: [P0,P1,P2,P3], then [P3,P4,P5,P6], … (3 new points each).
#[derive(Resource, Default)]
struct ControlPointOrder(Vec<Entity>);

#[derive(Resource, Default, Clone, Copy)]
enum DragState {
    #[default]
    Idle,
    Dragging {
        entity: Entity,
        /// World cursor minus point position at press (avoids snap-to-cursor).
        grab_offset: Vec2,
    },
    /// Control points changed; rebuild the spline once.
    NeedsSync,
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

fn hit_test(points: &Query<(Entity, &mut Transform), With<ControlPoint>>, cursor: Vec2) -> Hit {
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
    mut points: Query<(Entity, &mut Transform), With<ControlPoint>>,
    mut drag: ResMut<DragState>,
    mut order: ResMut<ControlPointOrder>,
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
                let entity = commands
                    .spawn((
                        ControlPoint,
                        Mesh2d(meshes.add(Circle::new(DOT_RADIUS))),
                        MeshMaterial2d(materials.add(Color::BLACK)),
                        Transform::from_translation(cursor.extend(1.0)),
                    ))
                    .id();
                order.0.push(entity);
                *drag = DragState::NeedsSync;
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
        PointerPhase::Release => {
            *drag = match *drag {
                DragState::Dragging { .. } => DragState::NeedsSync,
                other => other,
            };
        }
        PointerPhase::Idle => {}
    }
}

fn sync_spline_from_points(
    mut drag: ResMut<DragState>,
    order: Res<ControlPointOrder>,
    points: Query<&Transform, With<ControlPoint>>,
    mut spline: ResMut<ActiveSpline>,
) {
    let should_sync = matches!(*drag, DragState::Dragging { .. } | DragState::NeedsSync);
    if !should_sync {
        return;
    }

    let mut positions = Vec::with_capacity(order.0.len());
    for entity in &order.0 {
        let Ok(tf) = points.get(*entity) else {
            spline.0 = Spline::default();
            if matches!(*drag, DragState::NeedsSync) {
                *drag = DragState::Idle;
            }
            return;
        };
        positions.push(tf.translation.truncate());
    }

    spline.0 = Spline::from_bezier_points(&positions);

    if matches!(*drag, DragState::NeedsSync) {
        *drag = DragState::Idle;
    }
}
