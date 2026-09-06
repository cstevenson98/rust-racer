use bevy::prelude::*;
use debug_draw::DebugDrawPlugin;
use player::PlayerPlugin;
use track_core::TrackCorePlugin;
use track_editor::TrackEditorPlugin;
use track_render::TrackRenderPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Hello, Bevy".into(),
                resolution: (800, 600).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins((
            TrackCorePlugin,
            TrackEditorPlugin,
            TrackRenderPlugin,
            DebugDrawPlugin,
            PlayerPlugin,
        ))
        .insert_resource(ClearColor(Color::srgb(0.12, 0.12, 0.16)))
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);
}
