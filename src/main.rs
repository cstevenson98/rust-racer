mod points;
mod spline;

use bevy::prelude::*;
use points::PointsPlugin;

fn main() {
    let _ = spline::Spline { segments: vec![] };

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Hello, Bevy".into(),
                resolution: (800, 600).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(PointsPlugin)
        .insert_resource(ClearColor(Color::srgb(0.12, 0.12, 0.16)))
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);
}
