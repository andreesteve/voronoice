use std::time::Instant;

use bevy::{prelude::*, render::{camera::Camera, pipeline::PrimitiveTopology}};

mod pipeline;
use pipeline::*;

mod voronoi;
use crate::voronoi::Voronoi;

fn main() {
    App::build()
        .add_plugins(DefaultPlugins)
        .add_plugin(VertexColorPlugin)
        .add_resource(ClearColor(Color::rgb(0., 0., 0.))) //background
        .add_startup_system(setup.system())
        .add_system(handle_input.system())
        .run();
}

fn color_white(_i: usize) -> Color {
    Color::WHITE
}

fn color_red(_i: usize) -> Color {
    Color::RED
}

fn generate_voronoi(size: usize) -> Voronoi {
    let start = Instant::now();
    //let range = (-1.0, 1.0);
    //let voronoi = Voronoi::new_with_random_sites(size, range, range);
    //let voronoi = Voronoi::new(voronoi::generate_circle_sites(size));
    //let voronoi = Voronoi::new(voronoi::generate_square_sites(2, 2));
    //let voronoi = Voronoi::new(voronoi::generate_triangle_sites());
    let voronoi = Voronoi::new(voronoi::generate_special_case_1());

    println!("Generated new voronoi of size {} in {:?}", size, start.elapsed());

    voronoi
}

struct VoronoiMeshOptions {
    voronoi_topoloy: PrimitiveTopology,
    delauney_topoloy: PrimitiveTopology,
}

impl Default for VoronoiMeshOptions {
    fn default() -> Self {
        VoronoiMeshOptions {
            voronoi_topoloy: PrimitiveTopology::LineList,
            delauney_topoloy: PrimitiveTopology::PointList
        }
    }
}

fn spawn_voronoi(commands: &mut Commands, mut meshes: ResMut<Assets<Mesh>>, voronoi: &Voronoi, options: &VoronoiMeshOptions) {
    let start = Instant::now();
    let voronoi_generator = voronoi::VoronoiMeshGenerator { voronoi: &voronoi, coloring: color_red, topology: options.voronoi_topoloy };
    let triangle_generator = voronoi::VoronoiMeshGenerator { voronoi: &voronoi, coloring: color_white, topology: options.delauney_topoloy };

    commands
        .spawn(
        ColorBundle {
                mesh: meshes.add(voronoi_generator.build_voronoi_mesh()),
                transform: Transform::from_translation(Vec3::new(
                    0.0,
                    0.0,
                    0.0,
                )),
                ..Default::default()
            })
        .spawn(
            ColorBundle {
                    mesh: meshes.add(triangle_generator.build_delauney_mesh()),
                    transform: Transform::from_translation(Vec3::new(
                        0.0,
                        0.0,
                        0.0,
                    )),
                    ..Default::default()
        })
    ;

    println!("Generated new voronoi meshes in {:?}", start.elapsed());
}

const CAMERA_Y: f32 = 6.0;

// right hand
// triangulation anti-clockwise
fn setup(
    commands: &mut Commands
) {
    // let mut voronoi_loyd_1 = voronoi.loyd_relaxation();
    // voronoi_loyd_1.build();

    // let mut voronoi_loyd_2 = voronoi_loyd_1.loyd_relaxation();
    // voronoi_loyd_2.build();

    // let mut voronoi_loyd_3 = voronoi_loyd_2.loyd_relaxation();
    // voronoi_loyd_3.build();

    let camera_pos = Vec3::new(0.000001, CAMERA_Y, 0.0);
    let mut camera_t = Transform::from_translation(camera_pos)
        .looking_at(Vec3::default(), Vec3::unit_y());
    // roll camera so Z point up, and X right
    camera_t.rotate(Quat::from_rotation_ypr(0.0, 0.0, 180f32.to_radians()));

    commands
        // camera
        .spawn(Camera3dBundle {
            transform: camera_t,
            ..Default::default()
        });
}

fn get_closest_site(voronoi: &Voronoi, pos: Vec3) -> Option<(usize, f32)> {
    voronoi.sites.iter().enumerate().map(|(i, p)| (i, Vec3::new(p.y as f32, 0.0, p.x as f32).distance(pos)))
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
}

#[derive(Default)]
struct State {
    voronoi_opts: VoronoiMeshOptions,
    voronoi: Option<Voronoi>,
    size: usize,
}
fn handle_input(
    mut state: Local<State>,
    input: Res<Input<KeyCode>>,
    mouse_button_input: Res<Input<MouseButton>>,
    windows: Res<Windows>,
    meshes: ResMut<Assets<Mesh>>,
    commands: &mut Commands,
    query: Query<Entity, With<VertexColor>>,
    mut camera_query: Query<(&mut Transform, &GlobalTransform, &Camera), With<Camera>>) {

    let mut respawn = false;

    // no voronoi, generate random one
    if !state.voronoi.is_some() {
        respawn = true;
        state.size = 20;
        state.voronoi = Some(generate_voronoi(state.size));
    }

    // span new voronoi with new rendering but same points
    if input.just_pressed(KeyCode::P) {
        let options = &mut state.voronoi_opts;
        options.voronoi_topoloy = match options.voronoi_topoloy {
            PrimitiveTopology::TriangleList => PrimitiveTopology::LineList,
            PrimitiveTopology::LineList => PrimitiveTopology::PointList,
            _ => PrimitiveTopology::TriangleList,
        };

        respawn = true;
    } else if input.just_pressed(KeyCode::O) {
        let options = &mut state.voronoi_opts;
        options.delauney_topoloy = match options.delauney_topoloy {
            PrimitiveTopology::TriangleList => PrimitiveTopology::LineList,
            PrimitiveTopology::LineList => PrimitiveTopology::PointList,
            _ => PrimitiveTopology::TriangleList,
        };

        respawn = true;
    } else if input.just_pressed(KeyCode::L) {
        // run loyd relaxation
        state.voronoi = Some(state.voronoi.take().unwrap().lloyd_relaxation());
        respawn = true;
    }


    if mouse_button_input.just_pressed(MouseButton::Left) || mouse_button_input.just_pressed(MouseButton::Right) {
        let (camera_transform, _, camera) = camera_query.iter_mut().next().unwrap();
        let window = windows.iter().next().unwrap();
        let screen_size = Vec2::from([window.width() as f32, window.height() as f32]);
        let cursor_screen_pos = window.cursor_position().unwrap_or(Vec2::zero());

        // normalize cursor coords (-1 to 1)
        let cursor_pos_normalized = (2.0 * (cursor_screen_pos / screen_size) - Vec2::new(1.0, 1.0)).extend(1.0);
        let view_matrix = camera_transform.compute_matrix();
        let screen_normal_coords_to_world = view_matrix * camera.projection_matrix.inverse();

        let cursor_world_pos = screen_normal_coords_to_world.transform_point3(cursor_pos_normalized);
        let ray: Vec3 = cursor_world_pos - camera_transform.translation;

        // FIXME I put this together to debug voronoi generator
        // this is assuming camera looking down Y
        // genealize this ray-plane intersection logic based on the camera forward vector
        let mut world_pos = -camera_transform.translation.y * (ray / ray.y);
        world_pos.y = 0.0;

        // take sites and change based on type of click
        let point = voronoi::Point { x: world_pos.z as f64, y: world_pos.x  as f64 };
        let mut old = state.voronoi.take().unwrap();
        let closest = get_closest_site(&old, world_pos);
        if mouse_button_input.just_pressed(MouseButton::Left) {
            // do not let adding points extremelly close as this degenerate triangulation
            if closest.is_none() || closest.unwrap().1 > 0.001 {
                old.sites.push(point);
                info!("Site added: {:?}", world_pos);
            }
        } else {
            // if right click, get closest point and remove it
            if let Some((i, _)) = closest {
                old.sites.remove(i);
                info!("Site removed: {}", i);
            }
        }

        state.voronoi = Some(Voronoi::new(old.sites));
        respawn = true;
    }

    // change number of points
    if input.just_pressed(KeyCode::Up) {
        respawn = true;
        state.size += 100;
        state.voronoi = Some(generate_voronoi(state.size));
    } else if input.just_pressed(KeyCode::Down) {
        respawn = true;
        state.size = state.size.max(120) - 100;
        state.voronoi = Some(generate_voronoi(state.size));
    } else if input.just_pressed(KeyCode::PageUp) {
        respawn = true;
        state.size += 1000;
        state.voronoi = Some(generate_voronoi(state.size));
    } else if input.just_pressed(KeyCode::PageDown) {
        respawn = true;
        state.size = state.size.max(1020) - 1000;
        state.voronoi = Some(generate_voronoi(state.size));
    }

    // span new voronoi with new points
    if input.just_pressed(KeyCode::G) {
        respawn = true;
        state.voronoi = Some(generate_voronoi(state.size));
    }

    if respawn {
        for e in query.iter() {
            commands.despawn(e);
        }

        spawn_voronoi(commands, meshes, state.voronoi.as_ref().expect("Where is my voronoi"), &state.voronoi_opts);
    }

    if input.pressed(KeyCode::W) {
        for (mut t, _, _) in camera_query.iter_mut() {
            let y_move = 0.1f32.min((t.translation.y - 0.7).powf(10.0));
            t.translation.y -= y_move;
        }
    } else if input.pressed(KeyCode::S) {
        for (mut t, _, _) in camera_query.iter_mut() {
            t.translation.y += 0.1;
        }
    } else if input.pressed(KeyCode::R) {
        for (mut t, _, _) in camera_query.iter_mut() {
            t.translation.y = CAMERA_Y;
        }
    }

    if input.just_pressed(KeyCode::B) {
        println!("{:#?}", state.voronoi);
    }
}

// Created 10000000 random points in 2074149 micrseconds
// delaunator: 10000000 points processed in 10,111,821 micrseconds
// Created 10000000 random points in 2053817 micrseconds
// voronoi: 10000000 points processed in 6,6532,576 micrseconds

// Created 10000 random points in 2066 micrseconds
// delaunator: 10000 points processed in 3796 micrseconds
// Created 10000 random points in 2048 micrseconds
// voronoi: 10000 points processed in 32993 micrseconds
// [andre@scout voronoi]$ cargo run
//     Finished dev [unoptimized + debuginfo] target(s) in 0.01s
//      Running `target/debug/terrain`
// Created 10000 random points in 2075 micrseconds
// delaunator: 10000 points processed in 3710 micrseconds
// Created 10000 random points in 2050 micrseconds
// voronoi: 10000 points processed in 31867 micrseconds
// [andre@scout voronoi]$ cargo run
//    Compiling terrain v0.1.0 (/home/andre/projects/learn/rust/voronoi)
//     Finished dev [unoptimized + debuginfo] target(s) in 0.29s
//      Running `target/debug/terrain`
// Created 100000 random points in 20829 micrseconds
// delaunator: 100000 points processed in 52593 micrseconds
// Created 100000 random points in 20560 micrseconds
// voronoi: 100000 points processed in 415524 micrseconds
// [andre@scout voronoi]$ cargo run
//    Compiling terrain v0.1.0 (/home/andre/projects/learn/rust/voronoi)
//     Finished dev [unoptimized + debuginfo] target(s) in 0.28s
//      Running `target/debug/terrain`
// Created 1000000 random points in 209930 micrseconds
// delaunator: 1000000 points processed in 744958 micrseconds
// Created 1000000 random points in 206342 micrseconds
// voronoi: 1000000 points processed in 5113890 micrseconds
// [andre@scout voronoi]$ cargo run
//    Compiling terrain v0.1.0 (/home/andre/projects/learn/rust/voronoi)
//     Finished dev [unoptimized + debuginfo] target(s) in 0.28s
//      Running `target/debug/terrain`
// Created 10000000 random points in 2074149 micrseconds
// delaunator: 10000000 points processed in 10111821 micrseconds
// Created 10000000 random points in 2053817 micrseconds
// voronoi: 10000000 points processed in 66532576 micrseconds
// [andre@scout voronoi]$ cargo run --release
