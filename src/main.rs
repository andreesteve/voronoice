use std::{collections::LinkedList, time::Instant};

use bevy::{prelude::*, render::{camera::{Camera, PerspectiveProjection}, pipeline::PrimitiveTopology}};

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
        .add_system(calculate_mouse_world_coords.system())
        .add_system(handle_input.system())
        .add_system(move_camera.system())
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
    let voronoi = Voronoi::new(voronoi::generate_special_case_2());

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
            delauney_topoloy: PrimitiveTopology::LineList
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
    commands: &mut Commands,
    asset_server: Res<AssetServer>
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
        // ui camera
        .spawn(CameraUiBundle::default())
        // camera
        .spawn(Camera3dBundle {
            transform: camera_t,
            ..Default::default()
        })
        .spawn(TextBundle {
            style: Style {
                position_type: PositionType::Absolute,
                ..Default::default()
            },
            text: Text {
                value: "(0, 0)".to_string(),
                font: asset_server.load("fonts/FiraSans-Bold.ttf"),
                style: TextStyle {
                    font_size: 25.0,
                    color: Color::WHITE,
                    ..Default::default()
                },
            },
            ..Default::default()
        })
        .with(Mouse::default());
}

fn get_closest_site(voronoi: &Voronoi, pos: Vec3) -> Option<(usize, f32)> {
    voronoi.sites.iter().enumerate().map(|(i, p)| (i, Vec3::new(p.y as f32, 0.0, p.x as f32).distance(pos)))
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
}

#[derive(Default)]
#[derive(Debug)]
struct Mouse {
    world_pos : Vec3
}
const MOUSE_TEXT_OFFSET: f32 = 15.0;
fn calculate_mouse_world_coords(mut mouse_query: Query<(&mut Mouse, &mut Text, &mut Style)>, query: Query<(&Transform, &Camera), With<PerspectiveProjection>>, windows: Res<Windows>) {
    let (mut mouse,  mut text, mut text_style) = mouse_query.iter_mut().next().unwrap();

    for ((camera_transform, camera), window) in query.iter().zip(windows.iter()) {
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
        mouse.world_pos = world_pos;
        text.value = format!("({:.2}, {:.2})", mouse.world_pos.z, mouse.world_pos.x);

        text_style.position.left = Val::Px(cursor_screen_pos.x + MOUSE_TEXT_OFFSET);
        text_style.position.top = Val::Px(window.height() - cursor_screen_pos.y + MOUSE_TEXT_OFFSET);
    }
}

fn move_camera(input: Res<Input<KeyCode>>, mut camera_query: Query<&mut Transform, (With<Camera>, With<PerspectiveProjection>)>) {
    if input.pressed(KeyCode::W) {
        for mut t in camera_query.iter_mut() {
            let y_move = 0.1f32.min((t.translation.y - 0.7).powf(10.0));
            t.translation.y -= y_move;
        }
    } else if input.pressed(KeyCode::S) {
        for mut t in camera_query.iter_mut() {
            t.translation.y += 0.1;
        }
    } else if input.pressed(KeyCode::R) {
        for mut t in camera_query.iter_mut() {
            t.translation.y = CAMERA_Y;
        }
    }
}

#[derive(Default)]
struct State {
    voronoi_opts: VoronoiMeshOptions,
    voronoi: Option<Voronoi>,
    size: usize,
    undo_list: LinkedList<Voronoi>,
    forward_list: LinkedList<Voronoi>,
}

impl State {
    fn replace(&mut self, v: Voronoi) -> Option<&Voronoi> {
        if let Some(old) = self.voronoi.replace(v) {
            self.undo_list.push_front(old);

            self.undo_list.front()
        } else {
            None
        }
    }

    fn undo(&mut self) -> Option<&Voronoi> {
        if let Some(prev) = self.undo_list.pop_front() {
            if let Some(curr) = self.voronoi.replace(prev) {
                self.forward_list.push_front(curr);
                self.forward_list.front()
            } else {
                None
            }
        } else {
            None
        }
    }

    fn undo_forward(&mut self) -> Option<&Voronoi> {
        if let Some(prev) = self.forward_list.pop_front() {
            if let Some(curr) = self.voronoi.replace(prev) {
                self.undo_list.push_front(curr);
                self.undo_list.front()
            } else {
                None
            }
        } else {
            None
        }
    }

    fn clear(&mut self) {
        self.voronoi.take();
        self.undo_list.clear();
        self.forward_list.clear();
    }
}

fn handle_input(
    mut state: Local<State>,
    input: Res<Input<KeyCode>>,
    mouse_button_input: Res<Input<MouseButton>>,
    meshes: ResMut<Assets<Mesh>>,
    commands: &mut Commands,
    query: Query<Entity, With<VertexColor>>,
    mouse_query: Query<&Mouse>) {

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
        let new = state.voronoi.as_ref().unwrap().lloyd_relaxation();
        state.replace(new);
        respawn = true;
    }


    let mouse = mouse_query.iter().next().unwrap();
    if mouse_button_input.just_pressed(MouseButton::Left) || mouse_button_input.just_pressed(MouseButton::Right) || mouse_button_input.just_pressed(MouseButton::Middle) {
        // take sites and change based on type of click
        let point = voronoi::Point { x: mouse.world_pos.z as f64, y: mouse.world_pos.x  as f64 };

        let closest_site = get_closest_site(state.voronoi.as_ref().unwrap(), mouse.world_pos);
        if mouse_button_input.just_pressed(MouseButton::Left) {
            // do not let adding points extremelly close as this degenerate triangulation
            if closest_site.is_none() || closest_site.unwrap().1 > 0.001 {
                let mut points = state.voronoi.as_ref().unwrap().sites.clone();
                points.push(point);
                state.replace(Voronoi::new(points));
                info!("Site added: {:?}", mouse.world_pos);
                respawn = true;
            }
        } else if mouse_button_input.just_pressed(MouseButton::Right) && state.voronoi.as_ref().unwrap().sites.len() > 3 { // don't let it go below 3 as it won't triangulate
            // if right click, get closest point and remove it
            if let Some((i, dist)) = closest_site {
                if dist < 0.2 {
                    let mut points = state.voronoi.as_ref().unwrap().sites.clone();
                    points.push(point);
                    points.remove(i);
                    state.replace(Voronoi::new(points));
                    info!("Site removed: {}", i);
                    respawn = true;
                }
            }
        } else if mouse_button_input.just_pressed(MouseButton::Middle) {
            // print info for closest site
            if let Some((site, dist)) = closest_site {
                if dist < 0.2 {
                    let cell = state.voronoi.as_ref().unwrap().get_cell(site);
                    println!("{:#?}", cell);
                }
            }
        }
    }

    // change number of points
    if input.just_pressed(KeyCode::Up) {
        respawn = true;
        state.size += 100;
        let size = state.size;
        state.replace(generate_voronoi(size));
    } else if input.just_pressed(KeyCode::Down) {
        respawn = true;
        state.size = state.size.max(120) - 100;
        let size = state.size;
        state.replace(generate_voronoi(size));
    } else if input.just_pressed(KeyCode::PageUp) {
        respawn = true;
        state.size += 1000;
        let size = state.size;
        state.replace(generate_voronoi(size));
    } else if input.just_pressed(KeyCode::PageDown) {
        respawn = true;
        state.size = state.size.max(1020) - 1000;
        let size = state.size;
        state.replace(generate_voronoi(size));
    }

    if input.pressed(KeyCode::LControl) {
        if input.just_pressed(KeyCode::Z) {
            println!("Undoing. Undo list size: {}, forward list size: {}", state.undo_list.len(), state.forward_list.len());
            state.undo();
            respawn = true;
        } else if input.just_pressed(KeyCode::Y) {
            println!("Undoing forward. Undo list size: {}, forward list size: {}", state.undo_list.len(), state.forward_list.len());
            state.undo_forward();
            respawn = true;
        }
    }

    // span new voronoi with new points
    if input.just_pressed(KeyCode::G) {
        respawn = true;
        // clean up state so it gets fully regenerated
        state.clear();
    }

    if respawn {
        for e in query.iter() {
            commands.despawn(e);
        }

        // may not exist after clean up
        if let Some(voronoi) = &state.voronoi {
            spawn_voronoi(commands, meshes, voronoi, &state.voronoi_opts);
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
