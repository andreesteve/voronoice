use std::{collections::LinkedList, time::Instant};

use bevy::{prelude::*, render::{camera::{Camera, PerspectiveProjection}, mesh::Indices, pipeline::PrimitiveTopology}};

mod pipeline;
mod voronoi;

use pipeline::*;
use crate::voronoi::*;

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
struct StatusDisplay;

fn add_display_lines(commands: &mut ChildBuilder, font: &Handle<Font>) {
    commands.spawn(TextBundle {
        style: Style {
            size: Size::new(Val::Px(500.0), Val::Px(40.0)),
            ..Default::default()
        },
        text: Text {
            value: "".to_string(),
            font: font.clone(),
            style: TextStyle {
                font_size: 25.0,
                color: Color::WHITE,
                ..Default::default()
            },
        },
        ..Default::default()
    })
    .with(StatusDisplay);
}

// right hand
// triangulation anti-clockwise
fn setup(
    commands: &mut Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let camera_pos = Vec3::new(0.000001, CAMERA_Y, 0.0);
    let mut camera_t = Transform::from_translation(camera_pos)
        .looking_at(Vec3::default(), Vec3::unit_y());
    // roll camera so Z point up, and X right
    camera_t.rotate(Quat::from_rotation_ypr(0.0, 0.0, 180f32.to_radians()));

    let font_handle = asset_server.load("fonts/FiraSans-Bold.ttf");
    commands.spawn(NodeBundle{
        style: Style {
            size: Size::new(Val::Percent(100.0), Val::Percent(100.0)),
            //align_content: AlignContent::Center,
            flex_direction: FlexDirection::ColumnReverse,
            ..Default::default()
        },
        material: materials.add(Color::NONE.into()),
        ..Default::default()
    }).with_children(|parent| {
        add_display_lines(parent, &font_handle);
        add_display_lines(parent, &font_handle);
        add_display_lines(parent, &font_handle);
        add_display_lines(parent, &font_handle);
        add_display_lines(parent, &font_handle);
        add_display_lines(parent, &font_handle);
    });

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
                font: font_handle,
                style: TextStyle {
                    font_size: 25.0,
                    color: Color::WHITE,
                    ..Default::default()
                },
            },
            ..Default::default()
        })
        .with(Mouse::default())
        .spawn(PbrBundle {
            mesh: meshes.add(get_bounding_box(2.0)),
            ..Default::default()
        })
        .with(BoundingBox::new_centered_square(43.0)); // this value does not matter
}

fn get_bounding_box(size: f32) -> Mesh {
    let edge = size / 2.0;
    let pos = vec![
        [-edge, 0.0, -edge], // bottom left
        [-edge, 0.0, edge], // bottom right
        [edge, 0.0, edge], // top right
        [edge, 0.0, -edge], // top left
    ];

    let mut m = Mesh::new(PrimitiveTopology::LineStrip);
    m.set_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 1.0, 0.0]; pos.len()]);
    m.set_attribute(Mesh::ATTRIBUTE_UV_0, vec![[0.0, 0.0]; pos.len()]);
    m.set_attribute(Mesh::ATTRIBUTE_POSITION, pos);
    m.set_indices(Some(Indices::U32(vec![0, 1, 2, 3, 0])));

    m
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

#[derive(Debug)]
enum SiteType {
    Case1,
    Case2,
    Random,
    Circle,
    Triangle,
    Square
}
impl Default for SiteType {
    fn default() -> Self {
        SiteType::Case2
    }
}
#[derive(Default)]
struct State {
    voronoi_opts: VoronoiMeshOptions,
    voronoi: Option<Voronoi>,
    size: usize,
    undo_list: LinkedList<Voronoi>,
    forward_list: LinkedList<Voronoi>,
    hull_behavior: HullBehavior,
    bounding_box: BoundingBox,
    site_type: SiteType,
}
impl State {
    fn replace(&mut self, v: Option<Voronoi>) -> Option<&Voronoi> {
        let old = if let Some(new) = v  {
            self.voronoi.replace(new)
        } else {
            self.voronoi.take()
        };

        if let Some(old) = old {
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
        self.bounding_box = BoundingBox::new_centered_square(2.0);
    }

    fn new_builder(&self) -> VoronoiBuilder {
        let mut builder = VoronoiBuilder::default();
        builder
            .set_hull_behavior(self.hull_behavior)
            .set_bounding_box(self.bounding_box.clone());

        builder
    }

    fn new_voronoi(&mut self, size: usize) {
        let start = Instant::now();
        self.size = size;
        //let range = (-1.0, 1.0);
        //builder.random_sites(size, range, range);
        //builder.generate_circle_sites(size);
        //builder.generate_square_sites(2, 2);
        //builder.generate_triangle_sites();
        let mut builder = self.new_builder();
        let range = (self.bounding_box.width() / 2.0, self.bounding_box.height() / 2.0);

        match self.site_type {
            SiteType::Random => builder.generate_random_sites(self.size, range, range),
            SiteType::Circle => builder.generate_circle_sites(self.size),
            SiteType::Case1 =>builder.generate_special_case_1(),
            SiteType::Case2 =>builder.generate_special_case_2(),
            SiteType::Triangle =>builder.generate_triangle_sites(),
            SiteType::Square => builder.generate_square_sites(self.size, self.size),
        };

        let voronoi = builder.build();
        println!("Generated new voronoi of size {} in {:?}", self.size, start.elapsed());

        self.replace(voronoi);
    }

    fn add_site_to_voronoi(&mut self, site: Point) {
        let mut sites = self.voronoi.as_ref().unwrap().sites.clone();
        sites.push(site);

        let mut builder = self.new_builder();
        builder.set_sites(sites);
        self.replace(builder.build());
    }

    fn remove_site_to_voronoi(&mut self, site_index: usize) {
        let mut sites = self.voronoi.as_ref().unwrap().sites.clone();
        sites.remove(site_index);

        let mut builder = self.new_builder();
        builder.set_sites(sites);
        self.replace(builder.build());
    }
}

fn handle_input(
    mut state: Local<State>,
    input: Res<Input<KeyCode>>,
    mouse_button_input: Res<Input<MouseButton>>,
    meshes: ResMut<Assets<Mesh>>,
    commands: &mut Commands,
    query: Query<Entity, With<VertexColor>>,
    mut query_text: Query<&mut Text, With<StatusDisplay>>,
    mut query_box: Query<&mut Transform, With<BoundingBox>>,
    mouse_query: Query<&Mouse>) {

    let mut respawn = false;

    // no voronoi, generate random one
    if !state.voronoi.is_some() && state.undo_list.is_empty() {
        respawn = true;
        state.hull_behavior = HullBehavior::None;
        state.bounding_box = BoundingBox::new_centered_square(2.0);
        state.new_voronoi(20);
    }

    if input.just_pressed(KeyCode::PageUp) || input.just_pressed(KeyCode::PageDown) {
        if input.just_pressed(KeyCode::PageUp) {
            let size = state.bounding_box.width() + 0.1;
            state.bounding_box = BoundingBox::new(state.bounding_box.center().clone(), size, size);
        } else if input.just_pressed(KeyCode::PageDown) {
            let size = (state.bounding_box.width() - 0.1).max(0.1);
            state.bounding_box = BoundingBox::new(state.bounding_box.center().clone(), size, size);
        }

        respawn = true;
        if let Some(v) = state.voronoi.as_ref() {
            let mut builder : VoronoiBuilder = v.into();
            builder.set_bounding_box(state.bounding_box.clone());
            state.replace(builder.build());
        }
    }

    for mut box_t in query_box.iter_mut() {
        box_t.scale = Vec3::splat((state.bounding_box.width() / 2.0) as f32);
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
            if let Some(existing_voronoi) = state.voronoi.as_ref() {
            let mut builder: VoronoiBuilder = existing_voronoi.into();
            builder.set_lloyd_relaxation_iterations(1);
            state.replace(builder.build());
            respawn = true;
        }
    } else if input.just_pressed(KeyCode::H) {
        // change hull behavior
        state.hull_behavior = match state.hull_behavior {
            HullBehavior::None => HullBehavior::Extended,
            HullBehavior::Extended => HullBehavior::Closed,
            HullBehavior::Closed => HullBehavior::None
        };
        println!("Hull behavior set to {:?}", state.hull_behavior);

        if let Some(existing_voronoi) = state.voronoi.as_ref() {
            let mut builder: VoronoiBuilder = existing_voronoi.into();
            builder.set_hull_behavior(state.hull_behavior);
            state.replace(builder.build());
            respawn = true;
        }
    }

    let mouse = mouse_query.iter().next().unwrap();
    if mouse_button_input.just_pressed(MouseButton::Left) || mouse_button_input.just_pressed(MouseButton::Right) || mouse_button_input.just_pressed(MouseButton::Middle) {
        // take sites and change based on type of click
        let point = voronoi::Point { x: mouse.world_pos.z as f64, y: mouse.world_pos.x  as f64 };

        let (closest_site, num_of_sites) = if let Some(voronoi) = state.voronoi.as_ref() {
            (get_closest_site(voronoi, mouse.world_pos), voronoi.sites.len())
        } else {
            (None, 0)
        };

        if mouse_button_input.just_pressed(MouseButton::Left) {
            // do not let adding points extremelly close as this degenerate triangulation
            if closest_site.is_none() || closest_site.unwrap().1 > 0.001 {
                state.add_site_to_voronoi(point);
                info!("Site added: {:?}", mouse.world_pos);
                respawn = true;
            }
        } else if mouse_button_input.just_pressed(MouseButton::Right) && num_of_sites > 3 { // don't let it go below 3 as it won't triangulate
            // if right click, get closest point and remove it
            if let Some((i, dist)) = closest_site {
                if dist < 0.2 {
                    state.remove_site_to_voronoi(i);
                    info!("Site removed: {}", i);
                    respawn = true;
                }
            }
        } else if mouse_button_input.just_pressed(MouseButton::Middle) {
            // print info for closest site
            if let Some((site, dist)) = closest_site {
                if dist < 0.2 {
                    if let Some(v) = state.voronoi.as_ref() {
                        let cell = v.get_cell(site);
                        println!("{:#?}", cell);
                    } else {
                        println!("No voronoi");
                    }
                }
            }
        }
    }

    // change number of points
    let size = state.size;
    let change = if input.pressed(KeyCode::LShift) { 1000 } else { 100 };
    if input.just_pressed(KeyCode::Up) {
        respawn = true;
        state.new_voronoi(size + change);
    } else if input.just_pressed(KeyCode::Down) {
        respawn = true;
        state.new_voronoi((size as i64 - change as i64).max(120) as usize);
    } else if input.just_pressed(KeyCode::Home) {
        state.site_type = match state.site_type {
            SiteType::Circle => SiteType::Random,
            SiteType::Random => SiteType::Square,
            SiteType::Square => SiteType::Triangle,
            SiteType::Triangle => SiteType::Case1,
            SiteType::Case1 => SiteType::Case2,
            SiteType::Case2 => SiteType::Circle,
        };
        respawn = true;
        state.new_voronoi(size);
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

    let updates: [String; 5] = [
        format!("[H] Hull mode: {:?}", state.hull_behavior),
        format!("[P] Voronoi mesh render mode: {:?}", state.voronoi_opts.voronoi_topoloy),
        format!("[PgUp/PgDown] Bounding box: {:.2}", state.bounding_box.width()),
        format!("[Home] Site type: {:?}", state.site_type),
        format!("# of Sites: {}", state.voronoi.as_ref().map_or(0, |v| v.sites.len())),
    ];

    for (mut text, update) in query_text.iter_mut().zip(&updates) {
        text.value = update.clone();
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
