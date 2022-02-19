use delaunator::{EMPTY, next_halfedge};
use rand::Rng;
use voronoice::{BoundingBox, Point, Voronoi, VoronoiBuilder};
use std::{fs::File, io::Write};
const CANVAS_SIZE: f64 = 800.;
const CANVAS_MARGIN: f64 = 0.2;
const POINT_SIZE: usize = 2;
const SITE_COLOR: &str = "black";
const LINE_WIDTH: usize = 1;
const VORONOI_EDGE_COLOR: &str = "blue";
const TRIANGULATION_HULL_COLOR: &str = "green";
const TRIANGULATION_LINE_COLOR: &str = "grey";
const SIZE: usize = 10;

fn main() -> std::io::Result<()> {
    let rotation_angle: f64 = 0.;
    let loyd_iterations = 0;
    let mut rng = rand::thread_rng();
    let range = CANVAS_SIZE * 0.5 * (1. - CANVAS_MARGIN);
    let x_range = rand::distributions::Uniform::new(-range, range);
    let y_range = rand::distributions::Uniform::new(-range, range);

    // generate random sites
    let mut sites = (0..SIZE)
        .map(move |_| [rng.sample(x_range), rng.sample(y_range)])
        .collect::<Vec<_>>();

    // let sites = [
    //     [0., 0.],
    //     [100., 100.],
    //     [100., -100.],
    //     [-100., -100.],
    //     [0., -90.],
    //     [-100., 100.],
    // ];

    // let sites = [
    //     [513.3611229325845, 218.20628310225956],
    //     [313.9123513947884, 232.64372934487739],
    //     [317.26521105553684, 191.5099287605878],
    //     [431.7609494497947, 332.2194071471341],
    //     [522.8488628722222, 709.9493651854054],
    //     [493.079820505491, 635.2243211927607],
    //     [230.41888112405147, 128.8067699699542],
    //     [475.1975240795291, 167.20375411455706],
    //     [583.7516127789556, 140.76002001786293],
    //     [649.2958774279067, 515.6097774455709],
    // ].iter().map(|p| [ p[0] - CANVAS_SIZE / 2.0, p[1] - CANVAS_SIZE / 2.0 ]).collect::<Vec<_>>();

    // case with a second layer of out of box circumcenters
    let sites = [
        [606.4229737144186, 481.55648115353944],
        [280.5097027714764, 429.3065189591119],
        [264.25420839218674, 620.0469491315449],
        [697.655066100861, 97.56991131517866],
        [394.22359797844257, 268.4993068072491],
        [131.63601405327887, 538.0290022262941],
        [211.7146961386036, 207.4358955418078],
        [446.40388795003526, 608.2377277244302],
        [566.0599130430098, 147.00054219422935],
        [420.0726028967108, 454.3552770612433],
    ].iter().map(|p| [ p[0] - CANVAS_SIZE / 2.0, p[1] - CANVAS_SIZE / 2.0 ]).collect::<Vec<_>>();


    // let sites = [
    //     [100., -5.],
    //     [-100., 5.],
    //     [100., 5.],
    //     [-100., -5.],
    //     [0., 0.],
    // ];

    // let sites = [
    //     [250., -310.],
    //     [-240., 0.],
    //     [240., 200.],
    //     [-30., -140.],
    //     [-50., -250.],
    //     [-210., 220.],
    //     //[210., -30.],
    //     //[-300., -200.],
    //     [0., 0.],
    // ];

    let rotation_angle = rotation_angle.to_radians();
    let sites: Vec<Point> = sites.iter().map(|p| {
        // rotate and adjust canvas origin
        Point {
            x: p[0] *  rotation_angle.cos() - p[1] * rotation_angle.sin() + CANVAS_SIZE / 2.0,
            y: p[1] *  rotation_angle.cos() + p[0] * rotation_angle.sin() + CANVAS_SIZE / 2.0
        }
    }).collect();

    // build voronoi
    let voronoi = VoronoiBuilder::default()
        .set_sites(sites)
        .set_bounding_box(BoundingBox::new(Point { x: CANVAS_SIZE / 2.0, y: CANVAS_SIZE / 2.0 }, CANVAS_SIZE * (1.0 - CANVAS_MARGIN), CANVAS_SIZE * (1.0 - CANVAS_MARGIN)))
        .set_lloyd_relaxation_iterations(loyd_iterations)
        .build()
        .expect("Couldn't build voronoi");

    // generate SVG
    let contents = format!(
        r#"
<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">
<rect width="100%" height="100%" fill="white" />
<rect x="{bb_x}" y="{bb_y}" width="{bb_width}" height="{bb_height}" style="fill-opacity:0;stroke-opacity:0.25;stroke-width:3;stroke:rgb(0,0,0)" />
    {sites}
    {voronoi_edges}
    {triangles}
</svg>"#,
        width = CANVAS_SIZE,
        height = CANVAS_SIZE,
        bb_x = voronoi.bounding_box().top_right().x - voronoi.bounding_box().width(),
        bb_y = CANVAS_SIZE - voronoi.bounding_box().top_right().y,
        bb_width = voronoi.bounding_box().width(),
        bb_height = voronoi.bounding_box().height(),
        sites = render_point(voronoi.sites()),
        voronoi_edges = render_voronoi_edges(&voronoi),
        triangles = render_triangles(&voronoi),
    );

    println!("{:#?}", voronoi);
    println!("Hull: {:#?}", voronoi.triangulation().hull);

    File::create("example.svg")?.write_all(contents.as_bytes())
}

fn render_triangles(voronoi: &Voronoi) -> String {
    let triangulation = voronoi.triangulation();
    let points = voronoi.sites();

    (0..triangulation.triangles.len()).fold(String::new(), |acc, e| {
        if e > triangulation.halfedges[e] || triangulation.halfedges[e] == EMPTY {
            let start = &points[triangulation.triangles[e]];
            let end = &points[triangulation.triangles[next_halfedge(e)]];
            let color = if triangulation.halfedges[e] == EMPTY { TRIANGULATION_HULL_COLOR } else { TRIANGULATION_LINE_COLOR };
            let mid = Point { x: (start.x + end.x) / 2.0, y: (start.y + end.y) / 2.0 };

            let acc = acc + &format!(r#"<line id="dedge_{id}" stroke-dasharray="10,10" x1="{x0}" y1="{y0}" x2="{x1}" y2="{y1}" style="stroke:{color};stroke-width:{width}" />"#,
                id = e,
                x0 = start.x,
                y0 = start.y,
                x1=end.x,
                y1=end.y,
                width = LINE_WIDTH,
                color = color);

            acc + &format!(r#"<text x="{x}" y="{y}" style="stroke:{color};">{text}</text>"#, x = mid.x, y = mid.y, text = e)
        } else {
            acc
        }
    })
}

fn render_point(points: &[Point]) -> String {
    points
        .iter()
        .enumerate()
        .fold(String::new(), |acc, (i, p)| {
            acc + &format!(
                r#"<circle id="pt_{pi}" cx="{x}" cy="{y}" r="{size}" fill="{color}"/>"#,
                pi = i,
                x = p.x,
                y = p.y,
                size = POINT_SIZE,
                color = SITE_COLOR
            ) + &format!(r#"<text x="{x}" y="{y}" style="stroke:{color};">{text}</text>"#, x = p.x, y = p.y, text = i, color = SITE_COLOR)
        })
}

fn render_voronoi_edges(voronoi: &Voronoi) -> String {
    let mut rng = rand::thread_rng();

    let mut buffer = String::new();
    voronoi.iter_cells().for_each(|cell| {
        // TODO: cycle() needs clone, instead of returning Impl Iterator, return a proper iterator struct
        // using a ugly chain below to get (current, next) vertices of a cell
        if let Some(first) = cell.iter_vertices().next() {
            cell.iter_vertices().zip(cell.iter_vertices().skip(1).chain(std::iter::once(first))).for_each(|(start, end)|{
                buffer += &format!(r#"<line x1="{x0}" y1="{y0}" x2="{x1}" y2="{y1}" style="stroke:{color};stroke-width:{width}" />"#,
                    x0 = start.x,
                    y0 = start.y,
                    x1 = end.x,
                    y1 = end.y,
                    width = LINE_WIDTH,
                    color = VORONOI_EDGE_COLOR);
            });
        }
    });

    voronoi.vertices().iter().for_each(|v| {
        buffer += &format!(r#"<circle cx="{x}" cy="{y}" r="{size}" fill="{color}" />"#,
            x = v.x + rng.sample(rand::distributions::Uniform::new(-10., 10.)),
            y = v.y + rng.sample(rand::distributions::Uniform::new(-10., 10.)),
            size = POINT_SIZE,
            color = "red");
    });

    buffer
}