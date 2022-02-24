use clap::StructOpt;
use delaunator::{EMPTY, next_halfedge};
use rand::Rng;
use voronoice::{BoundingBox, Point, Voronoi, VoronoiBuilder, ClipBehavior};
use std::{fs::File, io::Write, path::PathBuf};
const CANVAS_SIZE: f64 = 800.;
const CANVAS_MARGIN: f64 = 0.2;
const POINT_SIZE: usize = 2;
const CIRCUMCENTER_CIRCLE_COLOR: &str = "black";
const SITE_COLOR: &str = "black";
const CIRCUMCENTER_COLOR: &str = "red";
const LINE_WIDTH: usize = 1;
const VORONOI_EDGE_COLOR: &str = "blue";
const TRIANGULATION_HULL_COLOR: &str = "green";
const TRIANGULATION_LINE_COLOR: &str = "grey";
const RENDER_LABELS: bool = true;
const JITTER_RANGE_VALUE: f64 = 15.;

#[derive(clap::Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Optional path to file to read input sites from (disable other options)
    #[clap(short, long)]
    path: Option<PathBuf>,

    /// Optional number of sites to generate
    #[clap(short, long)]
    size: Option<usize>,

    /// Enables debug output
    #[clap(short, long)]
    debug: bool,

    /// Reads the sites from stdin (e.g. [[1,0],[0,1],[1,1]])
    #[clap(long)]
    stdin: bool,

    /// Writes sites to file
    #[clap(short, long)]
    write_sites: Option<PathBuf>,

    /// Writes output svg to path
    #[clap(short, long)]
    output_path: PathBuf,

    /// Number of lloyd iterations to run
    #[clap(short, long, default_value_t = 0)]
    lloyd_iterations: usize,

    /// Number of lloyd iterations to run (None, RemoveSitesOutsideBoundingBoxOnly, Clip)
    #[clap(short, long, default_value_t = ClipBehavior::Clip)]
    clip_behavior: ClipBehavior,

    /// Rotates the sites by this many degrees
    #[clap(short, long, default_value_t = 0.)]
    rotate: f64,

    /// Draw circumcenter circles
    #[clap(long)]
    circumcenter: bool,

    // /// Zoom (scales) rendering
    // #[clap(short, long, default_value_t = 1.0)]
    // zoom: f64,
}

fn main() -> std::io::Result<()> {
    let args = Args::parse();

    let sites = if let Some(path) = args.path {
        // laod sites from file
        let file = File::open(path)?;
        serde_json::from_reader(file)?
    } else if args.stdin {
        serde_json::from_reader(std::io::stdin())?
    } else {
        let mut rng = rand::thread_rng();
        let range = CANVAS_SIZE * 0.5 * (1. - CANVAS_MARGIN);
        let x_range = rand::distributions::Uniform::new(-range, range);
        let y_range = rand::distributions::Uniform::new(-range, range);

        let size = args.size.unwrap_or(10);

        // generate random sites
        (0..size)
            .map(move |_| [rng.sample(x_range), rng.sample(y_range)])
            .collect::<Vec<_>>()
    };

    if let Some(write_sites) = args.write_sites {
        serde_json::to_writer_pretty(&File::create(write_sites)?, &sites)?
    }

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

    let sites = center_rotate_and_scale(&sites, args.rotate.to_radians());

    // build voronoi
    let voronoi = VoronoiBuilder::default()
        .set_sites(sites)
        .set_bounding_box(BoundingBox::new(Point { x: CANVAS_SIZE / 2.0, y: CANVAS_SIZE / 2.0 }, CANVAS_SIZE * (1.0 - CANVAS_MARGIN), CANVAS_SIZE * (1.0 - CANVAS_MARGIN)))
        .set_lloyd_relaxation_iterations(args.lloyd_iterations)
        .set_clip_behavior(args.clip_behavior)
        .build()
        .expect("Couldn't build voronoi");

    // generate SVG
    let contents = format!(
        r#"
<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">
<rect width="100%" height="100%" fill="white" />
<rect x="{bb_x}" y="{bb_y}" width="{bb_width}" height="{bb_height}" style="fill-opacity:0;stroke-opacity:0.25;stroke-width:3;stroke:rgb(0,0,0)" />
    {sites}
    {circumcenters}
    {voronoi_edges}
    {triangles}
    {circumcenter_circles}
</svg>"#,
        width = CANVAS_SIZE,
        height = CANVAS_SIZE,
        bb_x = voronoi.bounding_box().left(),
        bb_y = voronoi.bounding_box().top(),
        bb_width = voronoi.bounding_box().width(),
        bb_height = voronoi.bounding_box().height(),
        sites = render_point(voronoi.sites(), SITE_COLOR, false),
        circumcenters = render_point(voronoi.vertices(), CIRCUMCENTER_COLOR, true),
        voronoi_edges = render_voronoi_edges(&voronoi),
        triangles = render_triangles(&voronoi),
        circumcenter_circles = if args.circumcenter { render_circumcenters(&voronoi) } else { "".to_string() },
    );

    if args.debug {
        println!("{:#?}", voronoi);
        println!("Hull: {:#?}", voronoi.triangulation().hull);
    }

    File::create(args.output_path)?.write_all(contents.as_bytes())
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

            if RENDER_LABELS {
                acc + &format!(r#"<text x="{x}" y="{y}" style="stroke:{color};">{text}</text>"#, x = mid.x, y = mid.y, text = e)
            } else {
                acc
            }
        } else {
            acc
        }
    })
}

fn render_point(points: &[Point], color: &str, jitter: bool) -> String {
    let mut rng = rand::thread_rng();
    let jitter_range = rand::distributions::Uniform::new(-JITTER_RANGE_VALUE, JITTER_RANGE_VALUE);

    points
        .iter()
        .enumerate()
        .fold(String::new(), |acc, (i, p)| {
            let (x, y) = if jitter {
                (p.x + rng.sample(jitter_range), p.y + rng.sample(jitter_range))
            } else {
                 (p.x, p.y)
            };

            acc + &format!(
                r#"<circle id="pt_{pi}" cx="{x}" cy="{y}" r="{size}" fill="{color}"/>"#,
                pi = i,
                size = POINT_SIZE,
                color = color
            ) + &if RENDER_LABELS { format!(r#"<text x="{x}" y="{y}" style="stroke:{color};">{text}</text>"#, text = i) } else { "".to_string() }
        })
}

fn render_circumcenters(voronoi: &Voronoi) -> String {
    voronoi.vertices().iter().enumerate().fold(String::new(), |acc, (triangle, circumcenter)| {
        if triangle < voronoi.triangulation().triangles.len() / 3  {
            let point_on_circle = &voronoi.sites()[voronoi.triangulation().triangles[triangle * 3]];
            let radius = ((point_on_circle.x - circumcenter.x).powi(2) + (point_on_circle.y - circumcenter.y).powi(2)).sqrt();

            acc + &format!(
                r#"<circle id="ct_{pi}" cx="{x}" cy="{y}" r="{radius}" fill="none" stroke="{color}" stroke-opacity="0.25" />"#,
                pi = triangle,
                x = circumcenter.x,
                y = circumcenter.y,
                color = CIRCUMCENTER_CIRCLE_COLOR
            )
        } else {
            acc
        }
    })
}

fn render_voronoi_edges(voronoi: &Voronoi) -> String {
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

    buffer
}

/// Finds the center point and farthest point from it, then generates a new vector of
/// scaled and offset points such that they fit between [0..SIZE]
fn center_rotate_and_scale(points: &[[f64;2]], rotation: f64) -> Vec<Point> {
    let mut center = points.iter().fold([0., 0.], |acc, p| [acc[0] + p[0], acc[1] + p[1]]);
    center[0] /= points.len() as f64;
    center[1] /= points.len() as f64;

    let farthest_distance = points
        .iter()
        .map(|p| {
            let (x, y) = (center[0] - p[0], center[1] - p[1]);
            x * x + y * y
        })
        .reduce(f64::max)
        .unwrap()
        .sqrt();

    let scale = CANVAS_SIZE / (farthest_distance * 2.0);
    let offset = (
        (CANVAS_SIZE / 2.0) - (scale * center[0]),
        (CANVAS_SIZE / 2.0) - (scale * center[1]),
    );

    points
        .iter()
        .map(|p| Point {
            x: scale * (p[0] *  rotation.cos() - p[1] * rotation.sin()) + offset.0,
            y: scale * (p[1] *  rotation.cos() + p[0] * rotation.sin()) + offset.1,
        })
        .collect()
}