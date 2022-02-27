use clap::StructOpt;
use delaunator::{EMPTY, next_halfedge};
use rand::Rng;
use voronoice::{BoundingBox, Point, Voronoi, VoronoiBuilder, ClipBehavior};
use std::{fs::File, io::Write, path::PathBuf};
const CANVAS_SIZE: f64 = 800.;
const CANVAS_MARGIN: f64 = 0.;
const POINT_SIZE: usize = 2;
const CIRCUMCENTER_CIRCLE_COLOR: &str = "black";
const SITE_COLOR: &str = "black";
const CIRCUMCENTER_COLOR: &str = "red";
const LINE_WIDTH: usize = 1;
const VORONOI_EDGE_COLOR: &str = "blue";
const TRIANGULATION_HULL_COLOR: &str = "green";
const TRIANGULATION_LINE_COLOR: &str = "grey";
const JITTER_RANGE_VALUE: f64 = 5.;

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

    /// Zoom (scales) rendering
    #[clap(short, long, default_value_t = 1.0)]
    zoom: f64,

    /// Pans the view on x
    #[clap(long, default_value_t = 0.)]
    pan_x: f64,

    /// Pans the view on y
    #[clap(long, default_value_t = 0.)]
    pan_y: f64,

    /// Print only sites in this list
    #[clap(long)]
    filter_sites: Vec<usize>,

    /// Optional bounding box side length (width = height = side)
    #[clap(long)]
    bounding_box_side: Option<f64>,

    /// Whether to render voronoi edges or not
    #[clap(long)]
    render_voronoi_edges: Option<bool>,

    /// Whether to render labels
    #[clap(long)]
    render_labels: Option<bool>,

    /// Whether to render site labels
    #[clap(long)]
    render_site_labels: Option<bool>,

    /// Whether to render voronoi vertex (circumcenter) labels
    #[clap(long)]
    render_voronoi_vertex_labels: Option<bool>,

    /// Whether to render voronoi vertex (circumcenter) labels
    #[clap(long)]
    render_edge_labels: Option<bool>,
}

fn main() -> std::io::Result<()> {
    let mut args = Args::parse();
    args.filter_sites.sort();
    if args.render_labels == Some(false) {
        args.render_site_labels = args.render_labels;
        args.render_voronoi_vertex_labels = args.render_labels;
        args.render_edge_labels = args.render_labels;
    }

    let sites = if let Some(path) = &args.path {
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

    if let Some(write_sites) = &args.write_sites {
        serde_json::to_writer_pretty(&File::create(write_sites)?, &sites)?
    }

    let transform = build_transformation(&sites, &args);
    let sites = sites.iter().map(|&[x, y]| Point { x, y }).collect();

    // build voronoi
    let voronoi = VoronoiBuilder::default()
        .set_sites(sites)
        .set_bounding_box(transform.bounding_box(&args))
        .set_lloyd_relaxation_iterations(args.lloyd_iterations)
        .set_clip_behavior(args.clip_behavior)
        .build()
        .expect("Couldn't build voronoi");

    let bounding_box_top_left = transform.transform(&Point { x: voronoi.bounding_box().left(), y: voronoi.bounding_box().top() });
    let bounding_box_side = transform.transform(voronoi.bounding_box().bottom_left()).y - bounding_box_top_left.y;

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
        bb_x = bounding_box_top_left.x,
        bb_y = bounding_box_top_left.y,
        bb_width = bounding_box_side,
        bb_height = bounding_box_side,
        sites = render_point(&transform, voronoi.sites(), SITE_COLOR, false, args.render_site_labels.unwrap_or(true)),
        circumcenters = render_point(&transform, voronoi.vertices(), CIRCUMCENTER_COLOR, true, args.render_voronoi_vertex_labels.unwrap_or(true)),
        voronoi_edges = if args.render_voronoi_edges.unwrap_or(true) { render_voronoi_edges(&transform, &voronoi, &args) } else { "".to_string() },
        triangles = render_triangles(&transform, &voronoi, args.render_edge_labels.unwrap_or(true)),
        circumcenter_circles = if args.circumcenter { render_circumcenters(&transform, &voronoi) } else { "".to_string() },
    );

    if args.debug {
        println!("{:#?}", voronoi);
        println!("Hull: {:#?}", voronoi.triangulation().hull);
    }

    File::create(args.output_path)?.write_all(contents.as_bytes())
}

fn render_triangles(transform: &Transform, voronoi: &Voronoi, labels: bool) -> String {
    let triangulation = voronoi.triangulation();
    let points = voronoi.sites();

    (0..triangulation.triangles.len()).fold(String::new(), |acc, e| {
        if e > triangulation.halfedges[e] || triangulation.halfedges[e] == EMPTY {
            let start = transform.transform(&points[triangulation.triangles[e]]);
            let end = transform.transform(&points[triangulation.triangles[next_halfedge(e)]]);
            let mid = Point { x: (start.x + end.x) / 2.0, y: (start.y + end.y) / 2.0 };
            let (color, label) = if triangulation.halfedges[e] == EMPTY {
                (TRIANGULATION_HULL_COLOR, format!("{e}"))
            } else {
                (TRIANGULATION_LINE_COLOR, format!("{e} ({})", triangulation.halfedges[e]))
            };

            let acc = acc + &format!(r#"<line id="dedge_{id}" stroke-dasharray="10,10" x1="{x0}" y1="{y0}" x2="{x1}" y2="{y1}" style="stroke:{color};stroke-width:{width}" />"#,
                id = e,
                x0 = start.x,
                y0 = start.y,
                x1=end.x,
                y1=end.y,
                width = LINE_WIDTH,
                color = color);

            if labels {
                acc + &format!(r#"<text x="{x}" y="{y}" style="stroke:{color};">{label}</text>"#, x = mid.x, y = mid.y)
            } else {
                acc
            }
        } else {
            acc
        }
    })
}

fn render_point(transform: &Transform, points: &[Point], color: &str, jitter: bool, labels: bool) -> String {
    let mut rng = rand::thread_rng();
    let jitter_range = rand::distributions::Uniform::new(-JITTER_RANGE_VALUE, JITTER_RANGE_VALUE);

    points
        .iter()
        .enumerate()
        .fold(String::new(), |acc, (i, p)| {
            let p = transform.transform(p);
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
            ) + &if labels { format!(r#"<text x="{x}" y="{y}" style="stroke:{color};">{text}</text>"#, text = i) } else { "".to_string() }
        })
}

fn render_circumcenters(transform: &Transform, voronoi: &Voronoi) -> String {
    voronoi.vertices().iter().enumerate().fold(String::new(), |acc, (triangle, circumcenter)| {
        if triangle < voronoi.triangulation().triangles.len() / 3  {
            let circumcenter = transform.transform(circumcenter);
            let point_on_circle = transform.transform(&voronoi.sites()[voronoi.triangulation().triangles[triangle * 3]]);
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

fn render_voronoi_edges(transform: &Transform, voronoi: &Voronoi, args: &Args) -> String {
    let mut buffer = String::new();

    for cell in voronoi.iter_cells() {
        if args.filter_sites.len() > 0 && args.filter_sites.binary_search(&cell.site()).is_err() {
            // do not print site if not in filter list
            continue;
        }

        let render = |(start, end)| {
            let start = transform.transform(start);
            let end = transform.transform(end);

            buffer += &format!(r#"<line x1="{x0}" y1="{y0}" x2="{x1}" y2="{y1}" style="stroke:{color};stroke-width:{width}" />"#,
                x0 = start.x,
                y0 = start.y,
                x1 = end.x,
                y1 = end.y,
                width = LINE_WIDTH,
                color = VORONOI_EDGE_COLOR);
        };

        if let Some(first) = cell.iter_vertices().next() {
            if args.clip_behavior != ClipBehavior::Clip && cell.is_on_hull() {
                // hull cells are not closed when clipping is disabled so we should not render last edge
                cell.iter_vertices().zip(cell.iter_vertices().skip(1)).for_each(render);
            } else {
                cell.iter_vertices().zip(cell.iter_vertices().skip(1).chain(std::iter::once(first))).for_each(render);
            };
        }
    }

    buffer
}

#[derive(Default, Clone)]
struct Transform {
    scale: f64,
    center: Point,
    offset: Point,
    farthest_distance: f64,
    rotation: f64
}

impl Transform {
    fn transform<T : std::borrow::Borrow<Point>>(&self, p: T) -> Point {
        let p = p.borrow();
        Point {
            x: self.scale * (p.x *  self.rotation.cos() - p.y * self.rotation.sin()) + self.offset.x,
            y: self.scale * (p.y *  self.rotation.cos() + p.x * self.rotation.sin()) + self.offset.y,
        }
    }

    fn bounding_box(&self, args: &Args) -> BoundingBox {
        if let Some(side) = args.bounding_box_side {
            BoundingBox::new_centered_square(side)
        } else {
            let box_side = self.farthest_distance * 2.0 * (1.0 - CANVAS_MARGIN);
            BoundingBox::new(self.center.clone(), box_side, box_side)
        }
    }
}

/// Finds the center point and farthest point from it, then generates a new vector of
/// scaled and offset points such that they fit between [0..SIZE]
fn build_transformation(points: &[[f64;2]], args: &Args) -> Transform {
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

    let scale = args.zoom * CANVAS_SIZE / (farthest_distance * 2.0);
    let offset = Point {
        x: (CANVAS_SIZE / 2.0) - (scale * center[0]) - args.pan_x,
        y: (CANVAS_SIZE / 2.0) - (scale * center[1]) - args.pan_y,
    };

    let center = Point { x: center[0], y: center[1] };

    Transform {
        center,
        scale,
        offset,
        farthest_distance,
        rotation: args.rotate.to_radians(),
    }
}