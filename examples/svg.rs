use rand::Rng;
use voronoice::{BoundingBox, Point, Voronoi, VoronoiBuilder};
use std::{fs::File, io::Write};
const CANVAS_SIZE: f64 = 800.;
const POINT_SIZE: usize = 2;
const SITE_COLOR: &str = "black";
const LINE_WIDTH: usize = 1;
const VORONOI_EDGE_COLOR: &str = "blue";
const SIZE: usize = 10;

fn main() -> std::io::Result<()> {
    // generate random sites
    let mut rng = rand::thread_rng();
    let x_range = rand::distributions::Uniform::new(0., CANVAS_SIZE * 0.8);
    let y_range = rand::distributions::Uniform::new(0., CANVAS_SIZE * 0.8);
    let sites = (0..SIZE)
        .map(move |_| Point { x: rng.sample(x_range), y: rng.sample(y_range) })
        .collect::<Vec<Point>>();

        // build voronoi
    let voronoi = VoronoiBuilder::default()
        .set_sites(sites)
        .set_bounding_box(BoundingBox::new(Point { x: CANVAS_SIZE / 2.0, y: CANVAS_SIZE / 2.0 }, CANVAS_SIZE * 0.95, CANVAS_SIZE * 0.95))
        .set_lloyd_relaxation_iterations(5)
        .build()
        .expect("Couldn't build voronoi");

    // generate SVG
    let contents = format!(
        r#"
<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">
<rect width="100%" height="100%" fill="white" />
    {sites}
    {voronoi_edges}
</svg>"#,
        width = CANVAS_SIZE,
        height = CANVAS_SIZE,
        sites = render_point(voronoi.sites()),
        voronoi_edges = render_voronoi_edges(&voronoi)
    );
    File::create("example.svg")?.write_all(contents.as_bytes())
}

fn render_point(points: &[Point]) -> String {
    points
        .iter()
        .enumerate()
        .fold(String::new(), |acc, (_, p)| {
            acc + &format!(
                r#"<circle cx="{x}" cy="{y}" r="{size}" fill="{color}"/>"#,
                x = p.x,
                y = p.y,
                size = POINT_SIZE,
                color = SITE_COLOR
            )
        })
}

fn render_voronoi_edges(voronoi: &Voronoi) -> String {
    let mut buffer = String::new();
    voronoi.iter_cells().for_each(|cell| {
        // TODO: cycle() needs clone, instead of returning Impl Iterator, return a proper iterator struct
        // using a ugly chain below to get (current, next) vertices of a cell
        let first = cell.iter_vertices().next().expect("All cells must have at least 1 vertex");
        cell.iter_vertices().zip(cell.iter_vertices().skip(1).chain(std::iter::once(first))).for_each(|(start, end)|{
            buffer += &format!(r#"<line x1="{x0}" y1="{y0}" x2="{x1}" y2="{y1}" style="stroke:{color};stroke-width:{width}" />"#,
                x0 = start.x,
                y0 = start.y,
                x1 = end.x,
                y1 = end.y,
                width = LINE_WIDTH,
                color = VORONOI_EDGE_COLOR);
        });
    });

    buffer
}