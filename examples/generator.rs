use std::process::exit;

use rand::Rng;
use voronoice::{BoundingBox, VoronoiBuilder, Point, Voronoi};
const SIZE: usize = 5;
const TRIES: usize = 50_000;

fn main() {
    let mut rng = rand::thread_rng();
    let range = 10.0;
    let x_range = rand::distributions::Uniform::new(-range, range);
    let y_range = rand::distributions::Uniform::new(-range, range);

    println!("Checking {TRIES} voronoi diagrams of size {SIZE} if a voronoi vertex outside of the bounding box is NOT connected to the delaunay hull");

    for _ in 0..TRIES {
        // generate random sites
        let sites = (0..SIZE)
            .map(|_| Point { x: rng.sample(x_range), y: rng.sample(y_range) })
            .collect::<Vec<_>>();

        // build voronoi
        let voronoi = VoronoiBuilder::default()
            .set_sites(sites)
            .set_bounding_box(BoundingBox::new_centered_square(2.0 * range))
            .set_lloyd_relaxation_iterations(0)
            .set_clip_behavior(voronoice::ClipBehavior::Clip)
            .build()
            .expect("Couldn't build voronoi");

        for cell in voronoi.iter_cells() {
            let vertices: Vec<Point> = cell.iter_vertices().cloned().collect();

            let area = calculate_area(&vertices);
            if area <= 0. {
                println!("Cell {}: not counter-clockwise. Area is {area}. {:?}", cell.site(), cell.iter_triangles().collect::<Vec<usize>>());
                fail(&voronoi);
            }

            vertices.iter().enumerate().filter(|(_, p)| !voronoi.bounding_box().is_inside(p)).for_each(|(v, p)| {
                println!("Cell {}: vertex {v} {:?} is outside bounding box.", cell.site(), p);
                fail(&voronoi);
            });
        }
    }
}

fn fail(voronoi: &Voronoi) {
    println!("Found invalid. Printed to file...");
    let s = format!("{:?}", voronoi.sites());
    std::io::Write::write_all(&mut std::fs::File::create("sites.json").unwrap(), s.as_bytes()).unwrap();
    exit(-1);
}

/// Check that the cell is ordered counter-clockwise and inside the bounding box.
fn calculate_area(vertices: &Vec<Point>) -> f64 {
    vertices.iter().zip(vertices.iter().cycle().skip(1)).fold(0.0, |acc, (a, b)| {
            acc + ((b.x - a.x) * (b.y + a.y))
    })
}