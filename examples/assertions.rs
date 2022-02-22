use std::process::exit;

use rand::Rng;
use voronoice::{BoundingBox, VoronoiBuilder, Point, NeighborSiteIterator};
const SIZE: usize = 15;
const TRIES: usize = 50_000;

fn cicumcenter(a: &Point, b: &Point, c: &Point) -> Point {
    // move origin to a
    let b_x = b.x - a.x;
    let b_y = b.y - a.y;
    let c_x = c.x - a.x;
    let c_y = c.y - a.y;

    let bb = b_x * b_x + b_y * b_y;
    let cc = c_x * c_x + c_y * c_y;
    let d = 1.0 / (2.0 * (b_x * c_y - b_y * c_x));

    Point {
        x: a.x + d * (c_y * bb - b_y * cc),
        y: a.y + d * (b_x * cc - c_x * bb),
    }
}

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
            .set_clip_behavior(voronoice::ClipBehavior::None)
            .build()
            .expect("Couldn't build voronoi");

        // precompute which sites are on the hull
        let mut is_site_on_hull = vec![false; voronoi.sites().len()];
        for &h in &voronoi.triangulation().hull {
            is_site_on_hull[h] = true;
        }

        // check that all circumcenters outside of the bounding box are in the delaunay hull or neighbors in the hull
        for (triangle, circumcenter) in voronoi.vertices().iter().enumerate() {
            if !voronoi.bounding_box().is_inside(circumcenter) {

                // get sites associated with the triangle
                let (a, b, c) = (
                    voronoi.triangulation().triangles[3 * triangle],
                    voronoi.triangulation().triangles[3 * triangle + 1],
                    voronoi.triangulation().triangles[3 * triangle + 2]
                );

                assert_eq!(circumcenter, &cicumcenter(&voronoi.sites()[a], &voronoi.sites()[b], &voronoi.sites()[c]), "Triangle calculation is wrong.");

                let is_connected_hull =
                    is_site_on_hull[a] || NeighborSiteIterator::new(&voronoi, a).fold(false, |is_hull_neightbor, neighbor_site| is_hull_neightbor || is_site_on_hull[neighbor_site])
                    || is_site_on_hull[b] || NeighborSiteIterator::new(&voronoi, b).fold(false, |is_hull_neightbor, neighbor_site| is_hull_neightbor || is_site_on_hull[neighbor_site])
                    || is_site_on_hull[c] || NeighborSiteIterator::new(&voronoi, c).fold(false, |is_hull_neightbor, neighbor_site| is_hull_neightbor || is_site_on_hull[neighbor_site])
                    ;

                if !is_connected_hull {
                    println!("Found circumcenter {triangle} for triangle ({a}, {b}, {c}) outside bounding box that is not connected to the hull.");
                    println!("{:#?}", voronoi.sites());
                    exit(-1);
                }
            }
        }
    }

    println!("NONE found.");
}