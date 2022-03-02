use rand::Rng;
use voronoice::{VoronoiBuilder, BoundingBox, Point};
const SIZE: usize = 1_000_000;

fn main() {
    let mut rng = rand::thread_rng();
    let bbox = BoundingBox::default();

    let x_range = rand::distributions::Uniform::new(-bbox.width() / 2.0, bbox.width() / 2.0);
    let y_range = rand::distributions::Uniform::new(-bbox.height() / 2.0, bbox.height() / 2.0);
    let sites: Vec<Point> = (0..SIZE)
        .map(|_| Point { x: rng.sample(x_range), y: rng.sample(y_range) })
        .collect();

    VoronoiBuilder::default()
        .set_bounding_box(bbox)
        .set_sites(sites)
        .build()
        .expect("Expect voronoi");
}