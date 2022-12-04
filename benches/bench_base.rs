use criterion::{BatchSize, Bencher};
use rand::Rng;
use voronoice::{BoundingBox, Point, VoronoiBuilder};

pub fn create_random_builder(size: usize) -> VoronoiBuilder {
    let mut rng = rand::thread_rng();
    let builder = VoronoiBuilder::default();
    let bbox = BoundingBox::default();

    let x_range = rand::distributions::Uniform::new(-bbox.width() / 2.0, bbox.width() / 2.0);
    let y_range = rand::distributions::Uniform::new(-bbox.height() / 2.0, bbox.height() / 2.0);
    let sites = (0..size)
        .map(|_| Point { x: rng.sample(x_range), y: rng.sample(y_range) })
        .collect();

    builder
        .set_bounding_box(bbox)
        .set_sites(sites)
}

pub fn create_benchmark_fn(b: &mut Bencher, size: usize) {
    b.iter_batched(
        || create_random_builder(size),
        |b| b.build(),
        BatchSize::SmallInput);
}