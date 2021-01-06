use criterion::{BatchSize, Bencher, BenchmarkId, Criterion, criterion_group, criterion_main};
use delaunator::triangulate;
use voronoice::*;

fn create_random_sites(size: usize) -> Vec<Point> {
    use rand::Rng;
    let mut rng = rand::thread_rng();

    let x_range = rand::distributions::Uniform::new(-1.0, 1.0);
    let y_range = rand::distributions::Uniform::new(-1.0, 1.0);
    (0..size)
        .map(|_| Point { x: rng.sample(x_range), y: rng.sample(y_range) })
        .collect()
}

fn delauney_benchmark_fn(b: &mut Bencher, size: usize) {
    b.iter_batched(
        || create_random_sites(size),
        |sites| (triangulate(&sites), sites),
        BatchSize::SmallInput);
}

fn voronoi_benchmark_fn(b: &mut Bencher, size: usize) {
    b.iter_batched(
        || create_random_sites(size),
        |sites| VoronoiBuilder::default().set_sites(sites).build(),
        BatchSize::SmallInput);
}

fn criterion_benchmark(c: &mut Criterion) {
    let mut group = c.benchmark_group("Compare");
    let iterations: [usize;7] = [7, 100, 500, 1_000, 10_000, 100_000, 500_000];

    for size in iterations.iter() {
        group.bench_with_input(
            BenchmarkId::new("voronoi", size),
            size,
            |b, &bench_size| voronoi_benchmark_fn(b, bench_size)
        );

        group.bench_with_input(
            BenchmarkId::new("delauney", size),
            size,
            |b, &bench_size| delauney_benchmark_fn(b, bench_size)
        );
    }

    group.finish();
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);