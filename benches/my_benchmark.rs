use criterion::{BatchSize, Bencher, Criterion, criterion_group, criterion_main};
use voronoice::*;

fn create_random_builder(size: usize) -> VoronoiBuilder {
    VoronoiBuilder::default().generate_random_sites_constrained(size)
}

fn create_benchmark_fn(b: &mut Bencher, size: usize) {
    b.iter_batched(
        || create_random_builder(size),
        |b| b.build(),
        BatchSize::SmallInput);
}

fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("100 random sites", |b| create_benchmark_fn(b, 100));
    c.bench_function("1,000 random sites", |b| create_benchmark_fn(b, 1_000));
    c.bench_function("10,000 random sites", |b| create_benchmark_fn(b, 10_000));
    c.bench_function("100,000 random sites", |b| create_benchmark_fn(b, 100_000));
    c.bench_function("1,000,000 random sites", |b| create_benchmark_fn(b, 1_000_000));

    let mut large_group = c.benchmark_group("very_large");
    large_group.sample_size(10);
    large_group.bench_function("10,000,000 random sites", |b| create_benchmark_fn(b, 10_000_000));
    large_group.bench_function("20,000,000 random sites", |b| create_benchmark_fn(b, 20_000_000));
    large_group.finish();
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);