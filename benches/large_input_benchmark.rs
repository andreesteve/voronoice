use criterion::{Criterion, criterion_group, criterion_main};

mod bench_base;
use bench_base::*;

fn criterion_benchmark(c: &mut Criterion) {
    let mut large = c.benchmark_group("large");
    large.bench_function("100,000 random sites", |b| create_benchmark_fn(b, 100_000));
    large.bench_function("1,000,000 random sites", |b| create_benchmark_fn(b, 1_000_000));
    large.finish();

    // FIXME: this takes quite a bit of memory, need to play with large input bench configuration
    let mut very_large_group = c.benchmark_group("very_large");
    very_large_group.sample_size(10);
    very_large_group.bench_function("5,000,000 random sites", |b| create_benchmark_fn(b, 5_000_000));
    very_large_group.bench_function("10,000,000 random sites", |b| create_benchmark_fn(b, 10_000_000));
    very_large_group.finish();
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);