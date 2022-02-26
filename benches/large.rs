use criterion::{Criterion, criterion_group, criterion_main};

mod bench_base;
use bench_base::*;

fn criterion_benchmark(c: &mut Criterion) {
    let mut large = c.benchmark_group("large");
    large.bench_function("100,000 random sites", |b| create_benchmark_fn(b, 100_000));
    large.bench_function("1,000,000 random sites", |b| create_benchmark_fn(b, 1_000_000));
    large.finish();
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);