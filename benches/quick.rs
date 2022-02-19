use criterion::{Criterion, criterion_group, criterion_main};

mod bench_base;
use bench_base::*;

fn criterion_benchmark(c: &mut Criterion) {
    let mut group = c.benchmark_group("quick");
    group.bench_function("100 random sites", |b| create_benchmark_fn(b, 100));
    group.bench_function("1,000 random sites", |b| create_benchmark_fn(b, 1_000));
    group.bench_function("10,000 random sites", |b| create_benchmark_fn(b, 10_000));
    group.finish();
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);