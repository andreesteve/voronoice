# Voronoice

[![Crate](https://img.shields.io/crates/v/voronoice.svg)](https://crates.io/crates/voronoice)
[![API](https://docs.rs/voronoice/badge.svg)](https://docs.rs/voronoice)
![Build](https://github.com/andreesteve/voronoice/actions/workflows/build.yml/badge.svg)

A nice and fast way to construct 2D [Voronoi diagrams](https://en.wikipedia.org/wiki/Voronoi_diagram) written in Rust.

Voronoice builds Voronoi diagrams by first obtaining its [Delaunay triangulation](https://en.wikipedia.org/wiki/Delaunay_triangulation), through the really fast [delaunator](https://docs.rs/delaunator/*/delaunator) crate and then extracting its dual Voronoi diagram.

## Example

```rust
use voronoice::*;

// voronoi sites
let sites = vec![
    Point { x: 0.0, y: 0.0 }, Point { x: 1.0, y: 0.0 }, Point { x: 0.0, y: 1.0 }
];

// builds a voronoi diagram from the set of sites above, bounded by a square of size 4
let my_voronoi = VoronoiBuilder::default()
    .set_sites(sites)
    .set_bounding_box(BoundingBox::new_centered_square(4.0))
    .set_distance_function(DistanceFunction::Euclidean)
    .set_lloyd_relaxation_iterations(5)
    .build()
    .unwrap();

// inspect cells through iterators
my_voronoi.iter_cells().for_each(|cell| {
    println!("Vertices of cell: {:?}", cell.vertices().collect::<Vec<&Point>>())
});

// or probe cells individually
let my_cell = my_voronoi.cell(1);
println!("Second cell has site {:?}, voronoi vertices {:?} and delaunay triangles {:?}",
    my_cell.site_position(),
    my_cell.vertices().collect::<Vec<&Point>>(),
    my_cell.triangles().collect::<Vec<usize>>());

// or, for graphical applications, that benefit from index buffers
// you can access the raw, indexed data
let all_voronoi_cell_vertices = my_voronoi.vertices();
let indexed_voronoi_cells = my_voronoi.cells();
println!("The first vertex position for the first voronoi cell is at {:?}",
    all_voronoi_cell_vertices[indexed_voronoi_cells[0][0]]);
```

## Documentation

On [docs.rs](https://docs.rs/voronoice/*/voronoice/).

## Performance

Here are some generation times on a 3.5GHz Core i7 from 2012.

| Number of points | Time         |
| -----------------|--------------|
|      1,000       | 150 µs       |
|     10,000       | 1.5 ms       |
|    100,000       | 18 ms        |
|  1,000,000       | 270 ms       |
|  5,000,000       | 1.6 s        |
| 10,000,000       | 3.5 s        |

A comparison benchmark with other Rust Voronoi diagram generating libraries can be found [here](https://github.com/andreesteve/voronoi-benchmark-rs).

# Examples

## [svg](examples/svg.rs)

```cargo run --example svg -- -s10 -l2 -z0.8 -o example.svg --render-labels false```

Generates a SVG image to visualize the Voronoi-Delaunay graph. Provide ```--help``` to see other options.

![Example of SVG Voronoi](examples/assets/example.svg)

## [image](examples/image.rs)

```cargo run --release --example image -- examples/assets/mona_noice_small.jpg 300```

 Generates a color averaged image by overlaying a Voronoi diagram

![Original image](examples/assets/mona_noice.jpg)
![Image after voronoi cell averaging](examples/assets/mona_noice_voronoi.jpg)

## Others

* [voronoice-inspector](https://github.com/andreesteve/voronoice-inspector) - an interactive tool to render and inspect Voronoi diagrams