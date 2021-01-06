use super::{BoundingBox, ClipBehavior, Point, Voronoi};
use super::utils::calculate_approximated_cetroid;

/// Provides a convenient way to construct a Voronoi diagram.
#[derive(Default)]
pub struct VoronoiBuilder {
    sites: Option<Vec<Point>>,
    lloyd_iterations: usize,
    bounding_box: BoundingBox,
    clip_behavior: ClipBehavior,
}

impl VoronoiBuilder {

    /// Sets the [BoundingBox] that will be used to enclose the graph.
    pub fn set_bounding_box(mut self, bounding_box: BoundingBox) -> Self {
        self.bounding_box = bounding_box;
        self
    }

    /// Sets the [ClipBehavior] to be used when building the graph.
    pub fn set_clip_behavior(mut self, clip_behavior: ClipBehavior) -> Self {
        self.clip_behavior = clip_behavior;
        self
    }

    /// Sets a vector of [Point]s representing the sites of each Voronoi cell that should be constructed.
    pub fn set_sites(mut self, sites: Vec<Point>) -> Self {
        self.sites.replace(sites);
        self
    }

    /// Sets the number of [LLoyd relaxation](https://en.wikipedia.org/wiki/Lloyd%27s_algorithm) iterations that should be run as part of the graph generation.
    pub fn set_lloyd_relaxation_iterations(mut self, iterations: usize) -> Self {
        self.lloyd_iterations = iterations;
        self
    }

    /// Consumes this builder and generates a Voronoi diagram/graph.
    /// An ```Option<Voronoi>``` is returned. None may be a valid return value if the set of sites do not generate a valid graph.
    pub fn build(mut self) -> Option<Voronoi> {
        let v = Voronoi::new(
            self.sites.take().expect("Cannot build voronoi without sites. Call set_sites() first."),
            self.bounding_box.clone(),
            self.clip_behavior,
        );

        self.perform_lloyd_relaxation(v)
    }

    fn perform_lloyd_relaxation(&mut self, mut v: Option<Voronoi>) -> Option<Voronoi> {
        for _ in 0..self.lloyd_iterations {
            if let Some(voronoi) = v {
                // get vertices for each cell and approximate centroid
                let new_sites = voronoi.cells_iter().map(|c| calculate_approximated_cetroid(c.vertices()))
                    .collect::<Vec<Point>>();

                // recompute new voronoi with sites after relaxation
                v = Self::create_builder_from_voronoi_without_sites(&voronoi)
                    .set_sites(new_sites)
                    .build();
            } else {
                break;
            }
        }

        v
    }

    /// Generates sites in the format of a circle centered at the origin with ```size``` points and radius ```radius```.
    /// Internally calls [Self::set_sites] with the generated value.
    pub fn generate_circle_sites(self, size: usize, radius: f64) -> Self {
        let len = size;
        let r = radius;
        let mut sites = vec![];
        sites.push(Point { x: 0.0, y: 0.0 });
        for i in 0..len {
            let a = (i as f64 * 360.0 / len as f64).to_radians();
            sites.push(Point {
                x: r * a.sin(),
                y: r * a.cos()
            });
        }

        self.set_sites(sites)
    }

    /// Generates sites in the format of a rectangle centered at the origin with ```width``` and ```height``` and ```width``` times ```height``` points.
    /// Internally calls [Self::set_sites] with the generated value.
    pub fn generate_rect_sites(self, width: usize, height: usize) -> Self {
        let mut sites = vec![];
        let fwidth = width as f64;
        let fheight = height as f64;

        for i in 0..width {
            for j in 0..height {
                sites.push(Point {
                    x: i as f64 / fwidth - 0.5,
                    y: j as f64/ fheight - 0.5
                });
            }
        }
        self.set_sites(sites)
    }

    /// Generates sites in the format of a square centered at the origin with ```width``` and ```width``` square points.
    /// Internally calls [Self::set_sites] with the generated value.
    pub fn generate_square_sites(self, width: usize) -> Self {
        self.generate_rect_sites(width, width)
    }

    fn create_builder_from_voronoi_without_sites(v: &Voronoi) -> Self {
        Self {
            bounding_box: v.bounding_box.clone(),
            clip_behavior: v.clip_behavior,
            lloyd_iterations: 0,
            sites: None,
        }
    }
}

impl From<&Voronoi> for VoronoiBuilder {
    /// Creates a builder with same configurations that produced the original voronoi.
    /// Useful for performing Lloyd relaxation or storing the configuration to generate a identical diagram.
    fn from(v: &Voronoi) -> Self {
        let mut builder = Self::create_builder_from_voronoi_without_sites(v);
        builder.sites = Some(v.sites.clone());

        builder
    }
}

impl From<Voronoi> for VoronoiBuilder {
    /// Creates a builder with same configurations that produced the original voronoi, consuming it.
    /// Useful for performing Lloyd relaxation or storing the configuration to generate a identical diagram.
    fn from(v: Voronoi) -> Self {
        let mut builder = Self::create_builder_from_voronoi_without_sites(&v);
        builder.sites = Some(v.sites);

        builder
    }
}