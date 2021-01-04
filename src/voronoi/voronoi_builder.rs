use super::{BoundingBox, ClipBehavior, Point, Voronoi};
use super::utils::calculate_approximated_cetroid;

#[derive(Default)]
pub struct VoronoiBuilder {
    sites: Option<Vec<Point>>,
    lloyd_iterations: usize,
    bounding_box: BoundingBox,
    clip_behavior: ClipBehavior,
}

impl VoronoiBuilder {
    pub fn set_bounding_box(mut self, bounding_box: BoundingBox) -> Self {
        self.bounding_box = bounding_box;
        self
    }

    pub fn set_clip_behavior(mut self, clip_behavior: ClipBehavior) -> Self {
        self.clip_behavior = clip_behavior;
        self
    }

    pub fn set_sites(mut self, sites: Vec<Point>) -> Self {
        self.sites.replace(sites);
        self
    }

    pub fn set_lloyd_relaxation_iterations(mut self, iterations: usize) -> Self {
        self.lloyd_iterations = iterations;
        self
    }

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
                let new_sites = (0..voronoi.sites.len())
                    .map(|c| calculate_approximated_cetroid(voronoi.get_cell_vertices(c)))
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

    #[allow(dead_code)]
    pub fn generate_random_sites_constrained(self, size: usize) -> Self {
        let x_range = (-self.bounding_box.width() / 2.0, self.bounding_box.width() / 2.0 );
        let y_range = (-self.bounding_box.height() / 2.0, self.bounding_box.height() / 2.0 );
        self.generate_random_sites(size, x_range, y_range)
    }

    #[allow(dead_code)]
    pub fn generate_random_sites(self, size: usize, x_range: (f64, f64), y_range: (f64, f64)) -> Self {
        use rand::Rng;
        let mut rng = rand::thread_rng();

        let x_range = rand::distributions::Uniform::new(x_range.0, x_range.1);
        let y_range = rand::distributions::Uniform::new(y_range.0, y_range.1);
        let sites = (0..size)
            .map(|_| Point { x: rng.sample(x_range), y: rng.sample(y_range) })
            .collect();

        self.set_sites(sites)
    }

    #[allow(dead_code)]
    pub fn generate_circle_sites(self, size: usize) -> Self {
        let len = size;
        let r = 1.0;
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

    #[allow(dead_code)]
    pub fn generate_square_sites(self, width: usize, height: usize) -> Self {
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
    /// Useful for performing Lloyd relaxation.
    fn from(v: &Voronoi) -> Self {
        let mut builder = Self::create_builder_from_voronoi_without_sites(v);
        builder.sites = Some(v.sites.clone());

        builder
    }
}

impl From<Voronoi> for VoronoiBuilder {
    /// Creates a builder with same configurations that produced the original voronoi, consuming it.
    /// Useful for performing Lloyd relaxation.
    fn from(v: Voronoi) -> Self {
        let mut builder = Self::create_builder_from_voronoi_without_sites(&v);
        builder.sites = Some(v.sites);

        builder
    }
}