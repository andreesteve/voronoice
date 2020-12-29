use super::{Point, Voronoi, HullBehavior};
use super::utils::calculate_approximated_cetroid;

#[derive(Default)]
pub struct VoronoiBuilder {
    sites: Option<Vec<Point>>,
    hull_behavior: HullBehavior,
    lloyd_iterations: usize
}

impl VoronoiBuilder {
    pub fn close_hull(&mut self) -> &mut Self {
        self.set_hull_behavior(HullBehavior::Closed)
    }

    pub fn set_hull_behavior(&mut self, behavior: HullBehavior) -> &mut Self {
        self.hull_behavior = behavior;
        self
    }

    pub fn set_sites(&mut self, sites: Vec<Point>) -> &mut Self {
        self.sites.replace(sites);
        self
    }

    pub fn set_lloyd_relaxation_iterations(&mut self, iterations: usize) -> &mut Self {
        self.lloyd_iterations = iterations;
        self
    }

    pub fn build(mut self) -> Voronoi {
        let v = Voronoi::new(
            self.sites.take().expect("Cannot build voronoi without sites. Call set_sites() first."),
            self.hull_behavior
        );

        self.perform_lloyd_relaxation(v)
    }

    fn perform_lloyd_relaxation(&mut self, mut v: Voronoi) -> Voronoi {
        for _ in 0..self.lloyd_iterations {
            // get verteces for each cell and approximate centroid
            let new_sites = (0..v.sites.len())
                .map(|c| calculate_approximated_cetroid(v.get_cell_verteces(c)))
                .collect::<Vec<Point>>();

            // recompute new voronoi with sites after relaxation
            let mut builder = Self::create_builder_from_voronoi_without_sites(&v);
            builder.set_sites(new_sites);
            v = builder.build();
        }

        v
    }

    #[allow(dead_code)]
    pub fn generate_random_sites(&mut self, size: usize, x_range: (f64, f64), y_range: (f64, f64)) -> &mut Self {
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
    pub fn generate_circle_sites(&mut self, size: usize) -> &mut Self {
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
    pub fn generate_square_sites(&mut self, width: usize, height: usize) -> &mut Self {
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

    #[allow(dead_code)]
    pub fn generate_triangle_sites(&mut self) -> &mut Self {
        let mut sites = vec![];

        sites.push(Point { x: -0.3, y: -0.3 });
        sites.push(Point { x: 0.3, y: -0.3 });
        sites.push(Point { x: 0.0, y: 0.3 });

        sites.push(Point { x: -0.6, y: -0.6 });
        sites.push(Point { x: 0.6, y: -0.6 });
        sites.push(Point { x: 0.0, y: 0.6 });

        sites.push(Point { x: -1.0, y: -1.0 });
        sites.push(Point { x: -1.0, y: 1.0 });
        sites.push(Point { x: 1.0, y: -1.0 });
        sites.push(Point { x: 1.0, y: 1.0 });

        self.set_sites(sites)
    }

    #[allow(dead_code)]
    pub fn generate_special_case_1(&mut self) -> &mut Self {
        let mut sites = vec![];

        sites.push(Point { x: -0.5, y: -0.5 });
        sites.push(Point { x: -0.5, y: 0.0 });
        sites.push(Point { x: 0.0, y: 0.0 });
        sites.push(Point { x: 0.5, y: -1.0 });

        self.set_sites(sites)
    }

    /// When constraint to a (2, 2) bounding box, this creates a case in which a triangle has a extremelly distance circumcenter. When clipped to the bounding box, it causes the voronoi cell to not contain the site.
    #[allow(dead_code)]
    pub fn generate_special_case_2(&mut self) -> &mut Self {
        let mut sites = vec![];

        sites.push(Point { x: 0.0, y: 0.0 });
        sites.push(Point { x: 0.22, y: -0.75 });
        sites.push(Point { x: 0.25, y: -0.83 });
        sites.push(Point { x: -0.50, y: 0.0 });

        self.set_sites(sites)
    }

    fn create_builder_from_voronoi_without_sites(v: &Voronoi) -> Self {
        Self {
            hull_behavior: v.hull_behavior,
            ..Default::default()
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