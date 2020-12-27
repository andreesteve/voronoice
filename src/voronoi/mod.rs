mod into_triangle_list;
mod utils;
mod edges_around_site_iterator;
mod voronoi_mesh_generator;

use delaunator::{Triangulation, next_halfedge, triangulate};
use self::edges_around_site_iterator::EdgesAroundSiteIterator;

pub use self::voronoi_mesh_generator::VoronoiMeshGenerator;
pub use delaunator::Point;

/// The dual delauney-voronoi graph.
///
/// This relies on delaunator library processing and data structure. Refer to https://mapbox.github.io/delaunator/ for a complete understanding of the data structure used.
pub struct Voronoi {
    pub x_range: (f64, f64),
    pub y_range: (f64, f64),

    /// These are the sites of each voronoi cell.
    pub sites: Vec<Point>,

    num_of_triangles: usize,
    triangulation: Triangulation,

    /// The circumcenters of each triangle (indexed by triangle / triangle's starting half-edge).
    ///
    /// For a given voronoi cell, its verteces are the circumcenters of its associated triangles.
    ///
    /// # Examples
    /// ```
    /// // For the first triangle, get its circumcenter:
    /// let v = Voronoi { ... };
    /// let c = v.circumcenters[0];
    /// ```
    circumcenters: Vec<Point>

    //diag: Option<voronator::VoronoiDiagram>,
}

impl Default for Voronoi {
    fn default() -> Self {
        Voronoi {
            x_range: (-1.0, 1.0),
            y_range: (-1.0, 1.0),
            sites: vec![],
            circumcenters: vec![],
            triangulation: Triangulation { halfedges: vec![], hull: vec![], triangles: vec![] },
            num_of_triangles: 0,
        }
    }
}

fn cicumcenter(a: &Point, b: &Point, c: &Point) -> Point {
    // move origin to a
    let b_x = b.x - a.x;
    let b_y = b.y - a.y;
    let c_x = c.x - a.x;
    let c_y = c.y - a.y;

    let bb = b_x * b_x + b_y * b_y;
    let cc = c_x * c_x + c_y * c_y;
    let d = 1.0 / (2.0 * (b_x * c_y - b_y * c_x));

    Point {
        x: a.x + d * (c_y * bb - b_y * cc),
        y: a.y + d * (b_x * cc - c_x * bb),
    }
}

// When reading this code think about 'edge' as the starting edge for a triangle
// So you can say that the starting edge indexes the triangle
// For instances, diag.triangles.len() is the number of starting edges and triangles in the triangulation, you can think of diag.triangles[e] as 'e' as being both the index of the
// starting edge and the triangle it represents. When dealing with an arbitraty edge, it may not be a starting edge. You can get the starting edge by dividing the edge by 3 and flooring it.
impl Voronoi {
    pub fn build(&mut self) -> &Voronoi {
        // self.diag = voronator::VoronoiDiagram::from_tuple(
        // &(self.x_range.0, self.y_range.0),
        // &(self.x_range.1, self.y_range.1),
        // &self.points);

        let tri = triangulate(&self.sites).expect("Expected tris");
        let num_of_triangles = tri.triangles.len() / 3;
        assert_eq!(tri.triangles.len() % 3, 0, "Number of half-edges should be a multiple of 3 (each triangle has 3 half-edges).");
        //println!("Triangles: {}, half-edges: {}, opposite-half-edges: {}", num_of_triangles, tri.triangles.len(), tri.halfedges.len());

        // calculate circuncenter of each triangle
        let circumcenters = (0..num_of_triangles)
            .map(|t| cicumcenter(&self.sites[tri.triangles[3* t]], &self.sites[tri.triangles[3* t + 1]], &self.sites[tri.triangles[3* t + 2]]))
            .collect();

        self.triangulation = tri;
        self.circumcenters = circumcenters;
        self.num_of_triangles = num_of_triangles;

        self
    }

    pub fn randomize_points(&mut self, size: usize) -> &Voronoi {
        use rand::Rng;
        let mut rng = rand::thread_rng();

        let x_range = rand::distributions::Uniform::new(self.x_range.0, self.x_range.1);
        let y_range = rand::distributions::Uniform::new(self.y_range.0, self.y_range.1);
        self.sites = (0..size)
            .map(|_| Point { x: rng.sample(x_range), y: rng.sample(y_range) })
            .collect();

        self
    }

    #[allow(dead_code)]
    pub fn loyd_relaxation(&self) -> Voronoi {
        // fn calculate_approximated_cetroid(points: &Vec<Point>) -> Point {
        //     let mut r = Point { x: 0.0 , y: 0.0 };
        //     for p in points.iter() {
        //         r.x += p.x;
        //         r.y += p.y;
        //     }

        //     let n = points.len() as f64;
        //     r.x /= n;
        //     r.y /= n;

        //     r
        // }

        // let points = self.triangulation
        //     .cells
        //     .iter()
        //     .map(|(_, p)| {
        //         let x = calculate_approximated_cetroid(p);
        //         (x.x as f32, x.y as f32)
        //     })
        //     .collect::<Vec<(f32, f32)>>();

        // Voronoi {
        //     sites: points,
        //     diag: None,
        //     ..*self
        // }

        todo!();
    }

    /// Returns the index to the site that half-edge `e` points to.
    /// This is similar to `triangles`. Given an half-edge `e`, `triangles` returns the index of the site the half-edge start off. `site_of_incoming` returns the index of the site the half-edge points to.
    fn site_of_incoming(&self, e: usize) -> usize {
        self.triangulation.triangles[next_halfedge(e)]
    }

    /// Returns a EdgesAroundSiteIterator iterator.
    fn edges_around_site(&self, e: usize) -> EdgesAroundSiteIterator {
        EdgesAroundSiteIterator::new(&self.triangulation, e)
    }
}