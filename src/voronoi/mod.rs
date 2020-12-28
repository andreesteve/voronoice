mod into_triangle_list;
mod utils;
mod edges_around_site_iterator;
mod voronoi_mesh_generator;

use delaunator::{EMPTY, Triangulation, next_halfedge, triangulate};
use self::edges_around_site_iterator::EdgesAroundSiteIterator;

pub use self::voronoi_mesh_generator::VoronoiMeshGenerator;
pub use delaunator::Point;
pub use self::utils::to_f32_vec;
pub use self::utils::generate_circle_sites;
pub use self::utils::generate_square_sites;
pub use self::utils::generate_triangle_sites;
pub use self::utils::generate_special_case_1;

/// The dual delauney-voronoi graph.
///
/// This relies on delaunator library processing and data structure. Refer to https://mapbox.github.io/delaunator/ for a complete understanding of the data structure used.
pub struct Voronoi {
    // pub x_range: (f64, f64),
    // pub y_range: (f64, f64),

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
    circumcenters: Vec<Point>,

    /// A map of each site to its left-most incomig half-edge.
    site_to_incoming: Vec<usize>,

    /// A map for each voronoi cell and the associated delauney triangles whose centroids are the cell's vertices.
    /// For any site[s], the associated voronoi cell associated triangles are represented by cell_triangles[s].
    cell_triangles: Vec<Vec<usize>>
}

impl std::fmt::Debug for Voronoi {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.debug_struct("Voronoi")
            .field("sites", &self.sites)
            .field("circumcenters", &self.circumcenters)
            .field("cell_triangles", &self.cell_triangles)
            .finish()
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

/// Returns the index to the site that half-edge `e` points to.
/// This is similar to `triangles`. Given an half-edge `e`, `triangles` returns the index of the site the half-edge start off. `site_of_incoming` returns the index of the site the half-edge points to.
fn site_of_incoming(triangulation: &Triangulation, e: usize) -> usize {
    triangulation.triangles[next_halfedge(e)]
}

/// Calculate the triangles associated with each voronoi cell
fn calculate_cell_triangles(sites: &Vec<Point>, circumcenters: &mut Vec<Point>, triangulation: &Triangulation, site_to_leftmost_halfedge: &Vec<usize>, num_of_sites: usize) -> Vec<Vec<usize>> {
    let mut seen_sites = vec![false; num_of_sites];
    let mut cells = vec![Vec::new(); num_of_sites];

    for edge in 0..triangulation.triangles.len() {
        // triangle[edge] is the site 'edge' originates from, but EdgesAroundPointIterator
        // iterate over edges around the site 'edge' POINTS TO, thus to get that site
        // we need to take the next half-edge
        let site = site_of_incoming(&triangulation, edge);

        //println!("Site {} {:?}, incoming edge {} (from site {} {:?}), first time? {}", site, sites[site], edge, triangulation.triangles[edge], sites[triangulation.triangles[edge]], !seen_sites[site]);

        // if we have already created the cell for this site, move on
        if !seen_sites[site] {
            seen_sites[site] = true;

            // edge may or may not be the left-most incoming edge for site
            // thus get the one
            let leftmost_edge = site_to_leftmost_halfedge[site];

            // EdgesAroundSiteIterator::new(&triangulation, leftmost_edge).for_each(|e| {
            //     println!("  Incoming edge {} from site {} {:?}", e, triangulation.triangles[e], sites[triangulation.triangles[e]])
            // });

            let cell = &mut cells[site];
            cell.extend(
                EdgesAroundSiteIterator::new(&triangulation, leftmost_edge)
                        .map(|e| utils::triangle_of_edge(e))
            );

            // if there is no half-edge associated with the left-most edge, the edge is on the hull
            // thus this cell will not "close" and it needs to be extended and clipped
            if triangulation.halfedges[leftmost_edge] == EMPTY {
                // get the point that the edge comes from
                let source_site = triangulation.triangles[leftmost_edge];
                let source_point = &sites[source_site];
                let target_point = &sites[site];

                // the line extension must be perpendicular to the hull edge
                // get edge direction, rotated by 90 degree clock-wise as to point towards the "outside"
                let mut orthogonal = Point { x: 2.0 * (source_point.y - target_point.y), y: 2.0 * (target_point.x - source_point.x) };

                // get voronoi vertex that needs to be extended and extend it
                let cell_vertex = &circumcenters[cell[0]];
                orthogonal.x += cell_vertex.x;
                orthogonal.y += cell_vertex.y;

                // add extended vertex as a "fake" circumcenter
                let vertex_index = circumcenters.len();
                cell.push(vertex_index);
                circumcenters.push(orthogonal);
            }
        }
    }

    // we need to "close" the cell for the sites on the hull, as we have so far extended their edges
    // perform clock-wise walk on the sites on the hull
    let hull = &triangulation.hull;
    let mut hull_iter = hull.iter().rev().copied();
    let first_cell_index = hull_iter.next().unwrap();
    let mut prev_exteded_vertex = *cells[first_cell_index].last().unwrap();

    for site in hull_iter {
        let site = site;
        let cell = &mut cells[site];

        // keep track of current extension vertex
        let curr_exteded_vertex = *cell.last().unwrap();

        // close the cell by picking the previous site extension to close the polygon
        // each edge (and site) on the hull has an associated extension, which is the last value in the cell list
        cell.insert(cell.len() - 1, prev_exteded_vertex);

        prev_exteded_vertex = curr_exteded_vertex;
    }

    let first_cell = &mut cells[first_cell_index];
    first_cell.insert(first_cell.len() - 1, prev_exteded_vertex);

    if num_of_sites != cells.len() {
        println!("Different number of sites from cells. Sites: {}, cells: {}", num_of_sites, cells.len());
        seen_sites.iter().enumerate().filter(|(_, seen)| !**seen).for_each(|(s, _)| println!("Site {} was not seen", s));
        panic!("Sites missing");
    }

    cells
}

// When reading this code think about 'edge' as the starting edge for a triangle
// So you can say that the starting edge indexes the triangle
// For instances, diag.triangles.len() is the number of starting edges and triangles in the triangulation, you can think of diag.triangles[e] as 'e' as being both the index of the
// starting edge and the triangle it represents. When dealing with an arbitraty edge, it may not be a starting edge. You can get the starting edge by dividing the edge by 3 and flooring it.
impl Voronoi {
    pub fn new(sites: Vec<Point>) -> Self {
        let triangulation = triangulate(&sites).expect("Expected tris");
        let num_of_triangles = triangulation.triangles.len() / 3;
        let num_of_sites = sites.len();

        // calculate circuncenter of each triangle
        let mut circumcenters = (0..num_of_triangles)
            .map(|t| cicumcenter(
                &sites[triangulation.triangles[3* t]],
                &sites[triangulation.triangles[3* t + 1]],
                &sites[triangulation.triangles[3* t + 2]]))
            .collect();

        // create map between site and its left-most incoming half-edge
        // this is especially important for the sites along the convex hull boundary when iterating over its neighoring sites
        let mut site_to_halfedge = vec![EMPTY; num_of_sites];
        for e in 0..triangulation.triangles.len() {
            let s = site_of_incoming(&triangulation, e);
            if site_to_halfedge[s] == EMPTY || triangulation.halfedges[e] == EMPTY {
                site_to_halfedge[s] = e;
            }
        }

        let cell_triangles = calculate_cell_triangles(&sites, &mut circumcenters, &triangulation, &site_to_halfedge, num_of_sites);

        Voronoi {
            circumcenters,
            site_to_incoming: site_to_halfedge,
            triangulation,
            num_of_triangles,
            cell_triangles,
            sites
        }
    }

    #[allow(dead_code)]
    pub fn new_with_random_sites(size: usize, x_range: (f64, f64), y_range: (f64, f64)) -> Voronoi {
        use rand::Rng;
        let mut rng = rand::thread_rng();

        let x_range = rand::distributions::Uniform::new(x_range.0, x_range.1);
        let y_range = rand::distributions::Uniform::new(y_range.0, y_range.1);
        let sites = (0..size)
            .map(|_| Point { x: rng.sample(x_range), y: rng.sample(y_range) })
            .collect();

        Voronoi::new(sites)
    }

    /// Returns an iterator that walks through each vertex of a voronoi cell in counter-clockwise manner.
    pub fn get_cell_verteces(&self, cell: usize) -> impl Iterator<Item = &Point> {
        self.cell_triangles[cell].iter().map(move |t| {
            &self.circumcenters[*t]
        })
    }

    #[allow(dead_code)]
    pub fn lloyd_relaxation(&self) -> Voronoi {
        fn calculate_approximated_cetroid<'a>(points: impl Iterator<Item = &'a Point>) -> Point {
            let mut r = Point { x: 0.0 , y: 0.0 };
            let mut n = 0;
            for p in points {
                r.x += p.x;
                r.y += p.y;
                n += 1;
            }

            let n = n as f64;
            r.x /= n;
            r.y /= n;

            r
        }

        // get verteces for each cell and calculate centroid
        // use the approximated cell centroid as new sites
        let new_sites = (0..self.sites.len())
            .map(|c| calculate_approximated_cetroid(self.get_cell_verteces(c)))
            .collect();

        Voronoi::new(new_sites)
    }

    // /// Returns the index to the site that half-edge `e` points to.
    // /// This is similar to `triangles`. Given an half-edge `e`, `triangles` returns the index of the site the half-edge start off. `site_of_incoming` returns the index of the site the half-edge points to.
    // // fn site_of_incoming(&self, e: usize) -> usize {
    // //     site_of_incoming(&self.triangulation, e)
    // // }

    // // /// Returns a EdgesAroundSiteIterator iterator.
    // // fn edges_around_site(&self, site: usize) -> EdgesAroundSiteIterator {
    // //     EdgesAroundSiteIterator::for_site(&self, site)
    // // }
}