mod into_triangle_list;
mod utils;
mod edges_around_site_iterator;
mod voronoi_mesh_generator;
mod cell;
mod voronoi_builder;
//mod into_line_list;

use delaunator::{EMPTY, Triangulation, next_halfedge, triangulate};
use self::edges_around_site_iterator::EdgesAroundSiteIterator;
use self::cell::VoronoiCell;

pub use self::voronoi_builder::*;
pub use self::voronoi_mesh_generator::VoronoiMeshGenerator;
pub use delaunator::Point;
pub use self::utils::to_f32_vec;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HullBehavior {
    /// No processing is done for the sites on the hull.
    /// This means cells on the hull do not have its edges extended to the bounding box, nor closed.
    None,

    /// Cells on the hull are only extended to the bounding box, but no additional verteces are added to make it a valid polygon.
    Extended,

    /// Cells on the hull are extended and closed such that they form a valid polygon.
    Closed
}

impl Default for HullBehavior {
    fn default() -> Self {
        HullBehavior::Closed
    }
}

/// The dual delauney-voronoi graph.
///
/// This relies on delaunator library processing and data structure. Refer to https://mapbox.github.io/delaunator/ for a complete understanding of the data structure used.
pub struct Voronoi {
    // pub x_range: (f64, f64),
    // pub y_range: (f64, f64),

    /// These are the sites of each voronoi cell.
    pub sites: Vec<Point>,

    bounding_box: Point,
    num_of_triangles: usize,
    triangulation: Triangulation,
    hull_behavior: HullBehavior,

    /// The circumcenters of each triangle (indexed by triangle / triangle's starting half-edge).
    ///
    /// For a given voronoi cell, its verteces are the circumcenters of its associated triangles.
    /// Values whose indexes are greater than sites.len() - 1 are not actual triangle circumcenters but voronoi cell vertices added to close sites on the convex hull.
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
            .field("hull_behavior", &self.hull_behavior)
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

fn clip_vector_to_bounding_box(point: &Point, bounding_box: &Point) -> Point {
    let box_ratio = bounding_box.y / bounding_box.y;
    let tn = point.y / point.x;
    let mut p = Point { x: 0.0, y: 0.0 };

    if tn > box_ratio || tn < -box_ratio {
        // will hit box_y or -box_y
        let box_y = bounding_box.y * point.y.signum();
        p.x = box_y * tn.recip();
        p.y = box_y;
    } else {
        // will hit box_x or -box_x
        let box_x = bounding_box.x * point.x.signum();
        p.x = box_x;
        p.y = box_x * tn;
    }

    debug_assert!(is_in_box(&p, bounding_box), "Point {:?} clipped ({:?}) outside bounding box {:?}", point, p, bounding_box);
    debug_assert!(p.x.abs() == bounding_box.x || p.y.abs() == bounding_box.y.abs(), "Point {:?} clipped ({:?}) outside bounding box {:?}", point, p, bounding_box);

    p
}

fn project_point_on_bounding_box(point: &Point, direction: &Point, bounding_box: &Point) -> Point {
    let box_x = bounding_box.x * direction.x.signum();
    let box_y = bounding_box.y * direction.y.signum();

    let mut p = Point { x: 0.0, y: 0.0 };
    p.x = point.x + direction.x * (1.0 + (box_y - (point.y + direction.y)) / direction.y);
    p.y = point.y + direction.y * (1.0 + (box_x - (point.x + direction.x)) / direction.x);

    if p.x.abs() > bounding_box.x {
        p.x = box_x;
    }

    if p.y.abs() > bounding_box.y {
        p.y = box_y;
    }


    // if tn > box_ratio || tn < -box_ratio {
    //     // will hit box_y or -box_y
    //     let box_y = bounding_box.y * direction.y.signum();
    //     p.x = point.x + direction.x * (1.0 + (box_y - (point.y + direction.y)) / direction.y);
    //     p.y = box_y;
    // } else {
    //     // will hit box_x or - box_x
    //     let box_x = bounding_box.x * direction.x.signum();
    //     p.x = box_x;
    //     p.y = point.y + direction.y * (1.0 + (box_x - (point.x + direction.x)) / direction.x);
    // }

    debug_assert!(is_in_box(&p, bounding_box), "Point {:?} with direction {:?} projected ({:?}) outside bounding box {:?}", point, direction, p, bounding_box);
    debug_assert!(p.x.abs() == bounding_box.x || p.y.abs() == bounding_box.y.abs(), "Point {:?} with direction {:?} projected ({:?}) outside bounding box {:?}", point, direction, p, bounding_box);

    p
}

fn is_in_box(point: &Point, bounding_box: &Point) -> bool {
    point.x.abs() <= bounding_box.x && point.y.abs() <= bounding_box.y
}

fn close_hull(cells: &mut Vec<Vec<usize>>, circumcenters: &mut Vec<Point>, triangulation: &Triangulation, bounding_box: &Point) {
    // perform clock-wise walk on the sites on the hull
    let hull = &triangulation.hull;
    let mut hull_iter = hull.iter().rev().copied();
    let first_cell_index = hull_iter.next().unwrap();
    let mut prev_exteded_vertex = *cells[first_cell_index].last().unwrap();
    let first_vertex = prev_exteded_vertex;

    for site in hull_iter {
        let site = site;
        let cell = &mut cells[site];

        // keep track of current extension vertex
        let curr_exteded_vertex = *cell.last().unwrap();

        close_cell(cell, circumcenters, prev_exteded_vertex, curr_exteded_vertex, bounding_box);

        // let mine = &circumcenters[curr_exteded_vertex];
        // let prev = &circumcenters[prev_exteded_vertex];

        // // close the cell by picking the previous site extension to close the polygon
        // // each edge (and site) on the hull has an associated extension, which is the last value in the cell list
        // cell.insert(cell.len() - 1, prev_exteded_vertex);

        // if mine.x.abs() == bounding_box.x && prev.y.abs() == bounding_box.y {
        //     let p = Point { x: mine.x, y: prev.y };
        //     let i = circumcenters.len();
        //     circumcenters.push(p);
        //     cell.insert(cell.len() - 1, i);
        // } else if mine.y.abs() == bounding_box.y && prev.x.abs() == bounding_box.x {
        //     let p = Point { x: prev.x, y: mine.y };
        //     let i = circumcenters.len();
        //     circumcenters.push(p);
        //     cell.insert(cell.len() - 1, i);
        // }

        prev_exteded_vertex = curr_exteded_vertex;
    }

    let first_cell = &mut cells[first_cell_index];
    close_cell(first_cell, circumcenters, prev_exteded_vertex, first_vertex, bounding_box);
}

/// Calculate the triangles associated with each voronoi cell
fn calculate_cell_triangles(hull_behavior: HullBehavior, sites: &Vec<Point>, circumcenters: &mut Vec<Point>, triangulation: &Triangulation, site_to_leftmost_halfedge: &Vec<usize>, num_of_sites: usize, bounding_box: &Point) -> Vec<Vec<usize>> {
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
            if triangulation.halfedges[leftmost_edge] == EMPTY && (hull_behavior == HullBehavior::Extended || hull_behavior == HullBehavior::Closed)  {
                // get the point that the edge comes from
                let source_site = triangulation.triangles[leftmost_edge];
                let source_point = &sites[source_site];
                let target_point = &sites[site];

                // the line extension must be perpendicular to the hull edge
                // get edge direction, rotated by 90 degree clock-wise as to point towards the "outside"
                let orthogonal = Point { x: source_point.y - target_point.y, y: target_point.x - source_point.x };

                // get voronoi vertex that needs to be extended and extend it
                let cell_vertex = &circumcenters[cell[0]];
                let projected = project_point_on_bounding_box(cell_vertex, &orthogonal, bounding_box);

                // add extended vertex as a "fake" circumcenter
                let vertex_index = circumcenters.len();
                cell.push(vertex_index);
                circumcenters.push(projected);
            }
        }
    }

    // we need to "close" the cell for the sites on the hull, as we have so far extended their edges
    if hull_behavior == HullBehavior::Closed {
        close_hull(&mut cells, circumcenters, triangulation, bounding_box);
    }

    // first_cell.insert(first_cell.len() - 1, prev_exteded_vertex);

    // if mine.x.abs() == bounding_box.x && prev.y.abs() == bounding_box.y {
    //     let p = Point { x: mine.x, y: prev.y };
    //     let i = circumcenters.len();
    //     circumcenters.push(p);
    //     first_cell.insert(first_cell.len() - 1, i);
    // } else if mine.y.abs() == bounding_box.y && prev.x.abs() == bounding_box.x {
    //     let p = Point { x: prev.x, y: mine.y };
    //     let i = circumcenters.len();
    //     circumcenters.push(p);
    //     first_cell.insert(first_cell.len() - 1, i);
    // }

    if num_of_sites != cells.len() {
        println!("Different number of sites from cells. Sites: {}, cells: {}", num_of_sites, cells.len());
        seen_sites.iter().enumerate().filter(|(_, seen)| !**seen).for_each(|(s, _)| println!("Site {} was not seen", s));
        panic!("Sites missing");
    }

    cells
}

fn close_cell(cell: &mut Vec<usize>, circumcenters: &mut Vec<Point>, prev_vertex: usize, curr_vertex: usize, bounding_box: &Point) {
    let mine = circumcenters[curr_vertex].clone();
    let prev = circumcenters[prev_vertex].clone();

    // FIXME: use same points for corners
    // let top_right = Point { x: bounding_box.x, y: bounding_box.y };
    // let top_left = Point { x: -bounding_box.x, y: bounding_box.y };
    // let bottom_right = Point { x: bounding_box.x, y: -bounding_box.y };
    // let bottom_left = Point { x: -bounding_box.x, y: -bounding_box.y };

    // close the cell by picking the previous site extension to close the polygon
    // each edge (and site) on the hull has an associated extension, which is the last value in the cell list
    cell.insert(cell.len() - 1, prev_vertex);

    if mine.x.abs() == bounding_box.x && prev.x.abs() == bounding_box.x && mine.x != prev.x {
        println!("Case 2: mine {:?} prev {:?}", mine, prev);
        // cur and prev on oppositve X bounding lines
        let dir = if prev.y.signum() != mine.y.signum() && prev.y.abs() > mine.y.abs() {
            -mine.y.signum()
        } else {
            mine.y.signum()
        };

        let p = Point { x: prev.x, y: dir * bounding_box.y };
        println!("  Added {:?}", p);
        let i = circumcenters.len();
        circumcenters.push(p);
        cell.insert(cell.len() - 1, i);

        let p = Point { x: mine.x, y: dir * bounding_box.y };
        println!("  Added {:?}", p);
        let i = circumcenters.len();
        circumcenters.push(p);
        cell.insert(cell.len() - 1, i);
    } else if mine.y.abs() == bounding_box.y && prev.y.abs() == bounding_box.y && mine.y != prev.y {
        println!("Case 3: mine {:?} prev {:?}", mine, prev);
        // cur and prev on oppositve Y bounding lines
        let dir = if prev.x.signum() != mine.x.signum() && prev.x.abs() > mine.x.abs() {
            -mine.x.signum()
        } else {
            mine.x.signum()
        };

        let p = Point { x: dir * bounding_box.x, y: prev.y };
        println!("  Added {:?}", p);
        let i = circumcenters.len();
        circumcenters.push(p);
        cell.insert(cell.len() - 1, i);

        let p = Point { x: dir * bounding_box.x, y: mine.y };
        println!("  Added {:?}", p);
        let i = circumcenters.len();
        circumcenters.push(p);
        cell.insert(cell.len() - 1, i);
    } else if mine.x.abs() == bounding_box.x && prev.y.abs() == bounding_box.y {
        println!("Case 0: mine {:?} prev {:?}", mine, prev);
        let p = Point { x: mine.x, y: prev.y };
        let i = circumcenters.len();
        circumcenters.push(p);
        cell.insert(cell.len() - 1, i);
    } else if mine.y.abs() == bounding_box.y && prev.x.abs() == bounding_box.x {
        println!("Case 1: mine {:?} prev {:?}", mine, prev);
        let p = Point { x: prev.x, y: mine.y };
        let i = circumcenters.len();
        circumcenters.push(p);
        cell.insert(cell.len() - 1, i);
    }
}

// When reading this code think about 'edge' as the starting edge for a triangle
// So you can say that the starting edge indexes the triangle
// For instances, diag.triangles.len() is the number of starting edges and triangles in the triangulation, you can think of diag.triangles[e] as 'e' as being both the index of the
// starting edge and the triangle it represents. When dealing with an arbitraty edge, it may not be a starting edge. You can get the starting edge by dividing the edge by 3 and flooring it.
impl Voronoi {
    pub fn new(sites: Vec<Point>, hull_behavior: HullBehavior, bounding_box: Point) -> Self {
        // remove any points not within bounding box
        let sites = sites.into_iter().filter(|p| is_in_box(p, &bounding_box)).collect::<Vec<Point>>();

        let triangulation = triangulate(&sites).expect("Expected tris");
        let num_of_triangles = triangulation.triangles.len() / 3;
        let num_of_sites = sites.len();

        // calculate circuncenter of each triangle
        let mut circumcenters = (0..num_of_triangles)
            .map(|t| {
                let mut c= cicumcenter(
                    &sites[triangulation.triangles[3* t]],
                    &sites[triangulation.triangles[3* t + 1]],
                    &sites[triangulation.triangles[3* t + 2]]);

                    // FIXME: this is a good approximation when sites are close to origin
                    // need to instead project circumcenters, project the cell vectors instead
                    if !is_in_box(&c, &bounding_box) {
                        c = clip_vector_to_bounding_box(&c, &bounding_box);
                    }

                    c
            })
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

        let cell_triangles = calculate_cell_triangles(hull_behavior, &sites, &mut circumcenters, &triangulation, &site_to_halfedge, num_of_sites, &bounding_box);

        Voronoi {
            bounding_box,
            circumcenters,
            site_to_incoming: site_to_halfedge,
            triangulation,
            num_of_triangles,
            cell_triangles,
            sites,
            hull_behavior
        }
    }

    /// Returns an iterator that walks through each vertex of a voronoi cell in counter-clockwise manner.
    pub fn get_cell_verteces(&self, cell: usize) -> impl Iterator<Item = &Point> {
        self.cell_triangles[cell].iter().map(move |t| {
            &self.circumcenters[*t]
        })
    }

    /// Gets a representation of a voronoi cell based on its site index.
    pub fn get_cell(&self, site: usize) -> VoronoiCell {
        VoronoiCell::new(site, self)
    }

    pub fn cells<'v>(&'v self) -> impl Iterator<Item = VoronoiCell<'v>> + Clone {
        (0..self.sites.len())
            .map(move |s| self.get_cell(s))
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