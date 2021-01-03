mod into_triangle_list;
mod utils;
mod edges_around_site_iterator;
mod voronoi_mesh_generator;
mod cell;
mod voronoi_builder;
mod bounding_box;
mod cell_clipping;

use delaunator::{EMPTY, Triangulation, triangulate};
use self::{utils::{cicumcenter, site_of_incoming}};
use self::cell::VoronoiCell;
use self::cell_clipping::*;

pub use self::bounding_box::*;
pub use self::voronoi_builder::*;
pub use self::voronoi_mesh_generator::VoronoiMeshGenerator;
pub use delaunator::Point;
pub use self::utils::to_f32_vec;

/// Defines how voronoi generation will handle sites on the convex hull.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HullBehavior {
    /// No processing is done for the sites on the hull.
    /// This means cells on the hull do not have its edges extended to the bounding box, nor closed.
    None,

    /// Cells on the hull are extended to the bounding box and closed by adding additional vertices.
    ExtendAndClose
}

impl Default for HullBehavior {
    fn default() -> Self {
        HullBehavior::ExtendAndClose
    }
}

/// Defines how voronoi generation will handle clipping of voronoi edges within the bounding box.
///
/// Clipping is necessary to guarantee that all voronoi vertices are within the bounding box boundaries. Voronoi cell vertices may fall outside the boundary if sites end up lining up almost co-lineraly.
/// Such co-linear sites may end up forming degenerated triangles ([very] obtuse triangles) in the underlying Delauney triangulation, which will cause the voronoi cell edges (which are the circumcenter of such triangles)
/// to be far away from the actual site points. If you avoid such site composition before hand (i.e. a couple iterations of lloyd relaxation), you may opt for `None` or `RemoveSitesOutsideBoundingBoxOnly`.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ClipBehavior {
    /// No clipping will be performed. Any sites outside the bounding box will still be used for diagram generation.
    None,

    /// Removes any sites outside the bounding box, but does not perform any further clipping of voronoi cells that may end up generated outside of the bounding box.
    RemoveSitesOutsideBoundingBoxOnly,

    /// Removes sites outside bounding box and clips any voronoi edges that fall outside the edges of the bounding box.
    Clip,
}

impl Default for ClipBehavior {
    fn default() -> Self {
        ClipBehavior::Clip
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

    bounding_box: BoundingBox,
    triangulation: Triangulation,
    hull_behavior: HullBehavior,
    clip_behavior: ClipBehavior,

    /// The circumcenters of each triangle (indexed by triangle / triangle's starting half-edge).
    ///
    /// For a given voronoi cell, its vertices are the circumcenters of its associated triangles.
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
    site_to_incoming_leftmost_halfedge: Vec<usize>,

    /// A map for each voronoi cell and the associated delauney triangles whose centroids are the cell's vertices.
    /// For any site[s], the associated voronoi cell associated triangles are represented by cell_triangles[s].
    cells: Vec<Vec<usize>>
}

impl std::fmt::Debug for Voronoi {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.debug_struct("Voronoi")
            .field("hull_behavior", &self.hull_behavior)
            .field("sites", &self.sites)
            .field("circumcenters", &self.circumcenters)
            .field("cell_triangles", &self.cells)
            .finish()
    }
}

// fn close_hull(cells: &mut Vec<Vec<usize>>, circumcenters: &mut Vec<Point>, triangulation: &Triangulation, bounding_box: &BoundingBox) {
//     // perform clock-wise walk on the sites on the hull
//     let hull = &triangulation.hull;
//     let mut hull_iter = hull.iter().rev().copied();
//     let first_cell_index = hull_iter.next().unwrap();
//     let mut prev_exteded_vertex = *cells[first_cell_index].last().unwrap();
//     let first_vertex = prev_exteded_vertex;

//     for site in hull_iter {
//         let site = site;
//         let cell = &mut cells[site];

//         // keep track of current extension vertex
//         let curr_exteded_vertex = *cell.last().unwrap();

//         //close_cell(cell, circumcenters, prev_exteded_vertex, curr_exteded_vertex, bounding_box);

//         // let mine = &circumcenters[curr_exteded_vertex];
//         // let prev = &circumcenters[prev_exteded_vertex];

//         // // close the cell by picking the previous site extension to close the polygon
//         // // each edge (and site) on the hull has an associated extension, which is the last value in the cell list
//         // cell.insert(cell.len() - 1, prev_exteded_vertex);

//         // if mine.x.abs() == bounding_box.x && prev.y.abs() == bounding_box.y {
//         //     let p = Point { x: mine.x, y: prev.y };
//         //     let i = circumcenters.len();
//         //     circumcenters.push(p);
//         //     cell.insert(cell.len() - 1, i);
//         // } else if mine.y.abs() == bounding_box.y && prev.x.abs() == bounding_box.x {
//         //     let p = Point { x: prev.x, y: mine.y };
//         //     let i = circumcenters.len();
//         //     circumcenters.push(p);
//         //     cell.insert(cell.len() - 1, i);
//         // }

//         prev_exteded_vertex = curr_exteded_vertex;
//     }

//     let first_cell = &mut cells[first_cell_index];
//     //close_cell(first_cell, circumcenters, prev_exteded_vertex, first_vertex, bounding_box);
// }

// /// Calculate the triangles associated with each voronoi cell
// fn calculate_cell_triangles(hull_behavior: HullBehavior, sites: &Vec<Point>, circumcenters: &mut Vec<Point>, triangulation: &Triangulation, site_to_leftmost_halfedge: &Vec<usize>, num_of_sites: usize, bounding_box: &BoundingBox) -> Vec<Vec<usize>> {
//     let mut seen_sites = vec![false; num_of_sites];
//     let mut cells = vec![Vec::new(); num_of_sites];

//     for edge in 0..triangulation.triangles.len() {
//         // triangle[edge] is the site 'edge' originates from, but EdgesAroundPointIterator
//         // iterate over edges around the site 'edge' POINTS TO, thus to get that site
//         // we need to take the next half-edge
//         let site = site_of_incoming(&triangulation, edge);

//         //println!("Site {} {:?}, incoming edge {} (from site {} {:?}), first time? {}", site, sites[site], edge, triangulation.triangles[edge], sites[triangulation.triangles[edge]], !seen_sites[site]);

//         // if we have already created the cell for this site, move on
//         if !seen_sites[site] {
//             seen_sites[site] = true;

//             // edge may or may not be the left-most incoming edge for site
//             // thus get the one
//             let leftmost_edge = site_to_leftmost_halfedge[site];

//             // if there is no half-edge associated with the left-most edge, the edge is on the hull
//             let is_hull_site = triangulation.halfedges[leftmost_edge] == EMPTY;

//             let cell = &mut cells[site];
//             cell.extend(
//                 EdgesAroundSiteIterator::new(&triangulation, leftmost_edge)
//                         .map(|e| utils::triangle_of_edge(e))
//             );

//             // // // // FIXME: there are two extensions per site, I am doing just one here
//             // // // // cells on the hull need to have its edges extended to the edges of the box
//             // // // if is_hull_site && !cell.is_empty() && (hull_behavior == HullBehavior::Extended || hull_behavior == HullBehavior::Closed)  {
//             // // //     // during clipping, cells are shifted to the left, with previously first entry becoming last
//             // // //     // this happens if the cell is closed before clipping, since hull cells are open before clipping they are not shiffted
//             // // //     let index_of_cell_to_extend = 0;
//             // // //     // this is the vertex we will extend from
//             // // //     let cell_vertex_to_extend = &circumcenters[cell[index_of_cell_to_extend]];

//             // // //     // FIX ME: if the circumcenter is outside the box, the extension needs to be projected onto the far side
//             // // //     // if vertex is outside bounding box or on the box's edge, no need to extend it
//             // // //     if bounding_box.is_exclusively_inside(cell_vertex_to_extend) {
//             // // //         // get the point that the edge comes from
//             // // //         let source_site = triangulation.triangles[leftmost_edge];
//             // // //         let source_point = &sites[source_site];
//             // // //         let target_point = &sites[site];

//             // // //         // the line extension must be perpendicular to the hull edge
//             // // //         // get edge direction, rotated by 90 degree counterclock-wise as to point towards the "outside" (x -> y, y -> -x)
//             // // //         let orthogonal = Point { x: source_point.y - target_point.y, y: target_point.x - source_point.x };

//             // // //         // get voronoi vertex that needs to be extended and extend it
//             // // //         let projected = bounding_box.project_ray_closest(cell_vertex_to_extend, &orthogonal).expect("Expected intersection with box");

//             // // //         // add extended vertex as a "fake" circumcenter
//             // // //         let vertex_index = circumcenters.len();
//             // // //         // this point is orthogonally extended towards the outside from the current cell[0], thus it needs to come in first
//             // // //         // be keep vertices in counterclockwise order
//             // // //         cell.insert(index_of_cell_to_extend, vertex_index);
//             // // //         circumcenters.push(projected);
//             // // //     }
//             // // // }

//             // clip cell edges
//             // let (new_cell, _) = clip_cell(cell, circumcenters, bounding_box, !is_hull_site);
//             // *cell = new_cell;
//         }
//     }

//     // we need to "close" the cell for the sites on the hull, as we have so far extended their edges
//     if hull_behavior == HullBehavior::Closed {
//         close_hull(&mut cells, circumcenters, triangulation, bounding_box);
//     }

//     if num_of_sites != cells.len() {
//         println!("Different number of sites from cells. Sites: {}, cells: {}", num_of_sites, cells.len());
//         seen_sites.iter().enumerate().filter(|(_, seen)| !**seen).for_each(|(s, _)| println!("Site {} was not seen", s));
//         panic!("Sites missing");
//     }

//     cells
// }

fn calculate_circumcenters(sites: &Vec<Point>, triangulation: &Triangulation) -> Vec<Point> {
    let num_of_triangles = triangulation.triangles.len() / 3;

    // calculate circuncenter of each triangle
    (0..num_of_triangles).map(|t| {
        let c = cicumcenter(
            &sites[triangulation.triangles[3* t]],
            &sites[triangulation.triangles[3* t + 1]],
            &sites[triangulation.triangles[3* t + 2]]);

            // FIXME: this is a good approximation when sites are close to origin
            // need to instead project circumcenters, project the cell vectors instead
            // if !is_in_box(&c, &bounding_box) {
            //     c = clip_vector_to_bounding_box(&c, &bounding_box);
            // }

            c
    })
    .collect()
}

// When reading this code think about 'edge' as the starting edge for a triangle
// So you can say that the starting edge indexes the triangle
// For instances, diag.triangles.len() is the number of starting edges and triangles in the triangulation, you can think of diag.triangles[e] as 'e' as being both the index of the
// starting edge and the triangle it represents. When dealing with an arbitraty edge, it may not be a starting edge. You can get the starting edge by dividing the edge by 3 and flooring it.
impl Voronoi {
    pub fn new(sites: Vec<Point>, bounding_box: BoundingBox, hull_behavior: HullBehavior, clip_behavior: ClipBehavior) -> Option<Self> {
        // remove any points not within bounding box
        let sites = sites.into_iter().filter(|p| bounding_box.is_inside(p)).collect::<Vec<Point>>();
        let triangulation = triangulate(&sites)?;

        // calculate circuncenter of each triangle, t hese will be the vertices of the voronoi cells
        let circumcenters = calculate_circumcenters(&sites, &triangulation);

        // create map between site and its left-most incoming half-edge
        // this is especially important for the sites along the convex hull boundary when iterating over its neighoring sites
        let num_of_sites = sites.len();
        let mut site_to_incoming_leftmost_halfedge = vec![EMPTY; num_of_sites];
        for e in 0..triangulation.triangles.len() {
            let s = site_of_incoming(&triangulation, e);
            if site_to_incoming_leftmost_halfedge[s] == EMPTY || triangulation.halfedges[e] == EMPTY {
                site_to_incoming_leftmost_halfedge[s] = e;
            }
        }

        // create cell builder to build cells and update circumcenters
        let cell_builder = CellBuilder::new(circumcenters, bounding_box.clone(), hull_behavior, clip_behavior);
        let result = cell_builder.build(&sites, &triangulation, &site_to_incoming_leftmost_halfedge);

        Some(Voronoi {
            bounding_box,
            site_to_incoming_leftmost_halfedge,
            triangulation,
            sites,
            hull_behavior,
            clip_behavior,
            circumcenters: result.vertices,
            cells: result.cells
        })
    }

    /// Returns an iterator that walks through each vertex of a voronoi cell in counter-clockwise manner.
    pub fn get_cell_vertices(&self, cell: usize) -> impl Iterator<Item = &Point> {
        self.cells[cell].iter().map(move |t| {
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

    /// Gets the number of Delauney triangles associated with this Voronoi diagram.
    pub fn number_of_triangles(&self) -> usize {
        self.triangulation.triangles.len() / 3
    }

    pub fn hull_behavior(&self) -> HullBehavior {
        self.hull_behavior
    }

    pub fn clip_behavior(&self) -> ClipBehavior {
        self.clip_behavior
    }
}