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
            .field("sites", &self.sites)
            .field("circumcenters", &self.circumcenters)
            .field("cell_triangles", &self.cells)
            .finish()
    }
}

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
    pub fn new(sites: Vec<Point>, bounding_box: BoundingBox, clip_behavior: ClipBehavior) -> Option<Self> {
        // remove any points not within bounding box
        let sites = match clip_behavior {
            ClipBehavior::RemoveSitesOutsideBoundingBoxOnly | ClipBehavior::Clip => sites.into_iter().filter(|p| bounding_box.is_inside(p)).collect::<Vec<Point>>(),
            ClipBehavior::None => sites
        };

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
        let cell_builder = CellBuilder::new(circumcenters, bounding_box.clone(), clip_behavior);
        let result = cell_builder.build(&sites, &triangulation, &site_to_incoming_leftmost_halfedge);

        Some(Voronoi {
            bounding_box,
            site_to_incoming_leftmost_halfedge,
            triangulation,
            sites,
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
}