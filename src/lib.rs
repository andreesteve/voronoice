//! # voronoice
//!
//! This crate provides a nice and fast way to construct 2D [Voronoi diagrams](https://en.wikipedia.org/wiki/Voronoi_diagram) from a set of sites.
//! It builds Voronoi diagrams by first obtaining its [Delaunay triangulation](https://en.wikipedia.org/wiki/Delaunay_triangulation), through the really fast [delaunator](https://docs.rs/delaunator/*/delaunator) crate,
//! and then extracting its dual Voronoi diagram.
//!
//! Both the Voronoi graph and its dual Delaunay graph are exposed through the [Voronoi] type.
//!
//! # Example
//!
//!```rust
//! use voronoice::*;
//!
//! // voronoi sites
//! let sites = vec![
//!     Point { x: 0.0, y: 0.0 }, Point { x: 1.0, y: 0.0 }, Point { x: 0.0, y: 1.0 }
//! ];
//!
//! // builds a voronoi diagram from the set of sites above, bounded by a square of size 4
//! let my_voronoi = VoronoiBuilder::default()
//!     .set_sites(sites)
//!     .set_bounding_box(BoundingBox::new_centered_square(4.0))
//!     .set_lloyd_relaxation_iterations(5)
//!     .build()
//!     .unwrap();
//!
//! // inspect cells through iterators
//! my_voronoi.iter_cells().for_each(|cell| {
//!     println!("Vertices of cell: {:?}", cell.iter_vertices().collect::<Vec<&Point>>())
//! });
//!
//! // or probe cells individually
//! let my_cell = my_voronoi.cell(1);
//! println!("Second cell has site {:?}, voronoi vertices {:?} and delaunay triangles {:?}",
//!     my_cell.site_position(),
//!     my_cell.iter_vertices().collect::<Vec<&Point>>(),
//!     my_cell.iter_triangles().collect::<Vec<usize>>());
//!
//! // or, for graphical applications, that benefit from index buffers
//! // you can access the raw, indexed data
//! let all_voronoi_cell_vertices = my_voronoi.vertices();
//! let indexed_voronoi_cells = my_voronoi.cells();
//! println!("The first vertex position for the first voronoi cell is at {:?}",
//!     all_voronoi_cell_vertices[indexed_voronoi_cells[0][0]]);
//!```

mod bounding_box;
mod cell_builder;
mod voronoi_cell;
mod iterator;
mod utils;
mod voronoi_builder;

use delaunator::{EMPTY, Triangulation, triangulate};
use self::{
    utils::{cicumcenter, site_of_incoming},
    cell_builder::*
};

pub use voronoi_builder::VoronoiBuilder;
pub use bounding_box::BoundingBox;
pub use voronoi_cell::VoronoiCell;
pub use iterator::NeighborSiteIterator;
pub use iterator::CellPathIterator;
pub use delaunator::Point;

/// Defines how Voronoi generation will handle clipping of Voronoi cell edges within the bounding box.
///
/// Clipping is necessary to guarantee that all Voronoi vertices are within the bounding box boundary.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ClipBehavior {
    /// No clipping will be performed. Any sites outside the bounding box will still be used for diagram generation.
    None,

    /// Removes any sites outside the bounding box, but does not perform any further clipping of Voronoi cells that may end up outside of the bounding box.
    RemoveSitesOutsideBoundingBoxOnly,

    /// Removes sites outside bounding box and clips any Voronoi cell edges that fall outside of the bounding box.
    Clip,
}

impl Default for ClipBehavior {
    fn default() -> Self {
        ClipBehavior::Clip
    }
}

/// The dual Delaunay-Voronoi graph.
///
/// To obtain an instance of this type, use [VoronoiBuilder].
pub struct Voronoi {
    /// These are the sites of each voronoi cell.
    sites: Vec<Point>,

    bounding_box: BoundingBox,
    triangulation: Triangulation,
    clip_behavior: ClipBehavior,

    /// The circumcenter of each triangle (indexed by triangle / triangle's starting half-edge).
    ///
    /// For a given voronoi cell, its vertices are the circumcenters of its associated triangles.
    /// Values whose indexes are greater than sites.len() - 1 are not actual triangle circumcenters but
    /// voronoi cell vertices added to close sites on the convex hull or otherwise used for clipping edges that fell outside the bounding box region.
    circumcenters: Vec<Point>,

    /// A map of each site to its left-most incomig half-edge.
    site_to_incoming_leftmost_halfedge: Vec<usize>,

    /// A map for each voronoi cell and the associated delaunay triangles whose centroids are the cell's vertices.
    /// For any site ```i```, the associated voronoi cell associated triangles are represented by ```cell_triangles[i]```.
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

// When reading this code think about 'edge' as the starting edge for a triangle
// So you can say that the starting edge indexes the triangle
// For instances, diag.triangles.len() is the number of starting edges and triangles in the triangulation, you can think of diag.triangles[e] as 'e' as being both the index of the
// starting edge and the triangle it represents. When dealing with an arbitraty edge, it may not be a starting edge. You can get the starting edge by dividing the edge by 3 and flooring it.
impl Voronoi {
    fn new(sites: Vec<Point>, bounding_box: BoundingBox, clip_behavior: ClipBehavior) -> Option<Self> {
        // remove any points not within bounding box
        let sites = match clip_behavior {
            ClipBehavior::RemoveSitesOutsideBoundingBoxOnly | ClipBehavior::Clip => sites.into_iter().filter(|p| bounding_box.is_inside(p)).collect::<Vec<Point>>(),
            ClipBehavior::None => sites
        };

        let triangulation = triangulate(&sites)?;

        // triangulation.triangles is the indexing of each half-edge to source site
        // 3 * t, 3 * t + 1 and 3 * t + 2 are the vertices of a triangle in this vector
        let num_of_triangles = triangulation.triangles.len() / 3;
        let num_of_sites = sites.len();

        // calculate circuncenter of each triangle, these will be the vertices of the voronoi cells
        let circumcenters = (0..num_of_triangles).map(|t| cicumcenter(
            &sites[triangulation.triangles[3* t]],
            &sites[triangulation.triangles[3* t + 1]],
            &sites[triangulation.triangles[3* t + 2]])
        ).collect();

        // create map between site and its left-most incoming half-edge
        // this is especially important for the sites along the convex hull boundary when iterating over its neighoring sites
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

    /// Borrows an immutable reference to the sites associated with the Voronoi graph.
    /// This is a reference to the unaltered site-point collection provided for the construction of this Voronoi graph.
    #[inline]
    pub fn sites(&self) -> &Vec<Point> {
        &self.sites
    }

    /// Gets a representation of a Voronoi cell based on its site index.
    ///
    /// # Examples
    ///```
    /// use voronoice::*;
    /// let v = VoronoiBuilder::default()
    ///     .generate_square_sites(10)
    ///     .build()
    ///     .unwrap();
    /// println!("The following are the positions for the Voronoi cell 0: {:?}",
    ///     v.cell(0).iter_vertices().collect::<Vec<&Point>>());
    ///```
    #[inline]
    pub fn cell(&self, site: usize) -> VoronoiCell {
        VoronoiCell::new(site, self)
    }

    /// Gets an iterator to walk through all Voronoi cells.
    /// Cells are iterated in order with the vector returned by [Self::sites()].
    pub fn iter_cells<'v>(&'v self) -> impl Iterator<Item = VoronoiCell<'v>> + Clone {
        (0..self.sites.len())
            .map(move |s| self.cell(s))
    }

    /// Gets a vector of Voronoi cell vectors that index the cell vertex positions.
    ///
    /// Consider using [Self::iter_cells()] or [Self::cell()] instead for a more ergonomic way of accessing cell information.
    /// Use cases for accessing this directly would include graphical applications where you need to build an index buffer for point positions.
    ///
    /// # Examples
    ///```
    /// use voronoice::*;
    /// let v = VoronoiBuilder::default()
    ///     .generate_square_sites(10)
    ///     .build()
    ///     .unwrap();
    /// let first_cell = v.cells().first().unwrap();
    /// let vertices = v.vertices();
    /// println!("The following are the positions for the Voronoi cell 0: {:?}",
    ///     first_cell.iter().copied().map(|v| &vertices[v]).collect::<Vec<&Point>>());
    ///```
    #[inline]
    pub fn cells(&self) -> &Vec<Vec<usize>> {
        &self.cells
    }

    /// Gets the a vector of the Voronoi cell vertices. These vertices are indexed by [Self::cells()].
    /// Consider using [Self::iter_cells()] or [Self::cell()] instead for a more ergonomic way of accessing cell information.
    ///
    /// For a given Voronoi cell, its vertices are the circumcenters of its associated Delaunay triangles.
    /// Values whose indexes are greater than ```sites.len() - 1``` are not actual triangle circumcenters but
    /// Voronoi cell vertices added to "close" sites on the convex hull or otherwise used for clipping edges that fell outside the bounding box region.
    ///
    /// Please see [Self::cells()] documentation for examples.
    #[inline]
    pub fn vertices(&self) -> &Vec<Point> {
        &self.circumcenters
    }

    /// Gets a reference to a vector of indices to sites where each triple represents a triangle on the dual Delaunay triangulation associated with this Voronoi graph.
    /// All triangles are directed counter-clockwise.
    #[inline]
    pub fn delaunay_triangles(&self) -> &Vec<usize> {
        &self.triangulation.triangles
    }

    /// Gets a reference to the bounding box.
    pub fn bounding_box(&self) -> &BoundingBox {
        &self.bounding_box
    }

    /// Gets the number of Delaunay triangles.
    fn number_of_triangles(&self) -> usize {
        self.triangulation.triangles.len() / 3
    }
}

#[cfg(test)]
mod tests {
    use rand::Rng;

    use super::*;

    fn create_random_builder(size: usize) -> VoronoiBuilder {
        let mut rng = rand::thread_rng();
        let builder = VoronoiBuilder::default();
        let bbox = BoundingBox::default();

        let x_range = rand::distributions::Uniform::new(-bbox.width(), bbox.width());
        let y_range = rand::distributions::Uniform::new(-bbox.height(), bbox.height());
        let sites = (0..size)
            .map(|_| Point { x: rng.sample(x_range), y: rng.sample(y_range) })
            .collect();

        builder
            .set_bounding_box(bbox)
            .set_sites(sites)
    }

    #[test]
    fn random_site_generation_test() {
        create_random_builder(100_000)
            .build()
            .expect("Some voronoi expected.");
    }
}