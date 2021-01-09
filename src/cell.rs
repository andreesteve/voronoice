use std::fmt;
use super::Voronoi;
use super::Point;

/// Represents a Voronoi cell. This is an ergonomic way to access cell details.
///
/// Use [Voronoi::cell()] or [Voronoi::cell_iter()] to obtain an instance of this type.
#[derive(Clone)]
pub struct VoronoiCell<'v> {
    site: usize,
    voronoi: &'v Voronoi
}

impl<'v> VoronoiCell<'v> {
    pub (super) fn new(site: usize, voronoi: &'v Voronoi) -> Self {
        Self {
            site,
            voronoi
        }
    }

    /// Gets a reference to the position of the site associated with this cell.
    ///
    /// # Examples
    ///
    ///```
    /// use voronoice::*;
    /// let sites = vec![Point { x: 0.0, y: 0.0 }, Point { x: 0.5, y: 0.0 }, Point { x: 0.0, y: 0.5 }];
    /// let v = VoronoiBuilder::default()
    ///     .set_sites(sites.clone())
    ///     .build()
    ///     .unwrap();
    /// // the first site generated by generate_circle_sites is at the origin
    /// assert_eq!(&sites[0], v.cell(0).site_position());
    /// assert_eq!(&sites[1], v.cell(1).site_position());
    /// assert_eq!(&sites[2], v.cell(2).site_position());
    ///```
    pub fn site_position(&self) -> &Point {
        &self.voronoi.sites[self.site]
    }

    /// Gets an iterator the indices of the triangles of the dual Delauney triangulation that are associated with this cell.
    /// The Voronoi cell vertices are the circumcenters of the associated Delauney triangles. This is a way to index into the underlying Delauney triangles.
    ///
    /// If this cell is on the hull of the diagram (```cell.is_on_hull() == true```), or has had one of its edges clipped,
    #[inline]
    pub fn triangles(&self) -> impl Iterator<Item=usize> + 'v + Clone {
        self.voronoi.cells[self.site].iter().copied()
    }

    /// Gets an iterator for the vertices of
    #[inline]
    pub fn vertices(&self) -> impl Iterator<Item=&Point> {
        self.triangles().map(move |t| &self.voronoi.circumcenters[t])
    }

    /// Returns a boolean indicating whether this cell is on the hull (edge) of the diagram.
    pub fn is_on_hull(&self) -> bool {
        // TODO this will be slow for a large graph - checking edges will likely be faster
        self.voronoi.triangulation.hull.contains(&self.site)
    }
}

impl<'v> fmt::Debug for  VoronoiCell<'v> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        #[derive(Debug)]
        struct Edge {
            edge: usize,
            incoming_site: usize,
            outgoing_site: usize
        }

        #[derive(Debug)]
        struct Site {
            site: usize,
            position: Point,
            is_on_hull: bool,
            leftmost_incoming_edge: Edge
        }

        #[derive(Debug)]
        struct Cellvertices {
            /// Each vertex is the circumcenter of a associated Delauney triangle
            triangles: Vec<usize>,
            positions: Vec<Point>
        }
        let leftmost_edge = self.voronoi.site_to_incoming_leftmost_halfedge[self.site];

        f.debug_struct("VoronoiCell")
            .field("site", &Site {
                site: self.site,
                position: self.site_position().clone(),
                is_on_hull: self.is_on_hull(),
                leftmost_incoming_edge: Edge {
                    edge: leftmost_edge,
                    incoming_site: self.site,
                    outgoing_site: self.voronoi.triangulation.triangles[leftmost_edge]
                }
            })
            .field("vertices", &Cellvertices {
                triangles: self.triangles().collect(),
                positions: self.triangles().map(|t| self.voronoi.circumcenters[t].clone()).collect()
            })
            .finish()
    }
}