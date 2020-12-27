use delaunator::*;

use super::Voronoi;

/// Iterator that walks through all the edges connected to the starting point referenced by index 'start'.
/// 'start' must be an incoming half-edge to the site that needs to be iterated around, the iterated values will be incoming half-edges.
/// The iteration happens in a clock-wise manner.
///
/// # Examples
///
/// ```
/// // Get all the edges connected to the 5th point p
/// let start = 5;
/// let point = sites[start]; // this is the point coordinates
/// for edges in EdgesAroundPointIterator::new(start) {
///     println!("Edge {} is connected to {}", edge, point);
/// }
/// ```
#[derive(Clone)]
pub struct EdgesAroundSiteIterator<'t> {
    triangulation: &'t Triangulation,
    start: usize,
    next: usize
}
impl<'t> EdgesAroundSiteIterator<'t> {
    #[allow(dead_code)]
    pub fn for_site(voronoi: &'t Voronoi, site: usize) -> Self {
        EdgesAroundSiteIterator::new(&voronoi.triangulation, voronoi.site_to_incoming[site])
    }

    /// Creates iterator based on a incoming edge to a site.
    /// This must be the left-most incoming edge to the site to avoid early iteration stop around the convex hull.
    /// Use `for_site` to obtain such a guarantee.
    pub fn new(triangulation: &'t Triangulation, incoming_edge: usize) -> Self {
        // assert_eq!(0,
        //     incoming_edge % 2,
        //     "Half-edge {} is not incoming. Half-edge must be incoming to a site otherwise it risks early iteration stop at convex hull boundary",
        //     incoming_edge);

        Self {
            triangulation: triangulation,
            start: incoming_edge,
            next: incoming_edge
        }
    }
}
impl<'t> Iterator for EdgesAroundSiteIterator<'t> {
    type Item = usize;
    /// Walks all half-edges around the starting point and returning the associated surrounding sites
    fn next(&mut self) -> Option<Self::Item> {
        let incoming = self.next;

        if incoming != EMPTY {
            let outgoing = next_halfedge(incoming);

            // then take the oposite half-edge, it will be the incoming edge for the opposite triangle
            self.next = self.triangulation.halfedges[outgoing];

            // if we are back to the begining, there is nothing else to do
            if self.next == self.start {
                self.next = EMPTY
            }

            Some(incoming)
        } else {
            None
        }
    }
}