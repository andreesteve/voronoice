use delaunator::{Triangulation, next_halfedge};
use super::{EMPTY};

/// Iterator that walks through all the edges connected to a provided starting point.
/// The iteration happens in a counterclock-wise manner.
#[derive(Clone)]
pub struct EdgesAroundSiteIterator<'t> {
    triangulation: &'t Triangulation,
    start: usize,
    next: usize
}

impl<'t> EdgesAroundSiteIterator<'t> {
    /// Creates iterator based on a incoming edge to a site.
    /// This must be the left-most incoming edge to the site to avoid early iteration stop around the convex hull.
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