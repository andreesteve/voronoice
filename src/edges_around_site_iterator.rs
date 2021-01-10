use delaunator::{Triangulation, next_halfedge};
use crate::Voronoi;

use super::{EMPTY};

/// Iterator that walks through all the edges connected to a provided starting point.
/// The iteration happens in a counterclock-wise manner.
///
/// Note: this really only returns edges for triangles around the site. On the convex hull, the rightmost edge will not be returned because there is no incoming rightmost edge.
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
        Self {
            triangulation: triangulation,
            start: incoming_edge,
            next: incoming_edge
        }
    }
}

impl<'t> Iterator for EdgesAroundSiteIterator<'t> {
    type Item = usize;
    /// Walks all half-edges around the starting point and returning the associated surrounding incoming edges
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

/// Iterates over sites that neighbor another site.
#[derive(Clone)]
pub struct NeighborSiteIterator<'t> {
    iter: EdgesAroundSiteIterator<'t>,
    last: usize
}

impl<'t> NeighborSiteIterator<'t> {
    /// Creates iterator based on the site.
    pub fn new(voronoi: &'t Voronoi, site: usize) -> Self {
        let incoming_leftmost_edge = voronoi.site_to_incoming_leftmost_halfedge[site];
        Self {
            iter: EdgesAroundSiteIterator::new(&voronoi.triangulation, incoming_leftmost_edge),
            last: EMPTY,
        }
    }
}

impl<'t> Iterator for NeighborSiteIterator<'t> {
    type Item = usize;

    /// Walks all half-edges around the starting point and returning the associated surrounding sites
    fn next(&mut self) -> Option<Self::Item> {
        if let Some(incoming) = self.iter.next() {
            self.last = incoming;

            // get site from where the incoming edge came from
            let site = self.iter.triangulation.triangles[incoming];
            Some(site)
        } else if self.last != EMPTY {
            // check if there is a next site on the hull
            let outgoing = next_halfedge(self.last);
            if self.iter.triangulation.halfedges[outgoing] == EMPTY {
                // this means we are on the hull and reached the rightmost outgoing edge
                self.last = EMPTY;
                Some(self.iter.triangulation.triangles[next_halfedge(outgoing)])
            } else {
                // this means site is not on hull, and we have already iterated over all neighbors
                None
            }
        } else {
            None
        }
    }
}

#[cfg(test)]
mod test {
    use delaunator::Point;
    use crate::VoronoiBuilder;
    use super::*;

    #[test]
    fn iter_neighbors_hull_test() {
        let sites = vec![Point { x: -0.5, y: 0.0 }, Point { x: 0.5, y: 0.0 }, Point { x: 0.0, y: 0.0 }, Point { x: 0.0, y: 0.5 }, Point { x: 0.0, y: -0.5 }];
        let v = VoronoiBuilder::default()
            .set_sites(sites.clone())
            .build()
            .unwrap();
        let neighbors: Vec<usize> = NeighborSiteIterator::new(&v, 0).collect();
        assert_eq!(neighbors.len(), 3, "There are 3 neighboring sites");
        assert_eq!(neighbors[0], 4);
        assert_eq!(neighbors[1], 2);
        assert_eq!(neighbors[2], 3);
    }

    #[test]
    fn iter_neighbors_inner_test() {
        let sites = vec![Point { x: -0.5, y: 0.0 }, Point { x: 0.5, y: 0.0 }, Point { x: 0.0, y: 0.0 }, Point { x: 0.0, y: 0.5 }, Point { x: 0.0, y: -0.5 }];
        let v = VoronoiBuilder::default()
            .set_sites(sites.clone())
            .build()
            .unwrap();
        let neighbors: Vec<usize> = NeighborSiteIterator::new(&v, 2).collect();
        assert_eq!(neighbors.len(), 4, "There are 4 neighboring sites");
        assert_eq!(neighbors[0], 3);
        assert_eq!(neighbors[1], 0);
        assert_eq!(neighbors[2], 4);
        assert_eq!(neighbors[3], 1);
    }
}