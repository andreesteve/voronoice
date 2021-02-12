use delaunator::{Point, Triangulation, next_halfedge};
use crate::{Voronoi, utils::dist2};

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
    voronoi: &'t Voronoi,
    last: usize,
    source_site: usize,
}

impl<'t> NeighborSiteIterator<'t> {
    /// Creates iterator based on the site.
    pub fn new(voronoi: &'t Voronoi, site: usize) -> Self {
        let incoming_leftmost_edge = voronoi.site_to_incoming_leftmost_halfedge[site];
        Self {
            iter: EdgesAroundSiteIterator::new(&voronoi.triangulation, incoming_leftmost_edge),
            voronoi,
            source_site: site,
            last: EMPTY,
        }
    }
}

impl<'t> Iterator for NeighborSiteIterator<'t> {
    type Item = usize;

    /// Walks all half-edges around the starting point and returning the associated surrounding sites
    fn next(&mut self) -> Option<Self::Item> {
        let mut site = None;
        while let Some(incoming) = self.iter.next() {
            self.last = incoming;

            // get site from where the incoming edge came from
            let neighbor_site = self.iter.triangulation.triangles[incoming];

            // FIXME: topological neighbors may not be visually connected after clipping
            // voronoi sites are topologically connected to other sites based if there is a delaunay edge between then
            // however clipping may remove that edge and the associated cells in the voronoi diagram may not share a common edge
            //if common_edge(self.voronoi.cells().get(neighbor_site).unwrap(), self.voronoi.cells().get(self.source_site).unwrap()) {
                site = Some(neighbor_site);
                break;
            //}
        }

        if site.is_some() {
            site
        } else if self.last != EMPTY {
            // check if there is a next site on the hull
            let outgoing = next_halfedge(self.last);
            if self.iter.triangulation.halfedges[outgoing] == EMPTY {
                // this means we are on the hull and reached the rightmost outgoing edge
                self.last = EMPTY;

                let neighbor_site = self.iter.triangulation.triangles[next_halfedge(outgoing)];
                //if common_edge(self.voronoi.cells().get(neighbor_site).unwrap(), self.voronoi.cells().get(self.source_site).unwrap()) {
                    Some(neighbor_site)
                //} else {
                    //None
                //}
            } else {
                // this means site is not on hull, and we have already iterated over all neighbors
                None
            }
        } else {
            None
        }
    }
}

/// Iterator that produces a path between two point.
#[derive(Clone)]
pub struct CellPathIterator<'t, 'p> {
    site: usize,
    dest: &'p Point,
    distance: f64,
    voronoi: &'t Voronoi
}

impl<'t, 'p> CellPathIterator<'t, 'p> {
    /// Creates iterator based on the site and destination point.
    pub fn new(voronoi: &'t Voronoi, site: usize, dest: &'p Point) -> Self {
        let mut s = Self {
            site,
            dest,
            voronoi,
            distance: 0.0
        };

        s.distance = s.dist2_for_site(site);

        s
    }
}

impl<'t, 'p> CellPathIterator<'t, 'p> {
    fn dist2_for_site(&self, site: usize) -> f64 {
        let point = &self.voronoi.sites[site];
        dist2(point, &self.dest)
    }
}

impl<'t, 'p> Iterator for CellPathIterator<'t, 'p> {
    type Item = usize;

    /// Walks current site neighbor and find the next site in the path
    fn next(&mut self) -> Option<Self::Item> {
        let current_size = self.site;

        // take the neighbor that is closest to dest
        let closest = NeighborSiteIterator::new(self.voronoi, self.site)
            .map(|n| (n, self.dist2_for_site(n)))
            .min_by(|(_, dist0), (_, dist1)| dist0.partial_cmp(dist1).unwrap());

        // if neighbor is closer to destination than we are, it is next in the path
        if let Some((n, dist)) = closest {
            if dist < self.distance {
                self.distance = dist;
                self.site = n;
            } else {
                // reached end
                self.site = EMPTY;
            }
        } else {
            // reached end
            self.site = EMPTY;
        }

        if current_size != EMPTY {
            Some(current_size)
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

    #[test]
    fn iter_neighbors_edge_clipped_by_box_test() {
        // points 0 and 1 are neighbors if the bounding box is a square of side 7
        // when bounding box is a square of side 2, the edge between 0 and 1 is removed and 2 becomes a cell in between 0 and 1

        // another problematic set is:
        /*
            [-0.5, -0.8],
            [0, -0.5],
            [0.2, -0.5],
            [0.3, -0.5],
        */

        // need to find a way to remove delauney neighbors whose voronoi edges were clipped out
        // comparing edge is one way, comparing circumcenters is another https://github.com/d3/d3-delaunay/pull/98/files
        let sites = vec![Point { x: -1.0, y: -1.0 }, Point { x: 0.0, y: -1.0 }, Point { x: -0.45, y: -0.95 }];
        let v = VoronoiBuilder::default()
            .set_sites(sites.clone())
            .build()
            .unwrap();
        let mut neighbors = NeighborSiteIterator::new(&v, 0);
        assert_eq!(Some(2), neighbors.next());
        assert_eq!(None, neighbors.next());
    }

    #[test]
    fn iter_cell_path_test() {
        let sites = vec![
            Point { x: -0.5, y: 0.0 },
            Point { x: 0.0, y: 0.0 }, Point { x: 0.0, y: 0.5 }, Point { x: 0.0, y: -0.5 },
            Point { x: 0.2, y: 0.0 }, Point { x: 0.2, y: 0.5 }, Point { x: 0.2, y: -0.5 },
            Point { x: 0.5, y: 0.0 },
        ];
        let v = VoronoiBuilder::default()
            .set_sites(sites.clone())
            .build()
            .unwrap();
        let mut path = CellPathIterator::new(&v, 0, sites.last().unwrap());
        assert_eq!(Some(0), path.next());
        assert_eq!(Some(1), path.next());
        assert_eq!(Some(4), path.next());
        assert_eq!(Some(7), path.next());
        assert_eq!(None, path.next());
    }

    #[test]
    fn iter_cell_path_test_2() {
        let sites = vec![
            Point { x: -0.9, y: -0.9 },
            Point { x: -0.5, y: -0.8 }, Point { x: -0.8, y: -0.6 },
            Point { x: -0.5, y: -0.5 }, Point { x: -0.5, y: 0.0 },
            Point { x: 0.0, y: 0.0 }, Point { x: 0.0, y: 0.5 }, Point { x: 0.0, y: -0.5 },
            Point { x: 0.2, y: 0.0 }, Point { x: 0.2, y: 0.5 }, Point { x: 0.2, y: -0.5 },
            Point { x: 0.3, y: 0.0 }, Point { x: 0.3, y: 0.5 }, Point { x: 0.3, y: -0.5 },
            Point { x: 0.5, y: 0.0 },
            Point { x: 0.5, y: 0.5 },
        ];
        let v = VoronoiBuilder::default()
            .set_sites(sites.clone())
            .build()
            .unwrap();
        let mut path = CellPathIterator::new(&v, 0, sites.last().unwrap());
        assert_eq!(Some(0), path.next());
        assert_eq!(Some(1), path.next());
        // this fails because the point 13 is a neighbor of 1; this is technically true if we expand the bounding box to a large value
        // 13 and 1 share a voronoi edge, but that edge is clipped by the bounding box
        assert_eq!(Some(3), path.next());
        assert_eq!(Some(5), path.next());
        assert_eq!(Some(8), path.next());
        assert_eq!(None, path.next());
    }
}