use std::fmt;
use super::Voronoi;
use super::Point;

pub struct VoronoiCell<'v> {
    site: usize,
    voronoi: &'v Voronoi
}

impl<'v> VoronoiCell<'v> {
    pub fn new(site: usize, voronoi: &'v Voronoi) -> Self {
        Self {
            site,
            voronoi
        }
    }

    pub fn get_site_position(&self) -> &Point {
        &self.voronoi.sites[self.site]
    }

    /// Get the indices of the triangles of the dual Delauney triangulation that are associated with this Voronoi cell.
    pub fn get_triangles(&self) -> impl Iterator<Item=usize> + 'v + Clone {
        self.voronoi.cell_triangles[self.site].iter().copied()
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
        struct CellVerteces {
            /// Each vertex is the circumcenter of a associated Delauney triangle
            triangles: Vec<usize>,
            positions: Vec<Point>
        }
        let leftmost_edge = self.voronoi.site_to_incoming_leftmost_halfedge[self.site];

        f.debug_struct("VoronoiCell")
            .field("site", &Site {
                site: self.site,
                position: self.get_site_position().clone(),
                is_on_hull: self.is_on_hull(),
                leftmost_incoming_edge: Edge {
                    edge: leftmost_edge,
                    incoming_site: self.site,
                    outgoing_site: self.voronoi.triangulation.triangles[leftmost_edge]
                }
            })
            .field("verteces", &CellVerteces {
                triangles: self.get_triangles().collect(),
                positions: self.get_triangles().map(|t| self.voronoi.circumcenters[t].clone()).collect()
            })
            .finish()
    }
}

// #[derive(Debug, Clone, Copy)]
// pub struct VoronoiCellIterator<'v> {
//     iter:Iterator
// }
// impl<'v> Iterator for VoronoiCellIterator<'v> {
// }
// impl<'v> Iterator for VoronoiCellIterator<'v> {
//     type Item = VoronoiCell<'v>;

//     fn next(&mut self) -> Option<Self::Item> {

//     }
// }