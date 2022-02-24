use delaunator::{EMPTY, next_halfedge, Triangulation};
use crate::utils::{triangle_of_edge};

use super::{ClipBehavior, Point, bounding_box::{self, *}, iterator::EdgesAroundSiteIterator, utils::{self, site_of_incoming}};

#[derive(Debug)]
pub struct CellBuilder<'t> {
    triangulation: &'t Triangulation,
    sites: &'t Vec<Point>,
    vertices: Vec<Point>,
    site_to_incoming_leftmost_halfedge: Vec<usize>,
    bounding_box: BoundingBox,
    clip_behavior: ClipBehavior,
    top_left_corner_index: usize,
    bottom_left_corner_index: usize,
    top_right_corner_index: usize,
    bottom_right_corner_index: usize,
}

pub struct CellBuilderResult {
    pub cells: Vec<Vec<usize>>,
    pub vertices: Vec<Point>,
    pub site_to_incoming_leftmost_halfedge: Vec<usize>,
}

trait PushAndReturnIndex<T> {
    /// Pushes element into vector and returns its index.
    fn pushi(&mut self, e: T) -> usize;
}

impl<T> PushAndReturnIndex<T> for Vec<T> {
    fn pushi(&mut self, e: T) -> usize {
        let index = self.len();
        self.push(e);
        index
    }
}

impl<'t> CellBuilder<'t> {
    pub fn new(triangulation: &'t Triangulation, sites: &'t Vec<Point>, vertices: Vec<Point>, bounding_box: BoundingBox, clip_behavior: ClipBehavior) -> Self {
        let site_to_incoming_leftmost_halfedge = calculate_incoming_edges(triangulation, sites.len());

        Self {
            triangulation,
            sites,
            site_to_incoming_leftmost_halfedge,
            top_right_corner_index: 0,
            top_left_corner_index: 0,
            bottom_left_corner_index: 0,
            bottom_right_corner_index: 0,
            vertices,
            bounding_box,
            clip_behavior,
        }
    }

    pub fn build(mut self) -> CellBuilderResult {
        // adds the corners of the bounding box as potential vertices for the voronoi
        let cells = if self.clip_behavior == ClipBehavior::Clip {
            self.calculate_corners();
            self.build_cells()
        } else {
            self.build_cells_no_clip()
        };

        CellBuilderResult {
            vertices: self.vertices,
            site_to_incoming_leftmost_halfedge: self.site_to_incoming_leftmost_halfedge,
            cells
        }
    }

    fn build_cells_no_clip(&mut self) -> Vec<Vec<usize>> {
        let num_of_sites = self.sites.len();
        let mut cells: Vec<Vec<usize>> = vec![Vec::new(); num_of_sites];

        // fill in cells
        for edge in 0..self.triangulation.triangles.len() {
            let site = site_of_incoming(self.triangulation, edge);
            let cell = &mut cells[site];

            // if cell is empty, it hasn't been processed yet
            if cell.len() == 0 {
                let leftmost_incoming_edge = self.site_to_incoming_leftmost_halfedge[site];
                cell.extend(EdgesAroundSiteIterator::new(self.triangulation, leftmost_incoming_edge).map(utils::triangle_of_edge));
            }
        }

        cells
    }

    /// Given two vertices `a` and `b` that are on the bounding box edge(s), returns what vertices must be added between them to conform with the bounding box format.
    /// `a` and `b` are assumed to be ordered counter-clockwise.
    /// Returns how many new vertices were added.
    pub fn get_link_vertices_around_box_edge(&self, pa: &Point, pb: &Point) -> (usize, [usize;4]) {
        let (mut a_bt, mut a_lr) = self.bounding_box.which_edge(pa);
        let (b_bt, b_lr) = self.bounding_box.which_edge(pb);

        // It is expected that points ARE on the box edge
        assert!(!(a_bt == BoundingBoxTopBottomEdge::None && a_lr == BoundingBoxLeftRightEdge::None), "Point a [{:?}] was not located on any of the box's edge", pa);
        assert!(!(b_bt == BoundingBoxTopBottomEdge::None && b_lr == BoundingBoxLeftRightEdge::None), "Point b [{:?}] was not located on any of the box's edge", pb);

        // short-circuit for case where a and b are on the same edge, and b is ahead of a, such case there is nothing to do
        if (match (a_bt, b_bt) {
            (BoundingBoxTopBottomEdge::Bottom, BoundingBoxTopBottomEdge::Bottom) => pb.x > pa.x,
            (BoundingBoxTopBottomEdge::Top, BoundingBoxTopBottomEdge::Top) => pa.x > pb.x,
            (_, _) => false
        } || match (a_lr, b_lr) {
            (BoundingBoxLeftRightEdge::Right, BoundingBoxLeftRightEdge::Right) => pb.y > pa.y,
            (BoundingBoxLeftRightEdge::Left, BoundingBoxLeftRightEdge::Left) => pa.y > pb.y,
            (_, _) => false
        }) {
            return (0, [0, 0, 0, 0]);
        }

        // a full cycle in the box will yeild maximum
        const MAX_CORNERS: usize = 4;
        let mut new_vertices = [EMPTY; MAX_CORNERS];
        let mut new_vertice_count = 0;

        // we only have work to do if a and b are not on the same edge
        // walk on box edge, starting on 'a' until we reach 'b'
        // on each iteration, move to the next edge (counter-clockwise)
        // and return the corners until b is reached
        // note: a and b may start on the same edge, this means we need to do a full loop on the box's edge
        loop {
            match (a_bt, a_lr) {
                (BoundingBoxTopBottomEdge::Top, lr) => {
                    // move to left edge
                    a_bt = BoundingBoxTopBottomEdge::None;
                    a_lr = BoundingBoxLeftRightEdge::Left;

                    // add top left corner, if 'a' is not already such a point
                    if lr != BoundingBoxLeftRightEdge::Left {
                        new_vertices[new_vertice_count] = self.top_left_corner_index;
                        new_vertice_count += 1;
                    }
                },

                (tb, BoundingBoxLeftRightEdge::Left) => {
                    // move to bottom edge
                    a_bt = BoundingBoxTopBottomEdge::Bottom;
                    a_lr = BoundingBoxLeftRightEdge::None;

                    // add bottom left corner, if 'a' is not such a point
                    if tb != BoundingBoxTopBottomEdge::Bottom {
                        new_vertices[new_vertice_count] = self.bottom_left_corner_index;
                        new_vertice_count += 1;
                    }
                },

                (BoundingBoxTopBottomEdge::Bottom, lr) => {
                    // move to right edge
                    a_bt = BoundingBoxTopBottomEdge::None;
                    a_lr = BoundingBoxLeftRightEdge::Right;

                    // add bottom right corner, if 'a' is not such a point
                    if lr != BoundingBoxLeftRightEdge::Right {
                        new_vertices[new_vertice_count] = self.bottom_right_corner_index;
                        new_vertice_count += 1;
                    }
                },

                (tb, BoundingBoxLeftRightEdge::Right) => {
                    // move to top edge
                    a_bt = BoundingBoxTopBottomEdge::Top;
                    a_lr = BoundingBoxLeftRightEdge::None;

                    // add top right corner, if 'a' is not such a point
                    if tb != BoundingBoxTopBottomEdge::Top {
                        new_vertices[new_vertice_count] = self.top_right_corner_index;
                        new_vertice_count += 1;
                    }
                },

                (BoundingBoxTopBottomEdge::None, BoundingBoxLeftRightEdge::None) => panic!("It seems that either 'a' ({:?}) or 'b' ({:?}) are not on the box's edge. Cannot link vertices not on the edge.", pa, pb)
            }

            if (a_bt != BoundingBoxTopBottomEdge::None && a_bt == b_bt) || (a_lr != BoundingBoxLeftRightEdge::None && a_lr == b_lr) {
                break;
            } else if new_vertice_count == MAX_CORNERS {
                panic!("It seems that either 'a' ({:?}) or 'b' ({:?}) are not on the box's edge. Cannot link vertices not on the edge.", pa, pb);
            }
        }

        (new_vertice_count, new_vertices)
    }

    pub fn calculate_corners(&mut self) {
        // add all corners to the vertice list as they will be used for closing cells
        let top_right = self.bounding_box.top_right().clone();
        let top_left = Point { x: self.bounding_box.left(), y: self.bounding_box.top() };
        let bottom_left = self.bounding_box.bottom_left().clone();
        let bottom_right = Point { x: self.bounding_box.right(), y: self.bounding_box.bottom() };

        // counter-clockwise
        self.top_left_corner_index = self.vertices.pushi(top_left);
        self.bottom_left_corner_index = self.vertices.pushi(bottom_left);
        self.bottom_right_corner_index = self.vertices.pushi(bottom_right);
        self.top_right_corner_index = self.vertices.pushi(top_right);
    }

    /// Clip a voronoi edge on the bounding box's edge, if the edge crosses the bounding box.
    ///
    /// Returns an index to the new vertex where the clip has occured.
    fn clip_voronoi_edge(&mut self, inside: usize, outside: usize) -> Option<usize> {
        let inside_pos = &self.vertices[inside];
        let outside_pos = &self.vertices[outside];
        let clip = self.bounding_box.project_ray_closest(inside_pos, &Point { x: outside_pos.x - inside_pos.x, y: outside_pos.y - inside_pos.y });

        if let Some(clip) = clip {
            // TODO reuse point when same edge is provided by neighbor in the other direction
            self.vertices.push(clip);
            Some(self.vertices.len() - 1)
        } else {
            None
        }
    }

    fn extend_voronoi_vertex(&mut self, hull_edge: usize) -> Option<usize> {
        let circumcenter = triangle_of_edge(hull_edge);
        let circumcenter_pos = &self.vertices[circumcenter];

        let (a, b) = (self.triangulation.triangles[hull_edge], self.triangulation.triangles[next_halfedge(hull_edge)]);

        let a_pos = &self.sites[a];
        let b_pos = &self.sites[b];

        // the projection direction is orthogonal to the hull's edge (a -> b)
        // put it just beyong bounding box edge
        let mut orthogonal = Point { x: a_pos.y - b_pos.y , y: b_pos.x - a_pos.x };

        // normalizing the orthogonal vector
        let ortho_length = (orthogonal.x * orthogonal.x + orthogonal.y * orthogonal.y).sqrt();
        orthogonal.x *= 1.0 / ortho_length;
        orthogonal.y *= 1.0 / ortho_length;

        // project to inifity
        let infinity = 1e+10_f64;
        let projected = Point { x: circumcenter_pos.x + orthogonal.x * infinity, y: circumcenter_pos.y + orthogonal.y * infinity };
        let v = self.vertices.pushi(projected);

        #[cfg(debug_assertions)] println!("  Hull edge {hull_edge} (circumcenter {circumcenter}) extended orthogonally to {a} -> {b} at {}", v);
        Some(v)
    }

    /// Given a ```cell``` and a voronoi edge prev -> c, checks for an intersection of the edge with the bounding box
    /// and adds the clipped edge to the cell if there is an intersection.
    fn try_clip_edge(&mut self, cell: &mut Vec<usize>, prev: usize, c: usize) {
        let pos = &self.vertices[c];
        let prev_pos = &self.vertices[prev];
        let prev_to_pos = Point { x: pos.x - prev_pos.x, y: pos.y - prev_pos.y };
        let (first_clip, second_clip) = self.bounding_box.project_ray(prev_pos, &prev_to_pos);

        // intersection found, but does the intersection happen in the [prev -> c edge] or after it
        // i.e. check to see if first_clip is after c (pos) in the projection ray
        if first_clip.is_some() && bounding_box::order_points_on_ray(prev_pos, &prev_to_pos, Some(pos.clone()), first_clip.clone()).0 == first_clip {
            if let Some(second_clip) = second_clip {
                let first_clip = first_clip.unwrap();
                let (first, second) = (first_clip, second_clip);

                // edge clipped twice on the bounding box, need to wrap around as needed
                // FIXME: degenerated8 site 2 cell includes site 1 due to colinear degeneration - this case should not wrap around
                let (link_count, links) = self.get_link_vertices_around_box_edge(
                    &first,
                    &second);

                let f = self.vertices.pushi(first);
                cell.push(f);

                cell.extend_from_slice(&links[..link_count]);

                let s = self.vertices.pushi(second);
                cell.push(s);

                #[cfg(debug_assertions)] println!("  Edge {prev} {:?} -> {c} {:?}: outside the box, but crossed at {f} {:?} and {s} {:?}.", self.vertices[prev], self.vertices[c], self.vertices[f], self.vertices[s]);
            } else {
                // else the edge intersected in one of the corners only
                // FIXME what does this mean? do we need to wrap around?
                let f = self.vertices.pushi(first_clip.unwrap());
                cell.push(f);
                #[cfg(debug_assertions)] println!("  Edge {prev} -> {c}: outside the box, but crossed at {f} only.");
            }
        } else {
            #[cfg(debug_assertions)] println!("  Edge {prev} -> {c}: outside the box. Dropped.");
        }
    }

    /// Builds cells for each site.
    /// This won't extend not close the hull.
    /// This will not clip any edges to the bounding box.
    fn build_cells(&mut self) -> Vec<Vec<usize>> {
        let triangulation = self.triangulation;
        let sites: &Vec<Point> = self.sites;
        let num_of_sites = sites.len();
        let mut cells: Vec<Vec<usize>> = vec![Vec::new(); num_of_sites];

        let is_inside: Vec<bool> = self.vertices.iter().map(|c| self.bounding_box.is_inside(c)).collect();
        let mut tmp_cell = Vec::new();

        let is_inside = |c| {
            if c < is_inside.len() {
                is_inside[c]
            } else {
                false
            }
        };

        // fill in cells
        for edge in 0..triangulation.triangles.len() {
        //for edge in [39] {
        //for edge in [3] {
            let site = site_of_incoming(triangulation, edge);
            let cell = &mut cells[site];

            // if cell is empty, it hasn't been processed yet
            if cell.len() == 0 {
                // edge may or may not be the left-most incoming edge for site, thus get the one
                // if iterator doesn't start this way, we may end cell vertex iteration early because
                // we will hit the halfedge in the hull
                let leftmost_incoming_edge = self.site_to_incoming_leftmost_halfedge[site];
                let is_site_on_hull = triangulation.halfedges[leftmost_incoming_edge] == EMPTY;

                #[cfg(debug_assertions)] println!();
                #[cfg(debug_assertions)] println!("Site: {site}. {}. Leftmost incoming edge: {}", if is_site_on_hull { "HULL" } else { "" }, leftmost_incoming_edge);

                // tmp_cell will have vertices oriented clockwise because of EdgesAroundSiteIterator iterator
                tmp_cell.clear();
                let iter = EdgesAroundSiteIterator::new(triangulation, leftmost_incoming_edge);

                if is_site_on_hull {
                    let mut iter = iter;

                    // hull cells may need to have their first and last vertex extended
                    if let Some(extension) = self.extend_voronoi_vertex(leftmost_incoming_edge) {
                        tmp_cell.push(extension);
                    }

                    let mut last_edge = None;
                    while let Some(edge) = iter.next() {
                        tmp_cell.push(utils::triangle_of_edge(edge));
                        last_edge = Some(edge);
                    }

                    let last_edge = next_halfedge(last_edge.expect("Hull is at least a triangle"));
                    #[cfg(debug_assertions)] println!("  Hull last edge {last_edge}");
                    if let Some(extension) = self.extend_voronoi_vertex(last_edge) {
                        tmp_cell.push(extension);
                    }

                    // FIXME degenerated4 and 8 - not single triangle requirement, when is this wrong?
                    // if hull site has single triangle / circumcenter, then we cannot rely on the orientation
                    // and need to check for ourselves
                    let site_pos = &self.sites[site];
                    if robust::orient2d((&self.vertices[*tmp_cell.first().unwrap()]).into(), (&self.vertices[*tmp_cell.last().unwrap()]).into(), site_pos.into()) <= 0. {
                        tmp_cell.reverse();
                    };
                } else {
                    tmp_cell.extend(iter.map(utils::triangle_of_edge));
                }

                #[cfg(debug_assertions)] println!("  Temp: {:?}", tmp_cell);

                let (first_index, first, first_inside) = if let Some(inside) = tmp_cell.iter().enumerate().filter(|(_, &c)| is_inside(c)).next() {
                    (inside.0, *inside.1, true)
                } else {
                    // FIXME: degenerated4.json - site 1 is this case and the edge is not rendered correctly
                    #[cfg(debug_assertions)] println!("  [{site}/{edge}] Cell has no vertices inside the box. It only crosses the box.");
                    (0, tmp_cell[0], false)
                };

                // walk voronoi edges counter clockwise and clip as needed
                let mut prev = first;
                let mut prev_inside = first_inside;

                #[cfg(debug_assertions)] println!("  Start: Temp[{first_index}] = {first}.");

                // iterate over each voronoi edge starting on edge first -> first + 1
                // iterate on reverse to get counter-clockwise orientation
                for &c in tmp_cell.iter().rev().cycle().skip(tmp_cell.len() - first_index).take(tmp_cell.len()) {
                    let inside = is_inside(c);

                    match (prev_inside, inside) {
                        // voronoi edge has both vertices outside bounding box
                        (false, false) => {
                            // but may cross the box, so check for an intersection
                            self.try_clip_edge(cell, prev, c);
                        },

                        // entering bounding box - edge crosses bounding box edge from the outside
                        (false, true) => {
                            let clip = self.clip_voronoi_edge(c, prev).expect("Edge crosses box, intersection must exist.");

                            // we could have left from one edge of the box and entered from another, thus we need to wrap around the bounding box corners
                            // last vertex in the cell is the one clipped when exiting the bounding box
                            let (link_count, links) = self.get_link_vertices_around_box_edge(
                                &self.vertices[*cell.last().expect("Invariant by starting inside box")],
                                &self.vertices[clip]);

                            #[cfg(debug_assertions)] println!("  [{site}/{edge}] Edge {prev} -> {c}: Re-entering box at {clip} and wrapped around {:?}", &links[..link_count]);

                            cell.extend_from_slice(&links[..link_count]);
                            cell.push(clip);
                        },

                        // leaving bounding box - edge crosses bounding box edge from the inside
                        (true, false) => {
                            cell.pushi(prev);
                            cell.pushi(self.clip_voronoi_edge(prev, c).expect("Edge crosses box, intersection must exist."));
                            #[cfg(debug_assertions)] println!("  [{site}/{edge}] Edge {prev} -> {c}: Leaving box. Added {prev} and clipped at {}", cell.last().unwrap());
                        },

                        // edge inside box
                        (true, true) => {
                            #[cfg(debug_assertions)] println!("  [{site}/{edge}] Edge {prev} -> {c}: Inside box. Added {prev}.");
                            cell.push(prev);
                        },
                    }

                    prev = c;
                    prev_inside = inside;
                }

                #[cfg(debug_assertions)] println!("  [{site}/{edge}] Cell: {:?}", cell);
            }
        }

        cells
    }
}

fn calculate_incoming_edges(triangulation: &Triangulation, num_of_sites: usize) -> Vec<usize> {
    // create map between site and its left-most incoming half-edge
    // this is especially important for the sites along the convex hull boundary when iterating over its neighoring sites
    let mut site_to_incoming_leftmost_halfedge = vec![EMPTY; num_of_sites];

    for e in 0..triangulation.triangles.len() {
        let s = site_of_incoming(&triangulation, e);
        if site_to_incoming_leftmost_halfedge[s] == EMPTY || triangulation.halfedges[e] == EMPTY {
            site_to_incoming_leftmost_halfedge[s] = e;
        }
    }

    // if input sites has lot of coincident points (very very close), the underlying triangulation will be problematic and those points may be ignored in the triangulation
    // thus they won't be reacheable and this will lead to problems down the line as we build the voronoi graph
    debug_assert!(!site_to_incoming_leftmost_halfedge.iter().any(|e| *e == EMPTY), "One or more sites is not reacheable in the triangulation mesh. This usually indicate coincident points.");

    site_to_incoming_leftmost_halfedge
}

// #[cfg(test)]
// mod test {
//     use super::*;

//     // fn new_builder(vertices: Vec<Point>) -> CellBuilder {
//     //     CellBuilder::new(vertices, BoundingBox::new_centered_square(4.0), ClipBehavior::Clip)
//     // }

//     fn assert_same_elements(actual: &Vec<usize>, expected: &Vec<usize>, message: &str) {
//         assert_eq!(actual.len(), expected.len(), "Vectors differ in length. Actual: {:?}. Expected: {:?}. {}", actual, expected, message);
//         assert_eq!(0, actual.iter().copied().zip(expected.iter().copied()).filter(|(a,b)| a != b).count(), "Vectors have differing elements. Actual: {:?}. Expected: {:?}. {}", actual, expected, message);
//     }

//     #[test]
//     fn link_vertices_around_box_edge_same_edge_b_after_a() {
//         let mut builder = new_builder(vec![
//             Point { x: 1.0, y: 2.0 },
//             Point { x: -1.0, y: 2.0 },
//         ]);
//         let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut cell = original_cell.clone();
//         builder.calculate_corners();
//         builder.link_vertices_around_box_edge(&mut cell, 0, 1);
//         assert_same_elements(&cell, &original_cell, "No change expected. Same edge, b is after a");
//     }

//     #[test]
//     fn link_vertices_around_box_edge_same_edge_b_before_a() {
//         let mut builder = new_builder(vec![
//             Point { x: -1.0, y: 2.0 },
//             Point { x: 1.0, y: 2.0 },
//         ]);
//         let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut cell = original_cell;
//         builder.calculate_corners();
//         builder.link_vertices_around_box_edge(&mut cell, 0, 1);
//         assert_same_elements(&cell, &vec![0, builder.top_left_corner_index, builder.bottom_left_corner_index, builder.bottom_right_corner_index, builder.top_right_corner_index, 1], "Full circuit expected because b is before a");
//     }

//     #[test]
//     fn link_vertices_around_box_edge_same_edge_b_corner_before_a() {
//         let mut builder = new_builder(vec![
//             Point { x: -1.0, y: 2.0 },
//             Point { x: 2.0, y: 2.0 },
//         ]);
//         let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut cell = original_cell;
//         builder.calculate_corners();
//         builder.link_vertices_around_box_edge(&mut cell, 0, 1);
//         assert_same_elements(&cell, &vec![0, builder.top_left_corner_index, builder.bottom_left_corner_index, builder.bottom_right_corner_index, 1], "Corners not expected to be duplicated");
//     }

//     #[test]
//     fn link_vertices_around_box_edge_same_edge_a_top_left_b_bottom_right() {
//         let mut builder = new_builder(vec![
//             Point { x: -2.0, y: 2.0 },
//             Point { x: 2.0, y: -2.0 },
//         ]);
//         let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut cell = original_cell;
//         builder.calculate_corners();
//         builder.link_vertices_around_box_edge(&mut cell, 0, 1);
//         assert_same_elements(&cell, &vec![0, builder.bottom_left_corner_index, 1], "Corners not expected to be duplicated");
//     }

//     #[test]
//     fn link_vertices_around_box_edge_same_edge_a_right_b_left() {
//         let mut builder = new_builder(vec![
//             Point { x: 2.0, y: 1.0 },
//             Point { x: -2.0, y: 1.5 },
//         ]);
//         let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut cell = original_cell;
//         builder.calculate_corners();
//         builder.link_vertices_around_box_edge(&mut cell, 0, 1);
//         assert_same_elements(&cell, &vec![0, builder.top_right_corner_index, builder.top_left_corner_index, 1], "Corners not expected to be duplicated");
//     }

//     #[test]
//     fn link_vertices_around_box_edge_same_edge_a_left_b_right() {
//         let mut builder = new_builder(vec![
//             Point { x: -2.0, y: 1.0 },
//             Point { x: 2.0, y: 1.5 },
//         ]);
//         let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut cell = original_cell;
//         builder.calculate_corners();
//         builder.link_vertices_around_box_edge(&mut cell, 0, 1);
//         assert_same_elements(&cell, &vec![0, builder.bottom_left_corner_index, builder.bottom_right_corner_index, 1], "Corners not expected to be duplicated");
//     }


//     #[test]
//     fn link_vertices_around_box_edge_same_edge_a_bottom_b_top() {
//         let mut builder = new_builder(vec![
//             Point { x: 1.0, y: -2.0 },
//             Point { x: -1.5, y: 2.0 },
//         ]);
//         let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut cell = original_cell;
//         builder.calculate_corners();
//         builder.link_vertices_around_box_edge(&mut cell, 0, 1);
//         assert_same_elements(&cell, &vec![0, builder.bottom_right_corner_index, builder.top_right_corner_index, 1], "Corners not expected to be duplicated");
//     }

//     #[test]
//     fn clip_cell_edge_one_edge_crosses_one_box_edge() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 10.0, y: 0.0 },
//         ]);
//         let (a, b) = builder.clip_cell_edge(0, 1);
//         assert_eq!(a, 0, "A is inside box, should not change");
//         assert_eq!(b, 2, "New edge should have been added due to clipping");
//         assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[2], "Point should have been added for clipped edge.");
//     }

//     #[test]
//     fn clip_cell_edge_edge_outside_box_with_single_point_on_box() {
//         let mut builder = new_builder(vec![
//             Point { x: -2.0, y: 0.0 },
//             Point { x: -5.0, y: 0.0 },
//         ]);

//         /*
//                     |-----\
//                     |     \
//                 x--x     \
//                     |     \
//                     |-----\
//         */
//         let (a, b) = builder.clip_cell_edge(0, 1);
//         assert_eq!(a, EMPTY, "Edge must be removed");
//         assert_eq!(b, EMPTY, "Edge must be removed");
//     }

//     #[test]
//     fn clip_cell_edge_one_edge_crosses_one_box_edge_inverted() {
//         let mut builder = new_builder(vec![
//             Point { x: 10.0, y: 0.0 },
//             Point { x: 0.0, y: 0.0 },
//         ]);
//         let (a, b) = builder.clip_cell_edge(0, 1);
//         assert_eq!(a, 2, "New edge should have been added due to clipping");
//         assert_eq!(b, 1, "B is inside box, should not change");
//         assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[2], "Point should have been added for clipped edge.");
//     }

//     #[test]
//     fn clip_cell_edge_one_edge_crosses_two_box_edges() {
//         let mut builder = new_builder(vec![
//             Point { x: -10.0, y: 0.0 },
//             Point { x: 10.0, y: 0.0 },
//         ]);
//         let (a, b) = builder.clip_cell_edge(0, 1);
//         assert_eq!(a, 2, "New edge should have been added due to clipping");
//         assert_eq!(b, 3, "New edge should have been added due to clipping");
//         assert_eq!(Point { x: -2.0, y: 0.0 }, builder.vertices[2], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[3], "Point should have been added for clipped edge.");
//     }

//     #[test]
//     fn clip_and_close_cell_when_no_point_outside_box() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 1.0, y: 0.0 },
//             Point { x: 0.0, y: 1.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.clip_and_close_cell(&mut clipped_cell);
//         assert_same_elements(&clipped_cell, &vec![0, 1, 2], "No clipping expected");
//         assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
//     }

//     #[test]
//     fn clip_and_close_cells_with_shared_edge_intersecting_box() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 1.0, y: 0.0 },
//             Point { x: 0.0, y: 5.0 }, // outside of bounding box
//             Point { x: -1.0, y: 0.0 },
//         ]);

//         /*
//                              2
//                              |
//             ------------------------------------
//             |                 |                 \
//             |                 |                 \
//             |                 |                 \
//             |        3 ------ 0 ------ 1        \
//             |------------------------------------\
//         */
//         let cell: Vec<usize> = [0, 1, 2].to_vec();
//         let mut clipped_cell = cell;
//         builder.clip_and_close_cell(&mut clipped_cell);

//         assert_eq!(clipped_cell.len(), 4, "A new vertix must be added because two edges cross the box");
//         assert_eq!(builder.bounding_box.which_edge(&builder.vertices[clipped_cell[2]]), (BoundingBoxTopBottomEdge::Top, BoundingBoxLeftRightEdge::None), "Vertex must be on bottom of bounding box");
//         assert_eq!(builder.bounding_box.which_edge(&builder.vertices[clipped_cell[3]]), (BoundingBoxTopBottomEdge::Top, BoundingBoxLeftRightEdge::None), "Vertex must be on bottom of bounding box");
//         assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");

//         // edge 0 -> 2 is shared
//         let cell: Vec<usize> = [3, 0, 2].to_vec();
//         let mut neighbor_clipped_cell = cell;
//         builder.clip_and_close_cell(&mut neighbor_clipped_cell);

//         assert_eq!(neighbor_clipped_cell.len(), 4, "A new vertix must be added because two edges cross the box");
//         assert_eq!(neighbor_clipped_cell[2], clipped_cell[3], "Vertex on shared edge must be clipped to same value");
//         assert_cell_consistency(&neighbor_clipped_cell, &builder, "Cell consistency check");
//     }

//     #[test]
//     fn clip_triangular_cell_with_one_point_outside_box() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 0.0, y: -3.0 }, // outside
//             Point { x: 1.0, y: 0.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.clip_and_close_cell(&mut clipped_cell);
//         assert_same_elements(&clipped_cell, &vec![0, 3, 4, 2], "Clipped cell incorrect indices.");
//         assert_eq!(Point { x: 0.0, y: -2.0 }, builder.vertices[3], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 1.0 / 3.0, y: -2.0 }, builder.vertices[4], "Point should have been added for clipped edge.");
//         assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
//     }

//     #[test]
//     fn clip_triangular_cell_with_one_point_outside_box_and_last_crossing_box_edge() {
//         let mut builder = new_builder(vec![
//             Point { x: 1.0, y: 0.0 },
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 0.0, y: -3.0 }, // outside
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.clip_and_close_cell(&mut clipped_cell);
//         assert_same_elements(&clipped_cell, &vec![0, 1, 3, 4], "Clipped cell incorrect indices.");
//         assert_eq!(Point { x: 0.0, y: -2.0 }, builder.vertices[3], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 1.0 / 3.0, y: -2.0 }, builder.vertices[4], "Point should have been added for clipped edge.");
//         assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
//     }

//     #[test]
//     fn clip_triangular_cell_with_two_points_outside_and_one_edge_entirely_outside_box() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 0.0, y: -30.0 }, // leaves through the bottom
//             Point { x: 20.0, y: 0.0 }, // comes back through the right
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.calculate_corners();
//         builder.clip_and_close_cell(&mut clipped_cell);
//         // builder adds 4 vertices all the time, so next index is 7
//         assert_same_elements(&clipped_cell, &vec![0, 7, builder.bottom_right_corner_index, 8], "Clipped cell incorrect indices.");
//         assert_eq!(Point { x: 0.0, y: -2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
//         assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
//     }

//     #[test]
//     fn clip_triangular_cell_with_two_points_outside_and_edge_crossing_box_twice() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 0.0, y: -3.0 },
//             Point { x: 3.0, y: 0.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.calculate_corners();
//         builder.clip_and_close_cell(&mut clipped_cell);
//         // builder adds 4 vertices all the time, so next index is 7
//         assert_same_elements(&clipped_cell, &vec![0, 7, 8, 9, 10], "Clipped cell incorrect indices.");
//         assert_eq!(Point { x: 0.0, y: -2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 1.0, y: -2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 2.0, y: -1.0 }, builder.vertices[9], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[10], "Point should have been added for clipped edge.");
//         assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
//         //assert_eq!(is_closed, true, "No entire edge was outside the box, so the cell must stay closed.")
//     }

//     #[test]
//     fn clip_triangular_cell_with_one_edge_intersecting_box_corner() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 4.0, y: 0.0 },
//             Point { x: 0.0, y: 4.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.calculate_corners();
//         builder.clip_and_close_cell(&mut clipped_cell);
//         assert_same_elements(&clipped_cell, &vec![0, 7, builder.top_right_corner_index , 8], "Clipped cell incorrect indices.");
//         assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 0.0, y: 2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
//         assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
//     }

//     // same as above but with triangle inverted, no vertex inside box
//     #[test]
//     fn clip_triangular_cell_outside_with_one_edge_intersecting_box_corner() {
//         let mut builder = new_builder(vec![
//             Point { x: 4.0, y: 4.0 },
//             Point { x: 4.0, y: 0.0 },
//             Point { x: 0.0, y: 4.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.calculate_corners();
//         let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
//         assert_eq!(keep_cell, false, "A single intersection at the coner with all other points outside the cell, should not keep this cell.");
//         assert_same_elements(&clipped_cell, &vec![builder.top_right_corner_index], "Clipped cell incorrect indices.");
//     }

//     #[test]
//     fn clip_triangular_cell_with_one_point_inside_box_and_one_edge_parallel_to_box_edge() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 2.0, y: -4.0 },
//             Point { x: 2.0, y: 4.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.calculate_corners();
//         builder.clip_and_close_cell(&mut clipped_cell);
//         // FIXME: identify intersection at corners and return corner index instead of creating a new one
//         assert_same_elements(&clipped_cell, &vec![0, 7, 8, 9, 10], "Clipped cell incorrect indices.");
//         assert_eq!(Point { x: 1.0, y: -2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 2.0, y: -2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 2.0, y: 2.0 }, builder.vertices[9], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 1.0, y: 2.0 }, builder.vertices[10], "Point should have been added for clipped edge.");
//         assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
//     }

//     #[test]
//     fn clip_triangular_cell_with_no_point_inside_box_and_one_edge_parallel_to_box_edge() {
//         let mut builder = new_builder(vec![
//             Point { x: 4.0, y: 0.0 },
//             Point { x: 2.0, y: 4.0 },
//             Point { x: 2.0, y: -4.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.calculate_corners();
//         let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
//         assert_same_elements(&clipped_cell, &vec![7, 8], "Clipped cell incorrect indices.");
//         assert_eq!(Point { x: 2.0, y: 2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 2.0, y: -2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
//         assert_eq!(keep_cell, false, "A line shared with one box edge, should not keep this cell.");
//         //assert_eq!(is_closed, false, "One edge outside box.")
//     }

//     // same case as above, but the entire parallel edge is shared with the box
//     #[test]
//     fn clip_triangular_cell_with_no_point_inside_box_and_one_shared_edge_parallel_to_box_edge() {
//         let mut builder = new_builder(vec![
//             Point { x: 4.0, y: 0.0 },
//             Point { x: 2.0, y: 2.0 },
//             Point { x: 2.0, y: -2.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.calculate_corners();
//         builder.clip_and_close_cell(&mut clipped_cell);
//         assert_same_elements(&clipped_cell, &vec![7, 1, 2, 8], "Clipped cell incorrect indices.");
//         assert_eq!(Point { x: 2.0, y: 2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 2.0, y: 2.0 }, builder.vertices[1], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 2.0, y: -2.0 }, builder.vertices[2], "Point should have been added for clipped edge.");
//         assert_eq!(Point { x: 2.0, y: -2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");

//         // FIXME: the result is a rectangle with area 0 - code think cell is closed, but it should be removed
//         //assert_cell_consistency(&clipped_cell, &builder);

//         // FIX ME: code thinks cell is closed, but that is not true in this case
//         // ignoring for now as this shouldn't really happen for voronoi
//         //assert_eq!(is_closed, false, "One edge outside box.");
//     }

//     #[test]
//     fn clip_triangular_cell_with_three_points_outside_box() {
//         let mut builder = new_builder(vec![
//             Point { x: 10.0, y: 0.0 },
//             Point { x: 10.0, y: -3.0 },
//             Point { x: 15.0, y: 0.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.calculate_corners();
//         let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
//         assert_eq!(keep_cell, false, "No intersection with box, all points outside, should not keep the cell.");
//         assert_same_elements(&clipped_cell, &vec![], "Clipped cell incorrect indices.");
//     }

//     #[test]
//     fn clip_square_cell_with_two_edges_outside_box() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 5.0, y: 0.0 },
//             Point { x: 5.0, y: 5.0 },
//             Point { x: 0.0, y: 5.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.calculate_corners();
//         let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
//         assert_eq!(keep_cell, true, "Intersection with box.");
//         assert_cell_vertex(&builder, &clipped_cell, "Incorrect clipping", vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 2.0, y: 0.0 },
//             Point { x: 2.0, y: 2.0 },
//             Point { x: 0.0, y: 2.0 },
//         ]);
//     }

//     #[test]
//     fn clip_square_cell_with_two_edges_outside_box_start_vertex_outside() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.0, y: 5.0 },
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 5.0, y: 0.0 },
//             Point { x: 5.0, y: 5.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.calculate_corners();
//         let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
//         assert_eq!(keep_cell, true, "Intersection with box.");
//         assert_cell_vertex(&builder, &clipped_cell, "Incorrect clipping", vec![
//             Point { x: 0.0, y: 2.0 },
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 2.0, y: 0.0 },
//             Point { x: 2.0, y: 2.0 },
//         ]);
//     }

//     #[test]
//     fn clip_polygon_cell_with_multiple_edges_removed() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 0.0, y: -5.0 },
//             Point { x: 1.0, y: -5.0 },
//             Point { x: 1.0, y: -1.0 },
//             Point { x: 5.0, y: -1.0 },
//             Point { x: 5.0, y: 0.0 },
//             Point { x: 1.0, y: 0.0 },
//             Point { x: 1.0, y: 5.0 },
//             Point { x: 0.0, y: 5.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.calculate_corners();
//         let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
//         assert_eq!(keep_cell, true, "Intersection with box.");
//         assert_cell_vertex(&builder, &clipped_cell, "Incorrect clipping", vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 0.0, y: -2.0 },
//             Point { x: 1.0, y: -2.0 },
//             Point { x: 1.0, y: -1.0 },
//             Point { x: 2.0, y: -1.0 },
//             Point { x: 2.0, y: 0.0 },
//             Point { x: 1.0, y: 0.0 },
//             Point { x: 1.0, y: 2.0 },
//             Point { x: 0.0, y: 2.0 },
//         ]);
//     }

//     #[test]
//     fn clip_polygon_cell_with_multiple_edges_removed_and_coners_added() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 0.0, y: -5.0 },
//             Point { x: 5.0, y: -5.0 },
//             Point { x: 5.0, y: -1.0 },
//             Point { x: 1.0, y: -1.0 },
//             Point { x: 1.0, y: 0.0 },
//             Point { x: 5.0, y: 0.0 },
//             Point { x: 5.0, y: 5.0 },
//             Point { x: 0.0, y: 5.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.calculate_corners();
//         let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
//         assert_eq!(keep_cell, true, "Intersection with box.");
//         assert_cell_vertex(&builder, &clipped_cell, "Incorrect clipping", vec![
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 0.0, y: -2.0 },
//             Point { x: 2.0, y: -2.0 },
//             Point { x: 2.0, y: -1.0 },
//             Point { x: 1.0, y: -1.0 },
//             Point { x: 1.0, y: 0.0 },
//             Point { x: 2.0, y: 0.0 },
//             Point { x: 2.0, y: 2.0 },
//             Point { x: 0.0, y: 2.0 },
//         ]);
//     }

//     #[test]
//     fn clip_polygon_cell_with_multiple_edges_removed_and_coners_added_starting_edge_outside() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.0, y: 5.0 },
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 0.0, y: -5.0 },
//             Point { x: 5.0, y: -5.0 },
//             Point { x: 5.0, y: -1.0 },
//             Point { x: 1.0, y: -1.0 },
//             Point { x: 1.0, y: 0.0 },
//             Point { x: 5.0, y: 0.0 },
//             Point { x: 5.0, y: 5.0 },
//         ]);
//         let cell: Vec<usize> = (0..builder.vertices.len()).collect();
//         let mut clipped_cell = cell;
//         builder.calculate_corners();
//         let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
//         assert_eq!(keep_cell, true, "Intersection with box.");
//         assert_cell_vertex(&builder, &clipped_cell, "Incorrect clipping", vec![
//             Point { x: 0.0, y: 2.0 },
//             Point { x: 0.0, y: 0.0 },
//             Point { x: 0.0, y: -2.0 },
//             Point { x: 2.0, y: -2.0 },
//             Point { x: 2.0, y: -1.0 },
//             Point { x: 1.0, y: -1.0 },
//             Point { x: 1.0, y: 0.0 },
//             Point { x: 2.0, y: 0.0 },
//             Point { x: 2.0, y: 2.0 },
//         ]);
//     }

//     #[test]
//     fn extend_vertex_test() {
//         let mut builder = new_builder(vec![
//             Point { x: 0.5, y: 0.0 }, // vertex to be extended
//         ]);

//         let ext = builder.extend_vertex(&Point { x: 1.0, y: 1.0 }, &Point { x: 0.0, y: 1.0 }, 1.0);
//         assert_eq!(Point { x: 0.5, y: 2.0 }, builder.vertices[ext], "Extension expected to be orthogonal to a -> b and on the bounding box edge.");
//     }

//     #[test]
//     fn extend_and_close_hull_test() {
//         let mut builder = new_builder(vec![
//             Point { x: -0.5, y: -0.25 }, // vertex to be extended
//         ]);
//         builder.calculate_corners();
//         let sites = vec![
//             Point { x: -0.5, y: 1.0 },
//             Point { x: -1.5, y: -1.0 },
//             Point { x: 0.5, y: -1.0 },
//         ];
//         assert_eq!(builder.vertices[0], utils::cicumcenter(&sites[0], &sites[1], &sites[2]), "I got the circumcenter wrong.");
//         let hull_sites = (0..sites.len()).collect();
//         let mut cells = vec![
//             vec![0],
//             vec![0],
//             vec![0],
//         ];
//         builder.extend_and_close_hull(&sites, &hull_sites, &mut cells);

//         assert_cell_vertex_without_bounds(&builder, &cells[0], "First cell", vec![
//             Point { x: -0.5, y: -0.25 },
//             Point { x: 7.155417527999327, y: 3.5777087639996634 },
//             Point { x: -8.155417527999326, y: 3.5777087639996634 },
//         ]);

//         assert_cell_vertex_without_bounds(&builder, &cells[1], "Second cell", vec![
//             Point { x: -0.5, y: -0.25 },
//             Point { x: -8.155417527999326, y: 3.5777087639996634 },
//             Point { x: -0.5, y: -9.0 },
//         ]);

//         assert_cell_vertex_without_bounds(&builder, &cells[2], "Third cell", vec![
//             Point { x: -0.5, y: -0.25 },
//             Point { x: -0.5, y: -9.0 },
//             Point { x: 7.155417527999327, y: 3.5777087639996634 },
//         ]);
//     }
// }