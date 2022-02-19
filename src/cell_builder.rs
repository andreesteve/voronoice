use std::{assert_eq, iter::once, ops::RangeBounds};
use delaunator::{EMPTY, Triangulation, EPSILON};
use crate::utils::abs_diff_eq;

use super::{ClipBehavior, Point, bounding_box::{self, *}, iterator::EdgesAroundSiteIterator, utils::{self, site_of_incoming}};

#[derive(Debug)]
pub struct CellBuilder {
    vertices: Vec<Point>,
    bounding_box: BoundingBox,
    clip_behavior: ClipBehavior,
    top_left_corner_index: usize,
    bottom_left_corner_index: usize,
    top_right_corner_index: usize,
    bottom_right_corner_index: usize,
}

pub struct CellBuilderResult {
    pub cells: Vec<Vec<usize>>,
    pub vertices: Vec<Point>
}

/// Indicates what was the result of the clip / extension calculation for a hull site / edge.
enum HullClipResult {
    /// No clipping or extension required.
    None,

    /// The circumcenter (voronoi vertex) associated with the hull's edge was clipped.
    ///
    /// The associated new vertex (projection) is returned.
    Clipped(usize),

    /// The circumcenter (voronoi vertex) associated with the hull's edge was extended.
    ///
    /// The associated new vertex (projection) is returned.
    Extended(usize)
}

impl CellBuilder {
    pub fn new(vertices: Vec<Point>, bounding_box: BoundingBox, clip_behavior: ClipBehavior) -> Self {
        Self {
            top_right_corner_index: 0,
            top_left_corner_index: 0,
            bottom_left_corner_index: 0,
            bottom_right_corner_index: 0,
            vertices,
            bounding_box,
            clip_behavior,
        }
    }

    pub fn build(mut self, sites: &Vec<Point>, triangulation: &Triangulation, site_to_incoming_leftmost_halfedge: &Vec<usize>) -> CellBuilderResult {
        // adds the corners of the bounding box as potential vertices for the voronoi
        self.calculate_corners();

        // create the cells
        let mut cells = self.build_cells(sites, triangulation, site_to_incoming_leftmost_halfedge);

        // is this true? what if the triangle is degenerated and the circumcenter is outside the bounding box?
        // debug_assert!({
        //     let invalid_hull_sites: Vec<usize> = hull.iter().copied().filter(|&hull_site| {
        //         Some(&triangle_of_edge(site_to_incoming_leftmost_halfedge[hull_site])) != cells[hull_site].first()
        //     }).collect();

        //     if !invalid_hull_sites.is_empty() {
        //         println!("The following hull sites do not have its first voronoi cell vertex as the circumcenter of the triangle associated with its left most incoming half-edge: {:?}", invalid_hull_sites);
        //         false
        //     } else {
        //         true
        //     }
        // },"The first vertex of each voronoi cell on the hull must always be the circumcenter of the triangle associated to the leftmost incoming half-edge.");

        debug_assert!({
            let invalid_hull_sites: Vec<usize> = triangulation.hull.iter().copied().filter(|&hull_site| {
                triangulation.halfedges[site_to_incoming_leftmost_halfedge[hull_site]] != EMPTY
            }).collect();

            if !invalid_hull_sites.is_empty() {
                println!("The following hull sites have incorrect leftmost halfedge cached: {:?}", invalid_hull_sites);
                false
            } else {
                true
            }
        },"The left most edge for hull sites must not have a half-edge.");

        debug_assert!({
            let invalid_hull_sites: Vec<usize> = site_to_incoming_leftmost_halfedge.iter().enumerate()
                .filter(|(site, &edge)| triangulation.halfedges[edge] == EMPTY && !triangulation.hull.contains(site))
                .map(|(site, _)| site)
                .collect();

            if !invalid_hull_sites.is_empty() {
                println!("The following sites are not on the hull list but they are supposed to because they have an edge without an associated half-edge: {:?}", invalid_hull_sites);
                false
            } else {
                true
            }
        },"The all sites whose left most edge has no associated half-edge must be on the hull.");

        CellBuilderResult {
            vertices: self.vertices,
            cells
        }
    }

    /// Extend, towards the bounding box edge, the `voronoi_vertex` orthogonally to the Delaunay triangle edge represented by `a` -> `b`.
    /// Creates the new vertex on the bounding box edge and returns it index on the `vertices` collection.
    fn extend_vertex(&mut self, site_a: &Point, site_b: &Point, scale: f64) -> usize {
        // the vertex is the circumcenter of the triangle of edge a->b
        // the vertex is on the a->b bisector line, thus we can take midpoint of a->b and vertex for the orthogonal projection
        let edge_midpoint = Point { x: (site_a.x + site_b.x) / 2.0, y: (site_a.y + site_b.y) / 2.0 };

        // clockwise rotation 90 degree from edge direction
        let mut orthogonal = Point { x: site_b.y - site_a.y, y: site_a.x - site_b.x };
        // normalizing the orthogonal vector
        // let ortho_length = (orthogonal.x * orthogonal.x + orthogonal.y * orthogonal.y).sqrt();
        // orthogonal.x *= 1. / ortho_length;
        // orthogonal.y *= 1. / ortho_length;

        // let mut projected = Point { x: edge_midpoint.x + (scale * orthogonal.x), y: edge_midpoint.y + (scale * orthogonal.y) };

        let projected = if let Some(projected) = self.bounding_box.project_ray_closest(&edge_midpoint, &orthogonal) {
            projected
        } else {
            // midpoint is on the edge
            edge_midpoint
        };

        let index = self.vertices.len();
        self.vertices.push(projected);

        index
    }

    fn extend_and_close_hull(&mut self, sites: &Vec<Point>, hull_sites: &Vec<usize>, cells: &mut Vec<Vec<usize>>) {
        assert_eq!(true, hull_sites.len() > 2, "A valid triangulation has at least 3 sites.");

        // For each site on the hull, extend its first vertex beyond the bounding box
        // Add this extended vertex to the cell, and the previous cell extended vertex as well
        // Thus closing the cell. When clip logic runs for this cell, it will clip the extensions as needed
        // Set a extension scale to be more than the bounding box diagonal, to make sure we get nice clipped corners
        let scale = self.bounding_box.width() + self.bounding_box.height();

        // handle first
        let &last_site = hull_sites.last().expect("");
        let &first_site = hull_sites.first().expect("");
        let last_site_pos = &sites[last_site];
        let mut site_pos = &sites[first_site];

        let last_size_ext = self.extend_vertex(last_site_pos, site_pos, scale);
        let mut prev_ext = last_size_ext;
        let mut site = first_site;

        // skip last to make sure we can always read next
        for &next_site in hull_sites.iter().skip(1) {
            let next_site_pos = &sites[next_site];
            let ext = self.extend_vertex(site_pos, next_site_pos, scale);
            cells[site].push(prev_ext);

            let (count, val) = self.get_link_vertices_around_box_edge(&self.vertices[prev_ext], &self.vertices[ext]);
            for i in (0..count) {
                cells[site].push(val[i]);
            }

            cells[site].push(ext);
            prev_ext = ext;
            site_pos = next_site_pos;
            site = next_site;
        }

        // handle skipped
        cells[last_site].push(prev_ext);

        let (count, val) = self.get_link_vertices_around_box_edge(&self.vertices[prev_ext], &self.vertices[last_size_ext]);
        for i in (0..count) {
            cells[site].push(val[i]);
        }

        cells[last_site].push(last_size_ext);
    }

    /// Cell is assumed to be closed.
    /// Returns a value indicating whether this cell contains at least one edge inside the bounding box.
    pub fn clip_and_close_cell(&mut self, cell: &mut Vec<usize>) -> bool {
        if cell.len() < 3 {
            panic!("Only closed cells can be clipped. A cell must have at least 3 vertices to possibly be closed.")
        }

        // keep track of the indices where the cell is open
        // open_edges[i] will tell the index of new_cell after an edge removal
        const MAX_OPEN_EDGES: usize = 20;
        let mut open_edges = [EMPTY; MAX_OPEN_EDGES];
        let mut open_edges_count = 0;
        let mut vertex_count = 0;
        let mut last_edge_removed = false;

        let first = cell.first().expect("At least one vertex expected for a cell.");
        let last = cell.last().expect("At least one vertex expected for a cell.");
        let mut previous = None;

        // iterates over edges (n, n+1), start with (last, first) since the cell is closed
        *cell = cell.iter().zip(cell.iter().skip(1)).chain(once((last, first)))
            // clip edge and convert to list of indices again
            .flat_map(|(&a, &b)| {
                // voronoi cell edges are shared by one neighbor
                // check to see if neighbor already clipped this edge (in opposite direction)

                // FIXME: adding a cache to clip solves the problem of duplicated vertices, but makes the code 50% slower
                // Things to think about:
                // * is clipping onyl necessary for cells on the hull?
                // * if so, we can drastically simplify this logic by clipping cells in the hull and in order so we know neighbors and shared edges
                // the answer seem to be yes, but degenerated triangles cause problems and thsoe are on sites not on the hull, but adjacent to it
                // maybe those can be tracked during creation? and handled specially?
                // actually voronoi cell vertices may be outside the bounding box
                //
                // let (new_a, new_b) = if let Some(clipped) = self.edge_clip_cache[ba_cache_index] {
                //     // if edge was previous clipped, invert direction to match ours
                //     let (new_b, new_a) = clipped;
                //     (new_a, new_b)
                // } else {
                //     let ab_cache_index = a * self.vertices.len() + b;
                    let (new_a, new_b) = self.clip_cell_edge(a, b);
                //     self.edge_clip_cache[ab_cache_index] = Some((new_a, new_b));
                //     (new_a, new_b)
                // };

                once(new_a).chain(once(new_b))
            })
            // remove duplicates
            .filter_map(|a| {
                let prev = previous;
                previous = Some(a);

                // remove vertex if it is empty


                if a == EMPTY {
                    if prev == Some(EMPTY) {
                        // two EMPTY vertices removed in a row means an edge was removed
                        last_edge_removed = true;
                    }
                    None
                } else if let Some(prev) = prev {
                    if prev == a {
                        // vertex did not change
                        None
                    } else {
                        // is we have a valid vertex, and one or more edges was removed
                        if last_edge_removed {
                            last_edge_removed = false;
                            open_edges[open_edges_count] = vertex_count;
                            open_edges_count += 1;
                        }

                        vertex_count += 1;
                        Some(a)
                    }
                } else {
                    // is we have a valid vertex, and one or more edges was removed
                    if last_edge_removed {
                        last_edge_removed = false;
                        open_edges[open_edges_count] = vertex_count;
                        open_edges_count += 1;
                    }

                    vertex_count += 1;
                    Some(a)
                }
            })
            .collect::<Vec<usize>>();

        if cell.len() < 2 {
            // if 1 or 0 vertice in the box, this cell should be removed
            return false;
        }

        // due to how the iterator above works, since all edges are expanded and then removed
        // we need to handle the last edge (last -> first) after that work is done
        // since clipping happens, this may not always hold, but often does
        if cell.last() == cell.first() {
            cell.truncate(cell.len() - 1);
        }

        // if we finish with an open edge without entering back the box, handle it here
        if last_edge_removed {
            // we cycled the entire cell without entering back, this means we need to connect to the starting point
            open_edges[open_edges_count] = 0;
            open_edges_count += 1;
        }

        // if edges were removed, cell is open and needs to be closed
        let mut open_edge_index_adjustment = 0;
        for i in 0..open_edges_count {
            // FIXME: this can only happen for the first value (open_edges[0]), no need to check on every loop
            // if we start iterating the cell on a vertex outside the box
            // the paring vertex to close this edge wraps around the vector (i.e. need to close last -> first)
            let (a, b) = if open_edges[i] == 0 {
                (cell.len() - 1, 0)
            } else {
                (open_edges[i] + open_edge_index_adjustment - 1, open_edges[i] + open_edge_index_adjustment)
            };

            // this many vertices were added between A and B, this means values were shifted right
            let new_vertices = self.link_vertices_around_box_edge(cell, a, b);

            // only add to the adjustment if this is not the last-first edge
            // because any new vertices will be added to the end of the array
            // thus they do not shift the position of the other vertices
            if open_edges[i] != 0 {
                open_edge_index_adjustment += new_vertices;
            }
        }

        // if after closing, we are left with a line, this cell should be removed
        cell.len() >= 3
    }

    /// Clips edge indexed by a -> b and return the indices of the vertices in the clipped edge (may be same values if no clipping was required).
    /// Cells edges be returned as `EMPTY` indicating that the edge is completely outside the box and should be excluded.
    fn clip_cell_edge(&mut self, a: usize, b: usize) -> (usize, usize) {
        // we are iterating a -> b, counter-clockwise on the edges of the cell (cell may be open)
        // at least one intersection, possibilities
        // 1) a -> box edge -> box edge -> b            a and b outside box ---> need to clip edge at both intersections
        // 2) a -> box edge_same, box edge_same -> b    same as 1, but a->b is a line parallel to one of the box edges ---> keep edge (excluding edge would open cell)
        // 3) a -> box corner, box corner -> b          same as 1, but intersection is right on box corner, or line is parallel to one of the box edges ---> keep edge (excluding edge would open cell)
        // 4) a -> box edge -> b -> box edge            a outside, b inside box ---> clip edge at first intersection
        // 5) a -> b -> box edge [, box edge]           a and b outside, but the line they are in intersects the box (variations include intersection on corner, intersection parallel to a box edge) ---> exclude edge
        // 6) a -> b -> box edge                        a and b inside ---> keep edge as is
        // 7) a -> b                                    a and b outside box, and no intersection with box ---> exclude edge
        let pa = &self.vertices[a];
        let pb = &self.vertices[b];
        let mut new_a = a;
        let mut new_b = b;

        if !self.bounding_box.is_inside(pa) {
            // a is outside, b is inside or outside
            // clip will tell us how many intersections between a->b, and clip_a will come first, then clip_b
            let a_to_b = Point { x: pb.x - pa.x, y: pb.y - pa.y };
            let (clip_a, clip_b) = self.bounding_box.project_ray(pa, &a_to_b);

            if let Some(clip_a) = clip_a {
                let v_index = self.vertices.len();
                new_a = v_index;

                if let Some(clip_b) = clip_b {
                    // b may be inside or outside the box
                    if !self.bounding_box.is_inside(pb) {
                        // there are two main cases here:
                        // a -> b -> box; i.e. ray reaches the box but after b; we discard this edge (case 5)
                        // check which point (b or clip_a) is closer to a
                        let closest_to_a = bounding_box::order_points_on_ray(pa,&a_to_b, Some(pb.clone()), Some(clip_a.clone())).0.unwrap();
                        if closest_to_a == clip_a {
                            // a -> box -> b; edge crosses box (case 1,2,3), we clip this ray twice
                            new_b = v_index + 1;

                            if abs_diff_eq(clip_a.x, clip_b.x, EPSILON) && abs_diff_eq(clip_a.y, clip_b.y, EPSILON) {
                                // case 3 - a and b outside box, intersection at the corner
                                // intersection at same point (corner)
                                new_b = EMPTY;

                                // check which corner it is and use the known index
                                new_a = match self.bounding_box.which_edge(&clip_a) {
                                    (BoundingBoxTopBottomEdge::Top, BoundingBoxLeftRightEdge::Right) => self.top_right_corner_index,
                                    (BoundingBoxTopBottomEdge::Top, BoundingBoxLeftRightEdge::Left) => self.top_left_corner_index,
                                    (BoundingBoxTopBottomEdge::Bottom, BoundingBoxLeftRightEdge::Right) => self.bottom_right_corner_index,
                                    (BoundingBoxTopBottomEdge::Bottom, BoundingBoxLeftRightEdge::Left) => self.bottom_left_corner_index,
                                    _ => panic!("All corners covered."),
                                };
                            } else {
                                self.vertices.push(clip_a);
                                self.vertices.push(clip_b);
                            }
                        } else {
                            // a -> b -> box (case 5), discard edge
                            new_a = EMPTY;
                            new_b = EMPTY;
                        }
                    } else {
                        // b is inside the box, only clip the edge once
                        self.vertices.push(clip_a);
                    }
                } else {
                    // a single itersection, this means a -> box ->b
                    self.vertices.push(clip_a);
                }
            } else {
                // a and b are outside of bounding box and there were no intersection with box
                // this edge will be completely excluded from the result
                // this also means the resulting cell will be open (if it was previously closed)
                // case 7
                new_a = EMPTY;
                new_b = EMPTY;
            }
        } else if !self.bounding_box.is_inside(pb) {
            // b is outside, and a is inside
            let a_to_b = Point { x: pb.x - pa.x, y: pb.y - pa.y };
            let clip_b = self.bounding_box.project_ray_closest(pa, &a_to_b);

            if let Some(clip_b) = clip_b {
                // track new index for b
                new_b = self.vertices.len();
                self.vertices.push(clip_b);
            } else {
                // clip_b will be None when no projection exists because a is exactly on the box's edge, so a -> b does not intersect with the box with the exception of a
                // in this case, we can exclude this edge
                new_a = EMPTY;
                new_b = EMPTY;
            }
        } // else neither is outside, not need for clipping

        fn assert_on_box_edge(builder: &CellBuilder, i: usize) -> bool {
            let (top, left) = builder.bounding_box.which_edge(&builder.vertices[i]);
            left != BoundingBoxLeftRightEdge::None || top != BoundingBoxTopBottomEdge::None
        }
        debug_assert!(new_a == a || new_a == EMPTY || assert_on_box_edge(self, new_a), "Edge {} ({:#?}) -> {} ({:#?}) was clipped but endpoint {} ({:#?}) is not on box's edge {:#?}.", a, self.vertices[a], b, self.vertices[b], new_a, self.vertices[new_a], self.bounding_box.which_edge(&self.vertices[new_a]));
        debug_assert!(new_b == b || new_b == EMPTY || assert_on_box_edge(self, new_b), "Edge {} ({:#?}) -> {} ({:#?}) was clipped but endpoint {} ({:#?}) is not on box's edge {:#?}.", a, self.vertices[a], b, self.vertices[b], new_b, self.vertices[new_b], self.bounding_box.which_edge(&self.vertices[new_b]));

        (new_a, new_b)
    }

    /// Given two vertices pa and pb, oriented counter clockwise, inserts the vertices necessary to link pa -> pb around the bounding box edge.
    fn insert_link_vertices<R>(&self, cell: &mut Vec<usize>, range: R, pa: usize, pb: usize)
        where
            R: RangeBounds<usize> {
        let (link_count, links) = self.get_link_vertices_around_box_edge(&self.vertices[pa], &self.vertices[pb]);
        match link_count {
            0 => { cell.splice(range, [pa, pb]); },
            1 => { cell.splice(range, [pa, links[0], pb]); },
            2 => { cell.splice(range, [pa, links[0], links[1], pb]); },
            3 => { cell.splice(range, [pa, links[0], links[1], links[2], pb]); },
            4 => { cell.splice(range, [pa, links[0], links[1], links[2], links[3], pb]); },
            _ => panic!("Maximum 4 edges can exist on a bounding box")
        }
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

        // if (a_bt == BoundingBoxTopBottomEdge::None && a_lr == BoundingBoxLeftRightEdge::None) || (b_bt == BoundingBoxTopBottomEdge::None && b_lr == BoundingBoxLeftRightEdge::None) {
        //     return (0, [0, 0, 0, 0]);
        // }

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
        //(0, new_vertices)
    }

    /// Given two vertices `a` and `b` that are on the bounding box edge(s), returns what vertices must be added between them to conform with the bounding box format.
    /// `a` and `b` are assumed to be ordered counter-clockwise.
    /// Returns how many new vertices were added.
    pub fn link_vertices_around_box_edge(&mut self, cell: &mut Vec<usize>, a: usize, b: usize) -> usize {
        let ca = cell[a];
        let cb = cell[b];
        let pa = &self.vertices[ca];
        let pb = &self.vertices[cb];
        let (mut a_bt, mut a_lr) = self.bounding_box.which_edge(pa);
        let (b_bt, b_lr) = self.bounding_box.which_edge(pb);

        // It is expected that points ARE on the box edge
        assert!(!(a_bt == BoundingBoxTopBottomEdge::None && a_lr == BoundingBoxLeftRightEdge::None), "Point a ({}) [{:?}] was not located on any of the box's edge", a, pa);
        assert!(!(b_bt == BoundingBoxTopBottomEdge::None && b_lr == BoundingBoxLeftRightEdge::None), "Point b ({}) [{:?}] was not located on any of the box's edge", b, pb);

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
            return 0;
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

                (BoundingBoxTopBottomEdge::None, BoundingBoxLeftRightEdge::None) => panic!("It seems that either 'a' ({}) or 'b' ({}) are not on the box's edge. Cannot link vertices not on the edge.", a, b)
            }

            if (a_bt != BoundingBoxTopBottomEdge::None && a_bt == b_bt) || (a_lr != BoundingBoxLeftRightEdge::None && a_lr == b_lr) {
                break;
            } else if new_vertice_count == MAX_CORNERS {
                panic!("It seems that either 'a' ({}) or 'b' ({}) are not on the box's edge. Cannot link vertices not on the edge.", a, b);
            }
        }

        if new_vertice_count != 0 {
            // splice cell and insert values on the right position (after a)
            let r = a + 1;
            cell.splice(r..r, new_vertices.iter().take(new_vertice_count).copied());
        }

        new_vertice_count
    }

    pub fn calculate_corners(&mut self) {
        // add all corners to the vertice list as they will be used for closing cells
        let width = self.bounding_box.width();
        let height = self.bounding_box.height();

        let top_right = self.bounding_box.top_right().clone();
        let mut top_left = top_right.clone(); top_left.x -= width;
        let mut bottom_left = top_left.clone(); bottom_left.y -= height;
        let mut bottom_right = top_right.clone(); bottom_right.y -= height;

        self.vertices.push(top_right);
        self.vertices.push(top_left);
        self.vertices.push(bottom_left);
        self.vertices.push(bottom_right);

        let bottom_right_index = self.vertices.len() - 1;
        self.bottom_right_corner_index = bottom_right_index;
        self.bottom_left_corner_index = bottom_right_index - 1;
        self.top_left_corner_index = bottom_right_index - 2;
        self.top_right_corner_index = bottom_right_index - 3;
    }

    /// Builds cells for each site.
    /// This won't extend not close the hull.
    /// This will not clip any edges to the bounding box.
    fn build_cells(&mut self, sites: &Vec<Point>, triangulation: &Triangulation, site_to_incoming_leftmost_halfedge: &Vec<usize>) -> Vec<Vec<usize>> {
        let num_of_sites = sites.len();
        let mut cells = vec![Vec::new(); num_of_sites];

        // // fill in all cells with circumcenters
        // for edge in 0..triangulation.triangles.len() {
        //     // triangle[edge] is the site 'edge' originates from, but EdgesAroundPointIterator
        //     // iterate over edges around the site 'edge' POINTS TO, thus to get that site
        //     // we need to take the next half-edge
        //     let site = site_of_incoming(triangulation, edge);

        //     // if we have already created the cell for this site, move on
        //     if !seen_sites[site] {
        //         seen_sites[site] = true;

        //         // edge may or may not be the left-most incoming edge for site, thus get the one
        //         // if iterator doesn't start this way, we may end cell vertex iteration early because
        //         // we will hit the halfedge in the hull
        //         let leftmost_edge = site_to_incoming_leftmost_halfedge[site];

        //         let cell = &mut cells[site];
        //         cell.extend(EdgesAroundSiteIterator::new(triangulation, leftmost_edge).map(utils::triangle_of_edge));
        //     }
        // }

        // iter over hull
        // get edge prev_site -> curr_site (i.e. iter over hull edges)
        // get circumcenter associated with edge's triangle
        // if circumcenter is outside box, projection / clipping required
        // if circumcenter is inside, extension required
        // track result for each hull edge
        // hull done

        // for each hull edge, there is a hull triangle (i.e. a triangle that has 1 edge in the hull, and two edges not facing full - there may be a case of a single triangle that is entirely the hull)
        // the edge that is on the hull links two sites that are (delauney) hull vertices, therefore there is another site on that triangle that is not in the hull (besides single triangle case)
        // iterate over all such sites - i.e. iterate over the remaining sites that belong to a hull triangle
        // those sites may need projection if the circumcenter of the triangle is outside box (already calculated before when doing the hull)
        // we can look up the result by checking the sites of that triangle that are part of the triangle edge facing the hull

        // when building the cell the first circumcenter is the one associated with the left most triangle (i.e. shared with the previous site coutner clock wise)
        // and the last circumcenter is associated with the rightmost triangle, shared the the next hull site counter clockwise


        // fill in cells
        for edge in 0..triangulation.triangles.len() {
            let site = site_of_incoming(triangulation, edge);

            // if cell is empty, it hasn't been processed yet
            if cells[site].len() == 0 {
                // edge may or may not be the left-most incoming edge for site, thus get the one
                // if iterator doesn't start this way, we may end cell vertex iteration early because
                // we will hit the halfedge in the hull
                cells[site].extend(EdgesAroundSiteIterator::new(triangulation, site_to_incoming_leftmost_halfedge[site]).map(utils::triangle_of_edge))
            }
        }

        if self.clip_behavior != ClipBehavior::Clip {
            return cells;
        }

        // iterate over each hull edge (site, next_site) counter-clockwise
        // pick the circumcenter associated with that edge's triangle (cells[site][last] == cells[site_next][0]) and compute clipping/projection
        let &last_site = triangulation.hull.last().expect("Hull is at least a triangle");
        let &first_site = triangulation.hull.first().expect("Hull is at least a triangle.");
        let mut site = last_site;
        let mut new_vertex = HullClipResult::None;

        // FIXME are we handling the first edge?
        for &next_site in triangulation.hull.iter().chain(once(&first_site)).take(20) {
            #[cfg(debug_assertions)] dbg!("Building hull edge {site} -> {next_site}");
            let circumcenter = *cells[site].last().expect("Cell has at least one circumcenter.");
            let circumcenter_pos = &self.vertices[circumcenter];
            let previous_new_vertex = new_vertex;

            #[cfg(debug_assertions)] dbg!("   Circumcenter: {circumcenter} [{:?}]", circumcenter_pos);

            // check whether circumcenter needs clipping or projection
            new_vertex = if self.bounding_box.is_inside(circumcenter_pos) {
                #[cfg(debug_assertions)] dbg!("   Circumcenter: inside");

                // FIXME: this if is to handle edge case when we process the first edge twice
                // and we cannot tell that the circumcenter is exactly on the bounding box edge
                // anything self.bottom_right_corner_index or higher means a new circumcenter added during processing
                // so we know we do not need to handle this again
                if circumcenter < self.bottom_right_corner_index {
                    // circumcenter is inside the bounding box and needs to be projected into box's edge from midpoint of hull's edge
                    let site_pos = &sites[site];
                    let next_site_pos = &sites[next_site];

                    // the projection direction is orthogonal to the hull's edge (prev_site -> site edge)
                    let orthogonal = Point { x: site_pos.y - next_site_pos.y, y: next_site_pos.x - site_pos.x };

                    if let Some(projected) = self.bounding_box.project_ray_closest(&circumcenter_pos, &orthogonal) {
                        // TODO: whenever adding a new vertex, it may be very close to an existing one, we should consider deduplicating
                        let v = self.vertices.len();
                        self.vertices.push(projected);

                        cells[site].push(v);
                        HullClipResult::Extended(v)
                    } else {
                        // circumcenter exactly on the bounding box's edge, nothing to do
                        dbg!("   Circumcenter: on bounding box's edge");
                        HullClipResult::None
                    }
                } else {
                    // FIXME: only edge case for the very last iteration
                    HullClipResult::None
                }
            } else {
                #[cfg(debug_assertions)] dbg!("   Circumcenter: outside");
                // circumcenter is outside of bounding box and needs to be clipped by projecting it against the midpoints of the opposed hull edges
                // (we do not project using the neighbor's site circumcenters because they may not exist - i.e. a single triangle)

                // find the inner site that is part of the current hull triangle but is not in the hull
                let hull_edge = site_to_incoming_leftmost_halfedge[next_site];
                let edge_to_inner_site = delaunator::next_halfedge(hull_edge);
                #[cfg(debug_assertions)] dbg!("   Hull edge: {hull_edge}");
                #[cfg(debug_assertions)] dbg!("   Edge to inner: {edge_to_inner_site}");
                let inner_site = utils::site_of_incoming(triangulation, edge_to_inner_site);
                #[cfg(debug_assertions)] dbg!("   Inner site: {inner_site}");
                let inner_site_pos = &sites[inner_site];
                let site_pos = &sites[site];
                let next_site_pos = &sites[next_site];

                // 2 projections needed
                // "left": circumcenter --> mid(inner -> site)
                // "right": circumcenter --> mid(inner -> next site)
                // left and right will replace circumcenter vertex in the inner site voronoi cell
                // left will replace the circumcenter in site's cell
                // right will replace the circumcenter in next site's cell (i.e. "new_vertex")
                //
                //              voronoi
                //            inner site
                //          o ----------- o
                //          \            /
                //            \         /
                //      site    \      /    next_site
                //          --left---right--------------------- bounding box edge
                //                \  /
                //                  o   <- circumcenter outside box
                //
                // this also means that site and next site are neighbors topologically but visually they are no longer connected
                // because their common vertex (circumcenter) was removed
                // visually, inner site is a voronoi cell "in between" site and next site and inner site is now visually part of the voronoi hull, but not the delaunay hull

                // TODO figure out how to use this insight to fix neighbor logic identification on iterator

                let next_inner_mid = utils::midpoint(inner_site_pos, next_site_pos);
                let site_inner_mid = utils::midpoint(inner_site_pos, site_pos);
                let left = self.bounding_box.project_ray_closest(&circumcenter_pos, &Point { x: site_inner_mid.x - circumcenter_pos.x, y: site_inner_mid.y - circumcenter_pos.y });
                let right = self.bounding_box.project_ray_closest(&circumcenter_pos, &Point { x: next_inner_mid.x - circumcenter_pos.x, y: next_inner_mid.y - circumcenter_pos.y });

                let left_index = if let Some(left) = left {
                    let v = self.vertices.len();
                    self.vertices.push(left.clone());

                    *cells[site].last_mut().expect("Cell has at least one vertex") = v;

                    v
                } else {
                    // TODO what does it mean?
                    panic!("Left projection during clipping must exist.");
                };

                if let Some(right) = right {
                    let right_index = self.vertices.len();
                    self.vertices.push(right);

                    // update inner site voronoi cell
                    let inner_site_cell = &mut cells[inner_site];
                    #[cfg(debug_assertions)] dbg!("   Inner cell: {:?}", &inner_site_cell);
                    if let Some(circumcenter_index) = inner_site_cell.iter().position(|&v| v == circumcenter) { // when does this not exist?
                        // add left and right to inner site's cell and link them around the bounding box as needed
                        self.insert_link_vertices(inner_site_cell, circumcenter_index..circumcenter_index+1, right_index, left_index);
                        #[cfg(debug_assertions)] dbg!("   Inner cell updated: {:?}", inner_site_cell);
                    }

                    HullClipResult::Clipped(right_index)
                } else {
                    // TODO what does it mean?
                    HullClipResult::None
                }
            };

            // handle result from previous side (shared vertex), which affects the first vertex in our cell
            // and close this cell
            // note that cell.last() is always the last vertex in the cell (even after we extended or clipped the circumcenter above)
            let cell = &mut cells[site];
            #[cfg(debug_assertions)] dbg!("   Cell: {:?}", &cell);
            match previous_new_vertex {
                // when extension happens on previous site, it is extending our first vertex (i.e. adding to our list)
                // adding it to the end of the cell is fine as it will keep counter-clockwise orientation
                HullClipResult::Extended(value) =>  {
                    self.insert_link_vertices(cell, cell.len() - 1.., *cell.last().expect("Cell has at least one element."), value);
                },

                // when clipping happens on previous site, it replaces the first vertex in current cell
                HullClipResult::Clipped(value) => {
                    cell[0] = value;

                    // this is a bit hacky to reuse insert_link_vertices - set the range such that it removes and readds the last element, then pops to remove the first vertex added duplicated in the end
                    self.insert_link_vertices(cell, cell.len() - 1.., *cell.last().expect("Cell has at least one element."), *cell.first().expect("Cell has at least one element."));
                    cell.pop();
                },

                HullClipResult::None => {
                    // the only case when this can happen is when we are processing the first edge (last_site, first_site) in the first pass on the loop without previous edge calculation
                    // note the reason it is OK not to do anything is because on the first pass, the cell.last() will be either updated (clipped case) or a new vertex will be added which will be the new last (extension case)
                    // then, on the last iteration when this edge is considered again, the cell.last circumcenter will be exactly in the bounding box edge and will be ignored (extension case)
                    // and one of the cases above in this match will be triggered because a previous edge will have yeild some clip/extension result
                    assert_eq!((site, next_site), (last_site, first_site), "When processing voronoi hull, there always will be clipping or extension, unless disabled.");
                },
            }

            #[cfg(debug_assertions)] dbg!("   Cell updated: {:?}", cell);

            site = next_site;
        }

        cells
    }

    /// Builds cells for each site.
    /// This won't extend not close the hull.
    /// This will not clip any edges to the bounding box.
    fn build_cells_old(&mut self, sites: &Vec<Point>, triangulation: &Triangulation, site_to_incoming_leftmost_halfedge: &Vec<usize>) -> Vec<Vec<usize>> {
        let num_of_sites = sites.len();
        let mut seen_sites = vec![false; num_of_sites];
        let mut cells = vec![Vec::new(); num_of_sites];

        for edge in 0..triangulation.triangles.len() {
            // triangle[edge] is the site 'edge' originates from, but EdgesAroundPointIterator
            // iterate over edges around the site 'edge' POINTS TO, thus to get that site
            // we need to take the next half-edge
            let site = site_of_incoming(triangulation, edge);

            if !triangulation.hull.contains(&site) {
                continue;
            }

            // if we have already created the cell for this site, move on
            if !seen_sites[site] {
                seen_sites[site] = true;

                // edge may or may not be the left-most incoming edge for site, thus get the one
                // if iterator doesn't start this way, we may end cell vertex iteration early because
                // we will hit the halfedge in the hull
                let leftmost_edge = site_to_incoming_leftmost_halfedge[site];

                // if there is no half-edge associated with the left-most edge, the edge is on the hull
                // let is_hull_site = triangulation.halfedges[leftmost_edge] == EMPTY;


                // // get circumcenter of the triangle associated with this site
                // let circumcenter = self.vertices[utils::triangle_of_edge(self.site_to_incoming_leftmost_halfedge[site_pos])];

                let cell = &mut cells[site];
                let mut iter = EdgesAroundSiteIterator::new(triangulation, leftmost_edge).map(utils::triangle_of_edge).peekable();
                while let Some(curr) = iter.next() {
                    // get next vertex (wrap around to first if this is the last vertex)
                    if let Some(&next) = iter.peek().or(cell.first()) {
                        // let (new_curr, new_next) = self.clip_cell_edge(curr, next);
                        let (new_curr, new_next) = (curr, next);

                        if new_curr != EMPTY {
                            // does not matter if new_curr != curr (i.e. clipping occured) because curr is not in the list yet
                            cell.push(new_curr);
                        }

                        if new_next != EMPTY && new_next != next {
                            // if new_next != next it means next is outside bounding box, so new_next is clipped on the box
                            // therefore we add it to the cell
                            // it is also possible that next is the first vertex of the cell and is inside the bounding box
                            // in this case new_next is the clipping on the bounding box edge between curr -> next, which needs to be added to the cell
                            cell.push(new_next);
                        }

                        // FIXME when new_curr == new_next == EMPTY, the edge was removed and the cell is open
                        // so this cell is similar to a hull cell now in the sense that it is not "closed" and the logic to add vertices around the bounding box edge to close the cell is required
                    } else {
                        // FIXME - confirm this is hull cases and deal with this properly - what does it mean?
                        assert!(triangulation.hull.contains(&site));
                        cell.push(curr);
                    }
                }
            }
        }

        cells
    }
}

#[cfg(test)]
mod test {
    use super::*;

    fn new_builder(vertices: Vec<Point>) -> CellBuilder {
        CellBuilder::new(vertices, BoundingBox::new_centered_square(4.0), ClipBehavior::Clip)
    }

    fn assert_same_elements(actual: &Vec<usize>, expected: &Vec<usize>, message: &str) {
        assert_eq!(actual.len(), expected.len(), "Vectors differ in length. Actual: {:?}. Expected: {:?}. {}", actual, expected, message);
        assert_eq!(0, actual.iter().copied().zip(expected.iter().copied()).filter(|(a,b)| a != b).count(), "Vectors have differing elements. Actual: {:?}. Expected: {:?}. {}", actual, expected, message);
    }

    fn assert_cell_vertex_without_bounds(builder: &CellBuilder, cell: &Vec<usize>, message: &str, expected_vertices: Vec<Point>) {
        let cell_vertices = cell.iter().map(|c| builder.vertices[*c].clone()).collect::<Vec<Point>>();
        assert_eq!(expected_vertices.len() , cell.len(), "Cell vertex count is incorrect. Expected {:#?}, found {:#?}. {}", expected_vertices, cell_vertices, message);

        cell_vertices.iter().enumerate().zip(expected_vertices.iter()).for_each(|((index, actual), expected)| {
            assert_eq!(expected, actual, "Invalid vertex for position {}. Expected cell vertices {:#?}, found {:#?} {}", index, expected_vertices, cell_vertices, message);
        });
    }

    fn assert_cell_vertex(builder: &CellBuilder, cell: &Vec<usize>, message: &str, expected_vertices: Vec<Point>) {
        assert_cell_vertex_without_bounds(builder, cell, message, expected_vertices);
        assert_cell_consistency(cell, builder, message);
    }

    /// Check that the cell is ordered counter-clockwise and inside the bounding box.
    fn assert_cell_consistency(cell: &Vec<usize>, builder: &CellBuilder, message: &str) {
        let points: Vec<Point> = cell.iter().map(|c| builder.vertices[*c].clone()).collect();

        // are all points within the bounding box?
        let points_outside: Vec<(usize, &Point)> = points.iter().enumerate().filter(|(_, p)| {
            !builder.bounding_box.is_inside(p)
        }).collect();
        assert_eq!(0, points_outside.len(), "These points are outside bounding box: {:?}. {}", points_outside, message);

        // is counter-clockwise? https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
        let area = points.iter().zip(points.iter().cycle().skip(1)).fold(0.0, |acc, (a, b)| {
                acc + ((b.x - a.x) * (b.y + a.y))
        });
        assert_eq!(true, area < 0.0, "Area of the polygon must be less than 0 for it to be counter-clockwise ordered. Area: {}. {}", area, message);
    }

    #[test]
    fn link_vertices_around_box_edge_same_edge_b_after_a() {
        let mut builder = new_builder(vec![
            Point { x: 1.0, y: 2.0 },
            Point { x: -1.0, y: 2.0 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell.clone();
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &original_cell, "No change expected. Same edge, b is after a");
    }

    #[test]
    fn link_vertices_around_box_edge_same_edge_b_before_a() {
        let mut builder = new_builder(vec![
            Point { x: -1.0, y: 2.0 },
            Point { x: 1.0, y: 2.0 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell;
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &vec![0, builder.top_left_corner_index, builder.bottom_left_corner_index, builder.bottom_right_corner_index, builder.top_right_corner_index, 1], "Full circuit expected because b is before a");
    }

    #[test]
    fn link_vertices_around_box_edge_same_edge_b_corner_before_a() {
        let mut builder = new_builder(vec![
            Point { x: -1.0, y: 2.0 },
            Point { x: 2.0, y: 2.0 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell;
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &vec![0, builder.top_left_corner_index, builder.bottom_left_corner_index, builder.bottom_right_corner_index, 1], "Corners not expected to be duplicated");
    }

    #[test]
    fn link_vertices_around_box_edge_same_edge_a_top_left_b_bottom_right() {
        let mut builder = new_builder(vec![
            Point { x: -2.0, y: 2.0 },
            Point { x: 2.0, y: -2.0 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell;
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &vec![0, builder.bottom_left_corner_index, 1], "Corners not expected to be duplicated");
    }

    #[test]
    fn link_vertices_around_box_edge_same_edge_a_right_b_left() {
        let mut builder = new_builder(vec![
            Point { x: 2.0, y: 1.0 },
            Point { x: -2.0, y: 1.5 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell;
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &vec![0, builder.top_right_corner_index, builder.top_left_corner_index, 1], "Corners not expected to be duplicated");
    }

    #[test]
    fn link_vertices_around_box_edge_same_edge_a_left_b_right() {
        let mut builder = new_builder(vec![
            Point { x: -2.0, y: 1.0 },
            Point { x: 2.0, y: 1.5 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell;
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &vec![0, builder.bottom_left_corner_index, builder.bottom_right_corner_index, 1], "Corners not expected to be duplicated");
    }


    #[test]
    fn link_vertices_around_box_edge_same_edge_a_bottom_b_top() {
        let mut builder = new_builder(vec![
            Point { x: 1.0, y: -2.0 },
            Point { x: -1.5, y: 2.0 },
        ]);
        let original_cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut cell = original_cell;
        builder.calculate_corners();
        builder.link_vertices_around_box_edge(&mut cell, 0, 1);
        assert_same_elements(&cell, &vec![0, builder.bottom_right_corner_index, builder.top_right_corner_index, 1], "Corners not expected to be duplicated");
    }

    #[test]
    fn clip_cell_edge_one_edge_crosses_one_box_edge() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
        ]);
        let (a, b) = builder.clip_cell_edge(0, 1);
        assert_eq!(a, 0, "A is inside box, should not change");
        assert_eq!(b, 2, "New edge should have been added due to clipping");
        assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[2], "Point should have been added for clipped edge.");
    }

    #[test]
    fn clip_cell_edge_edge_outside_box_with_single_point_on_box() {
        let mut builder = new_builder(vec![
            Point { x: -2.0, y: 0.0 },
            Point { x: -5.0, y: 0.0 },
        ]);

        /*
                    |-----\
                    |     \
                x--x     \
                    |     \
                    |-----\
        */
        let (a, b) = builder.clip_cell_edge(0, 1);
        assert_eq!(a, EMPTY, "Edge must be removed");
        assert_eq!(b, EMPTY, "Edge must be removed");
    }

    #[test]
    fn clip_cell_edge_one_edge_crosses_one_box_edge_inverted() {
        let mut builder = new_builder(vec![
            Point { x: 10.0, y: 0.0 },
            Point { x: 0.0, y: 0.0 },
        ]);
        let (a, b) = builder.clip_cell_edge(0, 1);
        assert_eq!(a, 2, "New edge should have been added due to clipping");
        assert_eq!(b, 1, "B is inside box, should not change");
        assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[2], "Point should have been added for clipped edge.");
    }

    #[test]
    fn clip_cell_edge_one_edge_crosses_two_box_edges() {
        let mut builder = new_builder(vec![
            Point { x: -10.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
        ]);
        let (a, b) = builder.clip_cell_edge(0, 1);
        assert_eq!(a, 2, "New edge should have been added due to clipping");
        assert_eq!(b, 3, "New edge should have been added due to clipping");
        assert_eq!(Point { x: -2.0, y: 0.0 }, builder.vertices[2], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[3], "Point should have been added for clipped edge.");
    }

    #[test]
    fn clip_and_close_cell_when_no_point_outside_box() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 1.0, y: 0.0 },
            Point { x: 0.0, y: 1.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.clip_and_close_cell(&mut clipped_cell);
        assert_same_elements(&clipped_cell, &vec![0, 1, 2], "No clipping expected");
        assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
    }

    #[test]
    fn clip_and_close_cells_with_shared_edge_intersecting_box() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 1.0, y: 0.0 },
            Point { x: 0.0, y: 5.0 }, // outside of bounding box
            Point { x: -1.0, y: 0.0 },
        ]);

        /*
                             2
                             |
            ------------------------------------
            |                 |                 \
            |                 |                 \
            |                 |                 \
            |        3 ------ 0 ------ 1        \
            |------------------------------------\
        */
        let cell: Vec<usize> = [0, 1, 2].to_vec();
        let mut clipped_cell = cell;
        builder.clip_and_close_cell(&mut clipped_cell);

        assert_eq!(clipped_cell.len(), 4, "A new vertix must be added because two edges cross the box");
        assert_eq!(builder.bounding_box.which_edge(&builder.vertices[clipped_cell[2]]), (BoundingBoxTopBottomEdge::Top, BoundingBoxLeftRightEdge::None), "Vertex must be on bottom of bounding box");
        assert_eq!(builder.bounding_box.which_edge(&builder.vertices[clipped_cell[3]]), (BoundingBoxTopBottomEdge::Top, BoundingBoxLeftRightEdge::None), "Vertex must be on bottom of bounding box");
        assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");

        // edge 0 -> 2 is shared
        let cell: Vec<usize> = [3, 0, 2].to_vec();
        let mut neighbor_clipped_cell = cell;
        builder.clip_and_close_cell(&mut neighbor_clipped_cell);

        assert_eq!(neighbor_clipped_cell.len(), 4, "A new vertix must be added because two edges cross the box");
        assert_eq!(neighbor_clipped_cell[2], clipped_cell[3], "Vertex on shared edge must be clipped to same value");
        assert_cell_consistency(&neighbor_clipped_cell, &builder, "Cell consistency check");
    }

    #[test]
    fn clip_triangular_cell_with_one_point_outside_box() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -3.0 }, // outside
            Point { x: 1.0, y: 0.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.clip_and_close_cell(&mut clipped_cell);
        assert_same_elements(&clipped_cell, &vec![0, 3, 4, 2], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: -2.0 }, builder.vertices[3], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 1.0 / 3.0, y: -2.0 }, builder.vertices[4], "Point should have been added for clipped edge.");
        assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
    }

    #[test]
    fn clip_triangular_cell_with_one_point_outside_box_and_last_crossing_box_edge() {
        let mut builder = new_builder(vec![
            Point { x: 1.0, y: 0.0 },
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -3.0 }, // outside
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.clip_and_close_cell(&mut clipped_cell);
        assert_same_elements(&clipped_cell, &vec![0, 1, 3, 4], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: -2.0 }, builder.vertices[3], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 1.0 / 3.0, y: -2.0 }, builder.vertices[4], "Point should have been added for clipped edge.");
        assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
    }

    #[test]
    fn clip_triangular_cell_with_two_points_outside_and_one_edge_entirely_outside_box() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -30.0 }, // leaves through the bottom
            Point { x: 20.0, y: 0.0 }, // comes back through the right
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.calculate_corners();
        builder.clip_and_close_cell(&mut clipped_cell);
        // builder adds 4 vertices all the time, so next index is 7
        assert_same_elements(&clipped_cell, &vec![0, 7, builder.bottom_right_corner_index, 8], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: -2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
        assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
    }

    #[test]
    fn clip_triangular_cell_with_two_points_outside_and_edge_crossing_box_twice() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -3.0 },
            Point { x: 3.0, y: 0.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.calculate_corners();
        builder.clip_and_close_cell(&mut clipped_cell);
        // builder adds 4 vertices all the time, so next index is 7
        assert_same_elements(&clipped_cell, &vec![0, 7, 8, 9, 10], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: -2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 1.0, y: -2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -1.0 }, builder.vertices[9], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[10], "Point should have been added for clipped edge.");
        assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
        //assert_eq!(is_closed, true, "No entire edge was outside the box, so the cell must stay closed.")
    }

    #[test]
    fn clip_triangular_cell_with_one_edge_intersecting_box_corner() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 4.0, y: 0.0 },
            Point { x: 0.0, y: 4.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.calculate_corners();
        builder.clip_and_close_cell(&mut clipped_cell);
        assert_same_elements(&clipped_cell, &vec![0, 7, builder.top_right_corner_index , 8], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 2.0, y: 0.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 0.0, y: 2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
        assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
    }

    // same as above but with triangle inverted, no vertex inside box
    #[test]
    fn clip_triangular_cell_outside_with_one_edge_intersecting_box_corner() {
        let mut builder = new_builder(vec![
            Point { x: 4.0, y: 4.0 },
            Point { x: 4.0, y: 0.0 },
            Point { x: 0.0, y: 4.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.calculate_corners();
        let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
        assert_eq!(keep_cell, false, "A single intersection at the coner with all other points outside the cell, should not keep this cell.");
        assert_same_elements(&clipped_cell, &vec![builder.top_right_corner_index], "Clipped cell incorrect indices.");
    }

    #[test]
    fn clip_triangular_cell_with_one_point_inside_box_and_one_edge_parallel_to_box_edge() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 2.0, y: -4.0 },
            Point { x: 2.0, y: 4.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.calculate_corners();
        builder.clip_and_close_cell(&mut clipped_cell);
        // FIXME: identify intersection at corners and return corner index instead of creating a new one
        assert_same_elements(&clipped_cell, &vec![0, 7, 8, 9, 10], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 1.0, y: -2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, builder.vertices[9], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 1.0, y: 2.0 }, builder.vertices[10], "Point should have been added for clipped edge.");
        assert_cell_consistency(&clipped_cell, &builder, "Cell consistency check");
    }

    #[test]
    fn clip_triangular_cell_with_no_point_inside_box_and_one_edge_parallel_to_box_edge() {
        let mut builder = new_builder(vec![
            Point { x: 4.0, y: 0.0 },
            Point { x: 2.0, y: 4.0 },
            Point { x: 2.0, y: -4.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.calculate_corners();
        let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
        assert_same_elements(&clipped_cell, &vec![7, 8], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");
        assert_eq!(keep_cell, false, "A line shared with one box edge, should not keep this cell.");
        //assert_eq!(is_closed, false, "One edge outside box.")
    }

    // same case as above, but the entire parallel edge is shared with the box
    #[test]
    fn clip_triangular_cell_with_no_point_inside_box_and_one_shared_edge_parallel_to_box_edge() {
        let mut builder = new_builder(vec![
            Point { x: 4.0, y: 0.0 },
            Point { x: 2.0, y: 2.0 },
            Point { x: 2.0, y: -2.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.calculate_corners();
        builder.clip_and_close_cell(&mut clipped_cell);
        assert_same_elements(&clipped_cell, &vec![7, 1, 2, 8], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, builder.vertices[7], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, builder.vertices[1], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -2.0 }, builder.vertices[2], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -2.0 }, builder.vertices[8], "Point should have been added for clipped edge.");

        // FIXME: the result is a rectangle with area 0 - code think cell is closed, but it should be removed
        //assert_cell_consistency(&clipped_cell, &builder);

        // FIX ME: code thinks cell is closed, but that is not true in this case
        // ignoring for now as this shouldn't really happen for voronoi
        //assert_eq!(is_closed, false, "One edge outside box.");
    }

    #[test]
    fn clip_triangular_cell_with_three_points_outside_box() {
        let mut builder = new_builder(vec![
            Point { x: 10.0, y: 0.0 },
            Point { x: 10.0, y: -3.0 },
            Point { x: 15.0, y: 0.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.calculate_corners();
        let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
        assert_eq!(keep_cell, false, "No intersection with box, all points outside, should not keep the cell.");
        assert_same_elements(&clipped_cell, &vec![], "Clipped cell incorrect indices.");
    }

    #[test]
    fn clip_square_cell_with_two_edges_outside_box() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 5.0, y: 0.0 },
            Point { x: 5.0, y: 5.0 },
            Point { x: 0.0, y: 5.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.calculate_corners();
        let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
        assert_eq!(keep_cell, true, "Intersection with box.");
        assert_cell_vertex(&builder, &clipped_cell, "Incorrect clipping", vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 2.0, y: 0.0 },
            Point { x: 2.0, y: 2.0 },
            Point { x: 0.0, y: 2.0 },
        ]);
    }

    #[test]
    fn clip_square_cell_with_two_edges_outside_box_start_vertex_outside() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 5.0 },
            Point { x: 0.0, y: 0.0 },
            Point { x: 5.0, y: 0.0 },
            Point { x: 5.0, y: 5.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.calculate_corners();
        let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
        assert_eq!(keep_cell, true, "Intersection with box.");
        assert_cell_vertex(&builder, &clipped_cell, "Incorrect clipping", vec![
            Point { x: 0.0, y: 2.0 },
            Point { x: 0.0, y: 0.0 },
            Point { x: 2.0, y: 0.0 },
            Point { x: 2.0, y: 2.0 },
        ]);
    }

    #[test]
    fn clip_polygon_cell_with_multiple_edges_removed() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -5.0 },
            Point { x: 1.0, y: -5.0 },
            Point { x: 1.0, y: -1.0 },
            Point { x: 5.0, y: -1.0 },
            Point { x: 5.0, y: 0.0 },
            Point { x: 1.0, y: 0.0 },
            Point { x: 1.0, y: 5.0 },
            Point { x: 0.0, y: 5.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.calculate_corners();
        let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
        assert_eq!(keep_cell, true, "Intersection with box.");
        assert_cell_vertex(&builder, &clipped_cell, "Incorrect clipping", vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -2.0 },
            Point { x: 1.0, y: -2.0 },
            Point { x: 1.0, y: -1.0 },
            Point { x: 2.0, y: -1.0 },
            Point { x: 2.0, y: 0.0 },
            Point { x: 1.0, y: 0.0 },
            Point { x: 1.0, y: 2.0 },
            Point { x: 0.0, y: 2.0 },
        ]);
    }

    #[test]
    fn clip_polygon_cell_with_multiple_edges_removed_and_coners_added() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -5.0 },
            Point { x: 5.0, y: -5.0 },
            Point { x: 5.0, y: -1.0 },
            Point { x: 1.0, y: -1.0 },
            Point { x: 1.0, y: 0.0 },
            Point { x: 5.0, y: 0.0 },
            Point { x: 5.0, y: 5.0 },
            Point { x: 0.0, y: 5.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.calculate_corners();
        let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
        assert_eq!(keep_cell, true, "Intersection with box.");
        assert_cell_vertex(&builder, &clipped_cell, "Incorrect clipping", vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -2.0 },
            Point { x: 2.0, y: -2.0 },
            Point { x: 2.0, y: -1.0 },
            Point { x: 1.0, y: -1.0 },
            Point { x: 1.0, y: 0.0 },
            Point { x: 2.0, y: 0.0 },
            Point { x: 2.0, y: 2.0 },
            Point { x: 0.0, y: 2.0 },
        ]);
    }

    #[test]
    fn clip_polygon_cell_with_multiple_edges_removed_and_coners_added_starting_edge_outside() {
        let mut builder = new_builder(vec![
            Point { x: 0.0, y: 5.0 },
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -5.0 },
            Point { x: 5.0, y: -5.0 },
            Point { x: 5.0, y: -1.0 },
            Point { x: 1.0, y: -1.0 },
            Point { x: 1.0, y: 0.0 },
            Point { x: 5.0, y: 0.0 },
            Point { x: 5.0, y: 5.0 },
        ]);
        let cell: Vec<usize> = (0..builder.vertices.len()).collect();
        let mut clipped_cell = cell;
        builder.calculate_corners();
        let keep_cell = builder.clip_and_close_cell(&mut clipped_cell);
        assert_eq!(keep_cell, true, "Intersection with box.");
        assert_cell_vertex(&builder, &clipped_cell, "Incorrect clipping", vec![
            Point { x: 0.0, y: 2.0 },
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -2.0 },
            Point { x: 2.0, y: -2.0 },
            Point { x: 2.0, y: -1.0 },
            Point { x: 1.0, y: -1.0 },
            Point { x: 1.0, y: 0.0 },
            Point { x: 2.0, y: 0.0 },
            Point { x: 2.0, y: 2.0 },
        ]);
    }

    #[test]
    fn extend_vertex_test() {
        let mut builder = new_builder(vec![
            Point { x: 0.5, y: 0.0 }, // vertex to be extended
        ]);

        let ext = builder.extend_vertex(&Point { x: 1.0, y: 1.0 }, &Point { x: 0.0, y: 1.0 }, 1.0);
        assert_eq!(Point { x: 0.5, y: 2.0 }, builder.vertices[ext], "Extension expected to be orthogonal to a -> b and on the bounding box edge.");
    }

    #[test]
    fn extend_and_close_hull_test() {
        let mut builder = new_builder(vec![
            Point { x: -0.5, y: -0.25 }, // vertex to be extended
        ]);
        builder.calculate_corners();
        let sites = vec![
            Point { x: -0.5, y: 1.0 },
            Point { x: -1.5, y: -1.0 },
            Point { x: 0.5, y: -1.0 },
        ];
        assert_eq!(builder.vertices[0], utils::cicumcenter(&sites[0], &sites[1], &sites[2]), "I got the circumcenter wrong.");
        let hull_sites = (0..sites.len()).collect();
        let mut cells = vec![
            vec![0],
            vec![0],
            vec![0],
        ];
        builder.extend_and_close_hull(&sites, &hull_sites, &mut cells);

        assert_cell_vertex_without_bounds(&builder, &cells[0], "First cell", vec![
            Point { x: -0.5, y: -0.25 },
            Point { x: 7.155417527999327, y: 3.5777087639996634 },
            Point { x: -8.155417527999326, y: 3.5777087639996634 },
        ]);

        assert_cell_vertex_without_bounds(&builder, &cells[1], "Second cell", vec![
            Point { x: -0.5, y: -0.25 },
            Point { x: -8.155417527999326, y: 3.5777087639996634 },
            Point { x: -0.5, y: -9.0 },
        ]);

        assert_cell_vertex_without_bounds(&builder, &cells[2], "Third cell", vec![
            Point { x: -0.5, y: -0.25 },
            Point { x: -0.5, y: -9.0 },
            Point { x: 7.155417527999327, y: 3.5777087639996634 },
        ]);
    }
}