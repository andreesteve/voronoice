use delaunator::{EMPTY, next_halfedge, Triangulation};
use crate::{utils::{triangle_of_edge}, BoundingBox, bounding_box};
use super::{ClipBehavior, Point, iterator::EdgesAroundSiteIterator, utils::{self, site_of_incoming}};

const VORONOI_INFINITY: f64 = 1e+10_f64;

#[derive(Debug)]
pub struct CellBuilder<'t> {
    triangulation: &'t Triangulation,
    sites: &'t Vec<Point>,
    vertices: Vec<Point>,
    site_to_incoming_leftmost_halfedge: Vec<usize>,
    corner_ownership: Vec<usize>,
    is_vertex_inside_bounding_box: Vec<bool>,
    bounding_box: BoundingBox,
    clip_behavior: ClipBehavior,
    first_corner_index: usize,
    number_of_circumcenters: usize,
}

pub struct CellBuilderResult {
    pub cells: Vec<Vec<usize>>,
    pub vertices: Vec<Point>,
    pub site_to_incoming_leftmost_halfedge: Vec<usize>,
}

impl<'t> CellBuilder<'t> {
    pub fn new(triangulation: &'t Triangulation, sites: &'t Vec<Point>, vertices: Vec<Point>, bounding_box: BoundingBox, clip_behavior: ClipBehavior) -> Self {
        let site_to_incoming_leftmost_halfedge = calculate_incoming_edges(triangulation, sites.len());
        let is_vertex_inside_bounding_box: Vec<bool> = vertices.iter().map(|c| bounding_box.is_inside(c)).collect();

        let corner_ownership = if clip_behavior == ClipBehavior::Clip {
            calculate_corner_ownership(&bounding_box.corners(), &triangulation, sites, &site_to_incoming_leftmost_halfedge)
        } else {
            Vec::with_capacity(0)
        };

        Self {
            triangulation,
            sites,
            site_to_incoming_leftmost_halfedge,
            is_vertex_inside_bounding_box,
            corner_ownership,
            first_corner_index: 0,
            number_of_circumcenters: vertices.len(),
            vertices,
            bounding_box,
            clip_behavior
        }
    }

    pub fn build(mut self) -> CellBuilderResult {
        // adds the corners of the bounding box as potential vertices for the voronoi
        if self.clip_behavior == ClipBehavior::Clip {
            self.calculate_corners();
        }

        let cells = self.build_cells();

        CellBuilderResult {
            vertices: self.vertices,
            site_to_incoming_leftmost_halfedge: self.site_to_incoming_leftmost_halfedge,
            cells
        }
    }

    pub fn calculate_corners(&mut self) {
        // add all corners to the vertice list as they will be used for closing cells

        self.first_corner_index = self.vertices.len();
        for corner in self.bounding_box.corners() {
            self.vertices.push(corner);
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
        let mut tmp_cell = Vec::new();

        if self.clip_behavior == ClipBehavior::Clip {
            // For each hull edge a->b, extend its associated circumcenter vertex beyond the bounding box
            // Add this extended vertex to its cell, and the hull previous cell that shares the same extended vertex
            for (&a, &b) in triangulation.hull.iter().zip(triangulation.hull.iter().cycle().skip(1)) {
                let hull_edge = self.site_to_incoming_leftmost_halfedge[b];
                let extension = self.extend_voronoi_vertex(hull_edge);

                // add extension as first vertex in incoming site
                cells[b].push(extension);
                cells[b].extend(EdgesAroundSiteIterator::new(triangulation, hull_edge).map(utils::triangle_of_edge));

                // add extension as last vertex in outgoing site
                cells[a].push(extension);
            }

            // Clip hull sites
            for &hull_site in &triangulation.hull {
                let hull_cell = &mut cells[hull_site];
                tmp_cell.clear();
                std::mem::swap(hull_cell, &mut tmp_cell);
                self.clip_cell(&tmp_cell, hull_cell, hull_site);
            }
        }

        // Build and clip remaining cells
        for edge in 0..triangulation.triangles.len() {
            // note: because we are not processing any hull sites here, all edges will be incoming edge of some site
            let site = site_of_incoming(triangulation, edge);
            let cell = &mut cells[site];

            // if cell is empty, it hasn't been processed yet
            if cell.len() == 0 {
                #[cfg(debug_logs)] println!();
                #[cfg(debug_logs)] println!("Site: {site}.");

                let circumcenter_iter = EdgesAroundSiteIterator::new(triangulation, edge).map(utils::triangle_of_edge);
                if self.clip_behavior == ClipBehavior::Clip {
                    tmp_cell.clear();
                    tmp_cell.extend(circumcenter_iter);
                    self.clip_cell(&tmp_cell, cell, site);
                } else {
                    cell.extend(circumcenter_iter);
                    cell.reverse();
                }

                #[cfg(debug_logs)] println!("  [{site}/{edge}] Cell: {:?}", cell);
            }
        }

        cells
    }

    fn clip_cell(&mut self, tmp_cell: &Vec<usize>, cell: &mut Vec<usize>, site: usize) {
        #[cfg(debug_logs)] println!("  Temp: {:?}", tmp_cell);

        // find the first vertex inside the bounding box
        let (first_index, first, first_inside) = if let Some(inside) = tmp_cell.iter().enumerate().filter(|(_, &c)| self.is_vertex_inside_bounding_box(c)).next() {
            (inside.0, *inside.1, true)
        } else {
            // when cell is entirely outside the bounding box, it will always fall into the loop case below that the edge needs to be clipped
            (0, tmp_cell[0], false)
        };

        // walk voronoi edges counter clockwise and clip as needed
        let mut prev = first;
        let mut prev_inside = first_inside;
        let mut cell_open = false; // everytime we leave the cell, we leave it open and we need to close it
        #[cfg(debug_logs)] println!("  Start: Temp[{first_index}] = {first}.");
        for &c in tmp_cell.iter().rev().cycle().skip(tmp_cell.len() - first_index).take(tmp_cell.len()) {
            let inside = self.is_vertex_inside_bounding_box(c);

            match (prev_inside, inside) {
                // voronoi edge has both vertices outside bounding box
                (false, false) => {
                    // but may cross the box, so check for an intersection
                    let (first_clip, second_clip) = self.clip_voronoi_edge(prev, c);
                    if let Some(first_clip) = first_clip {
                        if cell_open {
                            // an edge crossing the box still leaves the cell open, but it may be able to wrap around a corner
                            // this is an edge case, see degerated10.json input
                            self.insert_edge_and_wrap_around_corners(site, cell,
                                *cell.last().expect("Cell must not be empty because we started from a vertex inside the bounding box."),
                                first_clip);
                            #[cfg(debug_logs)] println!("  [{site}] Edge {prev} -> {c}. Edge outside box. The box was open.");
                        }
                        #[cfg(debug_logs)] println!("  [{site}] Edge {prev} -> {c}. First clip: {first_clip}. Second clip: {}", second_clip.unwrap_or_default());
                        self.insert_edge_and_wrap_around_corners(site, cell,
                           first_clip,
                second_clip.expect("Two intersection points need to occur when a line crosses the bounding box"));
                        #[cfg(debug_logs)] println!("  [{site}] Edge {prev} -> {c}. Edge outside box. Entered at {} and left at {}", first_clip, second_clip.unwrap());
                    } else {
                        #[cfg(debug_logs)] println!("  [{site}] Edge {prev} -> {c}. Edge outside box, no intersection.");
                    }
                },

                // entering bounding box - edge crosses bounding box edge from the outside
                (false, true) => {
                    let (first_clip, second_clip) = self.clip_voronoi_edge(c, prev);
                    let first_clip = first_clip.expect("Edge crosses box, intersection must exist.");
                    debug_assert!(second_clip.is_none(), "Cannot have two intersections with the bounding box when one of the edge's vertex is inside the bounding box");
                    self.insert_edge_and_wrap_around_corners(site, cell,
                        *cell.last().expect("Cell must not be empty because we started from a vertex inside the bounding box."),
                       first_clip);
                    cell_open = false;
                    #[cfg(debug_logs)] println!("  [{site}] Edge {prev} -> {c}: Entering box at {first_clip} (previously left from {})");
                },

                // leaving bounding box - edge crosses bounding box edge from the inside
                (true, false) => {
                    let (first_clip, second_clip) = self.clip_voronoi_edge(prev, c);
                    let first_clip = first_clip.expect("Edge crosses box, intersection must exist.");
                    debug_assert!(second_clip.is_none(), "Cannot have two intersections with the bounding box when one of the edge's vertex is inside the bounding box");

                    cell.push(prev);
                    cell.push(first_clip);
                    cell_open = true;
                    #[cfg(debug_logs)] println!("  [{site}] Edge {prev} -> {c}: Leaving box. Added {prev} and clipped at {}", cell.last().unwrap());
                },

                // edge inside box
                (true, true) => {
                    #[cfg(debug_logs)] println!("  [{site}] Edge {prev} -> {c}: Inside box. Added {prev}.");
                    cell.push(prev);
                },
            }

            prev = c;
            prev_inside = inside;
        }
    }

    /// Clip a voronoi edge on the bounding box's edge, if the edge crosses the bounding box.
    ///
    /// The edge may cross the bounding box once (when ```a``` is inside the box) or twice (when ```a``` and ```b``` are outside the box).
    /// Returns up to two indexes to the new vertex where the clip has occured.
    /// * edge is oriented a -> b, i.e. a comes before b
    fn clip_voronoi_edge(&mut self, a: usize, b: usize) -> (Option<usize>, Option<usize>) {
        let a_pos = &self.vertices[a];
        let b_pos = &self.vertices[b];
        let a_to_b = &Point { x: b_pos.x - a_pos.x, y: b_pos.y - a_pos.y };

        let result = match self.bounding_box.project_ray(a_pos, a_to_b) {
            // single intersection (i.e a is inside bounding box and b is outside)
            (Some(first_clip), None) => {
                // insert intersection and return
                (Some(self.add_new_vertex(first_clip)), None)
            },

            // two intersecting points (i.e. a and b are outside bounding box but a->b cross bounding box)
            (Some(first_clip), Some(second_clip)) => {
                // it is possible that a -> b intersects the bounding box in a point after b, i.e. the edge does not cross the box
                // check if first_clip comes before b
                if bounding_box::order_points_on_ray(a_pos, a_to_b, Some(b_pos.clone()), Some(first_clip.clone())).0 == Some(first_clip.clone()) {
                    (Some(self.add_new_vertex(first_clip)), Some(self.add_new_vertex(second_clip)))
                } else {
                    // a -> b -> box: no intersection
                    (None, None)
                }
            },

            // invalid case
            (None, Some(_)) => panic!("project_ray must not return second intersection without returning the first intersection"),

            // no intersection
            (None, None) => (None, None),
        };

        result
    }

    /// Given two vertices on the bounding box boundary (first clip and second clip) check whether there is a need to add the bounding box corners in the cell
    fn insert_edge_and_wrap_around_corners(&mut self, site: usize, cell: &mut Vec<usize>, first_clip: usize, second_clip: usize) {
        if cell.last() != Some(&first_clip) {
            cell.push(first_clip);
        }

        let first_edge = self.bounding_box.which_edge(&self.vertices[first_clip]).expect("First clipped value is expected to be on the edge of the bounding box.");
        let second_edge = self.bounding_box.which_edge(&self.vertices[second_clip]).expect("Second clipped value is expected to be on the edge of the bounding box.");
        #[cfg(debug_logs)] let len = cell.len();

        // first_edge is to the right of first_clip -> second_clip
        // second_edge is to the left of first_clip -> second_clip
        if self.corner_ownership[first_edge] == site {
            // first_edge and all corners counter clockwise need to be added to this cell
            let mut edge = first_edge;
            while edge != second_edge && self.corner_ownership[edge] == site {
                cell.push(self.first_corner_index + edge);
                self.corner_ownership[edge] = EMPTY; // prevent another edge from duplicating this corner in the cell
                edge = bounding_box::next_edge(edge);
            }
            cell.push(second_clip);
        } else if self.corner_ownership[second_edge] == site {
            // second_edge and all corners clockwise need to be added to this cell
            cell.push(second_clip);
            let mut edge = second_edge;
            while edge != first_edge && self.corner_ownership[edge] == site {
                cell.push(self.first_corner_index + edge);
                self.corner_ownership[edge] = EMPTY;
                edge = bounding_box::next_edge(edge);
            }
        } else {
            // line first_clip -> second_clip crosses the box but does not need to wrap around corners
            cell.push(second_clip);
        }

        #[cfg(debug_logs)] println!("  [{site}] Edge {first_clip} ({first_edge}) -> {second_clip} ({second_edge}). Wrapping around {:?}", &cell[len-1..]);
    }

    fn extend_voronoi_vertex(&mut self, hull_edge: usize) -> usize {
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

        // project to "inifity"
        let projected = Point { x: circumcenter_pos.x + orthogonal.x * VORONOI_INFINITY, y: circumcenter_pos.y + orthogonal.y * VORONOI_INFINITY };
        let v = self.add_new_vertex(projected);

        #[cfg(debug_logs)] println!("  Hull edge {hull_edge} (circumcenter {circumcenter}) extended orthogonally to {a} -> {b} at {}", v);
        v
    }

    fn is_vertex_inside_bounding_box(&self, vertex: usize) -> bool {
        *self.is_vertex_inside_bounding_box.get(vertex).unwrap_or(&false)
    }

    /// Adds a new vertex if it doesn't already exist.
    ///
    /// Returns the index of the newly added vertex or index of existing vertex.
    fn add_new_vertex(&mut self, vertex: Point) -> usize {
        // the cost of deduplicating vertices is about 5-8% for 1M sites
        // cache hits really help here, but a future improvement would be test to see if a quadtree makes it faster for large inputs
        for (index, v) in self.vertices.iter().enumerate().skip(self.number_of_circumcenters) {
            if utils::abs_diff_eq(v.x, vertex.x, utils::EQ_EPSILON)
                && utils::abs_diff_eq(v.y, vertex.y, utils::EQ_EPSILON) {
                return index;
            }
        }

        let index = self.vertices.len();
        self.vertices.push(vertex);
        index
    }
}

/// Calculates to which sites each corner belongs to.
fn calculate_corner_ownership(corners: &[Point], triangulation: &Triangulation, sites: &Vec<Point>, site_to_incoming_leftmost_halfedge: &Vec<usize>) -> Vec<usize> {
    // corners counter-clockwise
    let mut corner_owners: Vec<usize> = Vec::with_capacity(corners.len());

    // the end of the shortest path between any site and a point is the site containing that point
    // we use this to figure out which sites own which corners
    // the site we use to build the path does not matter, but picking a closer site to start with is most efficient
    // thus we use the owning site for the last corner to calculate the next
    let mut site = *triangulation.hull.first().expect("Hull is at least a triangle.");
    for corner in corners {
        let owner = crate::iterator::shortest_path_iter_from_triangulation(
            triangulation,
            sites,
            site_to_incoming_leftmost_halfedge,
            site,
            corner.clone())
            .last()
            .expect("There must be one site that is the closest.");

        site = owner;
        corner_owners.push(owner);
    }

    corner_owners
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