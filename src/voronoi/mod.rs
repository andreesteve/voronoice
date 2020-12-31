mod into_triangle_list;
mod utils;
mod edges_around_site_iterator;
mod voronoi_mesh_generator;
mod cell;
mod voronoi_builder;
mod bounding_box;

use std::iter::{once};

use delaunator::{EMPTY, Triangulation, next_halfedge, triangulate};
use self::edges_around_site_iterator::EdgesAroundSiteIterator;
use self::cell::VoronoiCell;

pub use self::bounding_box::BoundingBox;
pub use self::voronoi_builder::*;
pub use self::voronoi_mesh_generator::VoronoiMeshGenerator;
pub use delaunator::Point;
pub use self::utils::to_f32_vec;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HullBehavior {
    /// No processing is done for the sites on the hull.
    /// This means cells on the hull do not have its edges extended to the bounding box, nor closed.
    None,

    /// Cells on the hull are only extended to the bounding box, but no additional verteces are added to make it a valid polygon.
    Extended,

    /// Cells on the hull are extended and closed such that they form a valid polygon.
    Closed
}

impl Default for HullBehavior {
    fn default() -> Self {
        HullBehavior::Closed
    }
}

/// The dual delauney-voronoi graph.
///
/// This relies on delaunator library processing and data structure. Refer to https://mapbox.github.io/delaunator/ for a complete understanding of the data structure used.
pub struct Voronoi {
    // pub x_range: (f64, f64),
    // pub y_range: (f64, f64),

    /// These are the sites of each voronoi cell.
    pub sites: Vec<Point>,

    bounding_box: BoundingBox,
    num_of_triangles: usize,
    triangulation: Triangulation,
    hull_behavior: HullBehavior,

    /// The circumcenters of each triangle (indexed by triangle / triangle's starting half-edge).
    ///
    /// For a given voronoi cell, its verteces are the circumcenters of its associated triangles.
    /// Values whose indexes are greater than sites.len() - 1 are not actual triangle circumcenters but voronoi cell vertices added to close sites on the convex hull.
    ///
    /// # Examples
    /// ```
    /// // For the first triangle, get its circumcenter:
    /// let v = Voronoi { ... };
    /// let c = v.circumcenters[0];
    /// ```
    circumcenters: Vec<Point>,

    /// A map of each site to its left-most incomig half-edge.
    site_to_incoming_leftmost_halfedge: Vec<usize>,

    /// A map for each voronoi cell and the associated delauney triangles whose centroids are the cell's vertices.
    /// For any site[s], the associated voronoi cell associated triangles are represented by cell_triangles[s].
    cell_triangles: Vec<Vec<usize>>
}

impl std::fmt::Debug for Voronoi {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.debug_struct("Voronoi")
            .field("hull_behavior", &self.hull_behavior)
            .field("sites", &self.sites)
            .field("circumcenters", &self.circumcenters)
            .field("cell_triangles", &self.cell_triangles)
            .finish()
    }
}

fn cicumcenter(a: &Point, b: &Point, c: &Point) -> Point {
    // move origin to a
    let b_x = b.x - a.x;
    let b_y = b.y - a.y;
    let c_x = c.x - a.x;
    let c_y = c.y - a.y;

    let bb = b_x * b_x + b_y * b_y;
    let cc = c_x * c_x + c_y * c_y;
    let d = 1.0 / (2.0 * (b_x * c_y - b_y * c_x));

    Point {
        x: a.x + d * (c_y * bb - b_y * cc),
        y: a.y + d * (b_x * cc - c_x * bb),
    }
}

/// Returns the index to the site that half-edge `e` points to.
/// This is similar to `triangles`. Given an half-edge `e`, `triangles` returns the index of the site the half-edge start off. `site_of_incoming` returns the index of the site the half-edge points to.
fn site_of_incoming(triangulation: &Triangulation, e: usize) -> usize {
    triangulation.triangles[next_halfedge(e)]
}

// fn clip_vector_to_bounding_box(point: &Point, bounding_box: &BoundingBox) -> Point {
//     let box_ratio = bounding_box.y / bounding_box.y;
//     let tn = point.y / point.x;
//     let mut p = Point { x: 0.0, y: 0.0 };

//     if tn > box_ratio || tn < -box_ratio {
//         // will hit box_y or -box_y
//         let box_y = bounding_box.y * point.y.signum();
//         p.x = box_y * tn.recip();
//         p.y = box_y;
//     } else {
//         // will hit box_x or -box_x
//         let box_x = bounding_box.x * point.x.signum();
//         p.x = box_x;
//         p.y = box_x * tn;
//     }

//     debug_assert!(bounding_box.  is_in_box(&p, bounding_box), "Point {:?} clipped ({:?}) outside bounding box {:?}", point, p, bounding_box);
//     debug_assert!(p.x.abs() == bounding_box.x || p.y.abs() == bounding_box.y.abs(), "Point {:?} clipped ({:?}) outside bounding box {:?}", point, p, bounding_box);

//     p
// }

fn close_hull(cells: &mut Vec<Vec<usize>>, circumcenters: &mut Vec<Point>, triangulation: &Triangulation, bounding_box: &BoundingBox) {
    // perform clock-wise walk on the sites on the hull
    let hull = &triangulation.hull;
    let mut hull_iter = hull.iter().rev().copied();
    let first_cell_index = hull_iter.next().unwrap();
    let mut prev_exteded_vertex = *cells[first_cell_index].last().unwrap();
    let first_vertex = prev_exteded_vertex;

    for site in hull_iter {
        let site = site;
        let cell = &mut cells[site];

        // keep track of current extension vertex
        let curr_exteded_vertex = *cell.last().unwrap();

        close_cell(cell, circumcenters, prev_exteded_vertex, curr_exteded_vertex, bounding_box);

        // let mine = &circumcenters[curr_exteded_vertex];
        // let prev = &circumcenters[prev_exteded_vertex];

        // // close the cell by picking the previous site extension to close the polygon
        // // each edge (and site) on the hull has an associated extension, which is the last value in the cell list
        // cell.insert(cell.len() - 1, prev_exteded_vertex);

        // if mine.x.abs() == bounding_box.x && prev.y.abs() == bounding_box.y {
        //     let p = Point { x: mine.x, y: prev.y };
        //     let i = circumcenters.len();
        //     circumcenters.push(p);
        //     cell.insert(cell.len() - 1, i);
        // } else if mine.y.abs() == bounding_box.y && prev.x.abs() == bounding_box.x {
        //     let p = Point { x: prev.x, y: mine.y };
        //     let i = circumcenters.len();
        //     circumcenters.push(p);
        //     cell.insert(cell.len() - 1, i);
        // }

        prev_exteded_vertex = curr_exteded_vertex;
    }

    let first_cell = &mut cells[first_cell_index];
    close_cell(first_cell, circumcenters, prev_exteded_vertex, first_vertex, bounding_box);
}

/// Clips edge indexed by a -> b and return the indices of the verteces in the clipped edge (may be same values if no clipping was required).
/// Cells edges be returned as `EMPTY` indicating that the edge is completely outside the box and should be excluded.
fn clip_cell_edge(a: usize, b: usize, circumcenters: &mut Vec<Point>, bounding_box: &BoundingBox) -> (usize, usize) {
    // we are iterating a -> b, counter-clockwise on the edges of the cell (cell may be open)
    // at lest one intersection, possibilities
    // 1) a -> box edge -> box edge -> b            a and b outside box ---> need to clip edge at both intersections
    // 2) a -> box edge_same, box edge_same -> b    same as 1, but a->b is a line parallel to one of the box edges ---> keep edge (excluding edge would open cell)
    // 3) a -> box corner, box corner -> b          same as 1, but intersection is right on box corner, or line is parallel to one of the box edges ---> keep edge (excluding edge would open cell)
    // 4) a -> box edge -> b -> box edge            a outside, b inside box ---> clip edge at first intersection
    // 5) a -> b -> box edge [, box edge]           a and b outside, but the line they are in intersects the box (variations include intersection on corner, intersection parallel to a box edge) ---> exclude edge
    // 6) a -> b -> box edge                        a and b inside ---> keep edge as is
    let pa = &circumcenters[a];
    let pb = &circumcenters[b];

    let mut new_a = a;
    let mut new_b = b;

    // if edge crosses box, then clip it
    if !bounding_box.is_inside(pa) {
        // a is outside, b is inside or outside
        // clip will tell us how many intersections between a->b
        let a_to_b = Point { x: pb.x - pa.x, y: pb.y - pa.y };
        let (clip_a, clip_b) = bounding_box.project_ray(pa, &a_to_b);

        if let Some(clip_a) = clip_a {
            let v_index = circumcenters.len();
            new_a = v_index;

            // b may be outside the box and need clipping to
            if let Some(clip_b) = clip_b {
                if !bounding_box.is_inside(pb) {
                    // track new index for the starting vertex
                    new_b = v_index + 1;

                    // FIXME: if both points are same, remove one (i.e. corner intersection)
                    circumcenters.push(clip_a);
                    circumcenters.push(clip_b);
                } else {
                    circumcenters.push(clip_a);
                }
            } else {
                circumcenters.push(clip_a);
            }
        } else {
            // b is outside of bounding box
            // because a is as well, this edge will be completely excluded from the result
            // this also means the resulting cell will be open (if it was previously closed)
            new_a = EMPTY;
            new_b = EMPTY;
        }
    } else if !bounding_box.is_inside(pb) {
        // b is outside, and a is inside
        let a_to_b = Point { x: pb.x - pa.x, y: pb.y - pa.y };
        let clip_b = bounding_box.project_ray_closest(pa, &a_to_b);

        // track new index for b
        new_b = circumcenters.len();
        circumcenters.push(clip_b.expect("Vertex 'b' is outside the bounding box. An intersection should have been returned."));
    } // else neither is outside, not need for clipping

    (new_a, new_b)
}

fn clip_cell(cell: &Vec<usize>, circumcenters: &mut Vec<Point>, bounding_box: &BoundingBox, is_closed: bool) -> (Vec<usize>, bool) {
    // keep state to later filter out duplicate edges
    let mut previous = if is_closed {
        // if cell is closed, the "previous" is the first because the iterator below loops (last -> first), which will yeild a result
        // like: [first, second ... last, first] before the duplication removal
        // by setting this value here, the duplication removal will convert such result to
        // [first, second, second, thrid, forth ... last, first] -> [second, thrid, forth, ... last, first]
        // virtually shiftting the result to the right. This does not change the counter-clockwise ordering of the verteces
        Some(*cell.first().expect("At least one vertex expected for a cell."))
    } else {
        None
    };

    // keeps track of whether this cell clipping opened up the cell
    let mut closed = is_closed;

    // iterates over (n, n+1)
    let new_cell = cell.iter().zip(cell.iter().skip(1))
        // if cell is closed, add (last, first) pair to the end to be handled too
        .chain(once((cell.last().unwrap(), cell.first().unwrap())).filter(|_| is_closed))
        // clip edge and convert to list of indices again
        .flat_map(|(a, b)| {
            let (new_a, new_b) = clip_cell_edge(*a, *b, circumcenters, bounding_box);
            once(new_a).chain(once(new_b))
        })
        // remove duplicates
        .filter_map(|a| {
            let prev = previous;
            previous = Some(a);

            // remove vertex if it is empty
            if a == EMPTY {
                // this will cause the cell to become open
                closed = false;
                None
            } else if let Some(prev) = prev {
                if prev == a {
                    // vertex did not change
                    None
                } else {
                    Some(a)
                }
            } else {
                Some(a)
            }
        })
        .collect::<Vec<usize>>();

    (new_cell, closed)
}


#[cfg(test)]
mod test {
    use super::*;

    fn assert_same_elements(a: &Vec<usize>, b: &Vec<usize>, message: &str) {
        assert_eq!(a.len(), b.len(), "Vectors differ in length.");
        assert_eq!(0, a.iter().copied().zip(b.iter().copied()).filter(|(a,b)| a != b).collect::<Vec<(usize, usize)>>().len(), "Vectors have differing elements. A: {:?}. B: {:?}. {}", a, b, message);
    }

    #[test]
    fn clip_cell_when_no_point_outside_box() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 1.0, y: 0.0 },
            Point { x: 0.0, y: 1.0 },
        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, is_closed) = clip_cell(&cell, &mut points, &bbox, false);
        assert_same_elements(&cell, &clipped_cell, "No clipping expected");
        assert_eq!(is_closed, false, "Clipping cannot close open cells.")
    }

    #[test]
    fn clip_cell_one_edge_crosses_one_box_edge() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, is_closed) = clip_cell(&cell, &mut points, &bbox, false);
        assert_same_elements(&clipped_cell, &vec![0, 2], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 2.0, y: 0.0 }, points[2], "Point should have been added for clipped edge.");
        assert_eq!(is_closed, false, "Clipping cannot close open cells.")
    }

    #[test]
    fn clip_cell_one_edge_crosses_one_box_edge_inverted() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: 10.0, y: 0.0 },
            Point { x: 0.0, y: 0.0 },
        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, is_closed) = clip_cell(&cell, &mut points, &bbox, false);
        assert_same_elements(&clipped_cell, &vec![2, 1], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 2.0, y: 0.0 }, points[2], "Point should have been added for clipped edge.");
        assert_eq!(is_closed, false, "Clipping cannot close open cells.")
    }

    #[test]
    fn clip_cell_one_edge_crosses_two_box_edges() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: -10.0, y: 0.0 },
            Point { x: 10.0, y: 0.0 },
        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, is_closed) = clip_cell(&cell, &mut points, &bbox, false);
        assert_same_elements(&clipped_cell, &vec![2, 3], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: -2.0, y: 0.0 }, points[2], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 0.0 }, points[3], "Point should have been added for clipped edge.");
        assert_eq!(is_closed, false, "Clipping cannot close open cells.")
    }

    #[test]
    fn clip_triangular_cell_with_one_point_outside_box() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -3.0 },
            Point { x: 1.0, y: 0.0 },
        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, is_closed) = clip_cell(&cell, &mut points, &bbox, true);
        assert_same_elements(&clipped_cell, &vec![3, 4, 2, 0], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: -2.0 }, points[3], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 1.0 / 3.0, y: -2.0 }, points[4], "Point should have been added for clipped edge.");
        assert_eq!(is_closed, true, "No edge was entirely outside of the box to cause the cell to be clipped open.")
    }

    #[test]
    fn clip_triangular_cell_with_one_point_outside_box_and_last_crossing_box_edge() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: 1.0, y: 0.0 },
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -3.0 },
        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, is_closed) = clip_cell(&cell, &mut points, &bbox, true);
        assert_same_elements(&clipped_cell, &vec![1, 3, 4, 0], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: -2.0 }, points[3], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 1.0 / 3.0, y: -2.0 }, points[4], "Point should have been added for clipped edge.");
        assert_eq!(is_closed, true, "No edge was entirely outside of the box to cause the cell to be clipped open.")
    }

    #[test]
    fn clip_triangular_cell_with_two_points_outside_and_one_edge_entirely_outside_box() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -30.0 },
            Point { x: 20.0, y: 0.0 },
        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, is_closed) = clip_cell(&cell, &mut points, &bbox, true);
        assert_same_elements(&clipped_cell, &vec![3, 4, 0], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: -2.0 }, points[3], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 0.0 }, points[4], "Point should have been added for clipped edge.");
        assert_eq!(is_closed, false, "One edge was entirely outside of the box to cause the cell to be clipped open.")
    }

    #[test]
    fn clip_triangular_cell_with_two_points_outside_and_edge_crossing_box_twice() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: -3.0 },
            Point { x: 3.0, y: 0.0 },
        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, is_closed) = clip_cell(&cell, &mut points, &bbox, true);
        assert_same_elements(&clipped_cell, &vec![3, 4, 5, 6, 0], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: -2.0 }, points[3], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 1.0, y: -2.0 }, points[4], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -1.0 }, points[5], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 0.0 }, points[6], "Point should have been added for clipped edge.");
        assert_eq!(is_closed, true, "No entire edge was outside the box, so the cell must stay closed.")
    }

    #[test]
    fn clip_triangular_cell_with_one_edge_intersecting_box_corner() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 0.0, y: 4.0 },
            Point { x: 4.0, y: 0.0 },
        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, is_closed) = clip_cell(&cell, &mut points, &bbox, true);
        assert_same_elements(&clipped_cell, &vec![3, 4, 5, 6, 0], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: 2.0 }, points[3], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, points[4], "Point should have been added for clipped edge."); // corner
        assert_eq!(Point { x: 2.0, y: 2.0 }, points[5], "Point should have been added for clipped edge."); // corner
        assert_eq!(Point { x: 2.0, y: 0.0 }, points[6], "Point should have been added for clipped edge.");
        assert_eq!(is_closed, true, "No entire edge was outside the box, so the cell must stay closed.")
    }

    #[test]
    fn clip_triangular_cell_with_one_edge_parallel_to_box_edge() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: 0.0, y: 0.0 },
            Point { x: 2.0, y: -4.0 },
            Point { x: 2.0, y: 4.0 },
        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, is_closed) = clip_cell(&cell, &mut points, &bbox, true);
        assert_same_elements(&clipped_cell, &vec![3, 4, 5, 6, 0], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 1.0, y: -2.0 }, points[3], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -2.0 }, points[4], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, points[5], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 1.0, y: 2.0 }, points[6], "Point should have been added for clipped edge.");
        assert_eq!(is_closed, true, "No entire edge was outside the box, so the cell must stay closed.")
    }

    #[test]
    fn clip_triangular_cell_with_three_points_outside_box() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: 10.0, y: 0.0 },
            Point { x: 10.0, y: -3.0 },
            Point { x: 15.0, y: 0.0 },

        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, is_closed) = clip_cell(&cell, &mut points, &bbox, true);
        assert_same_elements(&clipped_cell, &vec![], "Clipped cell incorrect indices.");
        assert_eq!(points.len(), 3, "No new points expected.");
        assert_eq!(is_closed, false, "A cell without verteces is by definition opened.");
    }
}

/// Calculate the triangles associated with each voronoi cell
fn calculate_cell_triangles(hull_behavior: HullBehavior, sites: &Vec<Point>, circumcenters: &mut Vec<Point>, triangulation: &Triangulation, site_to_leftmost_halfedge: &Vec<usize>, num_of_sites: usize, bounding_box: &BoundingBox) -> Vec<Vec<usize>> {
    let mut seen_sites = vec![false; num_of_sites];
    let mut cells = vec![Vec::new(); num_of_sites];

    for edge in 0..triangulation.triangles.len() {
        // triangle[edge] is the site 'edge' originates from, but EdgesAroundPointIterator
        // iterate over edges around the site 'edge' POINTS TO, thus to get that site
        // we need to take the next half-edge
        let site = site_of_incoming(&triangulation, edge);

        //println!("Site {} {:?}, incoming edge {} (from site {} {:?}), first time? {}", site, sites[site], edge, triangulation.triangles[edge], sites[triangulation.triangles[edge]], !seen_sites[site]);

        // if we have already created the cell for this site, move on
        if !seen_sites[site] {
            seen_sites[site] = true;

            // edge may or may not be the left-most incoming edge for site
            // thus get the one
            let leftmost_edge = site_to_leftmost_halfedge[site];

            // if there is no half-edge associated with the left-most edge, the edge is on the hull
            let is_hull_site = triangulation.halfedges[leftmost_edge] == EMPTY;

            let cell = &mut cells[site];
            cell.extend(
                EdgesAroundSiteIterator::new(&triangulation, leftmost_edge)
                        .map(|e| utils::triangle_of_edge(e))
            );

            // clip cell edges
            let (new_cell, _) = clip_cell(cell, circumcenters, bounding_box, !is_hull_site);
            *cell = new_cell;

            // cells on the hull need to have its edges extended to the edges of the box
            if is_hull_site && (hull_behavior == HullBehavior::Extended || hull_behavior == HullBehavior::Closed)  {
                // this is the vertex we will extend from
                let cell_vertex_to_extend = &circumcenters[cell[0]];

                // if vertex is outside bounding box, no need to extend it
                if !bounding_box.is_inside(cell_vertex_to_extend) {
                    // get the point that the edge comes from
                    let source_site = triangulation.triangles[leftmost_edge];
                    let source_point = &sites[source_site];
                    let target_point = &sites[site];

                    // the line extension must be perpendicular to the hull edge
                    // get edge direction, rotated by 90 degree counterclock-wise as to point towards the "outside" (x -> y, y -> -x)
                    let orthogonal = Point { x: source_point.y - target_point.y, y: target_point.x - source_point.x };

                    // get voronoi vertex that needs to be extended and extend it
                    let projected = bounding_box.project_ray_closest(cell_vertex_to_extend, &orthogonal).expect("Expected intersection with box");

                    // add extended vertex as a "fake" circumcenter
                    let vertex_index = circumcenters.len();
                    // this point is orthogonally extended towards the outside from the current cell[0], thus it needs to come in first
                    // be keep verteces in counterclockwise order
                    cell.insert(0, vertex_index);
                    circumcenters.push(projected);
                }
            }
        }
    }

    // we need to "close" the cell for the sites on the hull, as we have so far extended their edges
    if hull_behavior == HullBehavior::Closed {
        close_hull(&mut cells, circumcenters, triangulation, bounding_box);
    }

    if num_of_sites != cells.len() {
        println!("Different number of sites from cells. Sites: {}, cells: {}", num_of_sites, cells.len());
        seen_sites.iter().enumerate().filter(|(_, seen)| !**seen).for_each(|(s, _)| println!("Site {} was not seen", s));
        panic!("Sites missing");
    }

    cells
}

fn calculate_circumcenters(sites: &Vec<Point>, triangulation: &Triangulation) -> Vec<Point> {
    let num_of_triangles = triangulation.triangles.len() / 3;

    // calculate circuncenter of each triangle
    (0..num_of_triangles).map(|t| {
        let c = cicumcenter(
            &sites[triangulation.triangles[3* t]],
            &sites[triangulation.triangles[3* t + 1]],
            &sites[triangulation.triangles[3* t + 2]]);

            // FIXME: this is a good approximation when sites are close to origin
            // need to instead project circumcenters, project the cell vectors instead
            // if !is_in_box(&c, &bounding_box) {
            //     c = clip_vector_to_bounding_box(&c, &bounding_box);
            // }

            c
    })
    .collect()
}

fn close_cell(cell: &mut Vec<usize>, circumcenters: &mut Vec<Point>, prev_vertex: usize, curr_vertex: usize, bounding_box: &BoundingBox) {
    let mine = circumcenters[curr_vertex].clone();
    let prev = circumcenters[prev_vertex].clone();

    // FIXME: use same points for corners
    // let top_right = Point { x: bounding_box.x, y: bounding_box.y };
    // let top_left = Point { x: -bounding_box.x, y: bounding_box.y };
    // let bottom_right = Point { x: bounding_box.x, y: -bounding_box.y };
    // let bottom_left = Point { x: -bounding_box.x, y: -bounding_box.y };

    // close the cell by picking the previous site extension to close the polygon
    // each edge (and site) on the hull has an associated extension, which is the last value in the cell list
    cell.insert(cell.len() - 1, prev_vertex);
    let bounding_box = bounding_box.top_right();

    if mine.x.abs() == bounding_box.x && prev.x.abs() == bounding_box.x && mine.x != prev.x {
        println!("Case 2: mine {:?} prev {:?}", mine, prev);
        // cur and prev on oppositve X bounding lines
        let dir = if prev.y.signum() != mine.y.signum() && prev.y.abs() > mine.y.abs() {
            -mine.y.signum()
        } else {
            mine.y.signum()
        };

        let p = Point { x: prev.x, y: dir * bounding_box.y };
        println!("  Added {:?}", p);
        let i = circumcenters.len();
        circumcenters.push(p);
        cell.insert(cell.len() - 1, i);

        let p = Point { x: mine.x, y: dir * bounding_box.y };
        println!("  Added {:?}", p);
        let i = circumcenters.len();
        circumcenters.push(p);
        cell.insert(cell.len() - 1, i);
    } else if mine.y.abs() == bounding_box.y && prev.y.abs() == bounding_box.y && mine.y != prev.y {
        println!("Case 3: mine {:?} prev {:?}", mine, prev);
        // cur and prev on oppositve Y bounding lines
        let dir = if prev.x.signum() != mine.x.signum() && prev.x.abs() > mine.x.abs() {
            -mine.x.signum()
        } else {
            mine.x.signum()
        };

        let p = Point { x: dir * bounding_box.x, y: prev.y };
        println!("  Added {:?}", p);
        let i = circumcenters.len();
        circumcenters.push(p);
        cell.insert(cell.len() - 1, i);

        let p = Point { x: dir * bounding_box.x, y: mine.y };
        println!("  Added {:?}", p);
        let i = circumcenters.len();
        circumcenters.push(p);
        cell.insert(cell.len() - 1, i);
    } else if mine.x.abs() == bounding_box.x && prev.y.abs() == bounding_box.y {
        println!("Case 0: mine {:?} prev {:?}", mine, prev);
        let p = Point { x: mine.x, y: prev.y };
        let i = circumcenters.len();
        circumcenters.push(p);
        cell.insert(cell.len() - 1, i);
    } else if mine.y.abs() == bounding_box.y && prev.x.abs() == bounding_box.x {
        println!("Case 1: mine {:?} prev {:?}", mine, prev);
        let p = Point { x: prev.x, y: mine.y };
        let i = circumcenters.len();
        circumcenters.push(p);
        cell.insert(cell.len() - 1, i);
    }
}

// When reading this code think about 'edge' as the starting edge for a triangle
// So you can say that the starting edge indexes the triangle
// For instances, diag.triangles.len() is the number of starting edges and triangles in the triangulation, you can think of diag.triangles[e] as 'e' as being both the index of the
// starting edge and the triangle it represents. When dealing with an arbitraty edge, it may not be a starting edge. You can get the starting edge by dividing the edge by 3 and flooring it.
impl Voronoi {
    pub fn new(sites: Vec<Point>, hull_behavior: HullBehavior, bounding_box: BoundingBox) -> Option<Self> {
        // remove any points not within bounding box
        let sites = sites.into_iter().filter(|p| bounding_box.is_inside(p)).collect::<Vec<Point>>();

        let triangulation = triangulate(&sites)?;


        let num_of_triangles = triangulation.triangles.len() / 3;
        let num_of_sites = sites.len();

        // calculate circuncenter of each triangle
        let mut circumcenters = calculate_circumcenters(&sites, &triangulation);

        // create map between site and its left-most incoming half-edge
        // this is especially important for the sites along the convex hull boundary when iterating over its neighoring sites
        let mut site_to_halfedge = vec![EMPTY; num_of_sites];
        for e in 0..triangulation.triangles.len() {
            let s = site_of_incoming(&triangulation, e);
            if site_to_halfedge[s] == EMPTY || triangulation.halfedges[e] == EMPTY {
                site_to_halfedge[s] = e;
            }
        }

        let cell_triangles = calculate_cell_triangles(hull_behavior, &sites, &mut circumcenters, &triangulation, &site_to_halfedge, num_of_sites, &bounding_box);

        Some(Voronoi {
            bounding_box,
            circumcenters,
            site_to_incoming_leftmost_halfedge: site_to_halfedge,
            triangulation,
            num_of_triangles,
            cell_triangles,
            sites,
            hull_behavior
        })
    }

    /// Returns an iterator that walks through each vertex of a voronoi cell in counter-clockwise manner.
    pub fn get_cell_verteces(&self, cell: usize) -> impl Iterator<Item = &Point> {
        self.cell_triangles[cell].iter().map(move |t| {
            &self.circumcenters[*t]
        })
    }

    /// Gets a representation of a voronoi cell based on its site index.
    pub fn get_cell(&self, site: usize) -> VoronoiCell {
        VoronoiCell::new(site, self)
    }

    pub fn cells<'v>(&'v self) -> impl Iterator<Item = VoronoiCell<'v>> + Clone {
        (0..self.sites.len())
            .map(move |s| self.get_cell(s))
    }

    // /// Returns the index to the site that half-edge `e` points to.
    // /// This is similar to `triangles`. Given an half-edge `e`, `triangles` returns the index of the site the half-edge start off. `site_of_incoming` returns the index of the site the half-edge points to.
    // // fn site_of_incoming(&self, e: usize) -> usize {
    // //     site_of_incoming(&self.triangulation, e)
    // // }

    // // /// Returns a EdgesAroundSiteIterator iterator.
    // // fn edges_around_site(&self, site: usize) -> EdgesAroundSiteIterator {
    // //     EdgesAroundSiteIterator::for_site(&self, site)
    // // }
}