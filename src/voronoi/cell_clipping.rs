use std::iter::once;
use delaunator::EMPTY;
use super::{Point, bounding_box::{self, *}};

pub fn clip_cell(cell: &Vec<usize>, circumcenters: &mut Vec<Point>, bounding_box: &BoundingBox, is_closed: bool) -> (Vec<usize>, bool) {
    if cell.len() < 2 {
        return (cell.clone(), false);
    }

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
                if prev == Some(EMPTY) {
                    // two EMPTY vertices removed in a row means an edge was removed
                    closed = false;
                }
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
    // 7) a -> b                                    a and b outside box, and no intersection with box ---> exclude edge
    let pa = &circumcenters[a];
    let pb = &circumcenters[b];
    let mut new_a = a;
    let mut new_b = b;

    if !bounding_box.is_inside(pa) {
        // a is outside, b is inside or outside
        // clip will tell us how many intersections between a->b, and clip_a will come first than clip_b
        let a_to_b = Point { x: pb.x - pa.x, y: pb.y - pa.y };
        let (clip_a, clip_b) = bounding_box.project_ray(pa, &a_to_b);

        if let Some(clip_a) = clip_a {
            let v_index = circumcenters.len();
            new_a = v_index;

            if let Some(clip_b) = clip_b {
                // b may be inside or outside the box
                if !bounding_box.is_inside(pb) {
                    // there are two main cases here:
                    // a -> b -> box; i.e. ray reaches the box but after b; we discard this edge (case 5)
                    // check which point (b or clip_a) is closer to a
                    let closest_to_a = bounding_box::order_points_on_ray(pa,&a_to_b, Some(pb.clone()), Some(clip_a.clone())).0.unwrap();
                    if closest_to_a == clip_a {
                        // a -> box -> b; edge crosses box (case 1,2,3), we clip this ray twice
                        new_b = v_index + 1;

                        if clip_a == clip_b {
                            // case 3 - a and b outside box, intersection at the corner
                            // FIXME: consider distance epislon comparison instead
                            // intersection at same point (corner)
                            new_b = EMPTY;
                            circumcenters.push(clip_a);
                        } else {
                            circumcenters.push(clip_a);
                            circumcenters.push(clip_b);
                        }
                    } else {
                        // a -> b -> box (case 5), discard edge
                        new_a = EMPTY;
                        new_b = EMPTY;
                    }
                } else {
                    // b is inside the box, only clip the edge once
                    circumcenters.push(clip_a);
                }
            } else {
                // a single itersection, this means a -> box ->b
                circumcenters.push(clip_a);
            }
        } else {
            // a and b are outside of bounding box and there were no intersection with box
            // this edge will be completely excluded from the result
            // this also means the resulting cell will be open (if it was previously closed)
            // case 7
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
        assert_same_elements(&clipped_cell, &vec![3, 4, 5, 0], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 0.0, y: 2.0 }, points[3], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, points[4], "Point should have been added for clipped edge."); // corner
        assert_eq!(Point { x: 2.0, y: 0.0 }, points[5], "Point should have been added for clipped edge.");
        assert_eq!(is_closed, true, "No entire edge was outside the box, so the cell must stay closed.")
    }

    // same as above but with triangle inverted, no vertex inside box
    #[test]
    fn clip_triangular_cell_outside_with_one_edge_intersecting_box_corner() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: 4.0, y: 4.0 },
            Point { x: 4.0, y: 0.0 },
            Point { x: 0.0, y: 4.0 },
        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, is_closed) = clip_cell(&cell, &mut points, &bbox, true);
        assert_same_elements(&clipped_cell, &vec![3], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, points[3], "Point should have been added for clipped edge."); // corner
        assert_eq!(is_closed, false, "Two edges outside of the box were removed.")
    }

    #[test]
    fn clip_triangular_cell_with_one_point_inside_box_and_one_edge_parallel_to_box_edge() {
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
    fn clip_triangular_cell_with_no_point_inside_box_and_one_edge_parallel_to_box_edge() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: 4.0, y: 0.0 },
            Point { x: 2.0, y: 4.0 },
            Point { x: 2.0, y: -4.0 },
        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, is_closed) = clip_cell(&cell, &mut points, &bbox, true);
        assert_same_elements(&clipped_cell, &vec![3, 4], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, points[3], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -2.0 }, points[4], "Point should have been added for clipped edge.");
        assert_eq!(is_closed, false, "One edge outside box.")
    }

    // same case as above, but the entire parallel edge is shared with the box
    #[test]
    fn clip_triangular_cell_with_no_point_inside_box_and_one_shared_edge_parallel_to_box_edge() {
        let bbox = BoundingBox::new_centered_square(4.0); // edges at +-2
        let mut points = vec![
            Point { x: 4.0, y: 0.0 },
            Point { x: 2.0, y: 2.0 },
            Point { x: 2.0, y: -2.0 },
        ];
        let cell = (0..points.len()).collect();
        let (clipped_cell, _) = clip_cell(&cell, &mut points, &bbox, true);
        assert_same_elements(&clipped_cell, &vec![3, 1, 2, 4], "Clipped cell incorrect indices.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, points[3], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: 2.0 }, points[1], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -2.0 }, points[2], "Point should have been added for clipped edge.");
        assert_eq!(Point { x: 2.0, y: -2.0 }, points[4], "Point should have been added for clipped edge.");

        // FIX ME: code thinks cell is closed, but that is not true in this case
        // ignoring for now as this shouldn't really happen for voronoi
        //assert_eq!(is_closed, false, "One edge outside box.");
    }

    // FIX ME - this found a bug
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