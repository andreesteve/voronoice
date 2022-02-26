use crate::utils::{abs_diff_eq, EQ_EPSILON};

use super::Point;

/// Defines a rectangular bounding box.
///
/// The Y axis convention is downwards.
#[derive(Debug, Clone)]
pub struct BoundingBox {
    /// The center point of a rectangle.
    center: Point,

    /// The top right point of a rectangle.
    top_right: Point,

    /// The bottom left point of a rectangle.
    bottom_left: Point,
}

impl Default for BoundingBox {
    fn default() -> Self {
        Self::new_centered_square(2.0) // square from [-1, 1] on xy
    }
}

impl BoundingBox {
    /// Constructs a new bounding box.
    ///
    /// # Arguments
    ///
    /// * `center` - The position of the center of the bounding box
    /// * `width` - The bounding box's width
    /// * `height` - The bounding box's height
    ///
    pub fn new(center: Point, width: f64, height: f64) -> Self {
        Self {
            top_right: Point { x: center.x + width / 2.0, y: center.y - height / 2.0 },
            bottom_left: Point { x: center.x - width / 2.0, y: center.y + height / 2.0 },
            center,
        }
    }

    /// Constructs a new bounding box centeterd at origin with the provided width and height.
    pub fn new_centered(width: f64, height: f64) -> Self {
        Self::new(Point { x: 0.0, y: 0.0 }, width, height)
    }

    /// Constructs a new square bounding box centeterd at origin with the provided width.
    pub fn new_centered_square(width: f64) -> Self {
        Self::new_centered(width, width)
    }

    /// Gets the position of the box's center.
    #[inline]
    pub fn center(&self) -> &Point {
        &self.center
    }

    /// Gets the position of the top right corner of the bounding box.
    #[inline]
    pub fn top_right(&self) -> &Point {
        &self.top_right
    }

    /// Gets the position of the bottom left corner of the bounding box.
    #[inline]
    pub fn bottom_left(&self) -> &Point {
        &self.bottom_left
    }

    /// Gets the width of the bounding box.
    #[inline]
    pub fn width(&self) -> f64 {
        self.top_right.x - self.bottom_left.x
    }

    /// Gets the height of the bounding box.
    #[inline]
    pub fn height(&self) -> f64 {
        self.bottom_left.y - self.top_right.y
    }

    /// Gets the Y coordinate of the top of the bounding box.
    #[inline]
    pub fn top(&self) -> f64 {
        self.top_right.y
    }

    /// Gets the Y coordinate of the bottom of the bounding box.
    #[inline]
    pub fn bottom(&self) -> f64 {
        self.bottom_left.y
    }

    /// Gets the X coordinate of the left of the bounding box.
    #[inline]
    pub fn left(&self) -> f64 {
        self.bottom_left.x
    }

    /// Gets the X coordinate of the right of the bounding box.
    #[inline]
    pub fn right(&self) -> f64 {
        self.top_right.x
    }

    /// Gets a slice of corners oriented counter-clockwise.
    pub fn corners(&self) -> [Point; 4] {
        [
            Point { x: self.left(), y: self.top() }, // top left
            self.bottom_left().clone(),
            Point { x: self.right(), y: self.bottom() }, // bottom right
            self.top_right().clone(),
        ]
    }

    /// Returns whether a given point is inside (or on the edges) of the bounding box.
    #[inline]
    pub fn is_inside(&self, point: &Point) -> bool {
        let horizonal_ok = point.x >= self.left() && point.x <= self.right();
        let vertical_ok = point.y >= self.top() && point.y <= self.bottom();

        horizonal_ok && vertical_ok
    }

    // /// Same as inside, but return false if point is on the box edge.
    #[inline]
    pub fn is_exclusively_inside(&self, point: &Point) -> bool {
        self.is_inside(point) && self.which_edge(point).is_none()
    }

    /// Returns the index of the corner representing the edge ```point``` is on.
    ///
    /// corners() method can be called to get the list of corners.
    ///
    /// # Example
    /// If point is on middle of the top edge, the top left corner will be returned.
    #[inline]
    pub (crate) fn which_edge(&self, point: &Point) -> Option<usize> {
        if abs_diff_eq(point.y, self.top(), EQ_EPSILON) {
            // top
            Some(0)
        } else if abs_diff_eq(point.y, self.bottom(), EQ_EPSILON) {
            // bottom
            Some(2)
        } else {
            if abs_diff_eq(point.x, self.right(), EQ_EPSILON) {
                // right
                Some(3)
            } else if abs_diff_eq(point.x, self.left(), EQ_EPSILON) {
                // left
                Some(1)
            } else {
                None
            }
        }
    }

    // /// Returns which edge, if any, the given `point` is located.
    // #[inline]
    // pub (crate) fn which_edge(&self, point: &Point) -> (BoundingBoxTopBottomEdge, BoundingBoxLeftRightEdge) {
    //     (
    //         // The cost of this macro is probably justified in this case, since equality (==) is necessary
    //         // Didn't check the performance impact, though.
    //         if abs_diff_eq(point.y, self.top(), EQ_EPSILON) {
    //             // top
    //             BoundingBoxTopBottomEdge::Top
    //         } else if abs_diff_eq(point.y, self.bottom(), EQ_EPSILON) {
    //             BoundingBoxTopBottomEdge::Bottom
    //         } else {
    //             BoundingBoxTopBottomEdge::None
    //         },

    //         if abs_diff_eq(point.x, self.right(), EQ_EPSILON) {
    //             // right
    //             BoundingBoxLeftRightEdge::Right
    //         } else if abs_diff_eq(point.x, self.left(), EQ_EPSILON) {
    //             // left
    //             BoundingBoxLeftRightEdge::Left
    //         } else {
    //             BoundingBoxLeftRightEdge::None
    //         }
    //     )
    // }

    /// Intersects a line represented by points 'a' and 'b' and returns the two intersecting points with the box, or None
    pub (crate) fn intersect_line(&self, a: &Point, b: &Point) -> (Option<Point>, Option<Point>) {
        let c_x = b.x - a.x;
        let c_y = b.y - a.y;
        let c = c_y / c_x;
        let d = a.y - (a.x * c);

        let mut f = None;
        let mut g = None;
        let mut h = None;
        let mut i = None;

        // intersection left, right edges
        if c_x.abs() > 4. * std::f64::EPSILON {
            // y = c*x + d
            let right_y = (self.right() * c) + d;
            let left_y = (self.left() * c) + d;

            if right_y >= self.top()
            && right_y <= self.bottom() {
                f = Some(Point { x: self.right(), y: right_y });
            }

            if left_y >= self.top()
            && left_y <= self.bottom() {
                g = Some(Point { x: self.left(), y: left_y })
            }

            if g.is_some() && f.is_some() {
                // can't have more than 2 intersections, we are done
                return (f, g);
            }
        } // else line is parallel to y, won't intersect with left/right

        // intersection top, bottom edges
        if c_y.abs() > 4. * std::f64::EPSILON {
            if c_x.abs() < 4. * std::f64::EPSILON {
                // line is parallel to y
                if a.x <= self.right()
                && a.x >= self.left() {
                    // and crosses box
                    return (
                        Some(Point { x: a.x, y: self.top() }),
                        Some(Point { x: a.x, y: self.bottom() })
                    );
                } else {
                    // does not cross box
                    return (None, None);
                }
            }

            // x = (y - d) / c
            let top_x = (self.top() - d) / c;
            let bottom_x = (self.bottom() - d) / c;

            if top_x <= self.right()
                && top_x >= self.left() {
                h = Some(Point { x: top_x, y: self.top() })
            }

            if bottom_x <= self.right()
                && bottom_x >= self.left() {
                i = Some(Point { x: bottom_x, y: self.bottom() })
            }

            if h.is_some() && i.is_some() {
                // can't have more than 2 intersections, we are done
                return (h, i);
            }
        } // else line is parallel to x, won't intersect with top / bottom

        (f.or(g), h.or(i))
    }

    /// Intersects a ray with the bounding box. The first intersection is returned first.
    pub (crate) fn project_ray(&self, point: &Point, direction: &Point) -> (Option<Point>, Option<Point>) {
        let b = Point { x: point.x + direction.x, y: point.y + direction.y };
        let (a, b) = self.intersect_line(point, &b);
        order_points_on_ray(point, direction, a, b)
    }
}

/// Given a ray defined by `point` and `direction`, and two points `a` and `b` on such ray, returns a tuple (w, z) where point <= w <= z.
/// If either `a` or `b` are smaller than `point`, None is returned.
pub (crate) fn order_points_on_ray(point: &Point, direction: &Point, a: Option<Point>, b: Option<Point>) -> (Option<Point>, Option<Point>) {
    match (a,b) {
        (None, None) => { // no points, no intersection
            (None, None)
        }
        (Some(va), Some(vb)) => { // both a and b are reachable
            // point a and b are just a scalar times direction, so we can compare any non-zero
            // direction component, use largest
            let (d, da, db) = if direction.x.abs() > direction.y.abs() {
                // use x for comparison
                (direction.x, va.x - point.x, vb.x - point.x)
            } else {
                (direction.y, va.y - point.y, vb.y - point.y)
            };

            match (d.signum() == da.signum(), d.signum() == db.signum()) {
                (true, true) => {
                    if da.abs() > db.abs() {
                        // b is closer
                        (Some(vb), Some(va))
                    } else {
                        // a is closer
                        (Some(va), Some(vb))
                    }
                },
                (true, false) => { // only a reachable
                    (Some(va), None)
                },
                (false, true) => { // only b reachably
                    (Some(vb), None)
                },
                (false, false) => { // neither a nor b is reachable, no intersection
                    (None, None)
                }
            }
        },
        (Some(va), None) => {
            if direction.x.signum() == va.x.signum() && direction.y.signum() == va.y.signum() {
                // a is in the right direction
                (Some(va), None)
            } else {
                // a can't be reached
                (None, None)
            }
        },
        (None, Some(vb)) => {
            if direction.x.signum() == vb.x.signum() && direction.y.signum() == vb.y.signum() {
                // b is in the right direction
                (Some(vb), None)
            } else {
                // b can't be reached
                (None, None)
            }
        }
    }
}

#[inline]
pub (crate) fn next_edge(edge: usize) -> usize {
    (edge + 1) % 4
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(x: f64, c: f64, d: f64) -> Point {
        Point { x, y: (x * c) + d }
    }
    // direction-vector b to a, not a to b!
    fn direction(a: &Point, b: &Point) -> Point {
        Point { x: a.x - b.x, y: a.y - b.y }
    }

    #[test]
    fn intersect_line_tests() {
        // square centered on origin with edges on x = +-1, y= +-1
        let bbox = BoundingBox::new_centered_square(2.0);

        // line parallel to x, outside box
        let (a, b) = bbox.intersect_line(&Point { x: 5.0, y: 0.0 }, &Point { x: 5.0, y: 1.0 }); // x = 5
        assert_eq!(a, None, "No intersection expected for a parallel line to X outside of the box");
        assert_eq!(b, None, "No intersection expected for a parallel line to X outside of the box");

        // line parallel to y, outside box
        let (a, b) = bbox.intersect_line(&Point { x: 0.0, y: 5.0 }, &Point { x: 1.0, y: 5.0 }); // y = 5
        assert_eq!(a, None, "No intersection expected for a parallel line to Y outside of the box");
        assert_eq!(b, None, "No intersection expected for a parallel line to Y outside of the box");

        // line parallel to x, crossing box
        let (a, b) = bbox.intersect_line(&Point { x: 0.0, y: 0.0 }, &Point { x: 0.0, y: 1.0 }); // x = 0
        assert_eq!(Some(Point { x: 0.0, y: bbox.top() }), a, "Expected intersection with top edge");
        assert_eq!(Some(Point { x: 0.0, y: bbox.bottom() }), b, "Expected intersection with bottom edge");

        // line parallel to y, crossing box
        let (a, b) = bbox.intersect_line(&Point { x: 0.0, y: 0.0 }, &Point { x: 1.0, y: 0.0 }); // y = 0
        assert_eq!(Some(Point { x: 1.0, y: 0.0 }), a, "Expected intersection with right edge");
        assert_eq!(Some(Point { x: -1.0, y: 0.0 }), b, "Expected intersection with left edge");

        // line congruent to top edge
        let (a, b) = bbox.intersect_line(&Point { x: 0.0, y: bbox.top() }, &Point { x: 1.0, y: bbox.top() }); // y = top
        assert_eq!(Some(Point { x: bbox.right(), y: bbox.top() }), a, "Expected intersection with top right corner");
        assert_eq!(Some(Point { x: bbox.left(), y: bbox.top() }), b, "Expected intersection with top left corner");

        // line congruent to bottom edge
        let (a, b) = bbox.intersect_line(&Point { x: 0.0, y: bbox.bottom() }, &Point { x: 1.0, y: bbox.bottom() }); // y = bottom
        assert_eq!(Some(Point { x: bbox.right(), y: bbox.bottom() }), a, "Expected intersection with bottom right corner");
        assert_eq!(Some(Point { x: bbox.left(), y: bbox.bottom() }), b, "Expected intersection with bottom left corner");

        // line congruent to right edge
        let (a, b) = bbox.intersect_line(&Point { x: bbox.right(), y: 0.0 }, &Point { x: bbox.right(), y: 1.0 }); // x = right
        assert_eq!(Some(Point { x: bbox.right(), y: bbox.top() }), a, "Expected intersection with top right corner");
        assert_eq!(Some(Point { x: bbox.right(), y: bbox.bottom() }), b, "Expected intersection with bottom right corner");

        // line congruent to left edge
        let (a, b) = bbox.intersect_line(&Point { x: bbox.left(), y: 0.0 }, &Point { x: bbox.left(), y: 1.0 }); // x = left
        assert_eq!(Some(Point { x: bbox.left(), y: bbox.top() }), a, "Expected intersection with top left corner");
        assert_eq!(Some(Point { x: bbox.left(), y: bbox.bottom() }), b, "Expected intersection with bottom left corner");

        // -45 degree line from box origin
        let (a, b) = bbox.intersect_line(&Point { x: 0.0, y: 0.0 }, &Point { x: bbox.right(), y: bbox.top() });
        assert_eq!(Some(Point { x: bbox.right(), y: bbox.top() }), a, "Expected intersection with top right corner");
        assert_eq!(Some(Point { x: bbox.left(), y: bbox.bottom() }), b, "Expected intersection with left bottom corner");

        // 45 degree line from box origin
        let (a, b) = bbox.intersect_line(&Point { x: 0.0, y: 0.0 }, &Point { x: bbox.left(), y: bbox.bottom() });
        assert_eq!(Some(Point { x: bbox.right(), y: bbox.top() }), a, "Expected intersection with top right corner");
        assert_eq!(Some(Point { x: bbox.left(), y: bbox.bottom() }), b, "Expected intersection with left bottom corner");

        // -45 degree line translated by (0.5,0.5) - top right ear
        let (a, b) = bbox.intersect_line(&Point { x: 0.5, y: 0.5 }, &Point { x: 0.4, y: 0.6 });
        assert_eq!(Some(Point { x: 1.0, y: 0.0 }), a, "Expected intersection with middle of the right edge");
        assert_eq!(Some(Point { x: 0.0, y: 1.0 }), b, "Expected intersection with middle of the top edge");

        // 45 degree line translated by (-0.5,0.5) - top left ear
        let (a, b) = bbox.intersect_line(&Point { x: -0.5, y: 0.5 }, &Point { x: -0.4, y: 0.6 });
        assert_eq!(Some(Point { x: -1.0, y: 0.0 }), a, "Expected intersection with middle of the left edge");
        assert_eq!(Some(Point { x: 0.0, y: 1.0 }), b, "Expected intersection with middle of the top edge");

        // -45 degree line translated by (-0.5,-0.5) - bottom left ear
        let (a, b) = bbox.intersect_line(&Point { x: -0.5, y: -0.5 }, &Point { x: -0.4, y: -0.6 });
        assert_eq!(Some(Point { x: -1.0, y: 0.0 }), a, "Expected intersection with middle of the left edge");
        assert_eq!(Some(Point { x: 0.0, y: -1.0 }), b, "Expected intersection with middle of the bottom edge");

        // 45 degree line translated by (0.5,-0.5) - bottom right ear
        let (a, b) = bbox.intersect_line(&Point { x: 0.5, y: -0.5 }, &Point { x: 0.4, y: -0.6 });
        assert_eq!(Some(Point { x: 1.0, y: 0.0 }), a, "Expected intersection with middle of the right edge");
        assert_eq!(Some(Point { x: 0.0, y: -1.0 }), b, "Expected intersection with middle of the bottom edge");
    }

    #[test]
    fn project_ray_tests_centered_square() {
        // square centered on origin with edges on x = +-1, y= +-1
        let bbox = BoundingBox::new_centered_square(2.0);

        // point to the right of right side, directed to origin
        let (a, b) = bbox.project_ray(&Point { x: 2.0, y: 0.0 }, &Point { x: -0.1, y: 0.0 });
        assert_eq!(Some(Point { x: 1.0, y: 0.0 }), a, "Expected to hit right side first");
        assert_eq!(Some(Point { x: -1.0, y: 0.0 }), b, "And then hit the left side");

        // point to the left of right side, inside, directed to origin
        let (a, b) = bbox.project_ray(&Point { x: 0.9, y: 0.0 }, &Point { x: -0.1, y: 0.0 });
        assert_eq!(Some(Point { x: -1.0, y: 0.0 }), a, "Expected to hit left side first");
        assert_eq!(None, b, "and only that");

        // point to the right of left side, inside, directed to origin
        let (a, b) = bbox.project_ray(&Point { x: -0.9, y: 0.0 }, &Point { x: 0.1, y: 0.0 });
        assert_eq!(Some(Point { x: 1.0, y: 0.0 }), a, "Expected to hit right side first");
        assert_eq!(None, b, "and only that");

        // point to the left of left side, directed to origin
        let (a, b) = bbox.project_ray(&Point { x: -2.0, y: 0.0 }, &Point { x: 0.1, y: 0.0 });
        assert_eq!(Some(Point { x: -1.0, y: 0.0 }), a, "Expected to hit left side first");
        assert_eq!(Some(Point { x: 1.0, y: 0.0 }), b, "And then hit the right side");

        // point to the top of top side, directed to origin
        let (a, b) = bbox.project_ray(&Point { x: 0.0, y: 3.0 }, &Point { x: 0.0, y: -10.0 });
        assert_eq!(Some(Point { x: 0.0, y: 1.0 }), a, "Expected to hit top side first");
        assert_eq!(Some(Point { x: 0.0, y: -1.0 }), b, "And then hit the bottom side");

        // point to the bottom of top side, inside, directed to origin
        let (a, b) = bbox.project_ray(&Point { x: 0.0, y: 0.5 }, &Point { x: 0.0, y: -10.0 });
        assert_eq!(Some(Point { x: 0.0, y: -1.0 }), a, "Expected to hit bottom side first");
        assert_eq!(None, b, "and only that");

        // point to the top of bottom side, directed to origin
        let (a, b) = bbox.project_ray(&Point { x: 0.0, y: -0.5 }, &Point { x: 0.0, y: 0.2 });
        assert_eq!(Some(Point { x: 0.0, y: 1.0 }), a, "Expected to hit top side first");
        assert_eq!(None, b, "and only that");

        // point to the bottom of the bottom side, directed to origin
        let (a, b) = bbox.project_ray(&Point { x: 0.0, y: -3.0 }, &Point { x: 0.0, y: 10.0 });
        assert_eq!(Some(Point { x: 0.0, y: -1.0 }), a, "Expected to hit bottom side first");
        assert_eq!(Some(Point { x: 0.0, y: 1.0 }), b, "And then hit the top side");

        // point to the right of top side, inside, directed to origin
        let (a, b) = bbox.project_ray(&Point { x: 0.0, y: 0.5 }, &Point { x: 0.0, y: -10.0 });
        assert_eq!(Some(Point { x: 0.0, y: -1.0 }), a, "Expected to hit bottom side first");
        assert_eq!(None, b, "and only that");

        // point to the right, outside box, negatively inclined
        let c = -0.8;
        let d = 1.0;
        let (a, b) = bbox.project_ray(&line(2.0, c, d), &direction(&line(-20.0, c, d), &line(2.0, c, d)));
        assert!(a.is_some() && b.is_some(), "Expected two intersections, a: {:?}, b: {:?}", a, b);
        assert!(
            abs_diff_eq(line(1.0, c, d).x, a.clone().unwrap().x, EQ_EPSILON)
            && abs_diff_eq(line(1.0, c, d).y, a.unwrap().y, EQ_EPSILON)
            , "Expected to hit right side first"
        );
        assert!(
            abs_diff_eq(line(0.0, c, d).x, b.clone().unwrap().x, EQ_EPSILON)
            && abs_diff_eq(line(0.0, c, d).y, b.unwrap().y, EQ_EPSILON)
            , "And then top side"
        );

        // point to the inside box, negatively inclined
        let (a, b) = bbox.project_ray(&Point { x: -0.5, y: 0.0 }, &Point { x: -1.0, y: 0.8 });
        assert_eq!(Some(Point { x: -1.0, y: 0.4 }), a, "Expected to hit left side first");
        assert_eq!(None, b, "And only that");

        // point to the left, outside box, positively inclined
        let (a, b) = bbox.project_ray(&Point { x: -10.0, y: 0.0 }, &Point { x: 1.0, y: 0.8 });
        assert!(a.is_none() && b.is_none(), "Expected no intersection");
    }

    #[test]
    /// Tests multiple non-centered rects in all 4 quadrants of the (x,y)-plane
    fn project_ray_tests_non_centered_rect() {
        let width_height_ratio = [
            1.5, //     exactly representable by float
            1.1, // not exactly representable by float (if only this fails == float equality checks might fail due to rounding errors)
            std::f64::consts::E // because why not
        ];
        let width = 1.;
        let base_origin = Point {x: 3.1, y: 2.6};

        // different rect side ratios
        for &ratio in width_height_ratio.iter() {
            let height = width * ratio;
            for i in 1..=2 {
                for j in 1..=2 {
                    // 4 (i,j)-combinations to mirror origin around x- and y-axes to obtain origins in all 4 quadrants
                    // -1^1 = -1
                    // -1^2 =  1
                    let origin = Point {
                        x: base_origin.x * -1_f64.powi(i),
                        y: base_origin.y * -1_f64.powi(j),
                    };

                    let bbox = BoundingBox::new(
                        origin.clone(),
                        width,
                        height
                    );

                    let top = origin.y + 0.5 * height;
                    let bottom = origin.y - 0.5 * height;
                    let right = origin.x + 0.5 * width;
                    let left = origin.x - 0.5 * width;

                    // point to the right of the right side, directed to origin
                    let point = &Point { x: origin.x + width, y: origin.y };
                    let dir = &Point { x: -0.1, y: 0.0 };
                    let (a, b) = bbox.project_ray(point, dir);
                    assert!(a.is_some() && b.is_some(), "Expected two intersections, a: {:?}, b: {:?}", a, b);
                    let expected_intersection_a = Point{x: right, y: origin.y};
                    assert!(
                        abs_diff_eq(expected_intersection_a.x, a.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_a.y, a.clone().unwrap().y, EQ_EPSILON)
                        , "Expected to hit right side first. found: {:?}, expected: {:?}", a.unwrap(), expected_intersection_a
                    );
                    let expected_intersection_b = Point{x: left, y: origin.y};
                    assert!(
                        abs_diff_eq(expected_intersection_b.x, b.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_b.y, b.clone().unwrap().y, EQ_EPSILON)
                        , "And then hit left side. found: {:?}, expected: {:?}", b.unwrap(), expected_intersection_b
                    );

                    // point to the left of left side, directed to origin
                    let point = &Point { x: origin.x - width, y: origin.y };
                    let dir = &Point { x: 0.1, y: 0.0 };
                    let (a, b) = bbox.project_ray(point, dir);
                    assert!(a.is_some() && b.is_some(), "Expected two intersections, a: {:?}, b: {:?}", a, b);
                    let expected_intersection_a = Point{x: left, y: origin.y};
                    assert!(
                        abs_diff_eq(expected_intersection_a.x, a.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_a.y, a.clone().unwrap().y, EQ_EPSILON)
                        , "Expected to hit left side first. found: {:?}, expected: {:?}", a.unwrap(), expected_intersection_a
                    );
                    let expected_intersection_b = Point{x: right, y: origin.y};
                    assert!(
                        abs_diff_eq(expected_intersection_b.x, b.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_b.y, b.clone().unwrap().y, EQ_EPSILON)
                        , "And then hit right side. found: {:?}, expected: {:?}", b.unwrap(), expected_intersection_b
                    );

                    // point to the left of right side, inside, directed to origin
                    let point = &Point { x: origin.x + 0.4 * width, y: origin.y };
                    let dir = &Point { x: -0.1, y: 0.0 };
                    let (a, b) = bbox.project_ray(point, dir);
                    assert!(a.is_some(), "Expected one intersection");
                    let expected_intersection_a = Point{x: left, y: origin.y};
                    assert!(
                        abs_diff_eq(expected_intersection_a.x, a.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_a.y, a.clone().unwrap().y, EQ_EPSILON)
                        , "Expected to hit left side first. found: {:?}, expected: {:?}", a.unwrap(), expected_intersection_a
                    );
                    assert_eq!(None, b, "And only that");

                    // point to the right of left side, inside, directed to origin
                    let point = &Point { x: origin.x - 0.4 * width, y: origin.y };
                    let dir = &Point { x: 0.1, y: 0.0 };
                    let (a, b) = bbox.project_ray(point, dir);
                    assert!(a.is_some(), "Expected one intersection");
                    let expected_intersection_a = Point{x: right, y: origin.y};
                    assert!(
                        abs_diff_eq(expected_intersection_a.x, a.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_a.y, a.clone().unwrap().y, EQ_EPSILON)
                        , "Expected to hit right side first. found: {:?}, expected: {:?}", a.unwrap(), expected_intersection_a
                    );
                    assert_eq!(None, b, "And only that");

                    // point to the top of top side, directed to origin
                    let point = &Point { x: origin.x, y: origin.y + height};
                    let dir = &Point { x: 0.0, y: -0.1 };
                    let (a, b) = bbox.project_ray(point, dir);
                    assert!(a.is_some() && b.is_some(), "Expected two intersections, a: {:?}, b: {:?}", a, b);
                    let expected_intersection_a = Point{x: origin.x, y: top};
                    assert!(
                        abs_diff_eq(expected_intersection_a.x, a.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_a.y, a.clone().unwrap().y, EQ_EPSILON)
                        , "Expected to hit top side first. found: {:?}, expected: {:?}", a.unwrap(), expected_intersection_a
                    );
                    let expected_intersection_b = Point{x: origin.x, y: bottom};
                    assert!(
                        abs_diff_eq(expected_intersection_b.x, b.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_b.y, b.clone().unwrap().y, EQ_EPSILON)
                        , "And then hit bottom side. found: {:?}, expected: {:?}", b.unwrap(), expected_intersection_b
                    );

                    // point to the bottom of the bottom side, directed to origin
                    let point = &Point { x: origin.x, y: origin.y - height};
                    let dir = &Point { x: 0.0, y: 0.1 };
                    let (a, b) = bbox.project_ray(point, dir);
                    assert!(a.is_some() && b.is_some(), "Expected two intersections, a: {:?}, b: {:?}", a, b);
                    let expected_intersection_a = Point{x: origin.x, y: bottom};
                    assert!(
                        abs_diff_eq(expected_intersection_a.x, a.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_a.y, a.clone().unwrap().y, EQ_EPSILON)
                        , "Expected to hit bottom side first. found: {:?}, expected: {:?}", a.unwrap(), expected_intersection_a
                    );
                    let expected_intersection_b = Point{x: origin.x, y: top};
                    assert!(
                        abs_diff_eq(expected_intersection_b.x, b.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_b.y, b.clone().unwrap().y, EQ_EPSILON)
                        , "And then hit top side. found: {:?}, expected: {:?}", b.unwrap(), expected_intersection_b
                    );

                    // point to the bottom of top side, inside, directed to origin
                    let point = &Point { x: origin.x, y: origin.y + 0.4 * height};
                    let dir = &Point { x: 0.0, y: -0.1 };
                    let (a, b) = bbox.project_ray(point, dir);
                    assert!(a.is_some(), "Expected one intersection");
                    let expected_intersection_a = Point{x: origin.x, y: bottom};
                    assert!(
                        abs_diff_eq(expected_intersection_a.x, a.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_a.y, a.clone().unwrap().y, EQ_EPSILON)
                        , "Expected to hit bottom side first. found: {:?}, expected: {:?}", a.unwrap(), expected_intersection_a
                    );
                    assert_eq!(None, b, "And only that");

                    // point to the top of bottom side, directed to origin
                    let point = &Point { x: origin.x, y: origin.y - 0.4 * height};
                    let dir = &Point { x: 0.0, y: 0.1 };
                    let (a, b) = bbox.project_ray(point, dir);
                    assert!(a.is_some(), "Expected one intersection");
                    let expected_intersection_a = Point{x: origin.x, y: top};
                    assert!(
                        abs_diff_eq(expected_intersection_a.x, a.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_a.y, a.clone().unwrap().y, EQ_EPSILON)
                        , "Expected to hit top side first. found: {:?}, expected: {:?}", a.unwrap(), expected_intersection_a
                    );
                    assert_eq!(None, b, "And only that");

                    // point to the right-top of of the origin, inside, directed to bottom
                    let point = &Point { x: origin.x + 0.3 * width, y: origin.y + 0.3 * height };
                    let dir = &Point { x: 0.0, y: -10.0 };
                    let (a, b) = bbox.project_ray(point, dir);
                    assert!(a.is_some(), "Expected one intersection");
                    let expected_intersection_a = Point{x: origin.x + 0.3 * width, y: bottom};
                    assert!(
                        abs_diff_eq(expected_intersection_a.x, a.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_a.y, a.clone().unwrap().y, EQ_EPSILON)
                        , "Expected to hit bottom side first. found: {:?}, expected: {:?}", a.unwrap(), expected_intersection_a
                    );
                    assert_eq!(None, b, "And only that");

                    // point to the right, outside box, negatively inclined
                    let c = -0.8 * ratio; // to make sure the line is not too steep and hits bottom instead of right side
                    let d = -(c * origin.x) + top; // d = -(c*x) + y -> provoke intersection at (origin.x, top)
                    let point = &line(right + 0.5, c, d); // right of right side
                    let dir = &direction(&line(left - 2., c, d), point); // pointing towards box
                    let (a, b) = bbox.project_ray(point, dir);
                    assert!(a.is_some() && b.is_some(), "Expected two intersections, a: {:?}, b: {:?}", a, b);
                    let expected_intersection_a = line(right, c, d);
                    assert!(
                        abs_diff_eq(expected_intersection_a.x, a.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_a.y, a.clone().unwrap().y, EQ_EPSILON)
                        , "Expected to hit right side first. found: {:?}, expected: {:?}", a.unwrap(), expected_intersection_a
                    );
                    let expected_intersection_b = line(origin.x, c, d);
                    assert!(
                        abs_diff_eq(line(origin.x, c, d).x, b.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(line(origin.x, c, d).y, b.clone().unwrap().y, EQ_EPSILON)
                        , "And then top side. found: {:?}, expected: {:?}", b.unwrap(), expected_intersection_b
                    );

                    // point to the top-left of of the origin, inside, pointing up-left outwards
                    let c = -0.8 * ratio; // to make sure the line is not too steep and unwantedly hits bottom instead of right side
                    let d = -(c * left) + origin.y; // d = -(c*x) + y -> provoke intersection at (left, origin.y)
                    let point = &line(left + 0.2 * width, c, d); // right of left side, inside
                    let dir = &direction(&line(left - width, c, d), point); // outward direction
                    let (a, b) = bbox.project_ray(point, dir);
                    assert!(a.is_some(), "Expected one intersection");
                    let expected_intersection_a = line(left, c, d);
                    assert!(
                        abs_diff_eq(expected_intersection_a.x, a.clone().unwrap().x, EQ_EPSILON)
                        && abs_diff_eq(expected_intersection_a.y, a.clone().unwrap().y, EQ_EPSILON)
                        , "Expected to hit left side first. found: {:?}, expected: {:?}", a.unwrap(), expected_intersection_a
                    );
                    assert_eq!(None, b, "And only that");

                    // point to the left, outside box, positively inclined
                    let point = &Point { x: left - width, y: origin.y};
                    let dir = &Point { x: 0.9 * width, y: ratio }; // makes sure that the incline is steep enough to pass the top-left corner on the outside
                    let (a, b) = bbox.project_ray(point, dir);
                    assert!(a.is_none() && b.is_none(), "Expected no intersection");
                }
            }
        }
    }
}