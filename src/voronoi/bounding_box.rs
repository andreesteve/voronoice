use delaunator::EPSILON;

use super::Point;
#[derive(Debug, Clone)]
pub struct BoundingBox {
    /// The center point of a rectangle.
    center: Point,

    /// The top right point of a rectangle.
    top_right: Point,
}

impl Default for BoundingBox {
    fn default() -> Self {
        Self::new_centered(1.0, 1.0)
    }
}

impl BoundingBox {
    pub fn new(origin: Point, width: f64, height: f64) -> Self {
        Self {
            top_right: Point { x: origin.x + width / 2.0, y: origin.y + height / 2.0 },
            center: origin,
        }
    }

    /// New bounding box centeterd at origin.
    pub fn new_centered(width: f64, height: f64) -> Self {
        Self::new(Point { x: 0.0, y: 0.0 }, width, height)
    }

    pub fn new_centered_square(width: f64) -> Self {
        Self::new_centered(width, width)
    }

    pub fn center(&self) -> &Point {
        &self.center
    }

    pub fn top_right(&self) -> &Point {
        &self.top_right
    }

    pub fn width(&self) -> f64 {
        2.0 * (self.top_right.x - self.center.x)
    }

    pub fn height(&self) -> f64 {
        2.0 * (self.top_right.y - self.center.y)
    }

    #[inline]
    pub fn is_inside(&self, point: &Point) -> bool {
        point.x.abs() <= self.top_right.x && point.y.abs() <= self.top_right.y
    }

    /// Same as inside, but return false if point is on the box edge.
    #[inline]
    pub fn is_exclusively_inside(&self, point: &Point) -> bool {
        point.x.abs() < self.top_right.x && point.y.abs() < self.top_right.y
    }

    /// Intersects a line represented by points 'a' and 'b' and returns the two intersecting points with the box, or None
    pub fn intersect_line(&self, a: &Point, b: &Point) -> (Option<Point>, Option<Point>) {
        let c_x = b.x - a.x;
        let c_y = b.y - a.y;
        let c = c_y / c_x;
        let d = a.y - (a.x * c);

        let mut f = None;
        let mut g = None;
        let mut h = None;
        let mut i = None;

        // intersection left, right edges
        if c_x.abs() > EPSILON {
            // y = c*x + d
            let right_y = (self.top_right.x * c) + d;
            let left_y = d - (self.top_right.x * c);

            if right_y.abs() <= self.top_right.y {
                f = Some(Point { x: self.top_right.x, y: right_y });
            }

            if left_y.abs() <= self.top_right.y {
                g = Some(Point { x: -self.top_right.x, y: left_y })
            }

            if g.is_some() && f.is_some() {
                // can't have more than 2 intersections, we are done
                return (f, g);
            }
        } // else line is parallel to y, won't intersect with left/right

        // intersection top, bottom edges
        if c_y.abs() > EPSILON {
            if c_x.abs() < EPSILON {
                // line is parallel to y
                if a.x.abs() <= self.top_right.x {
                    // and crosses box
                    return (
                        Some(Point { x: a.x, y: self.top_right.y }),
                        Some(Point { x: a.x, y: -self.top_right.y })
                    );
                } else {
                    // does not cross box
                    return (None, None);
                }
            }

            // x = (y - d) / c
            let top_x = (self.top_right.y - d) / c;
            let bottom_x = -(d + self.top_right.y) / c;

            if top_x.abs() <= self.top_right.x {
                h = Some(Point { x: top_x, y: self.top_right.y })
            }

            if bottom_x.abs() <= self.top_right.x {
                i = Some(Point { x: bottom_x, y: -self.top_right.y })
            }

            if h.is_some() && i.is_some() {
                // can't have more than 2 intersections, we are done
                return (h, i);
            }
        } // else line is parallel to x, won't intersect with top / bottom

        (f.or(g), h.or(i))
    }

    // /// Similar to intersect_line, but if the intersection points are not in the segment, None is returned instead.
    // pub fn intersect_line_segment(&self, a: &Point, b: &Point) -> (Option<Point>, Option<Point>) {
    //     let (max_x, min_x) = if a.x > b.x {
    //         (a.x, b.x)
    //     } else {
    //         (b.x, a.x)
    //     };

    //     let (max_y, min_y) = if a.y > b.y {
    //         (a.y, b.y)
    //     } else {
    //         (b.y, a.y)
    //     };

    //     let filter_on_segment = |p: Point| -> Option<Point> {
    //         if p.x <= max_x && p.x >= min_x && p.y <= max_y && p.y >= min_y {
    //             Some(p)
    //         } else {
    //             None
    //         }
    //     };

    //     let (c, d) = self.intersect_line(a, b);

    //     (
    //         c.map_or(None, filter_on_segment),
    //         d.map_or(None, filter_on_segment)
    //     )
    // }

    /// Intersects a ray with the bounding box. The first intersection is returned first.
    pub fn project_ray(&self, point: &Point, direction: &Point) -> (Option<Point>, Option<Point>) {
        let b = Point { x: point.x + direction.x, y: point.y + direction.y };
        let (a, b) = self.intersect_line(point, &b);
        order_points_on_ray(point, direction, a, b)
    }

    /// Same as `project_ray` when you don't care abount the second intersecting point.
    pub fn project_ray_closest(&self, point: &Point, direction: &Point) -> Option<Point> {
        self.project_ray(point, direction).0
    }
}

/// Given a ray defined by `point` and `direction, and two points on such ray `a` and `b`, returns a tuple (w, z) where point <= w <= z.
/// If either `a` or `b` are smaller than `point`, None is returned.
pub fn order_points_on_ray(point: &Point, direction: &Point, a: Option<Point>, b: Option<Point>) -> (Option<Point>, Option<Point>) {
    if let Some(va) = a {
        if let Some(vb) = b {
            // point, a and b are just an scalar times direction, so we can compare any non-zero
            // direction component, use the largest
            let (d, da, db) = if direction.x.abs() > direction.y.abs() {
                // use x for comparison
                (direction.x, va.x - point.x, vb.x - point.x)
            } else {
                (direction.y, va.y - point.y, vb.y - point.y)
            };

            if d.signum() == da.signum() {
                // a is reacheable
                if d.signum() == db.signum() {
                    // b is reacheable
                    if da.abs() > db.abs() {
                        // b is closer
                        (Some(vb), Some(va))
                    } else {
                        // a is closer
                        (Some(va), Some(vb))
                    }
                } else {
                    // b will never be reached
                    (Some(va), None)
                }
            } else if d.signum() == db.signum() {
                // a will never be reached
                (Some(vb), None)
            } else {
                // neither will be reached
                (None, None)
            }
        } else if direction.x.signum() == va.x.signum() && direction.y.signum() == va.y.signum() {
            // a is in the right direction
            (Some(va), None)
        } else {
            // a can't be reached
            (None, None)
        }
    } else {
        // no intersection
        (None, None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(x: f64, c: f64, d: f64) -> Point {
        Point { x, y: (x * c) + d }
    }
    fn direction(a: &Point, b: &Point) -> Point {
        Point { x: a.x - b.x, y: a.y - b.y }
    }

    fn assert_close_enough(a: Point, b: Option<Point>, message: &str) {
        let b = b.expect(&format!("Expected value, but found None. {}", message));
        let close_enough = 5.0 * EPSILON;
        assert!((a.x - b.x).abs() < close_enough, "a.x [{:?}] and b.x [{:?}] expected to be close enough. Difference was: [{}]. {}", a.x, b.x, (a.x - b.x).abs(), message);
        assert!((a.y - b.y).abs() < close_enough, "a.y [{:?}] and b.y [{:?}] expected to be close enough. Difference was: [{}]. {}", a.y, b.y, (a.y - b.y).abs(), message);
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
        assert_eq!(Some(Point { x: 0.0, y: 1.0 }), a, "Expected intersection with top edge");
        assert_eq!(Some(Point { x: 0.0, y: -1.0 }), b, "Expected intersection with bottom edge");

        // line parallel to y, crossing box
        let (a, b) = bbox.intersect_line(&Point { x: 0.0, y: 0.0 }, &Point { x: 1.0, y: 0.0 }); // y = 0
        assert_eq!(Some(Point { x: 1.0, y: 0.0 }), a, "Expected intersection with right edge");
        assert_eq!(Some(Point { x: -1.0, y: 0.0 }), b, "Expected intersection with left edge");

        // line congruent to top edge
        let (a, b) = bbox.intersect_line(&Point { x: 0.0, y: 1.0 }, &Point { x: 1.0, y: 1.0 }); // y = 1
        assert_eq!(Some(Point { x: 1.0, y: 1.0 }), a, "Expected intersection with top right corner");
        assert_eq!(Some(Point { x: -1.0, y: 1.0 }), b, "Expected intersection with top left corner");

        // line congruent to bottom edge
        let (a, b) = bbox.intersect_line(&Point { x: 0.0, y: -1.0 }, &Point { x: 1.0, y: -1.0 }); // y = -1
        assert_eq!(Some(Point { x: 1.0, y: -1.0 }), a, "Expected intersection with bottom right corner");
        assert_eq!(Some(Point { x: -1.0, y: -1.0 }), b, "Expected intersection with bottom left corner");

        // line congruent to right edge
        let (a, b) = bbox.intersect_line(&Point { x: 1.0, y: 0.0 }, &Point { x: 1.0, y: 1.0 }); // x = 1
        assert_eq!(Some(Point { x: 1.0, y: 1.0 }), a, "Expected intersection with top right corner");
        assert_eq!(Some(Point { x: 1.0, y: -1.0 }), b, "Expected intersection with bottom right corner");

        // line congruent to left edge
        let (a, b) = bbox.intersect_line(&Point { x: -1.0, y: 0.0 }, &Point { x: -1.0, y: 1.0 }); // x = -1
        assert_eq!(Some(Point { x: -1.0, y: 1.0 }), a, "Expected intersection with top left corner");
        assert_eq!(Some(Point { x: -1.0, y: -1.0 }), b, "Expected intersection with bottom left corner");

        // 45 degree line from box origin
        let (a, b) = bbox.intersect_line(&Point { x: 0.0, y: 0.0 }, &Point { x: 1.0, y: 1.0 });
        assert_eq!(Some(Point { x: 1.0, y: 1.0 }), a, "Expected intersection with top right corner");
        assert_eq!(Some(Point { x: -1.0, y: -1.0 }), b, "Expected intersection with left bottom corner");

        // -45 degree line from box origin
        let (a, b) = bbox.intersect_line(&Point { x: 0.0, y: 0.0 }, &Point { x: -1.0, y: -1.0 });
        assert_eq!(Some(Point { x: 1.0, y: 1.0 }), a, "Expected intersection with top right corner");
        assert_eq!(Some(Point { x: -1.0, y: -1.0 }), b, "Expected intersection with left bottom corner");

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
    fn project_ray_tests() {
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

        // point to the right of top side, inside, directed to origin
        let (a, b) = bbox.project_ray(&Point { x: 0.0, y: 0.5 }, &Point { x: 0.0, y: -10.0 });
        assert_eq!(Some(Point { x: 0.0, y: -1.0 }), a, "Expected to hit bottom side first");
        assert_eq!(None, b, "and only that");

        // point to the right, outside box, negatively inclined
        let c = -0.8;
        let d = 1.0;
        let (a, b) = bbox.project_ray(&line(2.0, c, d), &direction(&line(-20.0, c, d), &line(2.0, c, d)));
        assert_close_enough(line(1.0, c, d), a, "Expected to hit left side first");
        assert_close_enough(line(0.0, c, d), b, "And then top side");

        // point to the inside box, negatively inclined
        let (a, b) = bbox.project_ray(&Point { x: -0.5, y: 0.0 }, &Point { x: -1.0, y: 0.8 });
        assert_eq!(Some(Point { x: -1.0, y: 0.4 }), a, "Expected to hit bottom side first");
        assert_eq!(None, b, "No collision");

        // point to the right, outside box, positively inclined
        let (a, b) = bbox.project_ray(&Point { x: -10.0, y: 0.0 }, &Point { x: 1.0, y: 0.8 });
        assert_eq!(None, a, "No collision");
        assert_eq!(None, b, "No collision");
    }
}