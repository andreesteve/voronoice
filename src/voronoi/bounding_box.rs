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

    pub fn is_inside(&self, point: &Point) -> bool {
        point.x.abs() <= self.top_right.x && point.y.abs() <= self.top_right.y
    }

    pub fn project_vector(&self, point: &Point, direction: &Point) -> Point {
        let box_x = self.top_right.x * direction.x.signum();
        let box_y = self.top_right.y * direction.y.signum();

        let mut p = Point { x: 0.0, y: 0.0 };
        p.x = point.x + direction.x * (1.0 + (box_y - (point.y + direction.y)) / direction.y);
        p.y = point.y + direction.y * (1.0 + (box_x - (point.x + direction.x)) / direction.x);

        if p.x.abs() > self.top_right.x {
            p.x = box_x;
        }

        if p.y.abs() > self.top_right.y {
            p.y = box_y;
        }


        // if tn > box_ratio || tn < -box_ratio {
        //     // will hit box_y or -box_y
        //     let box_y = bounding_box.y * direction.y.signum();
        //     p.x = point.x + direction.x * (1.0 + (box_y - (point.y + direction.y)) / direction.y);
        //     p.y = box_y;
        // } else {
        //     // will hit box_x or - box_x
        //     let box_x = bounding_box.x * direction.x.signum();
        //     p.x = box_x;
        //     p.y = point.y + direction.y * (1.0 + (box_x - (point.x + direction.x)) / direction.x);
        // }

        debug_assert!(self.is_inside(&p), "Point {:?} with direction {:?} projected ({:?}) outside bounding box {:?}", point, direction, p, self);
        debug_assert!(p.x.abs() == self.top_right.x || p.y.abs() == self.top_right.y.abs(), "Point {:?} with direction {:?} projected ({:?}) outside bounding box {:?}", point, direction, p, self);

        p
    }
}