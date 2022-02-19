use delaunator::{Point, Triangulation, next_halfedge};

/// Gets the index of the triangle (starting half-edge) this half-edge belongs to.
#[inline]
pub fn triangle_of_edge(edge: usize) -> usize {
    edge / 3
}

/// Returns the index to the site that half-edge `e` points to.
/// This is similar to `triangles`. Given an half-edge `e`, `triangles` returns the index of the site the half-edge start off. `site_of_incoming` returns the index of the site the half-edge points to.
#[inline]
pub fn site_of_incoming(triangulation: &Triangulation, e: usize) -> usize {
    triangulation.triangles[next_halfedge(e)]
}

pub fn calculate_approximated_cetroid<'a>(points: impl Iterator<Item = &'a Point>) -> Point {
    let mut r = Point { x: 0.0 , y: 0.0 };
    let mut n = 0;
    for p in points {
        r.x += p.x;
        r.y += p.y;
        n += 1;
    }

    let n = n as f64;
    r.x /= n;
    r.y /= n;

    r
}

/// Calculates the midpoint of the vector a -> b.
pub fn midpoint(a: &Point, b: &Point) -> Point {
    Point { x: (a.x + b.x) / 2.0, y: (a.y + b.y) / 2.0 }
}

pub fn cicumcenter(a: &Point, b: &Point, c: &Point) -> Point {
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

/// Calculates the squared distance between a and b
pub fn dist2(a: &Point, b: &Point) -> f64 {
    let x = a.x - b.x;
    let y = a.y - b.y;
    (x * x) + (y * y)
}

#[inline]
pub fn abs_diff_eq(a: f64, b: f64, epsilon: f64) -> bool {
    (if a > b {
        a - b
    } else {
        b - a
    }) <= epsilon
}