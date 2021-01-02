use delaunator::{Point, Triangulation, next_halfedge};
use bevy::prelude::Color;

pub fn to_f32_vec(p: &Vec<Point>) -> Vec<[f32; 3]> {
    p
        .iter()
        .map(|c| { [c.y as f32, 0.0, c.x as f32] })
        .collect()
}

pub fn color_to_f32_vec(color: Color) -> [f32; 3] {
    [color.r(), color.g(), color.b()]
}

/// Returns an iterator that interleaves sequential pairs. The last element is paired with the first.
///
/// # Example
///```
/// let a = [1, 2, 3];
/// let iter = iter_line_list(a);
/// assert_eq!(1, iter.next());
/// assert_eq!(2, iter.next());
/// assert_eq!(2, iter.next());
/// assert_eq!(3, iter.next());
/// assert_eq!(3, iter.next());
/// assert_eq!(1, iter.next());
/// assert_eq!(None, iter.next());
///```
pub fn into_line_list_wrap<T>(iter: impl Iterator<Item = T> + Clone)-> impl Iterator<Item = T> {
    let c = iter.clone();
    iter.zip(c.cycle().skip(1))
        .flat_map(|(a, b)| std::iter::once(a).chain(std::iter::once(b)))
}

pub fn into_line_list<T>(iter: impl Iterator<Item = T> + Clone)-> impl Iterator<Item = T> {
    let c = iter.clone();
    iter.zip(c.skip(1))
        .flat_map(|(a, b)| std::iter::once(a).chain(std::iter::once(b)))
}

/// Gets the index of the triangle (starting half-edge) this half-edge belongs to.
pub fn triangle_of_edge(edge: usize) -> usize {
    edge / 3
}

/// Returns the index to the site that half-edge `e` points to.
/// This is similar to `triangles`. Given an half-edge `e`, `triangles` returns the index of the site the half-edge start off. `site_of_incoming` returns the index of the site the half-edge points to.
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