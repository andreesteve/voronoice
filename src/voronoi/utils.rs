use delaunator::Point;
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
pub fn into_line_list<T>(iter: impl Iterator<Item = T> + Clone)-> impl Iterator<Item = T> {
    let c = iter.clone();
    iter.zip(c.cycle().skip(1))
        .flat_map(|(a, b)| std::iter::once(a).chain(std::iter::once(b)))
}

/// Gets the index of the triangle (starting half-edge) this half-edge belongs to.
pub fn triangle_of_edge(edge: usize) -> usize {
    edge / 3
}

#[allow(dead_code)]
pub fn generate_circle_sites(size: usize) -> Vec<Point> {
    let len = size;
    let r = 1.0;
    let mut sites = vec![];
    sites.push(Point { x: 0.0, y: 0.0 });
    for i in 0..len {
        let a = (i as f64 * 360.0 / len as f64).to_radians();
        sites.push(Point {
            x: r * a.sin(),
            y: r * a.cos()
        });
    }

    sites
}

#[allow(dead_code)]
pub fn generate_square_sites(width: usize, height: usize) -> Vec<Point> {
    let mut sites = vec![];
    let fwidth = width as f64;
    let fheight = height as f64;

    for i in 0..width {
        for j in 0..height {
            sites.push(Point {
                x: i as f64 / fwidth - 0.5,
                y: j as f64/ fheight - 0.5
            });
        }
    }

    sites
}

#[allow(dead_code)]
pub fn generate_triangle_sites() -> Vec<Point> {
    let mut sites = vec![];

    sites.push(Point { x: -0.3, y: -0.3 });
    sites.push(Point { x: 0.3, y: -0.3 });
    sites.push(Point { x: 0.0, y: 0.3 });

    sites.push(Point { x: -0.6, y: -0.6 });
    sites.push(Point { x: 0.6, y: -0.6 });
    sites.push(Point { x: 0.0, y: 0.6 });

    sites.push(Point { x: -1.0, y: -1.0 });
    sites.push(Point { x: -1.0, y: 1.0 });
    sites.push(Point { x: 1.0, y: -1.0 });
    sites.push(Point { x: 1.0, y: 1.0 });

    sites
}

#[allow(dead_code)]
pub fn generate_special_case_1() -> Vec<Point> {
    let mut sites = vec![];

    sites.push(Point { x: -0.5, y: -0.5 });
    sites.push(Point { x: -0.5, y: 0.0 });
    sites.push(Point { x: 0.0, y: 0.0 });
    sites.push(Point { x: 0.5, y: -1.0 });

    sites
}

#[allow(dead_code)]
pub fn generate_special_case_2() -> Vec<Point> {
    let mut sites = vec![];

    sites.push(Point { x: 0.0, y: 0.0 });
    sites.push(Point { x: 0.22, y: -0.75 });
    sites.push(Point { x: 0.25, y: -0.83 });
    sites.push(Point { x: -0.50, y: 0.0 });

    sites
}