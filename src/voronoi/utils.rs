use delaunator::Point;
use bevy::prelude::Color;

pub fn to_f32_vec(p: &Vec<Point>) -> Vec<[f32; 3]> {
    p
        .iter()
        .map(|c| { [c.x as f32, 0.0, c.y as f32] })
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