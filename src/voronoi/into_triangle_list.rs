#[must_use = "iterators are lazy and do nothing unless consumed"]
pub struct IntoTriangleListIter<I, A> {
    first: Option<A>,
    second: Option<A>,
    third: Option<A>,
    iter: I,
    counter: usize
}

impl<A, I: Iterator<Item = A>> IntoTriangleListIter<I, A> where A : Copy {
    pub fn new(mut iter: I) -> Self {
        let first = iter.next();
        let second = iter.next();
        IntoTriangleListIter {
            first,
            second,
            iter,
            third: None,
            counter: 0
        }
    }
}

impl<A, I: Iterator<Item = A>> Iterator for IntoTriangleListIter<I, A> where A : Copy {
    type Item = A;
    fn next(&mut self) -> Option<Self::Item> {
        match self.counter {
            0 => {
                self.counter += 1;
                self.third = self.iter.next();
                self.third
            },
            1 => {
                self.counter += 1;
                // we flip triangles here because often the input is clockwise
                let second = self.second;
                self.second = self.third;
                second
            },
            _ => {
                self.counter = 0;
                self.first
            },
        }
    }
}

pub trait IntoTriangleList: Sized {
    type Item;
    fn into_triangle_list(self) -> IntoTriangleListIter<Self, Self::Item>;
}

impl<I: Iterator> IntoTriangleList for I where I::Item : Copy {
    type Item = I::Item;
    fn into_triangle_list(self) -> IntoTriangleListIter<Self, Self::Item> {
        IntoTriangleListIter::new(self)
    }
}

#[cfg(test)]
mod tests {
    use std::iter::empty;

    use super::*;
    #[test]
    fn into_triangle_list_expect_triangles() {
        let mut iter = [1, 2, 3, 4].iter().into_triangle_list();
        // first triangle
        assert_eq!(Some(&3), iter.next());
        assert_eq!(Some(&1), iter.next());
        assert_eq!(Some(&2), iter.next());

        // second triangle
        assert_eq!(Some(&4), iter.next());
        assert_eq!(Some(&1), iter.next());
        assert_eq!(Some(&3), iter.next());

        // nothing
        assert_eq!(None, iter.next());
    }

    #[test]
    fn into_triangle_list_expect_no_triangles() {
        let mut iter = [1, 2].iter().into_triangle_list();

        // nothing
        assert_eq!(None, iter.next());
    }

    #[test]
    fn into_triangle_list_expect_no_triangles_from_empty() {
        let mut iter = empty::<u32>().into_triangle_list();

        // nothing
        assert_eq!(None, iter.next());
    }
}