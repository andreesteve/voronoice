mod voronoi;
pub use voronoi::*;

#[cfg(test)]
mod tests {
    use rand::Rng;

    use super::*;

    fn create_random_builder(size: usize) -> VoronoiBuilder {
        let mut rng = rand::thread_rng();
        let builder = VoronoiBuilder::default();
        let bbox = BoundingBox::default();

        let x_range = rand::distributions::Uniform::new(-bbox.width(), bbox.width());
        let y_range = rand::distributions::Uniform::new(-bbox.height(), bbox.height());
        let sites = (0..size)
            .map(|_| Point { x: rng.sample(x_range), y: rng.sample(y_range) })
            .collect();

        builder
            .set_bounding_box(bbox)
            .set_sites(sites)
    }

    #[test]
    fn random_site_generation_test() {
        create_random_builder(100_000)
            .build()
            .expect("Some voronoi expected.");
    }
}