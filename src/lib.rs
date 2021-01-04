mod voronoi;
pub use voronoi::*;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn random_site_generation_test() {
        let v = VoronoiBuilder::default()
            .generate_random_sites_constrained(100_000)
            .build();
        v.expect("Some voronoi expected.");
    }
}