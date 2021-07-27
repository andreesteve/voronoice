extern crate image;
use rand::Rng;

use std::{env, u8};
use voronoice::{Voronoi, VoronoiBuilder, BoundingBox, Point};

fn main() {
    let path = env::args().nth(1).expect("path to image file expected as first parameter");
    let factor = env::args().nth(2).expect("expected secong argument to be an integer factor").parse::<f64>().expect("expected secong argument to be an integer factor") / 10000.0;
    let mut img = image::open(path).unwrap().into_rgb8();
    let width = img.width() as usize;
    let height = img.height() as usize;

    // generate sites
    let size  = ((width * height) as f64 * factor) as usize;
    println!("Generating {} sites (factor {}, image dimensions: {} x {}", size, factor, width, height);
    let mut rng = rand::thread_rng();
    let x_range = rand::distributions::Uniform::new(0, width);
    let y_range = rand::distributions::Uniform::new(0, height);
    let sites = (0..size)
        .map(move |_| Point { x: rng.sample(x_range) as f64, y: rng.sample(y_range) as f64 })
        .collect();

    println!("Generating voronoi diagram");
    let voronoi = VoronoiBuilder::default()
        .set_sites(sites)
        .set_clip_behavior(voronoice::ClipBehavior::None)
        // image origin is top left corner, center is width/2,height/2
        .set_bounding_box(BoundingBox::new(Point { x: width as f64 / 2.0, y: height as f64 / 2.0 }, width as f64, height as f64))
        .build()
        .unwrap();

    // keep track of accumulated color per cell
    let mut cells = vec![(0usize, 0usize, 0usize, 0usize); voronoi.sites().len()];
    let mut pixel_to_site = vec![0; width * height];

    println!("Accumulating cell colors");
    let mut last_size = 0;
    for x in 0..width-1 {
        for y in 0..height-1 {
            let pindex = width * y + x;
            let x = x as u32;
            let y = y as u32;

            // get site/voronoi cell for which pixel belongs to
            let site = get_cell(&voronoi, last_size, x, y);
            last_size = site;
            pixel_to_site[pindex] = site;

            // accumulate color per cell
            let pixel = img.get_pixel(x, y);
            let cell_site = &mut cells[site];

            cell_site.0 += pixel.0[0] as usize;
            cell_site.1 += pixel.0[1] as usize;
            cell_site.2 += pixel.0[2] as usize;
            cell_site.3 += 1;
        }
    }

    println!("Averaging cell colors");
    // average value per cell
    for cell in cells.iter_mut() {
        if cell.3 > 0 {
            cell.0 = cell.0 / cell.3;
            cell.1 = cell.1 / cell.3;
            cell.2 = cell.2 / cell.3;
        }
    }

    println!("Generating image");
    // assign averaged color to pixels in cells
    for x in 0..width-1 {
        for y in 0..height-1 {
            let pindex = width * y + x;
            let x = x as u32;
            let y = y as u32;
            let site = pixel_to_site[pindex];
            let color = cells[site];
            let pixel = img.get_pixel_mut(x, y);
            pixel.0[0] = color.0 as u8;
            pixel.0[1] = color.1 as u8;
            pixel.0[2] = color.2 as u8;
        }
    }

    // Write the contents of this image to the Writer in PNG format.
    img.save("out.jpg").unwrap();
}

fn get_cell(voronoi: &Voronoi, current_site: usize, x: u32, y: u32) -> usize {
    let p = Point { x: x as f64, y: y as f64 };
    voronoi
        .cell(current_site)
        .iter_path(p)
        .last()
        .expect("Expected to find site that contains point")
}