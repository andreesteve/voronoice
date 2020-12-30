use bevy::{
    prelude::*,
    render::{mesh::{Indices, Mesh}, pipeline::PrimitiveTopology},
};
use utils::{into_line_list_wrap, into_line_list};
use super::{
    utils,
    HullBehavior, Voronoi,
    into_triangle_list::*,
};

pub struct VoronoiMeshGenerator<'a> {
    pub voronoi: &'a Voronoi,
    pub coloring: fn(usize) -> Color,
    pub topology: PrimitiveTopology
}

impl VoronoiMeshGenerator<'_> {
    #[allow(dead_code)]
    pub fn build_circumcenters_mesh(&self) -> Mesh {
        let positions: Vec<[f32; 3]> = utils::to_f32_vec(&self.voronoi.circumcenters);
        let num_of_vertices = positions.len();
        let normals: Vec<[f32; 3]> = vec![[0.0, 1.0, 0.0]; num_of_vertices];
        let uvs: Vec<[f32; 2]> = vec![[0.0, 0.0]; num_of_vertices];
        let colors: Vec<[f32; 3]> = (0..num_of_vertices)
            .map(self.coloring)
            .map(utils::color_to_f32_vec)
            .collect();

        let indices = (0..num_of_vertices).map(|e| e as u32).collect();

        let mut mesh = Mesh::new(self.topology);
        mesh.set_indices(Some(Indices::U32(indices)));
        mesh.set_attribute(Mesh::ATTRIBUTE_POSITION, positions);
        mesh.set_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        mesh.set_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
        mesh.set_attribute("Vertex_Color", colors);
        mesh
    }

    pub fn build_delauney_mesh(&self) -> Mesh {
        let positions: Vec<[f32; 3]> = utils::to_f32_vec(&self.voronoi.sites);
        let num_of_vertices = positions.len();
        let normals: Vec<[f32; 3]> = vec![[0.0, 1.0, 0.0]; num_of_vertices];
        let uvs: Vec<[f32; 2]> = vec![[0.0, 0.0]; num_of_vertices];
        let colors: Vec<[f32; 3]> = (0..num_of_vertices)
            .map(self.coloring)
            .map(utils::color_to_f32_vec)
            .collect();

        // let e = 17;
        // indices.push(self.voronoi.triangulation.triangles[e] as u32);
        // indices.push(self.voronoi.triangulation.triangles[next_halfedge(e)] as u32);
        // println!("Half-edge for {} is {}", e, self.voronoi.triangulation.halfedges[e]);

        let mut indices: Vec<u32> = vec![];
        for t in 0..self.voronoi.num_of_triangles {
            indices.push(self.voronoi.triangulation.triangles[3 * t] as u32);
            indices.push(self.voronoi.triangulation.triangles[3 * t + 1] as u32);

            indices.push(self.voronoi.triangulation.triangles[3 * t + 1] as u32);
            indices.push(self.voronoi.triangulation.triangles[3 * t + 2] as u32);

            indices.push(self.voronoi.triangulation.triangles[3 * t + 2] as u32);
            indices.push(self.voronoi.triangulation.triangles[3 * t] as u32);
        }

        let mut mesh = Mesh::new(self.topology);
        mesh.set_indices(Some(Indices::U32(indices)));
        mesh.set_attribute(Mesh::ATTRIBUTE_POSITION, positions);
        mesh.set_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        mesh.set_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
        mesh.set_attribute("Vertex_Color", colors);
        mesh
    }

    pub fn build_voronoi_mesh(&self) -> Mesh {
        let positions: Vec<[f32; 3]> = utils::to_f32_vec(&self.voronoi.circumcenters);
        let num_of_vertices = positions.len();
        let normals: Vec<[f32; 3]> = vec![[0.0, 1.0, 0.0]; num_of_vertices];
        let uvs: Vec<[f32; 2]> = vec![[0.0, 0.0]; num_of_vertices];
        let colors: Vec<[f32; 3]> = (0..num_of_vertices)
            .map(self.coloring)
            .map(utils::color_to_f32_vec)
            .collect();
        let indices = self.build_voronoi_cell_index_buffer();

        let mut mesh = Mesh::new(self.topology);
        mesh.set_indices(Some(Indices::U32(indices)));
        mesh.set_attribute(Mesh::ATTRIBUTE_POSITION, positions);
        mesh.set_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        mesh.set_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
        mesh.set_attribute("Vertex_Color", colors);
        mesh
    }

    fn build_voronoi_cell_index_buffer(&self) -> Vec<u32> {
        match self.topology {
            PrimitiveTopology::LineList => {
                match self.voronoi.hull_behavior {
                    HullBehavior::None | HullBehavior::Extended => {
                        // when hull is not closed, we cannot wrap line list or it will render extra edges
                        let hull_verteces = self.voronoi.cells()
                            .filter(|c| c.is_on_hull())
                            .flat_map(|c| into_line_list(c.get_triangles()))
                            .map(|i| i as u32);

                        // for cells not on hull, they are always closed
                        let mut verteces = self.voronoi.cells()
                            .filter(|c| !c.is_on_hull())
                            .flat_map(|c| into_line_list_wrap(c.get_triangles()))
                            .map(|t| t as u32)
                            .collect::<Vec<u32>>();

                        verteces.extend(hull_verteces);
                        verteces
                    },

                    HullBehavior::Closed => {
                        // when hull is closed, all cells are closed
                        self.voronoi.cells()
                        .filter(|c| !c.is_on_hull())
                        .flat_map(|c| into_line_list_wrap(c.get_triangles()))
                        .map(|t| t as u32)
                        .collect::<Vec<u32>>()
                    }
                }
            },

            PrimitiveTopology::PointList | PrimitiveTopology::TriangleList => {
                // if cells on hull are not closed, they will not render correctly in this mode
                self.voronoi.cells()
                    .flat_map(|c| into_line_list(c.get_triangles()))
                    .map(|t| t as u32)
                    .into_triangle_list()
                    .collect::<Vec<u32>>()
            },

            _ => panic!("Topology {:?} not supported", self.topology)
        }
    }
}