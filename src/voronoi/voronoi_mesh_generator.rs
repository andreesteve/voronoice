use bevy::{
    prelude::*,
    render::{mesh::{Indices, Mesh}, pipeline::PrimitiveTopology},
};
use utils::into_line_list;

use super::{Voronoi, utils, into_triangle_list::*};

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

    /// Iterates over the graph and build the voronoi cells vertex list.
    /// It walks through all the delauney triangles associated with of a voronoi cell based on a starting delauney half-edge.
    /// It returns a list of triangles whose circumcenters are the verteces of the voronoi cell.
    ///
    /// If the starting edge is outgoing from a site, the iterator returns triangles in counter-clockwise order; for incoming edges the iteration is clock-wise.
    /// i.e. for any ```t``` returned by this iterator ```voronoi.circumcenters[t]``` is a vertex of the voronoi cell.
    /// Each voronoi cell half-edge is associated with a delauney half-edges. Thus any delauney half-edge associated with this cell will allow this iterator to list all vertices.
    ///
    /// # Examples
    ///
    /// ```
    /// // Counter clock-wise prints all verteces for a voronoi cell associated with a delaney half-edge
    /// let edge = 3;
    /// for edges in VoronoiCellTriangleIterator::new(voronoi, edge {
    ///     println!("Vertex: {}", voronoi.triangles[edge]);
    /// }
    /// ```
    fn build_voronoi_cell_index_buffer(&self) -> Vec<u32> {
        let voronoi = self.voronoi;

        match self.topology {
                PrimitiveTopology::LineList =>
                    (0..voronoi.sites.len())
                        .map(|s| {
                            into_line_list(
                                voronoi.cell_triangles[s]
                                        .iter()
                                        .map(|c| *c as u32)
                            )
                        })
                        .flatten()
                        .collect(),
                PrimitiveTopology::TriangleList | PrimitiveTopology::PointList =>
                    (0..voronoi.sites.len())
                        .map(|s| voronoi.cell_triangles[s]
                            .iter()
                            .map(|c| *c as u32)
                            .into_triangle_list())
                        .flatten()
                        .collect(),
                _ =>
                    panic!("Topology {:?} is not supported", self.topology),
        }
    }
}