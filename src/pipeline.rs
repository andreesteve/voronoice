use bevy::{
    prelude::*,
    asset::{Assets, HandleUntyped},
    reflect::TypeUuid,
    render::{
        render_graph::{
            RenderGraph, AssetRenderResourcesNode,
            base::{self, MainPass}
         },
        renderer::RenderResources,
        pipeline::{
            RenderPipeline,
            BlendDescriptor, BlendFactor, BlendOperation, ColorStateDescriptor, ColorWrite,
            CompareFunction, CullMode, DepthStencilStateDescriptor, FrontFace, PipelineDescriptor,
            RasterizationStateDescriptor, StencilStateDescriptor, StencilStateFaceDescriptor,
        },
        shader::{Shader, ShaderStage, ShaderStages},
        texture::TextureFormat,
    }
};

pub const FORWARD_PIPELINE_HANDLE: HandleUntyped =
    HandleUntyped::weak_from_u64(PipelineDescriptor::TYPE_UUID, 12148362314032771289);

pub(crate) fn build_forward_pipeline(shaders: &mut Assets<Shader>) -> PipelineDescriptor {
    PipelineDescriptor {
        rasterization_state: Some(RasterizationStateDescriptor {
            front_face: FrontFace::Ccw,
            cull_mode: CullMode::Back,
            depth_bias: 0,
            depth_bias_slope_scale: 0.0,
            depth_bias_clamp: 0.0,
            clamp_depth: false,
        }),
        depth_stencil_state: Some(DepthStencilStateDescriptor {
            format: TextureFormat::Depth32Float,
            depth_write_enabled: true,
            depth_compare: CompareFunction::Less,
            stencil: StencilStateDescriptor {
                front: StencilStateFaceDescriptor::IGNORE,
                back: StencilStateFaceDescriptor::IGNORE,
                read_mask: 0,
                write_mask: 0,
            },
        }),
        color_states: vec![ColorStateDescriptor {
            format: TextureFormat::default(),
            color_blend: BlendDescriptor {
                src_factor: BlendFactor::SrcAlpha,
                dst_factor: BlendFactor::OneMinusSrcAlpha,
                operation: BlendOperation::Add,
            },
            alpha_blend: BlendDescriptor {
                src_factor: BlendFactor::One,
                dst_factor: BlendFactor::One,
                operation: BlendOperation::Add,
            },
            write_mask: ColorWrite::ALL,
        }],
        ..PipelineDescriptor::new(ShaderStages {
            vertex: shaders.add(Shader::from_glsl(
                ShaderStage::Vertex,
                include_str!("forward.vert"),
            )),
            fragment: Some(shaders.add(Shader::from_glsl(
                ShaderStage::Fragment,
                include_str!("forward frag"),
            ))),
        })
    }
}

#[derive(Default)]
pub struct VertexColorPlugin;

impl Plugin for VertexColorPlugin {
    fn build(&self, app: &mut AppBuilder) {
        const COLOR_NODE: &str = "my_material_with_vertex_color_support";
        app.add_asset::<VertexColor>();
        let resources = app.resources();
        let mut graph = resources.get_mut::<RenderGraph>().unwrap();

        graph.add_system_node(
            COLOR_NODE,
            AssetRenderResourcesNode::<VertexColor>::new(true),
        );

        graph
            .add_node_edge(
                COLOR_NODE,
                base::node::MAIN_PASS,
            )
            .unwrap();

        let mut shaders = resources.get_mut::<Assets<Shader>>().unwrap();
        let mut pipelines = resources.get_mut::<Assets<PipelineDescriptor>>().unwrap();
        pipelines.set_untracked(
            FORWARD_PIPELINE_HANDLE,
            build_forward_pipeline(&mut shaders),
        );

        resources.get_mut::<Assets<VertexColor>>()
            .unwrap()
            .add(Default::default());
    }
}

#[derive(RenderResources, Default, TypeUuid)]
#[uuid = "0320b9b8-b3a3-4baa-8bfa-c94008177b17"]
pub struct VertexColor {}

#[derive(Bundle)]
pub struct ColorBundle {
    pub mesh: Handle<Mesh>,
    pub main_pass: MainPass,
    pub draw: Draw,
    pub visible: Visible,
    pub render_pipelines: RenderPipelines,
    pub transform: Transform,
    pub global_transform: GlobalTransform,
    pub color: VertexColor,
}

impl Default for ColorBundle {
    fn default() -> Self {
        Self {
            render_pipelines: RenderPipelines::from_pipelines(vec![RenderPipeline::new(
                FORWARD_PIPELINE_HANDLE.typed(),
            )]),
            mesh: Default::default(),
            visible: Default::default(),
            main_pass: Default::default(),
            draw: Default::default(),
            transform: Default::default(),
            global_transform: Default::default(),
            color: Default::default(),
        }
    }
}