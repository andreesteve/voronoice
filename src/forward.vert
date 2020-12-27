#version 450

layout(location = 0) in vec3 Vertex_Position;
layout(location = 1) in vec3 Vertex_Normal;
layout(location = 2) in vec2 Vertex_Uv;
layout(location = 3) in vec3 Vertex_Color;
//layout(location = 4) in vec3 Vertex_Centroid;

layout(location = 0) out vec3 v_Position;
layout(location = 1) out vec3 v_Normal;
layout(location = 2) out vec2 v_Uv;
layout(location = 3) out vec3 v_Color;

layout(set = 0, binding = 0) uniform Camera {
    mat4 ViewProj;
};

layout(set = 1, binding = 0) uniform Transform {
    mat4 Model;
};

void main() {
    v_Normal = mat3(Model) * Vertex_Normal;
    v_Color = Vertex_Color;
    v_Position = (Model * vec4(Vertex_Position, 1.0)).xyz;
    v_Uv = Vertex_Uv;
    gl_Position = ViewProj * vec4(v_Position, 1.0);
}