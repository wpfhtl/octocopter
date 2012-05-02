#version 420

layout(points) in;
layout(triangle_strip) out;
layout(max_vertices = 4) out;

in vec4 color[];
out vec4 colorGS_to_FS;
out vec2 texureCoordinate;

// We use a uniform buffer object to store things that multiple shaders need.
// This also means that no uniform can be inactive, because the compiler can't
// know whether other shaders actually use the uniform.
layout(std140) uniform GlobalValues
{
    mat4 matrixModelToWorld;
    mat4 matrixWorldToCamera;
    mat4 matrixCameraToClip;
};

uniform mat4 matProjection;
uniform mat4 matModelView;
uniform vec3 cameraPosition;
uniform float particleRadius;

void main()
{
    mat4 matModelViewProjection = matModelView * matProjection;
    vec3 pos = gl_in[0].gl_Position.xyz;

    vec3 toCamera = normalize(cameraPosition - pos);

    vec3 upWorld = vec3(0.0, 1.0, 0.0);

    vec3 right = cross(toCamera, upWorld) * particleRadius;

    vec3 up = cross(toCamera, normalize(-right)) * particleRadius;

    // bottom left
    pos -= right;
    pos -= up;
    gl_Position = matModelViewProjection * vec4(pos, 1.0);
    texureCoordinate = vec2(-1.0, -1.0);
    colorGS_to_FS = color[0];
    EmitVertex();

    // top left
    pos += up*2;
    gl_Position = matModelViewProjection * vec4(pos, 1.0);
    texureCoordinate = vec2(-1.0, 1.0);
    colorGS_to_FS = color[0];
    EmitVertex();

    // bottom right
    pos -= up*2;
    pos += right * 2;
    gl_Position = matModelViewProjection * vec4(pos, 1.0);
    texureCoordinate = vec2(1.0, -1.0);
    colorGS_to_FS = color[0];
    EmitVertex();

    // top right
    pos += up*2;
    gl_Position = matModelViewProjection * vec4(pos, 1.0);
    texureCoordinate = vec2(1.0, 1.0);
    colorGS_to_FS = color[0];
    EmitVertex();

    EndPrimitive();
}
