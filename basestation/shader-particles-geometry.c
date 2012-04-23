#version 420

layout(points) in;
layout(triangle_strip) out;
layout(max_vertices = 4) out;

in vec4 particleColorVS_to_GS[];
out vec4 particleColorGS_to_FS;
out vec2 texureCoordinate;

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
    particleColorGS_to_FS = particleColorVS_to_GS[0];
    EmitVertex();

    // top left
    pos += up*2;
    gl_Position = matModelViewProjection * vec4(pos, 1.0);
    texureCoordinate = vec2(-1.0, 1.0);
    particleColorGS_to_FS = particleColorVS_to_GS[0];
    EmitVertex();

    // bottom right
    pos -= up*2;
    pos += right * 2;
    gl_Position = matModelViewProjection * vec4(pos, 1.0);
    texureCoordinate = vec2(1.0, -1.0);
    particleColorGS_to_FS = particleColorVS_to_GS[0];
    EmitVertex();

    // top right
    pos += up*2;
    gl_Position = matModelViewProjection * vec4(pos, 1.0);
    texureCoordinate = vec2(1.0, 1.0);
    particleColorGS_to_FS = particleColorVS_to_GS[0];
    EmitVertex();

    EndPrimitive();
}
