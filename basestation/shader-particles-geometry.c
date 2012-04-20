#version 420

layout(points) in;
layout(triangle_strip) out;
layout(max_vertices = 4) out;

in vec4 particleColorVS_to_GS[];
out vec4 particleColorGS_to_FS;
out vec2 texureCoordinate;

uniform mat4 matModelViewProjection;
uniform vec3 cameraPosition;
uniform float particleRadius;

void main()
{
    vec3 Pos = gl_in[0].gl_Position.xyz;
    vec3 toCamera = normalize(cameraPosition - Pos);
    vec3 upWorld = vec3(0.0, 1.0, 0.0);
    vec3 right = cross(toCamera, upWorld) * particleRadius;
    vec3 up = cross(toCamera, normalize(-right)) * particleRadius;

    // bottom left
    Pos -= right;
    Pos -= up;
    gl_Position = matModelViewProjection * vec4(Pos, 1.0);
    texureCoordinate = vec2(-1.0, -1.0);
    particleColorGS_to_FS = particleColorVS_to_GS[0];
    EmitVertex();

    // top left
    Pos += up*2;
    gl_Position = matModelViewProjection * vec4(Pos, 1.0);
    texureCoordinate = vec2(-1.0, 1.0);
    particleColorGS_to_FS = particleColorVS_to_GS[0];
    EmitVertex();

    // bottom right
    Pos -= up*2;
    Pos += right * 2;
    gl_Position = matModelViewProjection * vec4(Pos, 1.0);
    texureCoordinate = vec2(1.0, -1.0);
    particleColorGS_to_FS = particleColorVS_to_GS[0];
    EmitVertex();

    // top right
    Pos += up*2;
    gl_Position = matModelViewProjection * vec4(Pos, 1.0);
    texureCoordinate = vec2(1.0, 1.0);
    particleColorGS_to_FS = particleColorVS_to_GS[0];
    EmitVertex();

    EndPrimitive();
}
