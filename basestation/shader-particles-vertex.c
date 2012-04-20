#version 420

// We can either specify location ("layout(location = 0) in vec4 position;"),
// or use glGetAttribLocation in client code to read its index

in vec4 in_particleColor;
in vec4 in_particlePosition;

out vec4 particleColorVS_to_GS;

uniform mat4 matModelViewProjection;

void main()
{
    gl_Position = matModelViewProjection * in_particlePosition;
    particleColorVS_to_GS = in_particleColor;
}