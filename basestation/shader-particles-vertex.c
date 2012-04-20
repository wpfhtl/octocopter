#version 420

// We can either specify location ("layout(location = 0) in vec4 position;"),
// or use glGetAttribLocation in client code to read its index

in vec4 in_particlePosition;
in vec4 in_particleColor;

out vec4 particleColorVS_to_GS;

//uniform mat4 matModelViewProjection;

void main()
{
    gl_Position = in_particlePosition;
    particleColorVS_to_GS = in_particleColor;
}