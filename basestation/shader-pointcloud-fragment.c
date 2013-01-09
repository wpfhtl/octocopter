#version 420

in vec2 texureCoordinate;
in vec4 color;

// If we define only one output, then OpenGL is smart enough to know this will be the color
out vec4 fragColor;

// For using fixed color, e.g. when rendering waypoints
uniform vec4 fixedColor;
uniform bool useFixedColor;

void main()
{
    if(useFixedColor)
    {
      fragColor = fixedColor;
    }
    else
    {
      fragColor = color;
    }
}