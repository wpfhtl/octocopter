#version 420

in vec2 texureCoordinate;
in float infgainfragshader;

// If we define only one output, then OpenGL is smart enough to know this will be the color
out vec4 fragColor;

uniform vec4 fixedColor;
uniform int activeWayPointIndex;

void main()
{
    vec4 c = fixedColor;
    c.a = infgainfragshader + 0.5;
    
    // Make real waypoints bright red.
    if(infgainfragshader > 0.0)
      c = vec4(1.0, 0.2, 0.2, 1.0);

    fragColor = c;
}