#version 420

in vec4 color;

// If we define only one output, then OpenGL is smart enough to know this will be the color
out vec4 fragColor;

void main()
{
      fragColor = color;
      //fragColor = vec4(1.0, 0.0, 0.0, 1.0);
}