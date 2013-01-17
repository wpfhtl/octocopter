#version 420

//in vec2 texureCoordinate;
in vec4 colorGS_to_FS;

// If we define only one output, then OpenGL is smart enough to know this will be the color
out vec4 fragColor;

void main()
{
  fragColor = colorGS_to_FS;
  //fragColor = vec4(1.0, 0.2, 0.2, 0.5);
}