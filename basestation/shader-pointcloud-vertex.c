#version 420

// use glEnableVertexAttribArray(0) and glVertexAttribPointer(0, ...) to define this input data
layout(location = 0) in vec4 in_position;
// use glEnableVertexAttribArray(1) and glVertexAttribPointer(1, ...) to define this input data
layout(location = 1) in vec4 in_color;

out vec4 color;

// We use a uniform buffer object to store things that multiple shaders need.
// This also means that no uniform can be inactive, because the compiler can't
// know whether other shaders actually use the uniform.
layout(std140) uniform GlobalValues
{
    // By using modelToWorld and worldToCamera, we'd lose precision
    // (http://www.arcsynthesis.org/gltut/Positioning/Tut07%20The%20Perils%20of%20World%20Space.html)
    // So, we use a modelToCamera matrix instead, skipping two chances of precision loss.
    mat4 matrixModelToCamera;
    mat4 matrixCameraToClip;
};

uniform mat4 matrixExtra;
uniform bool useMatrixExtra;
uniform float maxPointVisualizationDistance;

void main()
{
    //color = in_color;

    vec4 colorTemp = vec4(1.0, 1.0, 1.0, 0.3); // a is alpha, 1.0 is visible

    // the w-component stores the squared distance. For rendering, lets set it to 1.0.
    vec4 pos = in_position;
    if(maxPointVisualizationDistance < pos.w)
        colorTemp.a = 0.03;

    pos.w = 1.0;

    if(useMatrixExtra)
      gl_Position = matrixCameraToClip * matrixModelToCamera * matrixExtra * pos;
    else
      gl_Position = matrixCameraToClip * matrixModelToCamera * pos;

    float vmin = -2;
    float vmax = 18.0; // colormap repeats every 10 height-meters
    float dv = vmax - vmin;
    //float colorValue = abs(pos.y); // colormap repeats every 10 height-meters
    // ben: max 10m, repeat cycle (use mod())
    //float colorValue = mod(abs(pos.y), vmax); // colormap repeats every 10 height-meters
    float colorValue = pos.y; // colormap, non-repeating

    if(colorValue < vmin)
      colorValue = vmin;
    if(colorValue > vmax)
      colorValue = vmax;

    if(colorValue < (vmin + 0.25 * dv))
    {
      colorTemp.r = 0;
      colorTemp.g = 4 * (colorValue - vmin) / dv;
    }
    else if(colorValue < (vmin + 0.5 * dv))
    {
      colorTemp.r = 0;
      colorTemp.b = 1 + 4 * (vmin + 0.25 * dv - colorValue) / dv;
    }
    else if(colorValue < (vmin + 0.75 * dv))
    {
      colorTemp.r = 4 * (colorValue - vmin - 0.5 * dv) / dv;
      colorTemp.b = 0;
    }
    else
    {
      colorTemp.g = 1 + 4 * (vmin + 0.75 * dv - colorValue) / dv;
      colorTemp.b = 0;
    }

    color = colorTemp;
}
