#version 420

in vec4 in_position;
in vec4 in_color;

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

uniform mat4 matCameraClip;
uniform mat4 matModelCamera;


void main()
{
    color = in_color;
    //gl_Position = in_position;

    // from http://www.arcsynthesis.org/gltut/Positioning/Tut07%20Shared%20Uniforms.html
    //vec4 temp = matrixModelToCamera * in_position;
    //gl_Position = matrixCameraToClip * temp;

    //gl_Position = matrixCameraToClip * matrixModelToCamera * in_position;
    //gl_Position = matCameraClip * matModelCamera * in_position;
    gl_Position = matCameraClip * matModelCamera * in_position;
}