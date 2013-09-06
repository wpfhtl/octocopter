#version 420

// use glEnableVertexAttribArray(0) and glVertexAttribPointer(0, ...) to define this input data
layout(location = 0) in vec4 in_position;
// we don't need a color input anymore, as the color is derived from the in_position.w component

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

void main()
{
    // when a particle has collided, change its color!
    if(in_position.w > 1.05)
      color = vec4(1.0, 0.0, 0.0, 1.0);
    else
      color = vec4(0.5, 0.5, 0.5, 0.5);

    gl_Position = vec4(in_position.x, in_position.y, in_position.z, 1.0);
}
