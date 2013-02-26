#version 420

// use glEnableVertexAttribArray(0) and glVertexAttribPointer(0, ...) to define this input data
layout(location = 0) in float in_waypointpressure;

out float waypointpressure;
//out vec4 color;

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
    //color = in_color;

    gl_Position = vec4(0,0,0,0);
    waypointpressure = in_waypointpressure;
}