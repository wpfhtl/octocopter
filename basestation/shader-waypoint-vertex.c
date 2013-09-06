#version 420

// use glEnableVertexAttribArray(0) and glVertexAttribPointer(0, ...) to define this input data
layout(location = 0) in vec4 in_position;
layout(location = 1) in float in_informationgain;

in int gl_VertexID;

out float infgainfragshader;

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

uniform int activeWayPointIndex;

void main()
{
    gl_Position = matrixCameraToClip * matrixModelToCamera * in_position;

    infgainfragshader = in_informationgain;

    // Determine pointsize
    float pointSize = 5;
    
    // mark the current waypoint larger
    if(gl_VertexID == activeWayPointIndex) pointSize *= 2.0;

    // mark real SCAN waypoints larger
    if(in_informationgain > 0.0) pointSize *= 2.0;
    
    gl_PointSize = pointSize;
}