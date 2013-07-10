#version 420

layout(points) in;
layout(line_strip, max_vertices = 2) out; // cannot use lines...

out vec4 color;
in float raylength[];
in int gl_PrimitiveIDIn; // the current primitive (=cell) id

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
uniform int rayStride;
uniform int firstUsableDistance;

void main()
{
    color = vec4(1.0, 0.2, 0.2, 0.8);
    mat4 matModelViewProjection = matrixCameraToClip * matrixModelToCamera;

    if(useMatrixExtra)
        matModelViewProjection = matModelViewProjection * matrixExtra;

    // Emit first point at scanner origin
    gl_Position = matModelViewProjection * vec4(0.0, 0.0, 0.0, 1.0);
    EmitVertex();

    // Compute the position of the second point
    float distance = raylength[0] * 65.536;
//    if(distance < 0.1)
//    {
//        distance = 40.0;
//        color = vec4(1.0, 1.0, 1.0, 0.3);
//    }

    vec4 vectorScannerToPoint = vec4(
                sin(-0.0043633231299858238686 * ((rayStride+1) * gl_PrimitiveIDIn - 540 + firstUsableDistance)) * distance,
                0.0,
                -cos(0.0043633231299858238686 * ((rayStride+1) * gl_PrimitiveIDIn - 540 + firstUsableDistance)) * distance,
                1.0);

    gl_Position = matModelViewProjection * vectorScannerToPoint;
    EmitVertex();

    EndPrimitive();
}
