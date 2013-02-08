#version 420

in vec2 texureCoordinate;
in vec4 colorGS_to_FS;

// If we define only one output, then OpenGL is smart enough to know this will be the color
out vec4 fragColor;

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

uniform float particleRadius;

// For using fixed color, e.g. when rendering waypoints
uniform vec4 fixedColor;
uniform bool useFixedColor;


void main()
{

    vec3 vertex_light_position = (matrixModelToCamera * vec4(0.577, 0.577, 0.577, 1.0)).xyz;
    vertex_light_position = vec3(0.577, 0.577, 0.577);
    // r^2 = (x - x0)^2 + (y - y0)^2 + (z - z0)^2
    float x = texureCoordinate.x;
    float y = texureCoordinate.y;
    float zz = 1.0 - x*x - y*y;

    if (zz <= 0.0) discard;

    float z = sqrt(zz);

    vec3 normal = vec3(x, y, z);

    // Lighting
    float diffuse_value = max(dot(normal, vertex_light_position), 0.0);

    vec4 cameraPosition = inverse(matrixModelToCamera) * vec4(0,0,0,1);
    cameraPosition.z = 1.0;
    //vec4 pos = vec4(cameraPosition, 1.0);
//     cameraPosition.z += z*particleRadius;
//     cameraPosition = matProjection * matModelView * cameraPosition;

    //gl_FragDepth = (cameraPosition.z / cameraPosition.w + 1.0) / 2.0;
    //vec4 clipPos = matrixCameraToClip * cameraPosition;
//float ndcDepth = clipPos.z / clipPos.w;
//gl_FragDepth = ((gl_DepthRange.diff * ndcDepth) + gl_DepthRange.near + gl_DepthRange.far) / 2.0;


    if(useFixedColor)
    {
      fragColor = fixedColor * diffuse_value;
      //fragColor.w = 1.0;
    }
    else
    {
      fragColor = colorGS_to_FS * diffuse_value;
      fragColor.w = 1.0;
    }
}
