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
    mat4 matrixModelToWorld;
    mat4 matrixWorldToCamera;
    mat4 matrixCameraToClip;
};

uniform mat4 matProjection;
uniform mat4 matModelView;
uniform vec3 cameraPosition;
uniform float particleRadius;

void main()
{
    vec3 vertex_light_position = vec3(0.577, 0.577, 0.577);
    // r^2 = (x - x0)^2 + (y - y0)^2 + (z - z0)^2
    float x = texureCoordinate.x;
    float y = texureCoordinate.y;
    float zz = 1.0 - x*x - y*y;

    if (zz <= 0.0) discard;

    float z = sqrt(zz);

    vec3 normal = vec3(x, y, z);

    // Lighting
    float diffuse_value = max(dot(normal, vertex_light_position), 0.0);

    vec4 pos = vec4(cameraPosition, 1.0);
    pos.z += z*particleRadius;
    pos = matProjection * matModelView * pos;

    //gl_FragDepth = (pos.z / pos.w + 1.0) / 2.0;

    fragColor = max(colorGS_to_FS * diffuse_value, vec4(1.0));

    //fragColor = colorGS_to_FS * diffuse_value;
    //fragColor = colorGS_to_FS;
}