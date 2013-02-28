#version 420

layout(points) in;
layout(triangle_strip) out;
layout(max_vertices = 4) out;

in vec4 color[];
out vec4 colorGS_to_FS;
out vec2 texureCoordinate;

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

void main()
{
    if(color[0].a == 0.0) return;

    //mat4 matModelViewProjection = matModelView * matProjection;
    //mat4 matModelViewProjection = matrixModelToCamera * matrixCameraToClip;
    mat4 matModelViewProjection = matrixCameraToClip * matrixModelToCamera;

    vec3 pos = gl_in[0].gl_Position.xyz;

    vec4 cameraPosition = inverse(matrixModelToCamera) * vec4(0,0,0,1);

    vec3 toCamera = normalize(cameraPosition.xyz - pos);

    vec3 upWorld = vec3(0.0, 1.0, 0.0);

    vec3 right = normalize(-cross(toCamera, upWorld)) * particleRadius;

    vec3 up = normalize(cross(toCamera, normalize(-right))) * particleRadius;

    // bottom left
    pos -= right;
    pos -= up;
    gl_Position = matModelViewProjection * vec4(pos, 1.0);
    texureCoordinate = vec2(-1.0, -1.0);
    colorGS_to_FS = color[0];
    EmitVertex();

    // top left
    pos += up*2;
    gl_Position = matModelViewProjection * vec4(pos, 1.0);
    texureCoordinate = vec2(-1.0, 1.0);
    colorGS_to_FS = color[0];
    EmitVertex();

    // bottom right
    pos -= up*2;
    pos += right * 2;
    gl_Position = matModelViewProjection * vec4(pos, 1.0);
    texureCoordinate = vec2(1.0, -1.0);
    colorGS_to_FS = color[0];
    EmitVertex();

    // top right
    pos += up*2;
    gl_Position = matModelViewProjection * vec4(pos, 1.0);
    texureCoordinate = vec2(1.0, 1.0);
    colorGS_to_FS = color[0];
    EmitVertex();

    EndPrimitive();
}
