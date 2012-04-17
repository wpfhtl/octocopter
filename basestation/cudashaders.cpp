#define STRINGIFY(A) #A

// vertex shader
const char *vertexShader = STRINGIFY(
uniform float pointRadius;  // point size in world space
uniform float pointScale;   // scale to calculate size in pixels
uniform float densityScale;
uniform float densityOffset;
void main()
{
    // calculate window-space point size
    vec3 posEye = vec3(gl_ModelViewMatrix * vec4(gl_Vertex.xyz, 1.0));
    float dist = length(posEye);
    gl_PointSize = pointRadius * (pointScale / dist);

    gl_TexCoord[0] = gl_MultiTexCoord0;

    gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xyz, 1.0);

    gl_FrontColor = gl_Color;
}
);





const char *geometryShader = STRINGIFY(
#version 330

layout(points) in;
layout(triangle_strip) out;
layout(max_vertices = 4) out;

uniform mat4 gVP;
uniform vec3 gCameraPos;

out vec2 TexCoord;

void main()
{
    vec3 Pos = gl_in[0].gl_Position.xyz;
    vec3 toCamera = normalize(gCameraPos - Pos);
    vec3 up = vec3(0.0, 1.0, 0.0);
    vec3 right = cross(toCamera, up);

    Pos -= (right * 0.5);
    gl_Position = gVP * vec4(Pos, 1.0);
    TexCoord = vec2(0.0, 0.0);
    EmitVertex();

    Pos.y += 1.0;
    gl_Position = gVP * vec4(Pos, 1.0);
    TexCoord = vec2(0.0, 1.0);
    EmitVertex();

    Pos.y -= 1.0;
    Pos += right;
    gl_Position = gVP * vec4(Pos, 1.0);
    TexCoord = vec2(1.0, 0.0);
    EmitVertex();

    Pos.y += 1.0;
    gl_Position = gVP * vec4(Pos, 1.0);
    TexCoord = vec2(1.0, 1.0);
    EmitVertex();

    EndPrimitive();
}
);






// pixel shader for rendering points as shaded spheres
const char *fragmentShader = STRINGIFY(
void main()
{
    vec4 ld4 = gl_ModelViewMatrix * vec4(0.577, 0.577, 0.577, 0);
    vec3 lightDir = vec3(ld4.x, ld4.y, ld4.z);
    //lightDir = vec3(0.577, 0.577, 0.577);

    // calculate normal from texture coordinates
    vec3 N;
    N.xy = gl_TexCoord[0].xy*vec2(2.0, -2.0) + vec2(-1.0, 1.0);
    float mag = dot(N.xy, N.xy);
    if (mag > 1.0) discard;   // kill pixels outside circle
    N.z = sqrt(1.0-mag);

    // calculate lighting
    float diffuse = max(0.0, dot(lightDir, N));

    gl_FragColor = gl_Color * diffuse;
}
);


