#define STRINGIFY(A) #A

const char* vertexShader = "                                                        \n\
#version 330                                                                        \n\
                                                                                    \n\
layout (location = 0) in vec3 Position;                                             \n\
                                                                                    \n\
void main()                                                                         \n\
{                                                                                   \n\
    gl_Position = vec4(Position, 1.0);                                              \n\
}                                                                                   \n\
";


const char* geometryShader = "                                                      \n\
#version 330                                                                        \n\
                                                                                    \n\
layout(points) in;                                                                  \n\
layout(triangle_strip) out;                                                         \n\
layout(max_vertices = 4) out;                                                       \n\
                                                                                    \n\
uniform mat4 gViewProjection;                                                       \n\
uniform vec3 cameraPosition;                                                        \n\
uniform float particleRadius;                                                       \n\
                                                                                    \n\
out vec2 TexCoord;                                                                  \n\
                                                                                    \n\
void main()                                                                         \n\
{                                                                                   \n\
    vec3 Pos = gl_in[0].gl_Position.xyz;                                            \n\
    vec3 toCamera = normalize(cameraPosition - Pos);                                \n\
    vec3 upWorld = vec3(0.0, 1.0, 0.0);                                             \n\
    vec3 right = cross(toCamera, upWorld) * particleRadius;                         \n\
    vec3 up = cross(toCamera, normalize(-right)) * particleRadius;                  \n\
                                                                                    \n\
    // bottom left                                                                  \n\
    Pos -= right;                                                                   \n\
    Pos -= up;                                                                      \n\
    gl_Position = gViewProjection * vec4(Pos, 1.0);                                 \n\
    TexCoord = vec2(0.0, 1.0);                                                      \n\
    EmitVertex();                                                                   \n\
                                                                                    \n\
    // top left                                                                     \n\
    Pos += up*2;                                                                    \n\
    gl_Position = gViewProjection * vec4(Pos, 1.0);                                 \n\
    TexCoord = vec2(0.0, 0.0);                                                      \n\
    EmitVertex();                                                                   \n\
                                                                                    \n\
    // bottom right                                                                 \n\
    Pos -= up*2;                                                                    \n\
    Pos += right * 2;                                                               \n\
    gl_Position = gViewProjection * vec4(Pos, 1.0);                                 \n\
    TexCoord = vec2(1.0, 1.0);                                                      \n\
    EmitVertex();                                                                   \n\
                                                                                    \n\
    // top right                                                                    \n\
    Pos += up*2;                                                                    \n\
    gl_Position = gViewProjection * vec4(Pos, 1.0);                                 \n\
    TexCoord = vec2(1.0, 0.0);                                                      \n\
    EmitVertex();                                                                   \n\
                                                                                    \n\
    EndPrimitive();                                                                 \n\
}                                                                                   \n\
";


const char* fragmentShader = "                                                      \n\
#version 330                                                                        \n\
                                                                                    \n\
in vec2 TexCoord;                                                                   \n\
out vec4 FragColor;                                                                 \n\
                                                                                    \n\
uniform mat4 gViewProjection;                                                       \n\
uniform vec3 cameraPosition;                                                        \n\
uniform float particleRadius;                                                       \n\
                                                                                    \n\
                                                                                    \n\
void mainCircle()                                                                   \n\
{                                                                                   \n\
    float distance = distance(TexCoord.xy, vec2(0.5, 0.5));                         \n\
    if (distance < 0.45 || distance > 0.5) {                                        \n\
        discard;                                                                    \n\
    }                                                                               \n\
    FragColor = vec4(1.0, 0.0, 0.0, 1.0);                                           \n\
                                                                                    \n\
}                                                                                   \n\
                                                                                    \n\
                                                                                    \n\
                                                                                    \n\
                                                                                    \n\
    void main()                                                                   \n\
    {                                                                   \n\
    vec3 vertex_light_position = vec3(0.577, 0.577, 0.577);
        // r^2 = (x - x0)^2 + (y - y0)^2 + (z - z0)^2                                                                   \n\
        float x = TexCoord[0].x;                                                                   \n\
        float y = TexCoord[0].y;                                                                   \n\
        float zz = 1.0 - x*x - y*y;                                                                   \n\
\n\
        if (zz <= 0.0)                                                                   \n\
            discard;                                                                   \n\
\n\
        float z = sqrt(zz);                                                                   \n\
\n\
        vec3 normal = vec3(x, y, z);                                                                   \n\
\n\
        // Lighting                                                                   \n\
        float diffuse_value = max(dot(normal, vertex_light_position), 0.0);                                                                   \n\
\n\
        vec4 pos = cameraPosition;                                                                   \n\
        pos.z += z*particleRadius;                                                                   \n\
        pos = gViewProjection * pos;                                                                   \n\
\n\
        FragDepth = (pos.z / pos.w + 1.0) / 2.0;                                                                   \n\
        FragColor = gl_Color * diffuse_value;                                                                   \n\
    }";












// pixel shader for rendering points as shaded spheres
const char *afragmentShader = STRINGIFY(
void main()
{
    const vec3 lightDir = vec3(0.577, 0.577, 0.577);

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
