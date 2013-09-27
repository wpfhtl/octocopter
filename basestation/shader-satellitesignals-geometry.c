#version 420

layout(points) in;
layout(triangle_strip, max_vertices = 28) out;

in int gl_PrimitiveIDIn; // the current primitive (=cell) id

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

uniform mat4 matrixExtra;
uniform int numSatsGps;
uniform int numSatsGlonass;

// pos has z for depth!
void drawBox2d(in vec3 pos, in vec2 size, in vec4 clr)
{
    // top left
    gl_Position = vec4(pos.x - size.x/2, pos.y + size.y/2, pos.z, 1.0);
    color = clr;
    EmitVertex();

    // bottom left
    gl_Position = vec4(pos.x - size.x/2, pos.y - size.y/2, pos.z, 1.0);
    color = clr;
    EmitVertex();

    // bottom right
    gl_Position = vec4(pos.x + size.x/2, pos.y - size.y/2, pos.z, 1.0);
    color = clr;
    EmitVertex();

    // top right
    gl_Position = vec4(pos.x + size.x/2, pos.y + size.y/2, pos.z, 1.0);
    color = clr;
    EmitVertex();

    // top left
    gl_Position = vec4(pos.x - size.x/2, pos.y + size.y/2, pos.z, 1.0);
    color = clr;
    EmitVertex();

    EndPrimitive();
}



void drawBox3d(in mat4 matrix, in vec3 basePos, in vec3 size, in vec4 clr)
{
    vec3 v;

    // Draw sides
    // bottom left front
    v = basePos + vec3(-size.x/2, 0.0, size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();
    
    // top left front
    v = basePos + vec3(-size.x/2, size.y, size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();
    
    // bottom right front
    v = basePos + vec3(size.x/2, 0.0, size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();
    
    // top right front
    v = basePos + vec3(size.x/2, size.y, size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();
    
    // bottom right back
    v = basePos + vec3(size.x/2, 0.0, -size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();
    
    // top right back
    v = basePos + vec3(size.x/2, size.y, -size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();
    
    // left bottom back
    v = basePos + vec3(-size.x/2, 0.0, -size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();
    
    // left top back
    v = basePos + vec3(-size.x/2, size.y, -size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();
    
    // bottom left front
    v = basePos + vec3(-size.x/2, 0.0, size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();
    
    // top left front
    v = basePos + vec3(-size.x/2, size.y, size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();
    
    EndPrimitive();
    
    // Draw top plane    
    // top left front
    v = basePos + vec3(-size.x/2, size.y, size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();
    
    
    // left top back
    v = basePos + vec3(-size.x/2, size.y, -size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();

    // top right front
    v = basePos + vec3(size.x/2, size.y, size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();
    
    // top right back
    v = basePos + vec3(size.x/2, size.y, -size.z/2);
    gl_Position = matrix * vec4(v, 1.0);
    color = clr;
    EmitVertex();
    
    EndPrimitive();
}


void main()
{
    // GPS:
    //  0   Green:   L1-CA
    //  1   Blue:    L1-P(Y)
    //  2   Red:     L2-P(Y)
    //  3   Yellow:  L2C
    // GLONASS:
    //  8   Green:   L1-CA
    //  11  Blue:    L2-CA
  
    vec4 palette[4] = vec4[](
      vec4(0.0, 1.0, 0.0, 0.3),
      vec4(0.0, 0.0, 1.0, 0.3),
      vec4(1.0, 0.0, 0.0, 0.3),
      vec4(1.0, 1.0, 0.0, 0.3));

    uint constellation = int(gl_in[0].gl_Position.x);
    uint svid = int(gl_in[0].gl_Position.y);
    
    uint signalType = int(gl_in[0].gl_Position.z);
    vec4 color = palette[signalType];
    
    if(signalType == 8)
    {
      signalType = 0;
      color = palette[signalType];
    }
    if(signalType == 11)
    {
      signalType = 1;
      color = palette[signalType];
    }
    
    float signalStrength = gl_in[0].gl_Position.w / 4;

    // make glonass start right of gps, with one spacer in between
    if(constellation != 0) svid += numSatsGps+1;
    float xpos = -0.95 + float(svid)*0.08 + (0.016 * float(signalType));
    vec2 size = vec2(0.015, signalStrength/33.33333);
    vec3 pos = vec3(xpos, -0.9 + size.y/2, 1.0);
    
    drawBox2d(pos, size, color);
    
    // let different shaders draw different scales, so we can keep max_vertices low!
    if(gl_PrimitiveIDIn == 0) drawBox2d(vec3(0.0, -0.9, 0.1), vec2(1.95, 0.002), vec4(0.8,0.8,0.8,0.5));
    if(gl_PrimitiveIDIn == 1) drawBox2d(vec3(0.0, -0.6, 0.1), vec2(1.95, 0.002), vec4(0.8,0.8,0.8,0.5));
    if(gl_PrimitiveIDIn == 2) drawBox2d(vec3(0.0, -0.3, 0.1), vec2(1.95, 0.002), vec4(0.8,0.8,0.8,0.5));
    if(gl_PrimitiveIDIn == 3) drawBox2d(vec3(0.0,  0.0, 0.1), vec2(1.95, 0.002), vec4(0.8,0.8,0.8,0.5));
    if(gl_PrimitiveIDIn == 4) drawBox2d(vec3(0.0, +0.3, 0.1), vec2(1.95, 0.002), vec4(0.8,0.8,0.8,0.5));
    if(gl_PrimitiveIDIn == 5) drawBox2d(vec3(0.0, +0.6, 0.1), vec2(1.95, 0.002), vec4(0.8,0.8,0.8,0.5));
    if(gl_PrimitiveIDIn == 6) drawBox2d(vec3(0.0, +0.9, 0.1), vec2(1.95, 0.002), vec4(0.8,0.8,0.8,0.5));
}
