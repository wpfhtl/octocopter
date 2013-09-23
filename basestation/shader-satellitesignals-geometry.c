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

void drawBox(in mat4 matrix, in vec3 basePos, in vec3 size, in vec4 clr)
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
      vec4(0.0, 1.0, 0.0, 1.0),
      vec4(0.0, 0.0, 1.0, 1.0),
      vec4(1.0, 0.0, 0.0, 1.0),
      vec4(1.0, 1.0, 0.0, 1.0));
    
    mat4 matModelViewProjection = matrixCameraToClip * matrixModelToCamera * matrixExtra;

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
    
    float signalStrength = gl_in[0].gl_Position.w / 4.0;
    
    float xpos;
    // gps satellites to the left, glonass to the right!
    if(constellation == 0) // 0=gps, 1=glonass
      xpos = float(svid) - 0.8 - float(numSatsGps);
    else
      xpos = 0.8 + float(svid);
    
    xpos += 0.2 * float(signalType);
    
    vec3 pos = vec3(xpos, 0.0, 0.0);
    
    vec3 boxSize = vec3(0.19, signalStrength/10.0, 0.19);
    
    drawBox(matModelViewProjection, pos, boxSize, color);
    
    // dbg
    float scaleVeryLeft = -(numSatsGps + 0.9);
    float scaleVeryRight = +(numSatsGlonass + 0.1);
    float scaleWidth = scaleVeryRight - scaleVeryLeft;
    float scaleCenter = scaleVeryLeft + scaleWidth/2.0;
    
    // let different shaders draw different scales, so we can keep max_vertices low!
    if(gl_PrimitiveIDIn == 0)
      drawBox(matModelViewProjection, vec3(scaleCenter, 1.0, 0), vec3(scaleWidth + 0.1, 0.02, 0.4), vec4(1,1,1,0.5));
    if(gl_PrimitiveIDIn == 1)
      drawBox(matModelViewProjection, vec3(scaleCenter, 2.0, 0), vec3(scaleWidth + 0.1, 0.02, 0.4), vec4(1,1,1,0.5));
    if(gl_PrimitiveIDIn == 2)
      drawBox(matModelViewProjection, vec3(scaleCenter, 3.0, 0), vec3(scaleWidth + 0.1, 0.02, 0.4), vec4(1,1,1,0.5));
    if(gl_PrimitiveIDIn == 3)
      drawBox(matModelViewProjection, vec3(scaleCenter, 4.0, 0), vec3(scaleWidth + 0.1, 0.02, 0.4), vec4(1,1,1,0.5));
}
