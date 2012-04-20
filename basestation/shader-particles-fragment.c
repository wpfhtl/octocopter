#version 420

in vec2 texureCoordinate;
in vec4 particleColorGS_to_FS;

// If we define only one output, then OpenGL is smart enough to know this will be the color
out vec4 fragColor;

uniform mat4 matModelViewProjection;
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
 	pos = matModelViewProjection * pos;

	gl_FragDepth = (pos.z / pos.w + 1.0) / 2.0;
	//gl_FragColor = gl_Color * diffuse_value;
	fragColor = particleColorGS_to_FS * diffuse_value;

	// gl_FragCoord is vec3, where .xy-swizzle is the actual gl-window-postion (starting bottom-left)
	//float lerpValue = gl_FragCoord.y / 1500.0f;
	//fragColor = mix(vec4(1.0f, 1.0f, 1.0f, 1.0f), vec4(0.2f, 0.2f, 0.2f, 1.0f), lerpValue);

	//fragColor = particleColorGS_to_FS * diffuse_value;
	//fragColor = vec4(1.0f, 1.0f, 1.0f, 1.0f);// * diffuse_value;
}