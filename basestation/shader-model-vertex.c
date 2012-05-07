#version 330

//layout (std140) uniform Matrices {
//	mat4 projMatrix;
//	mat4 viewMatrix;
//	mat4 modelMatrix;
//};

layout(std140) uniform GlobalValues
{
    // By using modelToWorld and worldToCamera, we'd lose precision
    // (http://www.arcsynthesis.org/gltut/Positioning/Tut07%20The%20Perils%20of%20World%20Space.html)
    // So, we use a modelToCamera matrix instead, skipping two chances of precision loss.
    mat4 matrixModelToCamera;
    mat4 matrixCameraToClip;
};

uniform mat4 matrixModelSubMeshTransform;

in vec3 position;
in vec3 normal;
in vec2 texCoord;

out vec4 vertexPos;
out vec2 TexCoord;
out vec3 Normal;

void main()
{
	//Normal = normalize(vec3(viewMatrix * modelMatrix * vec4(normal,0.0)));
	Normal = normalize(vec3(matrixModelToCamera * vec4(normal,0.0)));
	TexCoord = vec2(texCoord);
	//gl_Position = projMatrix * viewMatrix * modelMatrix * vec4(position,1.0);
	gl_Position = matrixCameraToClip * matrixModelToCamera * matrixModelSubMeshTransform * vec4(position,1.0);
	//gl_Position = matrixCameraToClip * matrixModelToCamera * vec4(position,1.0);
}