#ifndef MODEL_H
#define MODEL_H

#include <QObject>
#include <QFile>
#include <QMatrix4x4>


#include <assimp/assimp.hpp>
#include <assimp/aiPostProcess.h>
#include <assimp/aiScene.h>


#include "shaderprogram.h"

class Model : public QObject
{
    Q_OBJECT

    ShaderProgram* mShaderProgram;
    QMatrix4x4 mModelTransform;
    QString mMediaPrefix;

    bool importFile(const QFile& modelFile);
    void loadGlTextures(const aiScene* scene);
    void generateVAOsAndUniformBuffer(const struct aiScene *scene);

    void renderRecursively(const struct aiScene *scene, const struct aiNode* node);


    void setFloat4(float f[4], float a, float b, float c, float d) const;
    void color4ToFloat4(const struct aiColor4D *c, float f[4]) const;

    // Information to render each assimp node
    struct MyMesh{

            GLuint vao;
            GLuint texIndex;
            GLuint uniformBlockIndex;
            int numFaces;
    };

    std::vector<struct MyMesh> myMeshes;

    // This is for a shader uniform block
    struct MyMaterial{

            float diffuse[4];
            float ambient[4];
            float specular[4];
            float emissive[4];
            float shininess;
            int texCount;
    };

    // Model Matrix (part of the OpenGL Model View Matrix)
//    float modelMatrix[16];

    // For push and pop matrix
//    std::vector<float *> matrixStack;

    // Vertex Attribute Locations
    GLuint mAttributeLocationVertex, mAttributeLocationNormal, mAttributeLocationTexCoord;

    // Uniform Bindings Points
    //GLuint matricesUniLoc = 11;
    GLuint mMaterialUniformLocation;

    // The sampler uniform for textured models
    // we are assuming a single texture so this will
    //always be texture unit 0
//    GLuint texUnit = 0;

    // Uniform Buffer for Matrices
    // this buffer will contain 3 matrices: projection, view and model
    // each matrix is a float array with 16 components
    GLuint matricesUniBuffer;
    #define MatricesUniBufferSize sizeof(float) * 16 * 3
    #define ProjMatrixOffset 0
    #define ViewMatrixOffset sizeof(float) * 16
    #define ModelMatrixOffset sizeof(float) * 16 * 2
    #define MatrixSize sizeof(float) * 16


    // Program and Shader Identifiers
//    GLuint program, vertexShader, fragmentShader;

    // Shader Names
//    char *vertexFileName = "dirlightdiffambpix.vert";
//    char *fragmentFileName = "dirlightdiffambpix.frag";

    // Create an instance of the Importer class
    Assimp::Importer mAssimpImporter;

    // the global Assimp scene object
    const aiScene* mAssimpScene;

    // scale factor for the model to fit in the window
//    float scaleFactor;


    // images / texture map image filenames to textureIds
    // pointer to texture Array
    std::map<std::string, GLuint> mTextureIdMap;

public:
    Model(const QFile& file, const QString& mediaPrefix = QString(), QObject *parent = 0);

    QMatrix4x4 modelTransform() {return mModelTransform;}

    void render();

public slots:
    void slotSetModelTransform(const QMatrix4x4 &transform) {mModelTransform = transform;}

};

#endif // MODEL_H
