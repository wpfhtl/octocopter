#include <GL/glew.h>
#include <IL/il.h> // DevIL for image loading

#include "model.h"

Model::Model(const QFile& file, const QString& mediaPrefix, QObject *parent) : QObject(parent)
{
    mShaderProgram = 0;
    mAssimpScene = 0;
    mMediaPrefix = mediaPrefix;

    if(!importFile(file)) return;

    // This needs to be unique!
    mMaterialUniformLocation = 2;

    loadGlTextures(mAssimpScene);

    mShaderProgram = new ShaderProgram(this, "shader-model-vertex.c", "", "shader-model-fragment.c");

    mShaderProgram->bindUniformBlockToPoint("Material", mMaterialUniformLocation);

    //program = setupShaders();
    generateVAOsAndUniformBuffer(mAssimpScene);

    //    glEnable(GL_DEPTH_TEST);
    //    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

    //
    // Uniform Block
    //
    /* superfluous, these are the global matrices we have in glwidget
    glGenBuffers(1,&matricesUniBuffer);
    glBindBuffer(GL_UNIFORM_BUFFER, matricesUniBuffer);
    glBufferData(GL_UNIFORM_BUFFER, MatricesUniBufferSize,NULL,GL_DYNAMIC_DRAW);
    glBindBufferRange(GL_UNIFORM_BUFFER, matricesUniLoc, matricesUniBuffer, 0, MatricesUniBufferSize);	//setUniforms();
    glBindBuffer(GL_UNIFORM_BUFFER,0);
*/
    //glEnable(GL_MULTISAMPLE);

}

bool Model::importFile(const QFile& modelFile)
{
    if(!modelFile.exists())
    {
        qDebug() << "Model::importFile(): import file" << modelFile.fileName() << "does not exist!";
        return false;
    }

    mAssimpScene = mAssimpImporter.ReadFile(modelFile.fileName().toStdString(), aiProcessPreset_TargetRealtime_Quality);

    // If the import failed, report it
    if(!mAssimpScene)
    {
        qDebug() << "Model::importFile(): couldn't import file" << modelFile.fileName() << ":" << mAssimpImporter.GetErrorString();
        return false;
    }
    else
    {
        qDebug() << "Model::importFile(): successfully imported file" << modelFile.fileName();
    }

    /* unnecessary, was to fit into a view window
        struct aiVector3D scene_min, scene_max, scene_center;
        getBoundingBox(&scene_min, &scene_max);
        float tmp;
        tmp = scene_max.x-scene_min.x;
        tmp = scene_max.y - scene_min.y > tmp ? scene_max.y - scene_min.y : tmp;
        tmp = scene_max.z - scene_min.z > tmp ? scene_max.z - scene_min.z : tmp;
        scaleFactor = 1.f / tmp;*/

    // We're done. Everything will be cleaned up by the importer destructor
    return true;
}


void Model::loadGlTextures(const aiScene* scene)
{
    ILboolean success;

    /* initialization of DevIL */
    ilInit();

    /* scan scene's materials for textures */
    for (unsigned int m=0; m<scene->mNumMaterials; ++m)
    {
        int texIndex = 0;
        aiString path;	// filename

        aiReturn textureLookupResult = scene->mMaterials[m]->GetTexture(aiTextureType_DIFFUSE, texIndex, &path);
        while (textureLookupResult == AI_SUCCESS) {
            //fill map with textures, OpenGL image ids set to 0
            mTextureIdMap[path.data] = 0;
            qDebug() << "Found texture:" << path.data;
            // more textures?
            texIndex++;
            textureLookupResult = scene->mMaterials[m]->GetTexture(aiTextureType_DIFFUSE, texIndex, &path);
        }
    }

    int numTextures = mTextureIdMap.size();

    /* create and fill array with DevIL texture ids */
    ILuint* imageIds = new ILuint[numTextures];
    ilGenImages(numTextures, imageIds);

    /* create and fill array with GL texture ids */
    GLuint* textureIds = new GLuint[numTextures];
    glGenTextures(numTextures, textureIds); /* Texture name generation */

    /* get iterator */
    std::map<std::string, GLuint>::iterator itr = mTextureIdMap.begin();
    int i=0;
    for (; itr != mTextureIdMap.end(); ++i, ++itr)
    {
        //save IL image ID
        std::string filename = (*itr).first;  // get filename
        QString fileNameAbsolute = QString(mMediaPrefix).append(QString::fromStdString(filename));
        qDebug() << "Model::loadGlTextures(): trying to load texture:" << fileNameAbsolute;
        (*itr).second = textureIds[i];	  // save texture id for filename in map

        ilBindImage(imageIds[i]); /* Binding of DevIL image name */
        ilEnable(IL_ORIGIN_SET);
        ilOriginFunc(IL_ORIGIN_LOWER_LEFT);
        //success = ilLoadImage((ILstring)filename.c_str());
        success = ilLoadImage((ILstring)qPrintable(fileNameAbsolute));


        if (success) {
            /* Convert image to RGBA */
            ilConvertImage(IL_RGBA, IL_UNSIGNED_BYTE);

            /* Create and load textures to OpenGL */
            glBindTexture(GL_TEXTURE_2D, textureIds[i]);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ilGetInteger(IL_IMAGE_WIDTH), ilGetInteger(IL_IMAGE_HEIGHT), 0, GL_RGBA, GL_UNSIGNED_BYTE, ilGetData());
        }
        else
        {
            qDebug() << "Model::loadGlTextures(): couldn't load image" << filename.c_str();
        }
    }

    /* Because we have already copied image data into texture data we can release memory used by image. */
    ilDeleteImages(numTextures, imageIds);

    //Cleanup
    delete [] imageIds;
    delete [] textureIds;
}

void Model::setFloat4(float f[4], float a, float b, float c, float d) const
{
    f[0] = a;
    f[1] = b;
    f[2] = c;
    f[3] = d;
}

void Model::color4ToFloat4(const struct aiColor4D *c, float f[4]) const
{
    f[0] = c->r;
    f[1] = c->g;
    f[2] = c->b;
    f[3] = c->a;
}

void Model::generateVAOsAndUniformBuffer(const struct aiScene *scene)
{
    struct MyMesh aMesh;
    struct MyMaterial aMat;
    GLuint buffer;

    // For each mesh
    for (unsigned int n = 0; n < scene->mNumMeshes; ++n)
    {
        const struct aiMesh* mesh = scene->mMeshes[n];
        //qDebug() << "Model::generateVAOsAndUniformBuffer(): loading mesh" << QString::fromUtf8(mesh->mName.data, mesh->mName.length) << "with" << mesh->mNumFaces << "faces.";

        // create array with faces
        // have to convert from Assimp format to array
        unsigned int *faceArray;
        faceArray = (unsigned int *)malloc(sizeof(unsigned int) * mesh->mNumFaces * 3);
        unsigned int faceIndex = 0;

        for (unsigned int t = 0; t < mesh->mNumFaces; ++t)
        {
            const struct aiFace* face = &mesh->mFaces[t];

            memcpy(&faceArray[faceIndex], face->mIndices,3 * sizeof(float));
            faceIndex += 3;
        }
        aMesh.numFaces = scene->mMeshes[n]->mNumFaces;

        // generate Vertex Array for mesh
        glGenVertexArrays(1,&(aMesh.vao));
        glBindVertexArray(aMesh.vao);

        // buffer for faces
        glGenBuffers(1, &buffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * mesh->mNumFaces * 3, faceArray, GL_STATIC_DRAW);

        // buffer for vertex positions
        if(mesh->HasPositions())
        {
            mAttributeLocationVertex = mShaderProgram->attributeLocation("position");
            Q_ASSERT(mAttributeLocationVertex != -1 && "Model::generateVAOsAndUniformBuffer(): cannot find shader attribute!");
            glGenBuffers(1, &buffer);
            glBindBuffer(GL_ARRAY_BUFFER, buffer);
            glBufferData(GL_ARRAY_BUFFER, sizeof(float)*3*mesh->mNumVertices, mesh->mVertices, GL_STATIC_DRAW);
            glEnableVertexAttribArray(mAttributeLocationVertex);
            glVertexAttribPointer(mAttributeLocationVertex, 3, GL_FLOAT, 0, 0, 0);
        }

        // buffer for vertex normals
        if (mesh->HasNormals())
        {
            mAttributeLocationNormal = mShaderProgram->attributeLocation("normal");
            Q_ASSERT(mAttributeLocationNormal != -1 && "Model::generateVAOsAndUniformBuffer(): cannot find shader attribute!");
            glGenBuffers(1, &buffer);
            glBindBuffer(GL_ARRAY_BUFFER, buffer);
            glBufferData(GL_ARRAY_BUFFER, sizeof(float)*3*mesh->mNumVertices, mesh->mNormals, GL_STATIC_DRAW);
            glEnableVertexAttribArray(mAttributeLocationNormal);
            glVertexAttribPointer(mAttributeLocationNormal, 3, GL_FLOAT, 0, 0, 0);
        }

        // buffer for vertex texture coordinates
        if (mesh->HasTextureCoords(0))
        {
            float *texCoords = (float *)malloc(sizeof(float)*2*mesh->mNumVertices);
            for (unsigned int k = 0; k < mesh->mNumVertices; ++k)
            {
                texCoords[k*2]   = mesh->mTextureCoords[0][k].x;
                texCoords[k*2+1] = mesh->mTextureCoords[0][k].y;
            }

            mAttributeLocationTexCoord = mShaderProgram->attributeLocation("texCoord");
            Q_ASSERT(mAttributeLocationTexCoord != -1 && "Model::generateVAOsAndUniformBuffer(): cannot find shader attribute!");
            glGenBuffers(1, &buffer);
            glBindBuffer(GL_ARRAY_BUFFER, buffer);
            glBufferData(GL_ARRAY_BUFFER, sizeof(float)*2*mesh->mNumVertices, texCoords, GL_STATIC_DRAW);
            glEnableVertexAttribArray(mAttributeLocationTexCoord);
            glVertexAttribPointer(mAttributeLocationTexCoord, 2, GL_FLOAT, 0, 0, 0);
        }

        // unbind buffers
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER,0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);

        // create material uniform buffer
        struct aiMaterial *mtl = scene->mMaterials[mesh->mMaterialIndex];

        aiString texPath;	//contains filename of texture
        if(AI_SUCCESS == mtl->GetTexture(aiTextureType_DIFFUSE, 0, &texPath))
        {
            //bind texture
            unsigned int texId = mTextureIdMap[texPath.data];
            aMesh.texIndex = texId;
            aMat.texCount = 1;
        }
        else
        {
            aMat.texCount = 0;
        }

        float c[4];
        setFloat4(c, 0.8f, 0.8f, 0.8f, 1.0f);
        aiColor4D diffuse;
        if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
        {
            color4ToFloat4(&diffuse, c);
        }
        memcpy(aMat.diffuse, c, sizeof(c));

        setFloat4(c, 0.2f, 0.2f, 0.2f, 1.0f);
        aiColor4D ambient;
        if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
        {
            color4ToFloat4(&ambient, c);
        }
        memcpy(aMat.ambient, c, sizeof(c));

        setFloat4(c, 0.0f, 0.0f, 0.0f, 1.0f);
        aiColor4D specular;
        if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
        {
            color4ToFloat4(&specular, c);
        }
        memcpy(aMat.specular, c, sizeof(c));

        setFloat4(c, 0.0f, 0.0f, 0.0f, 1.0f);
        aiColor4D emission;
        if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
        {
            color4ToFloat4(&emission, c);
        }
        memcpy(aMat.emissive, c, sizeof(c));

        float shininess = 0.0;
        unsigned int max;
        aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
        aMat.shininess = shininess;

        glGenBuffers(1,&(aMesh.uniformBlockIndex));
        glBindBuffer(GL_UNIFORM_BUFFER, aMesh.uniformBlockIndex);
        glBufferData(GL_UNIFORM_BUFFER, sizeof(aMat), (void *)(&aMat), GL_STATIC_DRAW);

        myMeshes.push_back(aMesh);
    }
}

void Model::render()
{
    if(!mShaderProgram)
    {
        qDebug() << "Model::render(): initialization failed, cannot render";
        return;
    }

    // use our shader
//    glUseProgram(program);
    mShaderProgram->bind();

    // we are only going to use texture unit 0
    // unfortunately samplers can't reside in uniform blocks
    // so we have set this uniform separately
    mShaderProgram->setUniformValue("texUnit", 0);
//    glUniform1i(glGetUniformLocation(p,"texUnit");,0);

//    glDisable(GL_CULL_FACE);
    glFrontFace(GL_CCW);

    glEnable (GL_BLEND); glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    renderRecursively(mAssimpScene, mAssimpScene->mRootNode);
    glDisable(GL_BLEND);
//    glEnable(GL_CULL_FACE);

    // swap buffers
//    glutSwapBuffers();
}

void Model::renderRecursively(const struct aiScene *scene, const struct aiNode* node)
{
/*
    // Get node transformation matrix
    struct aiMatrix4x4 m = node->mTransformation;
    // OpenGL matrices are column major
    m.Transpose();

    // save model matrix and apply node transformation
    pushMatrix();

    float aux[16];
    memcpy(aux,&m,sizeof(float) * 16);
    multMatrix(modelMatrix, aux);
    setModelMatrix();
*/

//    qDebug() << "Model::renderRecursively(): rendering scene" << scene << "and node" << node;

    // "push" the matrix :)
    const QMatrix4x4 matrixModelTransformOld = mModelTransform;

    // retrieve thisnode's transform (aiMatrix4x4 is row-major, like QMatrix4x4's c'tor)
    qreal valuesQReal[16];
    for(int i=0;i<16;i++) valuesQReal[i] = *(node->mTransformation[i]);
    QMatrix4x4 matrixModelSubMeshTransform(valuesQReal);
    matrixModelSubMeshTransform.optimize();

    // Combine matrices
  //  mModelTransform = mModelTransform * matrixModelSubMeshTransform;
    // Send this temporary matrix into the shaderprogram's uniform
    mModelTransform.copyDataTo(valuesQReal);

    mShaderProgram->setUniformValue("matrixModelSubMeshTransform", mModelTransform);
//    mShaderProgram->setUniformValue("matrixModelSubMeshTransform", QMatrix4x4());
    /*
    GLint loc = glGetUniformLocation(mShaderProgram->programId(), "matrixModelSubMeshTransform");
    if(loc != -1)
    {
       float valuesFloat[16];
       for(int i=0;i<16;i++) valuesFloat[i] = valuesQReal[i];
       glUniformMatrix4fv(loc, 1, GL_TRUE, valuesFloat);
    } else {
        qDebug() << "uniform not found!";
    }*/

    // draw all meshes assigned to this node
    for (unsigned int n=0; n < node->mNumMeshes; ++n)
    {
        // bind material uniform
        glBindBufferRange(GL_UNIFORM_BUFFER, mMaterialUniformLocation, myMeshes[node->mMeshes[n]].uniformBlockIndex, 0, sizeof(struct MyMaterial));
        // bind texture
        glBindTexture(GL_TEXTURE_2D, myMeshes[node->mMeshes[n]].texIndex);
        // bind VAO
        glBindVertexArray(myMeshes[node->mMeshes[n]].vao);
        // draw
        glDrawElements(GL_TRIANGLES, myMeshes[node->mMeshes[n]].numFaces * 3, GL_UNSIGNED_INT, 0);
    }

    // draw all children
    for (unsigned int n=0; n < node->mNumChildren; ++n)
    {
        renderRecursively(scene, node->mChildren[n]);
    }

    // popMatrix();
    mModelTransform = matrixModelTransformOld;
}


// Rendering Callback Function


/*
GLuint Model::setupShaders()
{
    char *vs = NULL,*fs = NULL,*fs2 = NULL;

    GLuint p,v,f;

    v = glCreateShader(GL_VERTEX_SHADER);
    f = glCreateShader(GL_FRAGMENT_SHADER);

    vs = textFileRead(vertexFileName);
    fs = textFileRead(fragmentFileName);

    const char * vv = vs;
    const char * ff = fs;

    glShaderSource(v, 1, &vv,NULL);
    glShaderSource(f, 1, &ff,NULL);

    free(vs);free(fs);

    glCompileShader(v);
    glCompileShader(f);

    printShaderInfoLog(v);
    printShaderInfoLog(f);

    p = glCreateProgram();
    glAttachShader(p,v);
    glAttachShader(p,f);


    // WARNING FIXME: What does this do? Not translated!
    glBindFragDataLocation(mShaderProgram->programId(), 0, "output");

//    glBindAttribLocation(p,mAttributeLocationVertex,"position");
//    glBindAttribLocation(p,mAttributeLocationNormal,"normal");
//    glBindAttribLocation(p,mAttributeLocationTexCoord,"texCoord");

//    glLinkProgram(p);
//    glValidateProgram(p);
//    printProgramInfoLog(p);

//    program = p;
//    vertexShader = v;
//    fragmentShader = f;

//    GLuint k = glGetUniformBlockIndex(p,"Matrices");
//    glUniformBlockBinding(p, k, matricesUniLoc);
//    glUniformBlockBinding(p, glGetUniformBlockIndex(p,"Material"), materialUniLoc);



//    return(p);
}

*/
