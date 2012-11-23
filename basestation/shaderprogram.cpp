#include <GL/glew.h>
#include "shaderprogram.h"

ShaderProgram::ShaderProgram(QObject *parent, const QString& shaderVertex, const QString& shaderGeometry, const QString& shaderFragment) : QGLShaderProgram(parent)
{
    QDir shaderPath = QDir::current();
    shaderPath.cdUp(); // because we're in the build/ subdir

    if(!shaderVertex.isEmpty())
    {
    if(!addShaderFromSourceFile(QGLShader::Vertex, shaderPath.absolutePath() + "/" + shaderVertex))
        qDebug() << "ShaderProgram::ShaderProgram(): compiling vertex shader" << shaderVertex << "failed, log:" << log();
//    else
//        qDebug() << "ShaderProgram::ShaderProgram(): compiling vertex shader" << shaderVertex << "succeeded, log:" << log();
    }

    if(!shaderGeometry.isEmpty())
    {
        if(!addShaderFromSourceFile(QGLShader::Geometry, shaderPath.absolutePath() + "/" + shaderGeometry))
            qDebug() << "ShaderProgram::ShaderProgram(): compiling geometry shader" << shaderGeometry << "failed, log:" << log();
//        else
//            qDebug() << "ShaderProgram::ShaderProgram(): compiling geometry shader" << shaderGeometry << "succeeded, log:" << log();
    }

    if(!shaderFragment.isEmpty())
    {
        if(!addShaderFromSourceFile(QGLShader::Fragment, shaderPath.absolutePath() + "/" + shaderFragment))
            qDebug() << "ShaderProgram::ShaderProgram(): compiling fragment shader" << shaderFragment << "failed, log:" << log();
//        else
//            qDebug() << "ShaderProgram::ShaderProgram(): compiling fragment shader" << shaderFragment << "succeeded, log:" << log();
    }

    if(link())
    {
//        qDebug() << "ShaderProgram::ShaderProgram(): linking shader program with id" << programId() << "succeeded, log:" << log();
        GLint numberOfActiveAttributes, numberOfAttachedShaders, numberOfActiveUniforms, numberOfGeometryVerticesOut = 0;
        glGetProgramiv(programId(), GL_ATTACHED_SHADERS, &numberOfAttachedShaders);
        glGetProgramiv(programId(), GL_ACTIVE_ATTRIBUTES, &numberOfActiveAttributes);
        glGetProgramiv(programId(), GL_ACTIVE_UNIFORMS, &numberOfActiveUniforms);

        if(!shaderGeometry.isEmpty())
            glGetProgramiv(programId(), GL_GEOMETRY_VERTICES_OUT, &numberOfGeometryVerticesOut);

//        qDebug() << "ShaderProgram::ShaderProgram(): shader program has" << numberOfAttachedShaders<< "shaders and" << numberOfGeometryVerticesOut << "vertices geometry shader output.";

        QStringList activeAttributes;
        for(int i=0; i < numberOfActiveAttributes; i++)
        {
            GLchar attributeName[1024];
            GLint attributeSize;
            GLenum attributeType;
            glGetActiveAttrib(programId(), i, 1024, NULL, &attributeSize, &attributeType, attributeName);
            QString attributeDescription = QString("%1 of size %2, type %3, location %4")
                    .arg(attributeName)
                    .arg(attributeSize)
                    .arg(attributeType)
                    .arg(glGetAttribLocation(programId(), attributeName));
            activeAttributes << attributeDescription;
        }

//        qDebug() << "ShaderProgram::ShaderProgram(): shader program has" << numberOfActiveAttributes << "active attributes:" << activeAttributes.join(", ");

        QStringList activeUniforms;
        for(int i=0; i < numberOfActiveUniforms; i++)
        {
            GLchar uniformName[1024];
            GLint uniformSize;
            GLenum uniformType;
            glGetActiveUniform(programId(), i, 1024, NULL, &uniformSize, &uniformType, uniformName);
            QString uniformDescription = QString("%1 of size %2, type %3, location %4")
                    .arg(uniformName)
                    .arg(uniformSize)
                    .arg(uniformType)
                    .arg(glGetUniformLocation(programId(), uniformName));
            activeUniforms << uniformDescription;
        }

//        qDebug() << "ShaderProgram::ShaderProgram(): shader program has" << numberOfActiveUniforms << "active uniforms:" << activeUniforms.join(", ");

        // Try to bind the program's uniform block "GlobalValues" to a constant point, defined in
        // blockBindingPoint. This is needed to conect the program's UB to a uniform-buffer-object,
        // which can be updated from GlWidget.
        // This is so universal that we try to make this onnection for every shader program. If it
        // fails because there is no such uniform, thats fine by me.
        bindUniformBlockToPoint("GlobalValues", blockBindingPointGlobalMatrices);
    }
    else
        qDebug() << "ShaderProgram::ShaderProgram(): linking shader program failed, log:" << log();
}

void ShaderProgram::bindUniformBlockToPoint(const QString& uniformBlockName, unsigned int bindingPoint)
{
    // Bind uniform block @uniformBlockName to constant binding point @bindingPoint.
    // This is needed to conect the program's UB to a uniform-buffer-object, which can be updated from
    // e.g. GlWidget.
    const GLuint uniformBlockIndex = glGetUniformBlockIndex(programId(), qPrintable(uniformBlockName));
    if(uniformBlockIndex == GL_INVALID_INDEX)
        qDebug() << "ShaderProgram::bindUniformBlockToPoint(): uniform block with name" << uniformBlockName << "couldn't be found!";
    else
        glUniformBlockBinding(programId(), uniformBlockIndex, bindingPoint);
}
