#include <GL/glew.h>
#include "shaderprogram.h"

ShaderProgram::ShaderProgram(QObject *parent) : QObject(parent)
{
    mShaderProgram = 0;
}


bool ShaderProgram::setShaders(
    const QString& shaderVertex,
    const QString& shaderGeometry,
    const QString& shaderFragment)
{
    mShaderProgram = new QGLShaderProgram(this);

    QDir shaderPath = QDir::current();
    shaderPath.cdUp(); // because we're in the build/ subdir

    if(!shaderVertex.isEmpty())
    {
    if(mShaderProgram->addShaderFromSourceFile(QGLShader::Vertex, shaderPath.absolutePath() + "/" + shaderVertex))
        qDebug() << "ShaderProgram::setShaders(): compiling vertex shader succeeded, log:" << mShaderProgram->log();
    else
        qDebug() << "ShaderProgram::setShaders(): compiling vertex shader failed, log:" << mShaderProgram->log();
    }

    if(!shaderGeometry.isEmpty())
    {
        if(mShaderProgram->addShaderFromSourceFile(QGLShader::Geometry, shaderPath.absolutePath() + "/" + shaderGeometry))
            qDebug() << "ShaderProgram::setShaders(): compiling geometry shader succeeded, log:" << mShaderProgram->log();
        else
            qDebug() << "ShaderProgram::setShaders(): compiling geometry shader failed, log:" << mShaderProgram->log();
    }

    if(!shaderFragment.isEmpty())
    {
        if(mShaderProgram->addShaderFromSourceFile(QGLShader::Fragment, shaderPath.absolutePath() + "/" + shaderFragment))
            qDebug() << "ShaderProgram::setShaders(): compiling fragment shader succeeded, log:" << mShaderProgram->log();
        else
            qDebug() << "ShaderProgram::setShaders(): compiling fragment shader failed, log:" << mShaderProgram->log();
    }

    if(mShaderProgram->link())
    {
        qDebug() << "ShaderProgram::setShaders(): linking shader program succeeded, log:" << mShaderProgram->log();
        GLint numberOfActiveAttributes, numberOfAttachedShaders, numberOfActiveUniforms, numberOfGeometryVerticesOut;
        glGetProgramiv(mShaderProgram->programId(), GL_ATTACHED_SHADERS, &numberOfAttachedShaders);
        glGetProgramiv(mShaderProgram->programId(), GL_ACTIVE_ATTRIBUTES, &numberOfActiveAttributes);
        glGetProgramiv(mShaderProgram->programId(), GL_ACTIVE_UNIFORMS, &numberOfActiveUniforms);
        glGetProgramiv(mShaderProgram->programId(), GL_GEOMETRY_VERTICES_OUT, &numberOfGeometryVerticesOut);

        qDebug() << "ShaderProgram::setShaders(): shader program has" << numberOfAttachedShaders<< "shaders and" << numberOfGeometryVerticesOut << "vertices geometry shader output.";

        QStringList activeAttributes;
        for(int i=0; i < numberOfActiveAttributes; i++)
        {
            GLchar attributeName[1024];
            GLint attributeSize;
            GLenum attributeType;
            glGetActiveAttrib(mShaderProgram->programId(), i, 1024, NULL, &attributeSize, &attributeType, attributeName);
            QString attributeDescription = QString("%1 of size %2, type %3, location %4")
                    .arg(attributeName)
                    .arg(attributeSize)
                    .arg(attributeType)
                    .arg(glGetAttribLocation(mShaderProgram->programId(), attributeName));
            activeAttributes << attributeDescription;
        }

        qDebug() << "ShaderProgram::setShaders(): shader program has" << numberOfActiveAttributes << "active attributes:" << activeAttributes.join(", ");

        QStringList activeUniforms;
        for(int i=0; i < numberOfActiveUniforms; i++)
        {
            GLchar uniformName[1024];
            GLint uniformSize;
            GLenum uniformType;
            glGetActiveUniform(mShaderProgram->programId(), i, 1024, NULL, &uniformSize, &uniformType, uniformName);
            QString uniformDescription = QString("%1 of size %2, type %3, location %4")
                    .arg(uniformName)
                    .arg(uniformSize)
                    .arg(uniformType)
                    .arg(glGetUniformLocation(mShaderProgram->programId(), uniformName));
            activeUniforms << uniformDescription;
        }

        qDebug() << "ShaderProgram::setShaders(): shader program has" << numberOfActiveUniforms << "active uniforms:" << activeUniforms.join(", ");
    }
    else
        qDebug() << "ShaderProgram::setShaders(): linking shader program failed, log:" << mShaderProgram->log();

}

void ShaderProgram::bind()
{
    if(mShaderProgram)
        mShaderProgram->bind();
    else
        qDebug() << "ShaderProgram::bind(): cannot bind, shader not yet defined.";
}

void ShaderProgram::release()
{
    if(mShaderProgram)
        mShaderProgram->release();
    else
        qDebug() << "ShaderProgram::release(): cannot release, shader not yet defined.";
}
