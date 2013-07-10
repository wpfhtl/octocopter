#ifndef SHADERPROGRAM_H
#define SHADERPROGRAM_H

#include <QObject>
#include <QDir>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>

class ShaderProgram : public QOpenGLShaderProgram, public OPENGL_FUNCTIONS_CLASS
{
    Q_OBJECT

    QString mNameShaderVertex, mNameShaderGeometry, mNameShaderFragment;

public:
    //ShaderProgram(QObject *parent = 0);
    ShaderProgram(QObject *parent = 0, const QString& shaderVertex = QString(), const QString& shaderGeometry = QString(), const QString& shaderFragment = QString());
    ~ShaderProgram();

    void bindUniformBlockToPoint(const QString& uniformBlockName, unsigned int bindingPoint);
    bool initialize();

    static const GLuint blockBindingPointGlobalMatrices = 1;

signals:

public slots:

};

#endif // SHADERPROGRAM_H
