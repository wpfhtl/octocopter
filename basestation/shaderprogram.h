#ifndef SHADERPROGRAM_H
#define SHADERPROGRAM_H

#include <QObject>
#include <QDir>
#include <QGLShader>
#include <QGLShaderProgram>

class ShaderProgram : public QGLShaderProgram
{
    Q_OBJECT

public:
    //ShaderProgram(QObject *parent = 0);
    ShaderProgram(QObject *parent = 0, const QString& shaderVertex = QString(), const QString& shaderGeometry = QString(), const QString& shaderFragment = QString());

    void bindUniformBlockToPoint(const QString& uniformBlockName, unsigned int bindingPoint);

    static const GLuint blockBindingPointGlobalMatrices = 1;

signals:

public slots:

};

#endif // SHADERPROGRAM_H
