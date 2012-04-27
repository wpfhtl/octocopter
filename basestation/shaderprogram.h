#ifndef SHADERPROGRAM_H
#define SHADERPROGRAM_H

#include <QObject>
#include <QDir>
#include <QGLShader>
#include <QGLShaderProgram>

class ShaderProgram : public QObject
{
    Q_OBJECT

    QGLShaderProgram* mShaderProgram;
public:
    ShaderProgram(QObject *parent = 0);

    bool setShaders(
        const QString& shaderVertex,
        const QString& shaderGeometry,
        const QString& shaderFragment);

    void bind();

    void release();

signals:

public slots:

};

#endif // SHADERPROGRAM_H
