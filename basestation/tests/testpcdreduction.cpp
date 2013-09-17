#include "testpcdreduction.h"
#include <QString>
#include <QSurfaceFormat>
#include <QScreen>

TestPcdReduction::TestPcdReduction()
{
    mPointCloudCuda = new PointCloudCuda(Box3D(), 8*1024*1024, "TestCloud");
    mPointCloudCuda->setMinimumPointDistance(0.02f);

    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setMajorVersion(OPENGL_VERSION_MAJOR);
    format.setMinorVersion(OPENGL_VERSION_MINOR);
    format.setSamples(4);
    format.setProfile(QSurfaceFormat::CoreProfile);

    mOffScreenSurface = new QOffscreenSurface;
    mOffScreenSurface->setFormat(format);
    mOffScreenSurface->create();

    // Create an OpenGL context
    mOpenGlContext = new QOpenGLContext;
    mOpenGlContext->setFormat(format);
    mOpenGlContext->create();
}

void GlWindow::slotInitialize()
{
    qDebug() << __PRETTY_FUNCTION__;
}

TestPcdReduction::~TestPcdReduction()
{
    mOpenGlContext->deleteLater();
    delete mPointCloudCuda;
}

void TestPcdReduction::doTest()
{
    mOpenGlContext->makeCurrent(mOffScreenSurface);
    CudaHelper::initializeCuda();

    const QString fileName = "../tests/cloud.ply";
    mPointCloudCuda->importFromFile(fileName, nullptr);
    qDebug() << __PRETTY_FUNCTION__ << "pointcloud contains" << mPointCloudCuda->getNumberOfPoints() << "points.";
    mPointCloudCuda->slotReduce();
    qDebug() << __PRETTY_FUNCTION__ << "pointcloud contains" << mPointCloudCuda->getNumberOfPoints() << "points.";
    mPointCloudCuda->exportToFile("TestPcdReduction.ply", nullptr);
}

int main(int argc, char *argv[])
{
    QGuiApplication a(argc, argv);
    TestPcdReduction t;
    t.doTest();
}
