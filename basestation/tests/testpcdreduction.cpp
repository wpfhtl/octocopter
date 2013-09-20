#include "testpcdreduction.h"
#include <QString>
#include <QSurfaceFormat>
#include <QScreen>

TestPcdReduction::TestPcdReduction()
{
    mPointCloudCuda = new PointCloudCuda(Box3D(), 8*1024*1024, "TestCloud");
    mPointCloudCuda->setMinimumPointDistance(0.2f);

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

void TestPcdReduction::doTest(const QString& fileIn, const QString& fileOut)
{
    mOpenGlContext->makeCurrent(mOffScreenSurface);
    CudaHelper::initializeCuda();

    mPointCloudCuda->importFromFile(fileIn, nullptr);
    qDebug() << __PRETTY_FUNCTION__ << "pointcloud contains" << mPointCloudCuda->getNumberOfPoints() << "points.";
    mPointCloudCuda->slotReduce();
    qDebug() << __PRETTY_FUNCTION__ << "pointcloud contains" << mPointCloudCuda->getNumberOfPoints() << "points.";
    mPointCloudCuda->exportToFile(fileOut, nullptr);
}

int main(int argc, char *argv[])
{
    QGuiApplication a(argc, argv);
    QString fileIn("../tests/cloud.ply");
    QString fileOut("TestPcdReduction.ply");

    const QStringList args = a.arguments();
    if(args.size() > 1)
    {
        fileIn = args.at(args.size() - 2);
        fileOut = args.at(args.size() - 1);
    }

    TestPcdReduction t;
    t.doTest(fileIn, fileOut);
}
