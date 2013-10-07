#include "testpcdreduction.h"
#include <QString>
#include <QSurfaceFormat>
#include <QScreen>

TestPcdReduction::TestPcdReduction()
{
    mPointCloudCuda = new PointCloudCuda(Box3D(QVector3D(-512, -512, -512), QVector3D(512, 512, 512)), 8*1024*1024, "TestCloud");
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

//    for(int i=0;i<10;i++) {
        mPointCloudCuda->importFromFile(fileIn, nullptr);
//        for(int j=0;j<5;j++) {
//            qDebug() << __PRETTY_FUNCTION__ << "before" << j << "pointcloud" << i << "contains" << mPointCloudCuda->getNumberOfPointsStored() << "points.";
            mPointCloudCuda->slotReduce();
//            qDebug() << __PRETTY_FUNCTION__ << "after" << j << "pointcloud" << i << "contains" << mPointCloudCuda->getNumberOfPointsStored() << "points.";
//            mPointCloudCuda->exportToFile(fileOut.arg(i).arg(j), nullptr);
            mPointCloudCuda->exportToFile(fileOut.arg(0).arg(0), nullptr);
//        }
        mPointCloudCuda->slotReset();
        usleep(100000);
//    }
}

int main(int argc, char *argv[])
{
    QGuiApplication a(argc, argv);
    QString fileIn("../tests/cloud2m.ply");
    QString fileOut("TestPcdReduction-%1-%2.ply");

    const QStringList args = a.arguments();
    if(args.size() > 1)
    {
        fileIn = args.at(args.size() - 2);
        fileOut = args.at(args.size() - 1);
    }

    TestPcdReduction test;
    QTime time;time.start();
    test.doTest(fileIn, fileOut);
    qDebug() << "total test time in msec:" << time.elapsed();
}
