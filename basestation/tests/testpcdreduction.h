#ifndef TESTPCDREDUCTION_H
#define TESTPCDREDUCTION_H

#include "../basestation.h"
#include "../pointcloudcuda.h"

#include <QOffscreenSurface>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFunctions_4_3_Core>

class TestPcdReduction : protected OPENGL_FUNCTIONS_CLASS
{
public:
    TestPcdReduction();
    ~TestPcdReduction();
    void doTest(const QString &fileIn, const QString &fileOut);

private:
    QOffscreenSurface* mOffScreenSurface;
    QOpenGLContext* mOpenGlContext;
    PointCloudCuda* mPointCloudCuda;
};

#endif
