#ifndef CAMERA_SOURCE_H
#define CAMERA_SOURCE_H

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <malloc.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <QtCore>
#include <QVector3D>
#include <QQuaternion>

typedef struct
{
    void *start;
    int length;
    long long timestamp;
} Frame;

class Camera : public QObject
{
    Q_OBJECT

public:
    Camera(const QString& device, const QSize& imageSize, const QVector3D position, const QQuaternion& orientaiton, quint8 fps);
    ~Camera();
    int retrieveFrame(Frame *frame);


    // helpers
    static inline u_char clampValue(int value);
    static void convertToBgr(u_char *src, u_char *dst, int width, int height);
    static void convertToRgb(u_char *src, u_char *dst, int width, int height);

private:
    QVector3D mPosition;
    QQuaternion mOrientation;
    QTimer* mCaptureTimer;
    QString mDeviceFile;
    int mFileDescriptor;
    int mBufferCount;
    quint16 mWidth, mHeight;
    Frame *mFrameBuffer;
    QByteArray *mImageData;
    quint8 mFps;

    int initDevice();
    int uninitDevice();
    int startCapturing();
    int stopCapturing();

    // helpers:
    static int xioctl(int fd, int request, void *arg);

public slots:
    void slotSetEmitFrameRate(const quint8 fps);
    void slotReadAndEmitCurrentFrame();

signals:
    void imageReady(const QString& name, const QVector3D& position, const QQuaternion& orientation, const QByteArray* image);
};

#endif // CAMERA_SOURCE_H
