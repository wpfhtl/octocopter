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
#include <QImage>

#include <pose.h>

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
    Camera(const QString& device, const QSize& imageSize, const Pose& pose, quint8 fps);
    ~Camera();
    const QSize getImageSize() const;
    int retrieveFrame(Frame *frame);

    // helpers
    static inline unsigned char clampValue(int value);
    static void convertYCbCr422ToBgr888(unsigned char *src, unsigned char *dst, int width, int height);
    static void convertYCbCr422ToRgb888(unsigned char *src, unsigned char *dst, int width, int height);
//    static void convertYCbCr422ToGray8(unsigned char *src, unsigned char *dst, int width, int height);

    // Convert Y'CbCr 4:2:2 to Gray8. Defined here because we need it in VisualOdometry
    static void convertYCbCr422ToGray8(unsigned char *src, unsigned char *dst, int width, int height)
    {
        int y0, y1, cb, cr;
        int w = width / 2;
        int h = height;

        // count down through the rows ...
        while (h--)
        {
            // ... and process the image row 4 bytes at a time
            while (w--)
            {
                y0 = *src++;
                cb = *src++ - 128;
                y1 = *src++;
                cr = *src++ - 128;

                // RGB pixel 1
                *dst++ = y0;//clampValue(y0 + ((cr * 359) / 256));
    //            *dst++ = clampValue(y0 - ((cb * 11) / 32) - ((cr * 91) / 128));
    //            *dst++ = clampValue(y0 + ((cb * 113) / 64));

                // RGB pixel 2
                *dst++ = y1;//clampValue(y1 + ((cr * 359) / 256));
    //            *dst++ = clampValue(y1 - ((cb * 11) / 32) - ((cr * 91) / 128));
    //            *dst++ = clampValue(y1 + ((cb * 113) / 64));
            }
            w = width / 2;
        }
    };

private:
    QSize mImageSize;
    Pose mPose;
    QTimer* mCaptureTimer;
    QString mDeviceFile;
    int mFileDescriptor;
    int mBufferCount;
    Frame *mFrameBuffer;
    QByteArray *mImageData;
    quint8 mFps;

    int initDevice();
    int uninitDevice();
    int startCapturing();
    int stopCapturing();

    // helpers:
    static int xioctl(int fd, int request, void *arg);

private slots:
    void slotReadAndEmitCurrentFrame();

signals:
    // Here, the imagedata is INvalid after the emit, so copy it immediately.
    void imageReadyJpeg(const QString& name, const QSize& imageSize, const Pose& pose, const QByteArray* image);
    // Here, the imagedata is valid until the next frame is taken
    void imageReadyYCbCr(const QString& name, const QSize& imageSize, const Pose& pose, const QByteArray image);
};

#endif // CAMERA_SOURCE_H
