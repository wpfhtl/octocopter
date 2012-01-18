#include "camera.h"

Camera::Camera(const QString& device, const QSize& imageSize, const Pose& pose, quint8 fps): QObject()
{
    mFileDescriptor = -1;
    mBufferCount = 0;
    mFps = fps;
    mImageSize = imageSize;
    mPose = pose;
    mDeviceFile = device;
    mImageData = new QByteArray(mImageSize.width() * mImageSize.height() * 3, 0); // RGB buffer, filled with zeroes

    mCaptureTimer = new QTimer(this);
    mCaptureTimer->setInterval(1000 / fps);
    connect(mCaptureTimer, SIGNAL(timeout()), SLOT(slotReadAndEmitCurrentFrame()));

    if (-1 == initDevice()) { exit(EXIT_FAILURE); }
    if (-1 == startCapturing()) { exit(EXIT_FAILURE); }
}

Camera::~Camera()
{
    if (-1 == stopCapturing()) { exit(EXIT_FAILURE); }
    if (-1 == uninitDevice()) { exit(EXIT_FAILURE); }
}

const QSize Camera::getImageSize() const
{
    return mImageSize;
}

int Camera::initDevice()
{
    /*** open device ***/
    if (-1 == (mFileDescriptor = open(mDeviceFile.toAscii().constData(), O_RDWR | O_NONBLOCK, 0)))
    {
        qDebug() << "Camera::initDevice(): Could not open device:" << mDeviceFile;
        return -1;
    }

    /*** check device capabilities ***/
    struct v4l2_capability caps;
    memset(&caps, 0, sizeof(caps));

    if (-1 == xioctl(mFileDescriptor, VIDIOC_QUERYCAP, &caps))
    {
        qDebug() << "Camera::initDevice(): Could not query device capabilities [" << strerror(errno) << "]";
        return -1;
    }
    if (!(caps.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        qDebug() << "Camera::initDevice(): Device does not support video capture";
        return -1;
    }
    if (!(caps.capabilities & V4L2_CAP_STREAMING))
    {
        qDebug() << "Camera::initDevice(): Device does not support streaming I/O";
        return -1;
    }

    /*** set frame size ***/
    struct v4l2_format format;
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = mImageSize.width();
    format.fmt.pix.height = mImageSize.height();

    if (-1 == xioctl(mFileDescriptor, VIDIOC_S_FMT, &format))
    {
        qDebug() << "Camera::initDevice(): Error while setting the frame size [" << strerror(errno) << "]";
        return -1;
    }

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1 == xioctl(mFileDescriptor, VIDIOC_G_FMT, &format))
    {
        qDebug() << "Camera::initDevice(): Error while querying the frame size [" << strerror(errno) << "]";
        return -1;
    }

    int frame_width = format.fmt.pix.width;
    int frame_height = format.fmt.pix.height;

    if (mImageSize.width() != frame_width || mImageSize.height() != frame_height)
    {
        qDebug() << "Camera::initDevice(): Could not set the specified frame size" << mImageSize << ", current frame size is:" << frame_width << "x" << frame_height;
        return -1;
    }

    /*** set frame rate ***/
    struct v4l2_streamparm streamparm;
    memset(&streamparm, 0, sizeof(streamparm));
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamparm.parm.capture.timeperframe.numerator = 1;
    streamparm.parm.capture.timeperframe.denominator = mFps;

    if (-1 == xioctl(mFileDescriptor, VIDIOC_S_PARM, &streamparm))
    {
        qDebug() << "Camera::initDevice(): Error while setting the frame rate [" << strerror(errno) << "]";
        return -1;
    }

    memset(&streamparm, 0, sizeof(streamparm));
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1 == xioctl(mFileDescriptor, VIDIOC_G_PARM, &streamparm))
    {
        qDebug() << "Camera::initDevice(): Error while querying the frame rate [" << strerror(errno) << "]";
        return -1;
    }

    int frames = streamparm.parm.capture.timeperframe.denominator;

    if (mFps != frames)
    {
        qDebug() << "Could not set the specified frame rate, current frame rate is: " << frames << "frames/second";
        return -1;
    }

    /*** set up memory mapped buffers ***/
    struct v4l2_requestbuffers reqbuf;
    memset(&reqbuf, 0, sizeof(reqbuf));
    reqbuf.count = 2;
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;

    // request device buffers
    if (-1 == xioctl(mFileDescriptor, VIDIOC_REQBUFS, &reqbuf))
    {
        qDebug() << "Camera::initDevice(): Error while requesting device buffers [" << strerror(errno) << "]";
        return -1;
    }

    // check if request was successful
    if (reqbuf.count < 2)
    {
        qDebug() << "Camera::initDevice(): Device has insufficient buffer memory";
        return -1;
    }

    // allocate local buffer memory and map it
    mFrameBuffer = (Frame *) malloc(sizeof(*mFrameBuffer) * reqbuf.count);

    for (mBufferCount = 0; mBufferCount < (int) reqbuf.count; mBufferCount++)
    {
        struct v4l2_buffer buffer;
        memset(&buffer, 0, sizeof(buffer));
        buffer.index = mBufferCount;
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(mFileDescriptor, VIDIOC_QUERYBUF, &buffer))
        {
            qDebug() << "Camera::initDevice(): Error while querying device buffers [" << strerror(errno) << "]";
            return -1;
        }

        mFrameBuffer[mBufferCount].length = buffer.length;
        mFrameBuffer[mBufferCount].start = mmap(0, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, mFileDescriptor, buffer.m.offset);

        if (MAP_FAILED == mFrameBuffer[mBufferCount].start)
        {
            qDebug() << "Camera::initDevice(): Could not map buffer memory [" << strerror(errno) << "]";
            return -1;
        }
    }

    qDebug() << "Camera::initDevice(): initialization complete.";

    return 0;
}

int Camera::uninitDevice()
{
    /*** unmap buffer memory ***/
    for (int i = 0; i < mBufferCount; i++)
    {
        if (-1 == munmap(mFrameBuffer[i].start, mFrameBuffer[i].length))
        {
            qDebug() << "Could not unmap buffer memory [" << strerror(errno) << "]";
            return -1;
        }
    }

    free(mFrameBuffer);

    /*** close device ***/
    if (-1 == close(mFileDescriptor))
    {
        qDebug() << "Could not close device [" << strerror(errno) << "]";
        return -1;
    }

    mFileDescriptor = -1;
    return 0;
}

int Camera::startCapturing()
{
    /*** enqueue the initial device buffer ***/
    for (int i = 0; i < mBufferCount; i++)
    {
        struct v4l2_buffer buffer;
        memset(&buffer, 0, sizeof(buffer));
        buffer.index = i;
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(mFileDescriptor, VIDIOC_QBUF, &buffer))
        {
            qDebug() << "Camera::startCapturing(): Error while queuing device buffer [" << strerror(errno) << "]";
            return -1;
        }
    }

    /*** turn on the data stream ***/
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1 == xioctl(mFileDescriptor, VIDIOC_STREAMON, &type))
    {
        qDebug() << "Camera::startCapturing(): Unable to turn on data stream [" << strerror(errno) << "]";
        return -1;
    }

    mCaptureTimer->start();

    qDebug() << "Camera::startCapturing(): done.";

    return 0;
}

int Camera::stopCapturing()
{
    /*** turn off the data stream ***/
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1 == xioctl(mFileDescriptor, VIDIOC_STREAMOFF, &type))
    {
        qDebug() << "Camera::stopCapturing(): Unable to turn off data stream [" << strerror(errno) << "]";
        return -1;
    }

    return 0;
}

int Camera::retrieveFrame(Frame *frame)
{
    forever
    {
        fd_set rfds;
        struct timeval tv = {2, 0};
        int ret;

        FD_ZERO(&rfds);
        FD_SET(mFileDescriptor, &rfds);

        if (-1 == (ret = select(mFileDescriptor + 1, &rfds, 0, 0, &tv)))
        {
            if (EINTR == errno)
            {
                continue;
            }
            else
            {
                qDebug() << "Camera::retrieveFrame(): Error while waiting for camera access [" << strerror(errno) << "]";
                return -1;
            }
        }

        if (!ret)
        {
            qDebug() << "Camera::retrieveFrame(): Timeout occured waiting for camera access";
            return -1;
        }

        break;
    }

    /*** dequeue a device buffer ***/
    struct v4l2_buffer buffer;
    memset(&buffer, 0, sizeof(buffer));
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(mFileDescriptor, VIDIOC_DQBUF, &buffer))
    {
        qDebug() << "Camera::retrieveFrame(): Error while dequeuing device buffer [" << strerror(errno) << "]";
        return -1;
    }

    /*** timestamp the frame and copy the struct ***/
    struct timeval tv;
    gettimeofday(&tv, 0);
    mFrameBuffer[buffer.index].timestamp = (tv.tv_sec * 1000) + (tv.tv_usec / 1000);

    *frame = mFrameBuffer[buffer.index];

    /*** put device buffer back into the queue ***/
    if (-1 == xioctl(mFileDescriptor, VIDIOC_QBUF, &buffer))
    {
        qDebug() << "Camera::retrieveFrame(): Error while queuing device buffer [" << strerror(errno) << "]";
        return -1;
    }

    return 0;
}

int Camera::xioctl(int fd, int request, void *arg)
{
    int ret;
    // repeat read/write access attempt if interrupted
    while (-1 == (ret = ioctl(fd, request, arg)) && EINTR == errno);
    return ret;
}


/*** clamp value outside the range of an unsigned byte ***/
inline unsigned char Camera::clampValue(int value)
{
    if (255 < value) { return 255; }
    else if (0 > value) { return 0; }
    else { return value; }
}

/*** convert Y'CbCr 4:2:2 to BGR888 ***/
inline void Camera::convertYCbCr422ToBgr888(unsigned char *src, unsigned char *dst, int width, int height)
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

            // BGR pixel 1
            *dst++ = clampValue(y0 + ((cb * 113) / 64));
            *dst++ = clampValue(y0 - ((cb * 11) / 32) - ((cr * 91) / 128));
            *dst++ = clampValue(y0 + ((cr * 359) / 256));

            // BGR pixel 2
            *dst++ = clampValue(y1 + ((cb * 113) / 64));
            *dst++ = clampValue(y1 - ((cb * 11) / 32) - ((cr * 91) / 128));
            *dst++ = clampValue(y1 + ((cr * 359) / 256));
        }
        w = width / 2;
    }
}

/*** convert Y'CbCr 4:2:2 to RGB888 ***/
inline void Camera::convertYCbCr422ToRgb888(unsigned char *src, unsigned char *dst, int width, int height)
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
            *dst++ = clampValue(y0 + ((cr * 359) / 256));
            *dst++ = clampValue(y0 - ((cb * 11) / 32) - ((cr * 91) / 128));
            *dst++ = clampValue(y0 + ((cb * 113) / 64));

            // RGB pixel 2
            *dst++ = clampValue(y1 + ((cr * 359) / 256));
            *dst++ = clampValue(y1 - ((cb * 11) / 32) - ((cr * 91) / 128));
            *dst++ = clampValue(y1 + ((cb * 113) / 64));
        }
        w = width / 2;
    }
}



void Camera::slotReadAndEmitCurrentFrame()
{
    Frame frame;
    retrieveFrame(&frame);

    // unused, we don't encode here, because it might not be used in that same format here
    /*
    convertYCbCr422ToRgb888((unsigned char *) frame.start, (unsigned char*)mImageData->data(), mImageSize.width(), mImageSize.height());
    QImage image((const uchar*)mImageData->constData(), mImageSize.width(), mImageSize.height(), QImage::Format_RGB888);
    QByteArray imageDataEncoded;
    QBuffer buffer(&imageDataEncoded);
    buffer.open(QIODevice::WriteOnly);
    image.save(&buffer, "JPG", 40);
    qDebug() << QDateTime::currentDateTime().toString("hh:mm:ss:zzz") << "Camera::slotReadAndEmitCurrentFrame(): sending image after compressing" << frame.length << "YCbCr bytes down to" << imageDataEncoded.size() << "jpg bytes";
    emit imageReadyJpg(mDeviceFile, mImageSize, mPose, &imageDataEncoded);
    */

    emit imageReadyYCbCr(mDeviceFile, mImageSize, mPose, QByteArray::fromRawData((const char*)frame.start, frame.length));


}
