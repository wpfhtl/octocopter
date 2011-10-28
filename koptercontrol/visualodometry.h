#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include <QObject>
#include "camera.h"
#include "common.h"

#include <opencv2/opencv.hpp>
#include <math.h>

//using namespace cv;

class VisualOdometry : public QObject
{
    Q_OBJECT
private:
    Camera* mCamera;
    IplImage *mImage1, *mImage2, *mImageDebug, *mImageEigen, *mImageTemp, *mImagePyramid1, *mImagePyramid2;
    CvSize mFrameSize;
    QFile* mLogFile;

    // This is the window size to use to avoid the aperture problem (see slide "Optical Flow: Overview").
    CvSize mOpticalFlowWindow;

    // This array will contain the locations of the points in frame 1
    CvPoint2D32f mFeaturesFrame1[400];

    // This array will contain the locations of the points from frame 1 in frame 2.
    CvPoint2D32f mFeaturesFrame2[400];

    int mNumberOfFeatures;

    int mLastAverageHorizontalSpeed;

    /* This termination criteria tells the algorithm when to stop (either x iterations or when
     * epsilon is better than y.  You can play with these parameters for speed vs. accuracy but these values
     * work pretty well in many situations. */
    CvTermCriteria mOpticalFlowTerminationCriteria;

    // The i-th element of this array will be non-zero if and only if the i-th feature of frame 1 was found in frame 2.
    char mOpticalFlowFoundFeature[400];

    /* The i-th element of this array is the error in the optical flow for the i-th feature
     * of frame1 as found in frame 2.  If the i-th feature was not found (see the array above)
     * I think the i-th entry in this array is undefined.
     */
    float mOpticalFlowFeatureError[400];

    int mNumberOfFramesProcessed;

//    inline static void allocateOnDemand( IplImage **img, CvSize size, int depth, int channels );

public:
    VisualOdometry(Camera* camera);
    ~VisualOdometry();

signals:
    void yaw(float);
    void pitch(float);
    void roll(float);

private slots:
    void slotProcessImage(const QString& name, const QSize& imageSize, const QVector3D& position, const QQuaternion& orientation, const QByteArray imageData);

};

#endif // VISUALODOMETRY_H
