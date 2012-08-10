#include "visualodometry.h"

VisualOdometry::VisualOdometry(Camera* camera) : QObject()
{
    mCamera = camera;

    mLastAverageHorizontalSpeed = 0;

    mFrameSize.height = mCamera->getImageSize().height();
    mFrameSize.width = mCamera->getImageSize().width();

    mNumberOfFeatures = 200;

    // Allocate memory for the two images.
    mImage1 = cvCreateImage(mFrameSize, IPL_DEPTH_8U, 1);
    mImage2 = cvCreateImage(mFrameSize, IPL_DEPTH_8U, 1);
    mImageDebug = cvCreateImage(mFrameSize, IPL_DEPTH_8U, 1);

    // This is some workspace for the algorithm, it actually carves the image into pyramids of different resolutions
    mImagePyramid1 = cvCreateImage(mFrameSize, IPL_DEPTH_8U, 1);
    mImagePyramid2 = cvCreateImage(mFrameSize, IPL_DEPTH_8U, 1);

    // Also temp storage for the algorithm
    mImageEigen = cvCreateImage(mFrameSize, IPL_DEPTH_32F, 1);
    mImageTemp = cvCreateImage(mFrameSize, IPL_DEPTH_32F, 1);

    mLogFile = new QFile(QString("visualodometry-%1-%2.log").arg(QString::number(QCoreApplication::applicationPid())).arg(QDateTime::currentDateTime().toString("yyyyMMdd-hhmmsszzz")));
    if(!mLogFile->open(QIODevice::WriteOnly | QIODevice::Text))
        qFatal("VisualOdometry::VisualOdometry(): Couldn't open logfile %s for writing, exiting.", qPrintable(mLogFile->fileName()));

    mOpticalFlowWindow = cvSize(3,3);

    mOpticalFlowTerminationCriteria = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3);

    mNumberOfFramesProcessed = 0;
}

VisualOdometry::~VisualOdometry()
{
    mLogFile->close();
}

void VisualOdometry::slotProcessImage(const QString& name, const QSize& imageSize, const QVector3D& position, const QQuaternion& orientation, const QByteArray imageData)
{
    mNumberOfFramesProcessed++;

    // first copy the current image into the previous image
    memcpy(mImage1->imageData, mImage2->imageData, mFrameSize.width * mFrameSize.height);

    // convert the YCbCr from the camera into the current gray image
    Camera::convertYCbCr422ToGray8((u_char*)imageData.constData(), (u_char*)mImage2->imageData, imageSize.width(), imageSize.height());

    // black out the top quater of the image because of propeller noise
    for(int i=0;i<mFrameSize.width*(mFrameSize.height/4);i++) mImage2->imageData[i]=0;

    // On the first execution, mImage1 is still undefined.
    if(mNumberOfFramesProcessed == 1) return;

    // reset debug image to black
    cvSet(mImageDebug, cvScalar(0));

    char filename[50];
    sprintf(filename, "image1_%d.jpg", mNumberOfFramesProcessed);
//    cvSaveImage(filename, mImage1);

    sprintf(filename, "image2_%d.jpg", mNumberOfFramesProcessed);
//    cvSaveImage(filename, mImage2);

//    qDebug() << "numberOfFeatures before is" << mNumberOfFeatures;
                cvGoodFeaturesToTrack(mImage1, mImageEigen, mImageTemp, mFeaturesFrame1, &mNumberOfFeatures, .01, .01, NULL);
//                qDebug() << "numberOfFeatures after  is" << mNumberOfFeatures;


                /* Actually run Pyramidal Lucas Kanade Optical Flow!!
                 * "frame1_1C" is the first frame with the known features.
                 * "frame2_1C" is the second frame where we want to find the first frame's features.
                 * "pyramid1" and "pyramid2" are workspace for the algorithm.
                 * "frame1_features" are the features from the first frame.
                 * "frame2_features" is the (outputted) locations of those features in the second frame.
                 * "number_of_features" is the number of features in the frame1_features array.
                 * "optical_flow_window" is the size of the window to use to avoid the aperture problem.
                 * "5" is the maximum number of pyramids to use.  0 would be just one level.
                 * "optical_flow_found_feature" is as described above (non-zero iff feature found by the flow).
                 * "optical_flow_feature_error" is as described above (error in the flow for this feature).
                 * "mOpticalFlowTerminationCriteria" is as described above (how long the algorithm should look).
                 * "0" means disable enhancements.  (For example, the second array isn't pre-initialized with guesses.)
                 */
                cvCalcOpticalFlowPyrLK(
                            mImage1,
                            mImage2,
                            mImagePyramid1,
                            mImagePyramid2,
                            mFeaturesFrame1,
                            mFeaturesFrame2,
                            mNumberOfFeatures,
                            mOpticalFlowWindow,
                            5,
                            mOpticalFlowFoundFeature,
                            mOpticalFlowFeatureError,
                            mOpticalFlowTerminationCriteria,
                            0);

                qint64 averageDirectionX = 0;
                qint64 averageDirectionY = 0;

                /* For fun (and debugging :)), let's draw the flow field. */
                for(int i = 0; i < mNumberOfFeatures; i++)
                {
                        /* If Pyramidal Lucas Kanade didn't really find the feature, skip it. */
                        if(mOpticalFlowFoundFeature[i] == 0) continue;

                        CvPoint p1,p2;

                        p1.x = (int) mFeaturesFrame1[i].x;
                        p1.y = (int) mFeaturesFrame1[i].y;
                        p2.x = (int) mFeaturesFrame2[i].x;
                        p2.y = (int) mFeaturesFrame2[i].y;

                        averageDirectionX += p1.x - p2.x;
                        averageDirectionY += p1.y - p2.y;

//                        const float angle = atan2( (double) p1.y - p2.y, (double) p1.x - p2.x);
//                        const float hypotenuse = sqrt(SQUARE(p1.y - p2.y) + SQUARE(p1.x - p2.x));
                        /* Here we lengthen the arrow by a factor of three. */
//                        p2.x = (int) (p1.x - 3 * hypotenuse * cos(angle));
//                        p2.y = (int) (p1.y - 3 * hypotenuse * sin(angle));

                        /* Now we draw the main line of the arrow. */
                        /* "mImage1" is the frame to draw on.
                         * "p" is the point where the line begins.
                         * "q" is the point where the line stops.
                         * "CV_AA" means antialiased drawing.
                         * "0" means no fractional bits in the center cooridinate or radius.
                         */
                        cvLine( mImage1, p1, p2, CV_RGB(255,255,255), 1, CV_AA, 0);

                        /*
                        // Now draw the tips of the arrow.  I do some scaling so that the tips look proportional to the main line of the arrow.
                        p.x = (int) (q.x + 9 * cos(angle + pi / 4));
                        p.y = (int) (q.y + 9 * sin(angle + pi / 4));
                        cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
                        p.x = (int) (q.x + 9 * cos(angle - pi / 4));
                        p.y = (int) (q.y + 9 * sin(angle - pi / 4));
                        cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
                        */
                }

                CvPoint p1, p2;
                p1.x = mFrameSize.width/2;
                p1.y = mFrameSize.height/2;

                p2.x = p1.x + (averageDirectionX/10) + mLastAverageHorizontalSpeed;
                p2.y = p1.y;// + (averageDirectionY/5);

                mLastAverageHorizontalSpeed = averageDirectionX/12;

                QTextStream out(mLogFile);
                out << "frame " << mNumberOfFramesProcessed << " " << averageDirectionX << " " << averageDirectionY << '\n';

                cvLine( mImage1, p1, p2, CV_RGB(255,255,255), 1, CV_AA, 0 );
                sprintf(filename, "frame_%d.jpg", mNumberOfFramesProcessed);
//                cvSaveImage(filename, mImage1);
                cv::imwrite(std::string(filename), mImage1);


                qDebug() << "AVG DIRECTION" << averageDirectionX << averageDirectionY;
}
