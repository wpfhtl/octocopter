#ifndef OGREWIDGET_H
#define OGREWIDGET_H

#include <QtGui>
#include <QApplication>
#include <QWidget>
#include <QDebug>
#include <QColor>
#include <QTimer>
#include <QMutex>
#include <QMutexLocker>
#include <Ogre.h>
#include <QX11Info>
#include <QKeyEvent>

#include "simulator.h"
#include "laserscanner.h"

class Simulator;

class OgreWidget : public QWidget
{
    Q_OBJECT

public:
    enum TranslationMode {TRANSLATION_RELATIVE, TRANSLATION_ABSOLUTE};

    OgreWidget(Simulator *simulator);
    ~OgreWidget();

    // Override QWidget::paintEngine to return NULL
    QPaintEngine* paintEngine() const; // Turn off QTs paint engine for the Ogre widget.

//    Ogre::SceneNode* getVehicleNode(void);

    Ogre::RaySceneQuery* createRaySceneQuery(void);

    // Creates a scannerNode and attaches a mesh to it.
    Ogre::SceneNode* createScannerNode(const QString name, const Ogre::Vector3 &relativePosition = Ogre::Vector3::ZERO, const Ogre::Quaternion &relativeRotation = Ogre::Quaternion::IDENTITY);

    // returns ogre-world-coordinate-vector.
    Ogre::Vector3 getVehiclePosition(void) const;

    Ogre::SceneManager* sceneManager();

    void createManualObject(
            const QString &name,
            Ogre::ManualObject** manualObject,
            Ogre::SceneNode** sceneNode,
            Ogre::MaterialPtr& material);

public slots:
    void setBackgroundColor(QColor c);
    void setCameraPosition(
            const Ogre::Vector3 &vector,
            const TranslationMode &mode = OgreWidget::TRANSLATION_RELATIVE,
            const Ogre::Node::TransformSpace &transformSpace = Ogre::Node::TS_LOCAL,
            const Ogre::SceneNode* lookAt = 0);

    void setVehiclePosition(
            const Ogre::Vector3 &vector,
            const TranslationMode &mode = OgreWidget::TRANSLATION_RELATIVE,
            const Ogre::Node::TransformSpace &transformSpace = Ogre::Node::TS_LOCAL,
            const bool reAimCamera = false);

//    void slotSetLaserScannerRay(const QString &name, const Ogre::Vector3 &start, const Ogre::Vector3 &end);

signals:
    void setupFinished();
    void cameraPositionChanged(const Ogre::Vector3 &pos);
    void currentRenderStatistics(QSize windowSize, int triangles, float fps);

protected:
    virtual void keyPressEvent(QKeyEvent *e);
    virtual void keyReleaseEvent(QKeyEvent *e);
    virtual void moveEvent(QMoveEvent *e);
    virtual void mouseDoubleClickEvent(QMouseEvent *e);
    virtual void mouseMoveEvent(QMouseEvent *e);
    virtual void mousePressEvent(QMouseEvent *e);
    virtual void mouseReleaseEvent(QMouseEvent *e);
    virtual void paintEvent(QPaintEvent *e);
    virtual void resizeEvent(QResizeEvent *e);
    virtual void showEvent(QShowEvent *e);
    virtual void wheelEvent(QWheelEvent *e);
    virtual void timerEvent( QTimerEvent * );

private:
    void initOgreSystem();
    void loadResources();
    void createScene();

    Simulator* mSimulator;

    static const Ogre::Real turboModifier;
    static const QPoint invalidMousePoint;

    mutable QMutex mMutex;

    QMap<int, Ogre::Vector3> mKeyCoordinateMapping;
    QList<int> mKeysPressed;

    Ogre::Root          *ogreRoot;
    Ogre::SceneManager  *ogreSceneManager;
    Ogre::RenderWindow  *ogreRenderWindow;
    Ogre::Viewport      *ogreViewport;
    Ogre::Camera        *mCamera;

    unsigned int mFrameCount;// currently only used to emit statistics not every frame
    QPoint oldPosL, oldPosR;
    bool btnL, btnR;
    int mUpdateTimerId;

//    QHash<QString, QPair<const Ogre::Vector3&, const Ogre::Vector3&> > mRaysCurrent;
    QList<Ogre::SceneNode*> mScannerNodes;
//    QHash<QString, Ogre::ManualObject*> mRayManualObjects;
//    QHash<QString, Ogre::SceneNode*> mRayNodes;
//    QHash<QString, Ogre::MaterialPtr> mRayMaterials;

    Ogre::SceneNode *selectedNode;
    Ogre::SceneNode *mCameraNode;
    Ogre::Entity *mVehicleEntity;
    Ogre::SceneNode *mVehicleNode;
};

#endif
