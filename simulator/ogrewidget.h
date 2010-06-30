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

#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>
#include "Terrain/OgreTerrainQuadTreeNode.h"
#include "Terrain/OgreTerrainMaterialGeneratorA.h"
#include <OgrePageManager.h>
#include <RTShaderSystem/OgreShaderGenerator.h>

#define TERRAIN_PAGE_MIN_X 0
#define TERRAIN_PAGE_MIN_Y 0
#define TERRAIN_PAGE_MAX_X 0
#define TERRAIN_PAGE_MAX_Y 0
#define TERRAIN_WORLD_SIZE 12000.0f
#define TERRAIN_SIZE 513

class Simulator;

//using namespace Ogre;

/** This class simply demonstrates basic usage of the CRTShader system.
It sub class the material manager listener class and when a target scheme callback
is invoked with the shader generator scheme it tries to create an equvialent shader
based technique based on the default technique of the given material.
*/
class ShaderGeneratorTechniqueResolverListener : public Ogre::MaterialManager::Listener
{
public:
    ShaderGeneratorTechniqueResolverListener(Ogre::RTShader::ShaderGenerator* pShaderGenerator)
    {
        mShaderGenerator = pShaderGenerator;
    }

    virtual Ogre::Technique* handleSchemeNotFound(
            unsigned short schemeIndex,
            const Ogre::String& schemeName,
            Ogre::Material* originalMaterial,
            unsigned short lodIndex,
            const Ogre::Renderable* rend)
    {
        // Case this is the default shader generator scheme.
        if (schemeName == Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME)
        {
            MaterialRegisterIterator itFind = mRegisteredMaterials.find(originalMaterial);
            bool techniqueCreated = false;

            // This material was not registered before.
            if (itFind == mRegisteredMaterials.end())
            {
                techniqueCreated = mShaderGenerator->createShaderBasedTechnique(
                        originalMaterial->getName(),
                        Ogre::MaterialManager::DEFAULT_SCHEME_NAME,
                        schemeName);
            }
            mRegisteredMaterials[originalMaterial] = techniqueCreated;
        }

        return NULL;
    }

protected:
    typedef std::map<Ogre::Material*, bool> MaterialRegisterMap;
    typedef MaterialRegisterMap::iterator MaterialRegisterIterator;


protected:
    // Registered material map.
    MaterialRegisterMap mRegisteredMaterials;

    // The shader generator instance.
    Ogre::RTShader::ShaderGenerator* mShaderGenerator;
};



class OgreWidget : public QWidget
{
    Q_OBJECT

public:
    enum TranslationMode {TRANSLATION_RELATIVE, TRANSLATION_ABSOLUTE};

    OgreWidget(Simulator *simulator);
    ~OgreWidget();

    // Override QWidget::paintEngine to return NULL
    QPaintEngine* paintEngine() const; // Turn off QTs paint engine for the Ogre widget.

    Ogre::RaySceneQuery* createRaySceneQuery(void);

    Ogre::SceneNode* createVehicleNode(const Ogre::String name, const Ogre::Vector3 position, const Ogre::Quaternion orientation);

    // Creates a scannerNode and attaches a mesh to it.
    Ogre::SceneNode* createScanner(
            const QString name,
            const Ogre::Vector3 &relativePosition = Ogre::Vector3::ZERO,
            const Ogre::Quaternion &relativeRotation = Ogre::Quaternion::IDENTITY);

    void destroyScanner(const QString name);

    // Creates a camera for rendering to texture
    void createRttCamera(Ogre::Camera** camera, Ogre::RenderTarget** renderTarget, const QString name, const int width, const int height);
    void destroyCamera(Ogre::RenderTarget* renderTarget, Ogre::Camera* camera);

    // returns ogre-world-coordinate-vector.
    Ogre::Vector3 getVehiclePosition(void) const;

    Ogre::SceneManager* sceneManager();

    // Public access needed for the laserscanners rayIntersects()
    Ogre::TerrainGroup* mTerrainGroup;

    void createManualObject(
            const QString &name,
            Ogre::ManualObject** manualObject,
            Ogre::SceneNode** sceneNode,
            Ogre::MaterialPtr& material);

    void destroyManualObject(Ogre::ManualObject* manualObject, Ogre::SceneNode* sceneNode);

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
    // Terrain:
    Ogre::TerrainGlobalOptions* mTerrainGlobals;
    Ogre::RTShader::ShaderGenerator* mShaderGenerator;

    // Shader generator material manager listener.
    ShaderGeneratorTechniqueResolverListener* mMaterialMgrListener;

    void configureTerrainDefaults(Ogre::Light* l);
    void defineTerrain(long x, long y, bool flat = false);
    void getTerrainImage(bool flipX, bool flipY, Ogre::Image& img);
    void initBlendMaps(Ogre::Terrain* terrain);
    bool initializeRTShaderSystem(Ogre::SceneManager* sceneMgr);

    Ogre::Vector3 mTerrainPos;
    bool mTerrainsImported;
    // end Terrain

    void initOgreSystem();
    void loadResources();
    void createScene();
    void setupTerrain();

    Simulator* mSimulator;

    static const Ogre::Real turboModifier;
    static const QPoint invalidMousePoint;

    mutable QMutex mMutex;

    QMap<int, Ogre::Vector3> mKeyCoordinateMapping;
    QList<int> mKeysPressed;

    Ogre::Root          *ogreRoot;
    Ogre::SceneManager  *mSceneManager;
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
