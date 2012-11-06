#include "TutorialApplication.h"
#include "X3DLoader.h"
#include "OgreNodeHandler.h"

class TutorialFrameListener : public ExampleFrameListener
{
public:
	TutorialFrameListener(RenderWindow* win, Camera* cam, SceneNode* sceneRoot) : ExampleFrameListener(win, cam), _sceneRoot(sceneRoot) {};
	virtual bool processUnbufferedKeyInput(const FrameEvent& evt) {
		if(mKeyboard->isKeyDown(OIS::KC_V))
		{
			_sceneRoot->_update(true, true);
			_sceneRoot->_updateBounds();
			std::cout << "Frame camera" << std::endl;
			AxisAlignedBox box = _sceneRoot->_getWorldAABB();
			std::cout << "Center: " << box.getCenter() << std::endl;
		}
		return ExampleFrameListener::processUnbufferedKeyInput(evt);
	}

	SceneNode* _sceneRoot;
};


void TutorialApplication::setX3DFile(const std::string& filename)
{
	_filename = filename;
}

void TutorialApplication::createScene()
{

	XIOT::X3DLoader loader;
	OgreNodeHandler* handler = new OgreNodeHandler();
	handler->setSceneManager(this->mSceneMgr);
	loader.setNodeHandler(handler);

	loader.load(_filename);
	
}


void TutorialApplication::createFrameListener(void)
{
        mFrameListener= new TutorialFrameListener(mWindow, mCamera, mSceneMgr->getRootSceneNode());
        mFrameListener->showDebugOverlay(true);
        mRoot->addFrameListener(mFrameListener);
}

void TutorialApplication::createCamera(void)
 {
	 // Create the camera
	 ExampleApplication::createCamera();

	SceneNode* camNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("CameraNode");
    camNode->attachObject(mCamera);

	// Create headlight
    Light* light = mSceneMgr->createLight("headlight");
    light->setType(Ogre::Light::LT_DIRECTIONAL);
    light->setDiffuseColour(1,1,1);
    Ogre::SceneNode* lightNode = camNode->createChildSceneNode("HeadlightNode");
    lightNode->rotate( Ogre::Vector3::UNIT_Y, Ogre::Degree(180), Ogre::Node::TS_LOCAL );
    lightNode->attachObject(light);
}