/*
-----------------------------------------------------------------------------
This source file is part of OGRE
    (Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (c) 2000-2006 Torus Knot Software Ltd
Also see acknowledgements in Readme.html

You may use this sample code for anything you like, it is not covered by the
LGPL like the rest of the engine.
-----------------------------------------------------------------------------
*/
/*
-----------------------------------------------------------------------------
Filename:    ExampleFrameListener.h
Description: Defines an example frame listener which responds to frame events.
This frame listener just moves a specified camera around based on
keyboard and mouse movements.
Mouse:    Freelook
W or Up:  Forward
S or Down:Backward
A:        Step left
D:        Step right
             PgUp:     Move upwards
             PgDown:   Move downwards
             F:        Toggle frame rate stats on/off
			 R:        Render mode
             T:        Cycle texture filtering
                       Bilinear, Trilinear, Anisotropic(8)
             P:        Toggle on/off display of camera position / orientation
-----------------------------------------------------------------------------
*/

#ifndef __ExampleFrameListener_H__
#define __ExampleFrameListener_H__

#include "Ogre.h"
#include "OgreStringConverter.h"
#include "OgreException.h"

//Use this define to signify OIS will be used as a DLL
//(so that dll import/export macros are in effect)
#define OIS_DYNAMIC_LIB
#include <OIS/OIS.h>

using namespace Ogre;

class ExampleFrameListener: public FrameListener, public WindowEventListener
{
protected:
	virtual void updateStats(void)
	{
		static String currFps = "Current FPS: ";
		static String avgFps = "Average FPS: ";
		static String bestFps = "Best FPS: ";
		static String worstFps = "Worst FPS: ";
		static String tris = "Triangle Count: ";
		static String batches = "Batch Count: ";

		// update stats when necessary
		try {
			OverlayElement* guiAvg = OverlayManager::getSingleton().getOverlayElement("Core/AverageFps");
			OverlayElement* guiCurr = OverlayManager::getSingleton().getOverlayElement("Core/CurrFps");
			OverlayElement* guiBest = OverlayManager::getSingleton().getOverlayElement("Core/BestFps");
			OverlayElement* guiWorst = OverlayManager::getSingleton().getOverlayElement("Core/WorstFps");

			const RenderTarget::FrameStats& stats = mWindow->getStatistics();
			guiAvg->setCaption(avgFps + StringConverter::toString(stats.avgFPS));
			guiCurr->setCaption(currFps + StringConverter::toString(stats.lastFPS));
			guiBest->setCaption(bestFps + StringConverter::toString(stats.bestFPS)
				+" "+StringConverter::toString(stats.bestFrameTime)+" ms");
			guiWorst->setCaption(worstFps + StringConverter::toString(stats.worstFPS)
				+" "+StringConverter::toString(stats.worstFrameTime)+" ms");

			OverlayElement* guiTris = OverlayManager::getSingleton().getOverlayElement("Core/NumTris");
			guiTris->setCaption(tris + StringConverter::toString(stats.triangleCount));

			OverlayElement* guiBatches = OverlayManager::getSingleton().getOverlayElement("Core/NumBatches");
			guiBatches->setCaption(batches + StringConverter::toString(stats.batchCount));

			OverlayElement* guiDbg = OverlayManager::getSingleton().getOverlayElement("Core/DebugText");
			guiDbg->setCaption(mDebugText);
		}
		catch(...) { /* ignore */ }
	}

public:
	// Constructor takes a RenderWindow because it uses that to determine input context
	ExampleFrameListener(RenderWindow* win, Camera* cam, bool bufferedKeys = false, bool bufferedMouse = false,
			     bool bufferedJoy = false ) :
		mCamera(cam), mTranslateVector(Vector3::ZERO), mCurrentSpeed(0), mWindow(win), mStatsOn(true), mNumScreenShots(0),
		mMoveScale(0.0f), mRotScale(0.0f), mTimeUntilNextToggle(0), mFiltering(TFO_BILINEAR),
		mAniso(1), mSceneDetailIndex(0), mMoveSpeed(100), mRotateSpeed(36), mDebugOverlay(0),
		mInputManager(0), mMouse(0), mKeyboard(0), mJoy(0)
	{

		mDebugOverlay = OverlayManager::getSingleton().getByName("Core/DebugOverlay");

		LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
		OIS::ParamList pl;
		size_t windowHnd = 0;
		std::ostringstream windowHndStr;

		win->getCustomAttribute("WINDOW", &windowHnd);
		windowHndStr << windowHnd;
		pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

		mInputManager = OIS::InputManager::createInputSystem( pl );

		//Create all devices (We only catch joystick exceptions here, as, most people have Key/Mouse)
		mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject( OIS::OISKeyboard, bufferedKeys ));
		mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject( OIS::OISMouse, bufferedMouse ));
		try {
			mJoy = static_cast<OIS::JoyStick*>(mInputManager->createInputObject( OIS::OISJoyStick, bufferedJoy ));
		}
		catch(...) {
			mJoy = 0;
		}

		//Set initial mouse clipping size
		windowResized(mWindow);

		showDebugOverlay(true);

		//Register as a Window listener
		WindowEventUtilities::addWindowEventListener(mWindow, this);
	}

	//Adjust mouse clipping area
	virtual void windowResized(RenderWindow* rw)
	{
		unsigned int width, height, depth;
		int left, top;
		rw->getMetrics(width, height, depth, left, top);

		const OIS::MouseState &ms = mMouse->getMouseState();
		ms.width = width;
		ms.height = height;
	}

	//Unattach OIS before window shutdown (very important under Linux)
	virtual void windowClosed(RenderWindow* rw)
	{
		//Only close for window that created OIS (the main window in these demos)
		if( rw == mWindow )
		{
			if( mInputManager )
			{
				mInputManager->destroyInputObject( mMouse );
				mInputManager->destroyInputObject( mKeyboard );
				mInputManager->destroyInputObject( mJoy );

				OIS::InputManager::destroyInputSystem(mInputManager);
				mInputManager = 0;
			}
		}
	}

	virtual ~ExampleFrameListener()
	{
		//Remove ourself as a Window listener
		WindowEventUtilities::removeWindowEventListener(mWindow, this);
		windowClosed(mWindow);
	}

	virtual bool processUnbufferedKeyInput(const FrameEvent& evt)
	{

		if(mKeyboard->isKeyDown(OIS::KC_A))
			mTranslateVector.x = -mMoveScale;	// Move camera left

		if(mKeyboard->isKeyDown(OIS::KC_D))
			mTranslateVector.x = mMoveScale;	// Move camera RIGHT

		if(mKeyboard->isKeyDown(OIS::KC_UP) || mKeyboard->isKeyDown(OIS::KC_W) )
			mTranslateVector.z = -mMoveScale;	// Move camera forward

		if(mKeyboard->isKeyDown(OIS::KC_DOWN) || mKeyboard->isKeyDown(OIS::KC_S) )
			mTranslateVector.z = mMoveScale;	// Move camera backward

		if(mKeyboard->isKeyDown(OIS::KC_PGUP))
			mTranslateVector.y = mMoveScale;	// Move camera up

		if(mKeyboard->isKeyDown(OIS::KC_PGDOWN))
			mTranslateVector.y = -mMoveScale;	// Move camera down

		if(mKeyboard->isKeyDown(OIS::KC_RIGHT))
			mCamera->yaw(-mRotScale);

		if(mKeyboard->isKeyDown(OIS::KC_LEFT))
			mCamera->yaw(mRotScale);

		if( mKeyboard->isKeyDown(OIS::KC_ESCAPE) || mKeyboard->isKeyDown(OIS::KC_Q) )
			return false;

       	if( mKeyboard->isKeyDown(OIS::KC_F) && mTimeUntilNextToggle <= 0 )
		{
			mStatsOn = !mStatsOn;
			showDebugOverlay(mStatsOn);
			mTimeUntilNextToggle = 1;
		}

		if( mKeyboard->isKeyDown(OIS::KC_T) && mTimeUntilNextToggle <= 0 )
		{
			switch(mFiltering)
			{
			case TFO_BILINEAR:
				mFiltering = TFO_TRILINEAR;
				mAniso = 1;
				break;
			case TFO_TRILINEAR:
				mFiltering = TFO_ANISOTROPIC;
				mAniso = 8;
				break;
			case TFO_ANISOTROPIC:
				mFiltering = TFO_BILINEAR;
				mAniso = 1;
				break;
			default: break;
			}
			MaterialManager::getSingleton().setDefaultTextureFiltering(mFiltering);
			MaterialManager::getSingleton().setDefaultAnisotropy(mAniso);

			showDebugOverlay(mStatsOn);
			mTimeUntilNextToggle = 1;
		}

		if(mKeyboard->isKeyDown(OIS::KC_SYSRQ) && mTimeUntilNextToggle <= 0)
		{
			std::ostringstream ss;
			ss << "screenshot_" << ++mNumScreenShots << ".png";
			mWindow->writeContentsToFile(ss.str());
			mTimeUntilNextToggle = 0.5;
			mDebugText = "Saved: " + ss.str();
		}

		if(mKeyboard->isKeyDown(OIS::KC_R) && mTimeUntilNextToggle <=0)
		{
			mSceneDetailIndex = (mSceneDetailIndex+1)%3 ;
			switch(mSceneDetailIndex) {
				case 0 : mCamera->setPolygonMode(PM_SOLID); break;
				case 1 : mCamera->setPolygonMode(PM_WIREFRAME); break;
				case 2 : mCamera->setPolygonMode(PM_POINTS); break;
			}
			mTimeUntilNextToggle = 0.5;
		}

		static bool displayCameraDetails = false;
		if(mKeyboard->isKeyDown(OIS::KC_P) && mTimeUntilNextToggle <= 0)
		{
			displayCameraDetails = !displayCameraDetails;
			mTimeUntilNextToggle = 0.5;
			if (!displayCameraDetails)
				mDebugText = "";
		}

		// Print camera details
		if(displayCameraDetails)
			mDebugText = "P: " + StringConverter::toString(mCamera->getDerivedPosition()) +
						 " " + "O: " + StringConverter::toString(mCamera->getDerivedOrientation());

		// Return true to continue rendering
		return true;
	}

	virtual bool processUnbufferedMouseInput(const FrameEvent& evt)
	{

		// Rotation factors, may not be used if the second mouse button is pressed
		// 2nd mouse button - slide, otherwise rotate
		const OIS::MouseState &ms = mMouse->getMouseState();
		if( ms.buttonDown( OIS::MB_Right ) )
		{
			mTranslateVector.x += ms.X.rel * 0.13;
			mTranslateVector.y -= ms.Y.rel * 0.13;
		}
		else
		{
			mRotX = Degree(-ms.X.rel * 0.13);
			mRotY = Degree(-ms.Y.rel * 0.13);
		}

		return true;
	}

	virtual void moveCamera()
	{
		// Make all the changes to the camera
		// Note that YAW direction is around a fixed axis (freelook style) rather than a natural YAW
		//(e.g. airplane)
		mCamera->yaw(mRotX);
		mCamera->pitch(mRotY);
		mCamera->moveRelative(mTranslateVector);
	}

	virtual void showDebugOverlay(bool show)
	{
		if (mDebugOverlay)
		{
			if (show)
				mDebugOverlay->show();
			else
				mDebugOverlay->hide();
		}
	}

	// Override frameRenderingQueued event to process that (don't care about frameEnded)
	bool frameRenderingQueued(const FrameEvent& evt)
	{

		if(mWindow->isClosed())	return false;

		mSpeedLimit = mMoveScale * evt.timeSinceLastFrame;

		//Need to capture/update each device
		mKeyboard->capture();
		mMouse->capture();
		if( mJoy ) mJoy->capture();

		bool buffJ = (mJoy) ? mJoy->buffered() : true;

    	Ogre::Vector3 lastMotion = mTranslateVector;

		//Check if one of the devices is not buffered
		if( !mMouse->buffered() || !mKeyboard->buffered() || !buffJ )
		{
			// one of the input modes is immediate, so setup what is needed for immediate movement
			if (mTimeUntilNextToggle >= 0)
				mTimeUntilNextToggle -= evt.timeSinceLastFrame;

			// Move about 100 units per second
			mMoveScale = mMoveSpeed * evt.timeSinceLastFrame;
			// Take about 10 seconds for full rotation
			mRotScale = mRotateSpeed * evt.timeSinceLastFrame;

			mRotX = 0;
			mRotY = 0;
			mTranslateVector = Ogre::Vector3::ZERO;

		}

		//Check to see which device is not buffered, and handle it
		if( !mKeyboard->buffered() )
			if( processUnbufferedKeyInput(evt) == false )
				return false;
		if( !mMouse->buffered() )
			if( processUnbufferedMouseInput(evt) == false )
				return false;

		// ramp up / ramp down speed
    	if (mTranslateVector == Ogre::Vector3::ZERO)
		{
			// decay (one third speed)
			mCurrentSpeed -= evt.timeSinceLastFrame * 0.3;
			mTranslateVector = lastMotion;
		}
		else
		{
			// ramp up
			mCurrentSpeed += evt.timeSinceLastFrame;

		}
		// Limit motion speed
		if (mCurrentSpeed > 1.0)
			mCurrentSpeed = 1.0;
		if (mCurrentSpeed < 0.0)
			mCurrentSpeed = 0.0;

		mTranslateVector *= mCurrentSpeed;


		if( !mMouse->buffered() || !mKeyboard->buffered() || !buffJ )
			moveCamera();

		return true;
	}

	bool frameEnded(const FrameEvent& evt)
	{
		updateStats();
		return true;
	}

protected:
	Camera* mCamera;

	Vector3 mTranslateVector;
	Real mCurrentSpeed;
	RenderWindow* mWindow;
	bool mStatsOn;

	std::string mDebugText;

	unsigned int mNumScreenShots;
	float mMoveScale;
	float mSpeedLimit;
	Degree mRotScale;
	// just to stop toggles flipping too fast
	Real mTimeUntilNextToggle ;
	Radian mRotX, mRotY;
	TextureFilterOptions mFiltering;
	int mAniso;

	int mSceneDetailIndex ;
	Real mMoveSpeed;
	Degree mRotateSpeed;
	Overlay* mDebugOverlay;

	//OIS Input devices
	OIS::InputManager* mInputManager;
	OIS::Mouse*    mMouse;
	OIS::Keyboard* mKeyboard;
	OIS::JoyStick* mJoy;
};

#endif
