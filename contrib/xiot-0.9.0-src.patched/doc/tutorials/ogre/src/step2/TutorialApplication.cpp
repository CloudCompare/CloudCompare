#include "TutorialApplication.h"
#include "X3DLoader.h"
#include "OgreNodeHandler.h"


void TutorialApplication::setX3DFile(const char* filename)
{
	_filename = filename;
}

void TutorialApplication::createScene()
{
	// New instance of X3DLoader
	XIOT::X3DLoader loader;
	// Create and set NodeHandler
	OgreNodeHandler handler;
	loader.setNodeHandler(&handler);
	// Start processing
	loader.load(_filename.c_str());
}

