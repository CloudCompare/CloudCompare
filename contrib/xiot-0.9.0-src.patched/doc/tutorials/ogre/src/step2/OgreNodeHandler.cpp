#include "OgreNodeHandler.h"
#include "X3DAttributes.h"
#include <cassert>

using namespace Ogre;
using namespace std;

OgreNodeHandler::OgreNodeHandler()
{
}

OgreNodeHandler::~OgreNodeHandler()
{
}

/// Called by all start callback functions
int OgreNodeHandler::startUnhandled(const char* nodeName, const XIOT::X3DAttributes &attr)
{
	cout << "Unhandled: <" << nodeName << ">" << endl;
	return 1;
}

/// Called by all end callback functions
int OgreNodeHandler::endUnhandled(const char* nodeName)
{
	cout << "Unhandled: </" << nodeName << ">" << endl;
	return 1;
}