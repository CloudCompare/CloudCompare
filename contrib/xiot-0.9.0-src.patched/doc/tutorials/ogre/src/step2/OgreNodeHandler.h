#ifndef X3D_X3DLogNodeHandler_H
#define X3D_X3DLogNodeHandler_H

#include "X3DDefaultNodeHandler.h"
#include "OgreMaterial.h"
#include "OgreMesh.h"
#include <stack>

namespace Ogre {
	class SceneManager;
	class SceneNode;
};

class IndexedFaceSet;

/**
 * Handler that logs all library events to a file.
 *
 * This is yet another class that derives from the X3DDefaultNodeHandler. 
 * It overrides each callback function to create a log file,
 * which reports every start and end of a XML node/event. 
 * Plus, it will count the number of  events - thus nodes - 
 * occuring throughout the file import.
 * @see X3DDefaultNodeHandler
 */
class OgreNodeHandler : public XIOT::X3DDefaultNodeHandler {

public:

  OgreNodeHandler();
  ~OgreNodeHandler();

  /// Called by all start callback functions
  virtual int startUnhandled(const char* nodeName, const XIOT::X3DAttributes &attr);

  /// Called by all end callback functions
  virtual int endUnhandled(const char* nodeName);

};

#endif

