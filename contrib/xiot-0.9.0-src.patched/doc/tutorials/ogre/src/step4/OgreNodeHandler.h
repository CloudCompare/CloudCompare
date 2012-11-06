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

class IndexedGeometry;

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

  void setSceneManager(Ogre::SceneManager* sceneMgr);

  void startDocument();
  void endDocument();

  int startShape(const XIOT::X3DAttributes &attr);
  int endShape();

  int startSphere(const XIOT::X3DAttributes &attr);

  int startBox(const XIOT::X3DAttributes &attr);

  int startAppearance(const XIOT::X3DAttributes &attr);
  int endAppearance();

  int startMaterial(const XIOT::X3DAttributes &attr);

  int startColor(const XIOT::X3DAttributes &attr);

  int startCoordinate(const XIOT::X3DAttributes &attr);
  int startNormal(const XIOT::X3DAttributes &attr);

  int startTransform(const XIOT::X3DAttributes &attr);
  int endTransform();

  int startIndexedFaceSet(const XIOT::X3DAttributes &attr);
  int endIndexedFaceSet();

  int startIndexedLineSet(const XIOT::X3DAttributes &attr);
  int endIndexedLineSet();

  int startDirectionalLight(const XIOT::X3DAttributes &attr);

  int startUnknown(const char* nodeName, const XIOT::X3DAttributes &attr);

  protected:
	  std::string createUniqueName(const XIOT::X3DAttributes &attr, const std::string& prefix="");

  private:
	  Ogre::SceneManager* _sceneManager;
	  std::stack<Ogre::SceneNode*> _nodeStack;
	  Ogre::MaterialPtr _currentMaterial;
	  Ogre::Entity* _currentEntity;
	  Ogre::ManualObject* _currentManualObject;
	  Ogre::MeshPtr _currentMesh;
	  IndexedGeometry* _currentGeometry;
	  std::string _shapeName;
	  int _objCount;
};

#endif

