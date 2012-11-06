#ifndef __IndexedGeometry_h
#define __IndexedGeometry_h

#include "X3DTypes.h"
#include <string>

namespace Ogre {
	class ManualObject;
}

class IndexedGeometry 
{

public:
 IndexedGeometry(const std::string &name);
 ~IndexedGeometry();

 // Flags
 void setNormalPerVertex(bool normalPerVertex);
 void setColorPerVertex(bool colorPerVertex);

 // Indices
 void setCoordIndex(const std::vector<int> &coordIndex);
 void setNormalIndex(const std::vector<int> &normalIndex);
 void setColorIndex(const std::vector<int> &colorIndex);
 void setTexCoordIndex(const std::vector<int> &texCoordIndex);

  // Data
 void setCoords(const std::vector<XIOT::SFVec3f> &coords);
 void setNormals(const std::vector<XIOT::SFVec3f> &normals);
 void setTexCoords(const std::vector<XIOT::SFVec2f> &texCoords);
 void setColors(const std::vector<XIOT::SFColor> &xolors);
  
 void createIndexedFaceSet();
 void createIndexedLineSet(Ogre::ManualObject*);

 inline std::string getName() { return _name; };
 
 protected:
	 std::string _name;

  bool _normalPerVertex;
  bool _colorPerVertex;
  bool _ccw;

  std::vector<int> _coordIndex;
  std::vector<int> _normalIndex;
  std::vector<int> _colorIndex;
  std::vector<int> _texCoordIndex;

  std::vector<XIOT::SFVec3f> _coords;
  std::vector<XIOT::SFVec3f> _normals;
  std::vector<XIOT::SFVec2f> _texCoords;
  std::vector<XIOT::SFColor> _colors;


private:

};


#endif
