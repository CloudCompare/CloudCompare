#ifndef __vtkX3DNodeHandler_h
#define __vtkX3DNodeHandler_h

#include <map>
#include <set>

#include <xiot/X3DDefaultNodeHandler.h>
#include <xiot/X3DAttributes.h>
#include "vtkRenderer.h"
#include "vtkX3DImporter.h"

class vtkActor;
class vtkTransform;
class vtkFloatArray;
class vtkPolyDataMapper;
class vtkPoints;
class vtkIdTypeArray;
class vtkCellArray;
class vtkX3DIndexedGeometrySource;
class vtkUnsignedCharArray;
class vtkX3DImporter;

/**
 * Handler that fills a vtkRenderer from X3D callbacks.
 *
 * The vtkX3DNodeHandler derives from the X3DDefaultNodeHandler. It overrides a selection of the callback functions
 * and is used to import a X3D file into a vtk-application. To construct the scene, the class provides an overloaded
 * constructor which is used to save a pointer to a vtkRenderer object.
 * @see X3DDefaultNodeHandler
 * @ingroup vtkX3DImporter
 */
class vtkX3DNodeHandler : public XIOT::X3DDefaultNodeHandler
{
public:
	
  vtkX3DNodeHandler(vtkRenderer* Renderer, vtkX3DImporter* Importer);
	~vtkX3DNodeHandler();

  void startDocument();

	int startUnhandled(const char* nodeName, const XIOT::X3DAttributes &attr);
	
	int startAppearance(const XIOT::X3DAttributes &attr);
	
	int startMaterial(const XIOT::X3DAttributes &attr);
	
	int startTransform(const XIOT::X3DAttributes &attr);
	int endTransform();

	int startSphere(const XIOT::X3DAttributes &attr);

	int startShape(const XIOT::X3DAttributes &attr);
	int endShape();

	int startIndexedLineSet(const XIOT::X3DAttributes &attr);
	int endIndexedLineSet();

	int startIndexedFaceSet(const XIOT::X3DAttributes &attr);
	int endIndexedFaceSet();


	int startCoordinate(const XIOT::X3DAttributes &attr);
	
	int startNormal(const XIOT::X3DAttributes &attr);
	
	int startColor(const XIOT::X3DAttributes &attr);
	
	int startTextureCoordinate(const XIOT::X3DAttributes &attr);
	
	int startPointSet(const XIOT::X3DAttributes &attr);
	int endPointSet();

	int startBox(const XIOT::X3DAttributes &attr);

	int startCone(const XIOT::X3DAttributes &attr);

	int startCylinder(const XIOT::X3DAttributes &attr);
	
	int startDirectionalLight(const XIOT::X3DAttributes &attr);
	
  int startPixelTexture(const XIOT::X3DAttributes &attr);
  int startImageTexture(const XIOT::X3DAttributes &attr);

  /// X3DBindables
  int startNavigationInfo(const XIOT::X3DAttributes &attr);
	int startBackground(const XIOT::X3DAttributes &attr);
  int startViewpoint(const XIOT::X3DAttributes &attr);

protected:
  template<class T>
  int checkReferencing(const XIOT::X3DAttributes &attr, T** obj, bool shouldDelete) {
    if (attr.isUSE())
      {
      vtkObject* p = this->DefMap[attr.getUSE()];
      if (shouldDelete)
        {
        (*obj)->Delete();
        }
      *obj = T::SafeDownCast(p);
      if(!obj)
        {
        vtkWarningWithObjectMacro(this->Importer, << "Could not find node of type <" << (*obj)->GetClassName() << "> with DEF=\"" << attr.getUSE() << "\"."); \
        }
      this->IsCurrentUSE = true;
      return true;
      }
    this->IsCurrentUSE = false;
    if (attr.isDEF())
      {
      (*obj)->Register(this->MapReferencer); // Reference count array
      this->DefMap[attr.getDEF()] = *obj;
      }
    return false;
    }

private:
	vtkRenderer			*Renderer;
  vtkX3DImporter  *Importer;
  vtkLight        *HeadLight;

	vtkActor                    *CurrentActor;
	vtkTransform                *CurrentTransform;
	vtkPoints                   *CurrentPoints;
	vtkFloatArray               *CurrentNormals;
	vtkFloatArray               *CurrentTCoords;
	vtkUnsignedCharArray        *CurrentColors;
	vtkX3DIndexedGeometrySource *CurrentIndexedGeometry;
  
  int IsCurrentUnlit;
  int IsCurrentUSE;

  double CurrentEmissiveColor[3];

  int SeenBindables;

  vtkObject *MapReferencer;

	std::map<std::string,vtkObject*>			DefMap;		// used for DEF/USE
	std::set<int> _ignoreNodes;

};

#endif

