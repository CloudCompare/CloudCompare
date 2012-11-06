#ifndef __vtkX3DImporter_h
#define __vtkX3DImporter_h

/**
 *  @defgroup vtkX3DImporter X3D Importer for VTK
 */
#include "vtkImporter.h"

/** 
 * Imports X3D files into the VTK rendering system.
 *
 * It implements 
 * <a href="http://www.vtk.org/doc/nightly/html/classvtkImporter.html">vtkImporter</a> 
 * to use the standard VTK import mechanism. 
 * @see vtkImporter
 * @warning Currently the importer supports or partly supports following nodes: 
 * <ul>
 * <li>Appearance</li>
 * <li>Material</li>
 * <li>Transform</li>
 * <li>Sphere</li>
 * <li>Shape</li>
 * <li>IndexedFaceSet</li>
 * <li>Coordinate</li>
 * <li>Normal</li>
 * <li>Color</li>
 * <li>TextureCoordinate</li>
 * <li>Box</li>
 * <li>Cone</li>
 * <li>DirectionalLight</li>
 * <li>PointSet</li>
 * <li>Background</li>
 * <li>Viewpoint</li>
 * <li>NavigationInfo</li>
 * </ul>
 * There is no event processing
 * @ingroup vtkX3DImporter
 */
class vtkX3DImporter : public vtkImporter
{
public:
  static vtkX3DImporter *New();
  vtkTypeRevisionMacro(vtkX3DImporter,vtkImporter);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Specify the name of the file to read.
  vtkSetStringMacro(FileName);
  vtkGetStringMacro(FileName);

  vtkSetMacro(CalculateNormals  ,int);
  vtkGetMacro(CalculateNormals ,int);
  vtkBooleanMacro(CalculateNormals ,int);

  virtual int ImportBegin ();
  virtual void ImportEnd ();

protected:
  vtkX3DImporter();
  ~vtkX3DImporter();
 
  char *FileName;
  int CalculateNormals;

private:

  vtkX3DImporter(const vtkX3DImporter&); // Not implemented.
  void operator=(const vtkX3DImporter&); // Not implemented.
};


#endif
