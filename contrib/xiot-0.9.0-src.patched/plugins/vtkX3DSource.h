#ifndef __vtkX3DSource_h
#define __vtkX3DSource_h

#include "vtkMultiBlockDataSetAlgorithm.h"

/**
 *  @defgroup pvX3DReader ParaView X3D Reader
 */


class vtkMultiBlockDataSet;
class vtkX3DImporter;

/**
 * Converts X3D importer to a source.
 * 
 * Since paraview can only use vtkMultiBlockDataSetAlgorithms,
 * the X3D importer is wrapped into this algorithm as a source.
 * It will extract one or more geometry objects from the imported
 * scene. The behaviour is similar to the one of vtkVRMLSource
 * for VRML scenes.
 * @see vtkVRMLSource
 * @ingroup pvX3DReader
 */
class VTK_EXPORT vtkX3DSource : public vtkMultiBlockDataSetAlgorithm
{
public:
  vtkTypeRevisionMacro(vtkX3DSource,vtkMultiBlockDataSetAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);
  static vtkX3DSource *New();

  // Description:
  // X3D file name.  Set
  vtkSetStringMacro(FileName);
  vtkGetStringMacro(FileName);

  // Description: 
  // Descided whether to generate color arrays or not.
  vtkSetMacro(Color,int);
  vtkGetMacro(Color,int);
  vtkBooleanMacro(Color,int);

  // Description:
  // This method allows all parts to be put into a single output.
  // By default this flag is on.
  vtkSetMacro(Append,int);
  vtkGetMacro(Append,int);
  vtkBooleanMacro(Append,int);

protected:
  vtkX3DSource();
  ~vtkX3DSource();

  int RequestData(vtkInformation*, 
                  vtkInformationVector**, 
                  vtkInformationVector*);

  void InitializeImporter();
  void CopyImporterToOutputs(vtkMultiBlockDataSet*);

  char* FileName;
  vtkX3DImporter *Importer;
  int Color;
  int Append;

private:
  vtkX3DSource(const vtkX3DSource&);  // Not implemented.
  void operator=(const vtkX3DSource&);  // Not implemented.
};

#endif

