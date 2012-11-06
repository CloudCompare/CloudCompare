#ifndef __vtkX3DIndexedGeometrySource_h
#define __vtkX3DIndexedGeometrySource_h

#include <vtkPolyDataAlgorithm.h>

class vtkIdTypeArray;
class vtkFloatArray;
class vtkUnsignedCharArray;

/**
 * Generate vtkPolyData from X3D IndexedFaceSet / IndexedLineSet
 *
 * Helper class to generate a vtkPolyData structure from 
 * an IndexedFaceSet or IndexedLineSet data structure. This class converts index
 * lists and resolves multiple used coordinate data.
 * @see <A href="http://www.vtk.org/doc/nightly/html/classvtkPolyDataAlgorithm.html"/>vtkPolyDataAlgorithm</A>
 * @see <A href="http://www.web3d.org/x3d/specifications/ISO-IEC-FDIS-19775-1.2-X3D-AbstractSpecification/Part01/components/geometry3D.html#IndexedFaceSet"/>IndexedFaceSet</A>
 * @ingroup vtkX3DImporter
 */
class vtkX3DIndexedGeometrySource : public vtkPolyDataAlgorithm
{
public:
  static vtkX3DIndexedGeometrySource *New();
  vtkTypeRevisionMacro(vtkX3DIndexedGeometrySource,vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

//BTX
enum GeometryFormatEnum
  {
  INDEXED_FACESET,
  INDEXED_LINESET,
  };
//ETX

  vtkSetClampMacro(GeometryFormat, int, INDEXED_FACESET, INDEXED_LINESET);
  vtkGetMacro(GeometryFormat, int);
  void SetGeometryFormatToIndexedFaceSet()
    {this->SetGeometryFormat(INDEXED_FACESET);};
  void SetGeometryFormatToIndexedLineSet()
    {this->SetGeometryFormat(INDEXED_LINESET);};



  vtkSetMacro(NormalPerVertex  ,int);
  vtkGetMacro(NormalPerVertex ,int);
  vtkBooleanMacro(NormalPerVertex ,int);

  vtkSetMacro(ColorPerVertex ,int);
  vtkGetMacro(ColorPerVertex,int);
  vtkBooleanMacro(ColorPerVertex,int);

  vtkSetMacro(CreaseAngle, double);
  vtkGetMacro(CreaseAngle, double);

  vtkSetMacro(CalculateNormals  ,int);
  vtkGetMacro(CalculateNormals ,int);
  vtkBooleanMacro(CalculateNormals ,int);


  // Indices

  // Description:
  // Set the coord index array defining vertices.
  // This is in X3D format with -1 splitting cells
  void SetCoordIndex(vtkIdTypeArray * v);
  vtkIdTypeArray *GetCoordIndex() { return this->CoordIndex; };
  
  void SetNormalIndex(vtkIdTypeArray * v);
  vtkIdTypeArray *GetNormalIndex() { return this->NormalIndex; };

  void SetColorIndex(vtkIdTypeArray * v);
  vtkIdTypeArray *GetColorIndex() { return this->ColorIndex; };

  void SetTexCoordIndex(vtkIdTypeArray * v);
  vtkIdTypeArray *GetTexCoordIndex() { return this->TexCoordIndex; };

  // Data
  void SetCoords(vtkPoints *p);
  vtkPoints* GetCoords() { return this->Coords; };

  void SetNormals(vtkFloatArray* p);
  vtkFloatArray* GetNormals() { return this->Normals; };

  void SetTexCoords(vtkFloatArray* p);
  vtkFloatArray* GetTexCoords(){ return this->TexCoords; };

  void SetColors(vtkUnsignedCharArray* p);
  vtkUnsignedCharArray* GetColors(){ return this->Colors; };
  

protected:
  vtkX3DIndexedGeometrySource();
  ~vtkX3DIndexedGeometrySource();

  // Usual data generation method
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);
  int processIndexedFaceSet(vtkPolyData* pd, vtkCellArray* cells);
  int processIndexedLineSet(vtkPolyData* pd, vtkCellArray* cells);
  
  void ProcessColor(vtkPolyData* pd);

  int GeometryFormat;

  int NormalPerVertex;
  int ColorPerVertex;
  int CalculateNormals;

  double CreaseAngle;

  vtkIdTypeArray* CoordIndex;
  vtkIdTypeArray* NormalIndex;
  vtkIdTypeArray* ColorIndex;
  vtkIdTypeArray* TexCoordIndex;

  vtkPoints *Coords;
  vtkFloatArray *Normals;
  vtkFloatArray *TexCoords;
  vtkUnsignedCharArray* Colors;


private:


  vtkX3DIndexedGeometrySource(const vtkX3DIndexedGeometrySource&); // Not implemented.
  void operator=(const vtkX3DIndexedGeometrySource&); // Not implemented.
};


#endif
