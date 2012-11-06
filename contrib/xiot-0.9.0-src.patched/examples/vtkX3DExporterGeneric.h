/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkX3DExporterGeneric.h,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtX3DExporter - create an x3d file
// .SECTION Description
// vtkX3DExporter is a render window exporter which writes out the renderered
// scene into an X3D file. X3D is an XML-based format for representation 
// 3D scenes (similar to VRML). Check out http://www.web3d.org/x3d/ for more
// details.
// .SECTION Thanks
// X3DExporter is contributed by Christophe Mouton at EDF and implemented 
// by Kristian Sons at Supporting GmbH (xiot@supporting.com).
#ifndef __vtkX3DExporterGeneric_h
#define __vtkX3DExporterGeneric_h

#include "vtkExporter.h"

class vtkLight;
class vtkActor;
class vtkActor2D;
class vtkPoints;
class vtkDataArray;
class vtkUnsignedCharArray;
class vtkX3DExporterWriter;
class vtkRenderer;

namespace XIOT {
	class X3DWriter;
}


class vtkX3DExporterGeneric : public vtkExporter
{
public:
  static vtkX3DExporterGeneric *New();
  vtkTypeRevisionMacro(vtkX3DExporterGeneric,vtkExporter);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Set/Get the output file name.
  vtkSetStringMacro(FileName);
  vtkGetStringMacro(FileName);

  // Description:
  // Set/Get the file name written to the file
  // This is for comparision with other exporter
  // version
  vtkSetStringMacro(FakeFileName);
  vtkGetStringMacro(FakeFileName);


  // Description:
  // Specify the Speed of navigation. Default is 4.
  vtkSetMacro(Speed,double);
  vtkGetMacro(Speed,double);

  // Description:
  // Turn on binary mode
  vtkSetClampMacro(Binary, int, 0, 1);
  vtkBooleanMacro(Binary, int);
  vtkGetMacro(Binary, int);

  // Description:
  // In binary mode use fastest instead of best compression
  vtkSetClampMacro(Fastest, int, 0, 1);
  vtkBooleanMacro(Fastest, int);
  vtkGetMacro(Fastest, int);

protected:
  vtkX3DExporterGeneric();
  ~vtkX3DExporterGeneric();

  // Description:
  // Write data to output.
  void WriteData();

  void WriteALight(vtkLight *aLight, XIOT::X3DWriter* writer);
  void WriteAnActor(vtkActor *anActor, XIOT::X3DWriter* writer,
    int index);
  void WritePointData(vtkPoints *points, vtkDataArray *normals,
    vtkDataArray *tcoords, vtkUnsignedCharArray *colors,
    XIOT::X3DWriter* writer, int index);
  void WriteATextActor2D(vtkActor2D *anTextActor2D,
    XIOT::X3DWriter* writer);
  void WriteATexture(vtkActor *anActor, XIOT::X3DWriter* writer);
  void WriteAnAppearance(vtkActor *anActor, bool writeEmissiveColor, XIOT::X3DWriter* writer);
  int HasHeadLight(vtkRenderer* ren);
  char *FileName;
  char *FakeFileName;
  double Speed;
  int Binary;
  int Fastest;

private:

  vtkX3DExporterGeneric(const vtkX3DExporterGeneric&); // Not implemented.
  void operator=(const vtkX3DExporterGeneric&); // Not implemented.
};


#endif
