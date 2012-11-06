/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkX3DExporterGeneric.cxx,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen, Kristian Sons
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkX3DExporterGeneric.h"

#include "vtkActor2DCollection.h"
#include "vtkActor2D.h"
#include "vtkAssemblyPath.h"
#include "vtkCamera.h"
#include "vtkCellArray.h"
#include "vtkCellData.h"
#include "vtkGeometryFilter.h"
#include "vtkImageData.h"
#include "vtkLightCollection.h"
#include "vtkLight.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper2D.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkRendererCollection.h"
#include "vtkRenderWindow.h"
#include "vtkSmartPointer.h"
#include "vtkTextActor.h"
#include "vtkTextProperty.h"
#include "vtkTexture.h"
#include "vtkTransform.h"
#include "vtkDataArray.h"

#include "xiot/X3DWriterXML.h"
#include "xiot/X3DWriterFI.h"
#include "xiot/X3DTypes.h"

#include <vtksys/ios/sstream>
#include <cassert>



using namespace XIOT;

// forward declarations
static bool vtkX3DExporterWriterUsingCellColors(vtkActor* anActor);
static bool vtkX3DExporterWriterRenderFaceSet(
  int cellType,
  int representation,
  vtkPoints* points,
  vtkIdType cellOffset,
  vtkCellArray* cells,
  vtkUnsignedCharArray* colors, bool cell_colors,
  vtkDataArray* normals, bool cell_normals, 
  vtkDataArray* tcoords, 
  bool common_data_written, int index, X3DWriter* writer);
static void vtkX3DExporterWriteData(vtkPoints *points, 
  vtkDataArray *normals,
  vtkDataArray *tcoords, 
  vtkUnsignedCharArray *colors,
  int index, X3DWriter* writer);
static void vtkX3DExporterUseData(bool normals, bool tcoords, bool colors, int index, 
  X3DWriter* writer);
static bool vtkX3DExporterWriterRenderVerts(
  vtkPoints* points, vtkCellArray* cells,
  vtkUnsignedCharArray* colors, bool cell_colors,  X3DWriter* writer);
static bool vtkX3DExporterWriterRenderPoints(
  vtkPolyData* pd, 
  vtkUnsignedCharArray* colors,
  bool cell_colors,  
  X3DWriter* writer);

vtkCxxRevisionMacro(vtkX3DExporterGeneric, "$Revision: 1.19 $");
vtkStandardNewMacro(vtkX3DExporterGeneric);

//----------------------------------------------------------------------------
vtkX3DExporterGeneric::vtkX3DExporterGeneric()
{
  this->Speed = 4.0;
  this->FileName = NULL;
  this->FakeFileName = NULL;
  this->Binary = 0;
  this->Fastest = 0;
}
//----------------------------------------------------------------------------
vtkX3DExporterGeneric::~vtkX3DExporterGeneric()
{
  this->SetFileName(0);
  this->SetFakeFileName(0);
}


//----------------------------------------------------------------------------
void vtkX3DExporterGeneric::WriteData()
{
  X3DWriter *writer;
  vtkRenderer *ren;
  vtkActorCollection *ac;
  vtkActor2DCollection *a2Dc;
  vtkActor *anActor, *aPart;
  vtkActor2D *anTextActor2D, *aPart2D;
  vtkLightCollection *lc;
  vtkLight *aLight;
  vtkCamera *cam;

  // make sure the user specified a FileName or FilePointer
  if (this->FileName == NULL)
    {
    vtkErrorMacro(<< "Please specify FileName to use");
    return;
    }

  // Let's assume the first renderer is the right one
  // first make sure there is only one renderer in this rendering window
  //if (this->RenderWindow->GetRenderers()->GetNumberOfItems() > 1)
  //  {
  //  vtkErrorMacro(<< "X3D files only support one renderer per window.");
  //  return;
  //  }

  // get the renderer
  ren = this->RenderWindow->GetRenderers()->GetFirstRenderer();

  // make sure it has at least one actor
  if (ren->GetActors()->GetNumberOfItems() < 1)
    {
    vtkErrorMacro(<< "no actors found for writing X3D file.");
    return;
    }

  // try opening the files
  if (this->Binary)
    {
    
	X3DWriterFI* temp = new X3DWriterFI();
	temp->setProperty(Property::IntEncodingAlgorithm, (void*)Encoder::DeltazlibIntArrayEncoder);
	if (this->GetFastest())
		temp->setProperty(Property::FloatEncodingAlgorithm, (void*)Encoder::BuiltIn);
	else
		temp->setProperty(Property::FloatEncodingAlgorithm, (void*)Encoder::QuantizedzlibFloatArrayEncoder);
	
	writer = temp;
	
    }
  else
    {
		writer = new X3DWriterXML();
    }


  if (!writer->openFile(this->FileName))
    {
    vtkErrorMacro(<< "unable to open X3D file " << this->FileName);
    return;
    }

  //
  //  Write header
  //
  vtkDebugMacro("Writing X3D file");
  
  vtksys_ios::ostringstream ss;
  ss << ren->GetActors()->GetNumberOfItems();

  std::multimap<std::string, std::string> meta;
  meta.insert(std::pair<std::string, std::string>("filename", this->FakeFileName ? this->FakeFileName : this->FileName));
  meta.insert(std::pair<std::string, std::string>("generator", "Visualization ToolKit X3D exporter v0.9.1"));
  meta.insert(std::pair<std::string, std::string>("numberofelements", ss.str()));
  writer->startX3DDocument(Immersive, VERSION_3_0, &meta, false);

  double *dp;
  // Start write the Background
  writer->startNode(ID::Background);
  writer->setSFColor(ID::skyColor, ren->GetBackground());
  writer->endNode();
  // End of Background

  // Start write the Camera
  cam = ren->GetActiveCamera();
  writer->startNode(ID::Viewpoint);
  writer->setSFFloat(ID::fieldOfView, vtkMath::RadiansFromDegrees(cam->GetViewAngle()));
  
  writer->setSFVec3f(ID::position, cam->GetPosition());
  writer->setSFString(ID::description, "Default View");
  dp = cam->GetOrientationWXYZ();
  SFRotation orientation(dp[1], dp[2], dp[3], vtkMath::RadiansFromDegrees(-dp[0]));
  writer->setSFRotation(ID::orientation, orientation);
  writer->setSFVec3f(ID::centerOfRotation, cam->GetFocalPoint());
  writer->endNode();
  // End of Camera

  MFString navigationTypes;
  writer->startNode(ID::NavigationInfo);
  navigationTypes.push_back("EXAMINE");
  navigationTypes.push_back("FLY");
  navigationTypes.push_back("ANY");
  writer->setMFString(ID::type, navigationTypes);

  writer->setSFFloat(ID::speed, this->Speed);
  writer->setSFBool(ID::headlight, this->HasHeadLight(ren) ? true : false);
  writer->endNode();

  // do the lights first the ambient then the others
  writer->startNode(ID::DirectionalLight);
  writer->setSFFloat(ID::ambientIntensity, 1.0f);
  writer->setSFFloat(ID::intensity, 0.0f);
  dp = ren->GetAmbient();
  writer->setSFColor(ID::color, dp[0], dp[1], dp[2]);
  writer->endNode();

   // label ROOT
  static double n[] = {0.0, 0.0, 0.0};
  writer->startNode(ID::Transform);
  writer->setSFString(ID::DEF, "ROOT");
  writer->setSFVec3f(ID::translation, n[0], n[1], n[2]);


  // Write out the lights now
  lc = ren->GetLights();
  vtkCollectionSimpleIterator lsit;
  for (lc->InitTraversal(lsit); (aLight = lc->GetNextLight(lsit)); )
    {
    if (!aLight->LightTypeIsHeadlight())
      {
      this->WriteALight(aLight, writer);
      }
    }


 

  // do the actors now
  ac = ren->GetActors();
  vtkAssemblyPath *apath;
  vtkCollectionSimpleIterator ait;
  int index=0;
  for (ac->InitTraversal(ait); (anActor = ac->GetNextActor(ait)); )
    {
    for (anActor->InitPathTraversal(); (apath=anActor->GetNextPath()); )
      {
      if(anActor->GetVisibility()!=0)
        {
        aPart=static_cast<vtkActor *>(apath->GetLastNode()->GetViewProp());
        this->WriteAnActor(aPart, writer, index);
        index++;
        }
      }
    }
  writer->endNode(); // ROOT Transform


  //////////////////////////////////////////////
  // do the 2D actors now
  a2Dc = ren->GetActors2D();

  if(a2Dc->GetNumberOfItems()!=0)
    {
    static double s[] = {1000000.0, 1000000.0, 1000000.0};
    writer->startNode(ID::ProximitySensor);
    writer->setSFString(ID::DEF, "PROX_LABEL");
    writer->setSFVec3f(ID::size, s[0], s[1], s[2]);
    writer->endNode();

    //disable collision for the text annotations
    writer->startNode(ID::Collision);
	writer->setSFBool(ID::enabled, false);

    //add a Label TRANS_LABEL for the text annotations and the sensor
    writer->startNode(ID::Transform);
    writer->setSFString(ID::DEF, "TRANS_LABEL");

    vtkAssemblyPath *apath2D;
    vtkCollectionSimpleIterator ait2D;
    for (a2Dc->InitTraversal(ait2D);
      (anTextActor2D = a2Dc->GetNextActor2D(ait2D)); )
      {

      for (anTextActor2D->InitPathTraversal();
        (apath2D=anTextActor2D->GetNextPath()); )
        {
        aPart2D=
              static_cast<vtkActor2D *>(apath2D->GetLastNode()->GetViewProp());
        this->WriteATextActor2D(aPart2D, writer);
        }
      }
    writer->endNode(); // Transform
    writer->endNode(); // Collision

    writer->startNode(ID::ROUTE);
    writer->setSFString(ID::fromNode, "PROX_LABEL");
    writer->setSFString(ID::fromField, "position_changed");
    writer->setSFString(ID::toNode, "TRANS_LABEL");
    writer->setSFString(ID::toField, "set_translation");
    writer->endNode(); // Route

    writer->startNode(ID::ROUTE);
    writer->setSFString(ID::fromNode, "PROX_LABEL");
    writer->setSFString(ID::fromField, "orientation_changed");
    writer->setSFString(ID::toNode, "TRANS_LABEL");
    writer->setSFString(ID::toField, "set_rotation");
    writer->endNode(); // Route
    }
  /////////////////////////////////////////////////

  writer->endX3DDocument();
  writer->closeFile();
}


//----------------------------------------------------------------------------
void vtkX3DExporterGeneric::WriteALight(vtkLight *aLight,
  X3DWriter* writer)
{
  double *pos, *focus, *colord;
  double *dp;
  double dir[3];

  pos = aLight->GetPosition();
  focus = aLight->GetFocalPoint();
  colord = aLight->GetColor();

  dir[0] = focus[0] - pos[0];
  dir[1] = focus[1] - pos[1];
  dir[2] = focus[2] - pos[2];
  vtkMath::Normalize(dir);

  if (aLight->GetPositional())
    {
    if (aLight->GetConeAngle() >= 180.0)
      {
      writer->startNode(ID::PointLight);
      }
    else
      { 
      writer->startNode(ID::SpotLight);
      writer->setSFVec3f(ID::direction, dir[0], dir[1], dir[2]);
      writer->setSFFloat(ID::cutOffAngle,aLight->GetConeAngle());
      }
    writer->setSFVec3f(ID::location, pos[0], pos[1], pos[2]);
	dp = aLight->GetAttenuationValues();
    writer->setSFVec3f(ID::attenuation, dp[0], dp[1], dp[2]);

    }
  else
    {
    writer->startNode(ID::DirectionalLight);
    writer->setSFVec3f(ID::direction, dir[0], dir[1], dir[2]);
    }

  // TODO: Check correct color
  writer->setSFColor(ID::color, colord[0], colord[1], colord[2]);
  writer->setSFFloat(ID::intensity, aLight->GetIntensity());
  writer->setSFBool(ID::on, aLight->GetSwitch() ? true : false); 
  writer->endNode();
  writer->flush();
}

//----------------------------------------------------------------------------
void vtkX3DExporterGeneric::WriteAnActor(vtkActor *anActor,
  X3DWriter* writer, int index)
{
  vtkDataSet *ds;
  vtkPolyData *pd;
  vtkSmartPointer<vtkGeometryFilter> gf;
  vtkPointData *pntData;
  vtkCellData *cellData;
  vtkPoints *points;
  vtkDataArray *normals = NULL;
  vtkDataArray *tcoords = NULL;
  vtkProperty *prop;
  vtkUnsignedCharArray *colors;
  vtkSmartPointer<vtkTransform> trans;

  // to be deleted
  vtksys_ios::ostringstream appearance_stream;
  vtksys_ios::ostringstream ostr;

  // see if the actor has a mapper. it could be an assembly
  if (anActor->GetMapper() == NULL)
    {
    return;
    }

  // Essential to turn of interpolate scalars otherwise GetScalars() may return
  // NULL. We restore value before returning.
  int isbm = anActor->GetMapper()->GetInterpolateScalarsBeforeMapping();
  anActor->GetMapper()->SetInterpolateScalarsBeforeMapping(0);

  // first stuff out the transform
  trans = vtkSmartPointer<vtkTransform>::New();
  trans->SetMatrix(anActor->vtkProp3D::GetMatrix());

  double *dp;
  writer->startNode(ID::Transform);
  dp = trans->GetPosition();
  writer->setSFVec3f(ID::translation, dp[0], dp[1], dp[2]);
  dp = trans->GetOrientationWXYZ();
  writer->setSFRotation(ID::rotation, dp[1], dp[2], dp[3], vtkMath::RadiansFromDegrees(-dp[0]));
  dp = trans->GetScale();
  writer->setSFVec3f(ID::scale, dp[0], dp[1], dp[2]);

  // get the mappers input and matrix
  ds = anActor->GetMapper()->GetInput();

  // we really want polydata
  if ( ds->GetDataObjectType() != VTK_POLY_DATA )
    {
    gf = vtkSmartPointer<vtkGeometryFilter>::New();
    gf->SetInput(ds);
    gf->Update();
    pd = gf->GetOutput();
    }
  else
    {
    pd = static_cast<vtkPolyData *>(ds);
    }

  prop = anActor->GetProperty();
  points = pd->GetPoints();
  pntData = pd->GetPointData();
  tcoords = pntData->GetTCoords();
  cellData = pd->GetCellData();

  colors  = anActor->GetMapper()->MapScalars(255.0);

  // Are we using cell colors.
  bool cell_colors = vtkX3DExporterWriterUsingCellColors(anActor);

  normals = pntData->GetNormals();

  // Are we using cell normals.
  bool cell_normals = false;
  if (prop->GetInterpolation() == VTK_FLAT || !normals)
    {
    // use cell normals, if any.
    normals = cellData->GetNormals();
    cell_normals = true;
    }


  // if we don't have colors and we have only lines & points
  // use emissive to color them
  bool writeEmissiveColor = !(normals || colors || pd->GetNumberOfPolys() ||
    pd->GetNumberOfStrips());

  // write out the material properties to the mat file
  int representation = prop->GetRepresentation();

  if (representation == VTK_POINTS)
    {
    // If representation is points, then we don't have to render different cell
    // types in separate shapes, since the cells type no longer matter.
    if (true)
      {
      writer->startNode(ID::Shape);
      this->WriteAnAppearance(anActor, writeEmissiveColor, writer);
      vtkX3DExporterWriterRenderPoints(pd, colors, cell_colors, writer);
      writer->endNode();
      }
    }
  else
    {
    // When rendering as lines or surface, we need to respect the cell
    // structure. This requires rendering polys, tstrips, lines, verts in
    // separate shapes.
    vtkCellArray* verts = pd->GetVerts();
    vtkCellArray* lines = pd->GetLines();
    vtkCellArray* polys = pd->GetPolys();
    vtkCellArray* tstrips = pd->GetStrips();

    vtksys_ios::ostringstream geometry_stream;

    vtkIdType numVerts = verts->GetNumberOfCells();
    vtkIdType numLines = lines->GetNumberOfCells();
    vtkIdType numPolys = polys->GetNumberOfCells();
    vtkIdType numStrips = tstrips->GetNumberOfCells();

    bool common_data_written = false;
    if (numPolys > 0)
      {
      writer->startNode(ID::Shape);
      // Write Appearance
      this->WriteAnAppearance(anActor, writeEmissiveColor, writer);
      // Write Geometry
      vtkX3DExporterWriterRenderFaceSet(VTK_POLYGON, representation, points,
        (numVerts+numLines), polys, 
        colors, cell_colors, normals, cell_normals, 
        tcoords, common_data_written, index, writer);
      writer->endNode();  // close the  Shape
      common_data_written = true;
      }

    if (numStrips > 0)
      {
      writer->startNode(ID::Shape);
      // Write Appearance
      this->WriteAnAppearance(anActor, writeEmissiveColor, writer);
      // Write Geometry
      vtkX3DExporterWriterRenderFaceSet(VTK_TRIANGLE_STRIP,
        representation, points,
        (numVerts+numLines+numPolys), tstrips, 
        colors, cell_colors, normals, cell_normals, 
        tcoords, common_data_written, index, writer);
      writer->endNode();  // close the  Shape
      common_data_written = true;
      }

    if (numLines > 0)
      {
      writer->startNode(ID::Shape);
      // Write Appearance
      this->WriteAnAppearance(anActor, writeEmissiveColor, writer);
      // Write Geometry
      vtkX3DExporterWriterRenderFaceSet(VTK_POLY_LINE,
        (representation==VTK_SURFACE? VTK_WIREFRAME:representation), 
        points, (numVerts), lines, 
        colors, cell_colors, normals, cell_normals, 
        tcoords, common_data_written, index, writer);
      writer->endNode();  // close the  Shape
      common_data_written = true;
      }      

    if (numVerts > 0)
      {
      writer->startNode(ID::Shape);
      this->WriteAnAppearance(anActor, writeEmissiveColor, writer);
      vtkX3DExporterWriterRenderVerts(
        points, verts,
        colors, cell_normals, writer);
      writer->endNode();  // close the  Shape
      }

    }
  writer->endNode(); // close the original transform
  anActor->GetMapper()->SetInterpolateScalarsBeforeMapping(isbm);
}

//----------------------------------------------------------------------------
void vtkX3DExporterGeneric::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  if (this->FileName)
    {
    os << indent << "FileName: " << this->FileName << "\n";
    }
  else
    {
    os << indent << "FileName: (null)\n";
    }
  os << indent << "Speed: " << this->Speed << "\n";
  os << indent << "Binary: " << this->Binary << "\n";
  os << indent << "Fastest: " << this->Fastest << endl;
}




//----------------------------------------------------------------------------
void vtkX3DExporterGeneric::WriteATextActor2D(vtkActor2D *anTextActor2D,
  X3DWriter* writer)
{
  char *ds;
  vtkTextActor *ta;
  vtkTextProperty *tp;

  if (!anTextActor2D->IsA("vtkTextActor"))
    {
    return;
    }

  ta = static_cast<vtkTextActor*>(anTextActor2D);
  tp = ta->GetTextProperty();
  ds = NULL;
  ds = ta->GetInput();

  if (ds==NULL)
    {
    return;
    }

  double temp[3];

  writer->startNode(ID::Transform);
  temp[0] = ((ta->GetPosition()[0])/(this->RenderWindow->GetSize()[0])) - 0.5;
  temp[1] = ((ta->GetPosition()[1])/(this->RenderWindow->GetSize()[1])) - 0.5;
  temp[2] = -2.0;
  writer->setSFVec3f(ID::translation, temp[0], temp[1], temp[2]);
  temp[0] = temp[1] = temp[2] = 0.002;
  writer->setSFVec3f(ID::scale, temp[0], temp[1], temp[2]);

  writer->startNode(ID::Shape);

  writer->startNode(ID::Appearance);

  writer->startNode(ID::Material);
  temp[0] = 0.0; temp[1] = 0.0; temp[2] = 1.0;
  writer->setSFColor(ID::diffuseColor, temp[0], temp[1], temp[2]);
  tp->GetColor(temp);
  writer->setSFColor(ID::emissiveColor, temp[0], temp[1], temp[2]);
  writer->endNode(); // Material

  writer->endNode(); // Appearance

  writer->startNode(ID::Text);
  writer->setSFString(ID::string, ds);

  vtkstd::string familyStr;
  switch(tp->GetFontFamily())
    {
  case 0:
  default:
    familyStr = "\"SANS\"";
    break;
  case 1:
    familyStr = "\"TYPEWRITER\"";
    break;
  case 2:
    familyStr = "\"SERIF\"";
    break;
    }

  vtkstd::string justifyStr;
  switch  (tp->GetJustification())
    {
  case 0:
  default:
    justifyStr += "\"BEGIN\"";
    break;
  case 2:
    justifyStr += "\"END\"";
    break;
    }

  justifyStr += " \"BEGIN\"";

  writer->startNode(ID::FontStyle);
  writer->setSFString(ID::family, familyStr);
  writer->setSFBool(ID::topToBottom, tp->GetVerticalJustification() == 2);
  writer->setSFString(ID::justify, justifyStr);
  writer->setSFInt32(ID::size, tp->GetFontSize());
  writer->endNode(); // FontStyle
  writer->endNode(); // Text
  writer->endNode(); // Shape
  writer->endNode(); // Transform
}

void vtkX3DExporterGeneric::WriteAnAppearance(vtkActor *anActor, bool emissive,
  X3DWriter* writer)
{
  double tempd[3];
  double tempf2;

  vtkProperty* prop = anActor->GetProperty();

  writer->startNode(ID::Appearance);
  writer->startNode(ID::Material);
  writer->setSFFloat(ID::ambientIntensity,prop->GetAmbient());

  if (emissive)
    {
    tempf2 = prop->GetAmbient();
    prop->GetAmbientColor(tempd);
    tempd[0]*=tempf2;
    tempd[1]*=tempf2;
    tempd[2]*=tempf2;
    }
  else
    {
    tempd[0] = tempd[1] = tempd[2] = 0.0f;
    }
  writer->setSFColor(ID::emissiveColor, tempd[0], tempd[1], tempd[2]);

  // Set diffuse color
  tempf2 = prop->GetDiffuse();
  prop->GetDiffuseColor(tempd);
  tempd[0]*=tempf2;
  tempd[1]*=tempf2;
  tempd[2]*=tempf2;
  writer->setSFColor(ID::diffuseColor, tempd[0], tempd[1], tempd[2]);

  // Set specular color
  tempf2 = prop->GetSpecular();
  prop->GetSpecularColor(tempd);
  tempd[0]*=tempf2;
  tempd[1]*=tempf2;
  tempd[2]*=tempf2;
  writer->setSFColor(ID::specularColor, tempd[0], tempd[1], tempd[2]);  

  // Material shininess
  writer->setSFFloat(ID::shininess,prop->GetSpecularPower()/128.0);
  // Material transparency
  writer->setSFFloat(ID::transparency,1.0 - prop->GetOpacity());
  writer->endNode(); // close material

  // is there a texture map
  if (anActor->GetTexture())
    {
    this->WriteATexture(anActor, writer);
    }
  writer->endNode(); // close appearance
}

void vtkX3DExporterGeneric::WriteATexture(vtkActor *anActor,
  X3DWriter* writer)
{
  vtkTexture *aTexture = anActor->GetTexture();
  int *size, xsize, ysize;
  vtkDataArray *scalars;
  vtkDataArray *mappedScalars;
  unsigned char *txtrData;
  int totalValues;


  // make sure it is updated and then get some info
  if (aTexture->GetInput() == NULL)
    {
    vtkErrorMacro(<< "texture has no input!\n");
    return;
    }
  aTexture->GetInput()->Update();
  size = aTexture->GetInput()->GetDimensions();
  scalars = aTexture->GetInput()->GetPointData()->GetScalars();

  // make sure scalars are non null
  if (!scalars) 
    {
    vtkErrorMacro(<< "No scalar values found for texture input!\n");
    return;
    }

  // make sure using unsigned char data of color scalars type
  if (aTexture->GetMapColorScalarsThroughLookupTable () ||
    (scalars->GetDataType() != VTK_UNSIGNED_CHAR) )
    {
    mappedScalars = aTexture->GetMappedScalars ();
    }
  else
    {
    mappedScalars = scalars;
    }

  // we only support 2d texture maps right now
  // so one of the three sizes must be 1, but it 
  // could be any of them, so lets find it
  if (size[0] == 1)
    {
    xsize = size[1]; ysize = size[2];
    }
  else
    {
    xsize = size[0];
    if (size[1] == 1)
      {
      ysize = size[2];
      }
    else
      {
      ysize = size[1];
      if (size[2] != 1)
        {
        vtkErrorMacro(<< "3D texture maps currently are not supported!\n");
        return;
        }
      }
    }

  vtkstd::vector<int> imageDataVec;
  imageDataVec.push_back(xsize);
  imageDataVec.push_back(ysize);
  imageDataVec.push_back(mappedScalars->GetNumberOfComponents());

  totalValues = xsize*ysize;
  txtrData = static_cast<vtkUnsignedCharArray*>(mappedScalars)->
    GetPointer(0);
  for (int i = 0; i < totalValues; i++)
    {
    int result = 0;
    for(int j = 0; j < imageDataVec[2]; j++)
      {
      result = result << 8;
      result += *txtrData;
      txtrData++;
      }
    imageDataVec.push_back(result);
    }




  writer->startNode(ID::PixelTexture);
  writer->setSFImage(ID::image, imageDataVec);
  if (!(aTexture->GetRepeat()))
    {
		writer->setSFBool(ID::repeatS, false);
		writer->setSFBool(ID::repeatT, false);
    }
  writer->endNode();
}
//----------------------------------------------------------------------------
int vtkX3DExporterGeneric::HasHeadLight(vtkRenderer* ren)
{
  // make sure we have a default light
  // if we dont then use a headlight
  vtkLightCollection* lc = ren->GetLights();
  vtkCollectionSimpleIterator lsit;
  vtkLight* aLight=0;
  for (lc->InitTraversal(lsit); (aLight = lc->GetNextLight(lsit)); )
    {
    if (aLight->LightTypeIsHeadlight())
      {
      return 1;
      }
    }
  return 0;
}

static bool vtkX3DExporterWriterUsingCellColors(vtkActor* anActor)
{
  int cellFlag = 0;
  vtkMapper* mapper = anActor->GetMapper();
  vtkAbstractMapper::GetScalars(
    mapper->GetInput(), 
    mapper->GetScalarMode(), 
    mapper->GetArrayAccessMode(),
    mapper->GetArrayId(),
    mapper->GetArrayName(), cellFlag);
  return (cellFlag == 1);
}

//----------------------------------------------------------------------------
static bool vtkX3DExporterWriterRenderFaceSet(
  int cellType,
  int representation,
  vtkPoints* points,
  vtkIdType cellOffset,
  vtkCellArray* cells,
  vtkUnsignedCharArray* colors, bool cell_colors,
  vtkDataArray* normals, bool cell_normals, 
  vtkDataArray* tcoords, 
  bool common_data_written, int index, X3DWriter* writer)
{
  vtkstd::vector<int> coordIndexVector;
  vtkstd::vector<int> cellIndexVector;

  coordIndexVector.reserve(cells->GetNumberOfConnectivityEntries());
  cellIndexVector.reserve(cells->GetNumberOfCells());

  vtkIdType npts = 0;
  vtkIdType *indx = 0;

  if (cellType == VTK_POLYGON || cellType == VTK_POLY_LINE)
    {
    for (cells->InitTraversal(); cells->GetNextCell(npts,indx); cellOffset++)
      {
      for (vtkIdType cc=0; cc < npts; cc++)
        {
        coordIndexVector.push_back(static_cast<int>(indx[cc]));
        }

      if (representation == VTK_WIREFRAME && npts>2 && cellType == VTK_POLYGON)
        {
        // close the polygon.
        coordIndexVector.push_back(static_cast<int>(indx[0]));
        }
      coordIndexVector.push_back(-1);

      vtkIdType cellid = cellOffset;
      cellIndexVector.push_back(cellid);
      }
    }
  else // cellType == VTK_TRIANGLE_STRIP
    {
    for (cells->InitTraversal(); cells->GetNextCell(npts,indx); cellOffset++)
      {
      for (vtkIdType cc=2; cc < npts; cc++)
        {
        vtkIdType i1;
        vtkIdType i2;
        if (cc%2)
          {
          i1 = cc - 1;
          i2 = cc - 2;
          }
        else
          {
          i1 = cc - 2;
          i2 = cc - 1;
          }
        coordIndexVector.push_back(static_cast<int>(indx[i1]));
        coordIndexVector.push_back(static_cast<int>(indx[i2]));
        coordIndexVector.push_back(static_cast<int>(indx[cc]));

        if (representation == VTK_WIREFRAME)
          {
          // close the polygon when drawing lines
            coordIndexVector.push_back(static_cast<int>(indx[i1]));
          }
        coordIndexVector.push_back(-1);

        vtkIdType cellid = cellOffset;
        cellIndexVector.push_back(static_cast<int>(cellid));
        }
      }
    }

  if (representation == VTK_SURFACE)
    {
    writer->startNode(ID::IndexedFaceSet);
    writer->setSFBool(ID::solid, false);
    writer->setSFBool(ID::colorPerVertex, !cell_colors);
    writer->setSFBool(ID::normalPerVertex, !cell_normals);
	  writer->setMFInt32(ID::coordIndex, coordIndexVector);
    }
  else
    {
    // don't save normals/tcoords when saving wireframes.
    normals = 0;
    tcoords = 0;

    writer->startNode(ID::IndexedLineSet);
    writer->setSFBool(ID::colorPerVertex, !cell_colors);
	  writer->setMFInt32(ID::coordIndex, coordIndexVector);
    }

  if (normals && cell_normals && representation == VTK_SURFACE)
    {
		writer->setMFInt32(ID::normalIndex, cellIndexVector);
    }

  if (colors && cell_colors)
    {
		writer->setMFInt32(ID::colorIndex, cellIndexVector);
    }

  // Now save Coordinate, Color, Normal TextureCoordinate nodes.
  // Use DEF/USE to avoid duplicates.
  if (!common_data_written)
    {
    vtkX3DExporterWriteData(points, normals, tcoords, colors, index, writer);
    }
  else
    {
    vtkX3DExporterUseData((normals != NULL), (tcoords != NULL), (colors!= NULL), index, writer);
    }

  writer->endNode(); // end IndexedFaceSet or IndexedLineSet
  return true;
}

static void vtkX3DExporterWriteData(vtkPoints *points, 
  vtkDataArray *normals,
  vtkDataArray *tcoords, 
  vtkUnsignedCharArray *colors,
  int index, 
  X3DWriter* writer)
{
  char indexString[100];
  sprintf(indexString, "%04d", index);

  std::vector<float> vec(points->GetNumberOfPoints()*3);
  vtkDataArray* data = points->GetData();
  double t[3];
  for(int i = 0, j = 0; i < data->GetNumberOfTuples(); i++) {
    data->GetTuple(i, t);
	  vec[j++] = t[0];
    vec[j++] = t[1];
    vec[j++] = t[2];
  }
  // write out the points
  vtkstd::string defString = "VTKcoordinates";
  writer->startNode(ID::Coordinate);
  writer->setSFString(ID::DEF, defString.append(indexString).c_str());
  writer->setMFVec3f(ID::point, vec);
  writer->endNode();
  

  // write out the normals
  if (normals)
    {
    vec.resize(normals->GetNumberOfTuples()*3);
    for(int i = 0, j = 0; i < normals->GetNumberOfTuples(); i++) 
      {
      normals->GetTuple(i, t);
	    vec[j++] = t[0];
      vec[j++] = t[1];
      vec[j++] = t[2];
      }
    defString="VTKnormals";
    writer->startNode(ID::Normal);
    writer->setSFString(ID::DEF, defString.append(indexString).c_str());
    writer->setMFVec3f(ID::vector, vec);
    writer->endNode();
    } // normals
  

  // write out the texture coordinates
  if (tcoords)
    {
    vec.resize(tcoords->GetNumberOfTuples()*2);
		for(int i = 0, j = 0; i < tcoords->GetNumberOfTuples(); i++)
      {
      tcoords->GetTuple(i, t);
	    vec[j++] = t[0];
      vec[j++] = t[1];
      }
    defString="VTKtcoords";
    writer->startNode(ID::TextureCoordinate);
    writer->setSFString(ID::DEF, defString.append(indexString).c_str());
	  writer->setMFVec2f(ID::point, vec);
    writer->endNode();
    }

  // write out the colors
  if (colors)
    {
    unsigned char c[4];
    vec.resize(colors->GetNumberOfTuples()*3);
    for (int i = 0, j = 0; i < colors->GetNumberOfTuples(); i++)
      {
      colors->GetTupleValue(i,c);
      vec[j++] = c[0]/255.0f;
      vec[j++] = c[1]/255.0f;
      vec[j++] = c[2]/255.0f;
      }
    defString="VTKcolors";
    writer->startNode(ID::Color);
    writer->setSFString(ID::DEF, defString.append(indexString).c_str());
    writer->setMFColor(ID::color, vec);
    writer->endNode();
    vec.clear();
    }
}

static void vtkX3DExporterUseData(bool normals, bool tcoords, bool colors, int index, 
  X3DWriter* writer)
{
  char indexString[100];
  sprintf(indexString, "%04d", index);
  vtkstd::string defString = "VTKcoordinates";
  writer->startNode(ID::Coordinate);
  writer->setSFString(ID::USE, defString.append(indexString).c_str());
  writer->endNode();

  // write out the point data
  if (normals)
    {
    defString = "VTKnormals";
    writer->startNode(ID::Normal);
    writer->setSFString(ID::USE, defString.append(indexString).c_str());
    writer->endNode();
    }

  // write out the point data
  if (tcoords)
    {
    defString = "VTKtcoords";
    writer->startNode(ID::TextureCoordinate);
    writer->setSFString(ID::USE, defString.append(indexString).c_str());
    writer->endNode();
    }

  // write out the point data
  if (colors)
    {
    defString = "VTKcolors";
    writer->startNode(ID::Color);
    writer->setSFString(ID::USE, defString.append(indexString).c_str());
    writer->endNode();
    }
}

static bool vtkX3DExporterWriterRenderVerts(
  vtkPoints* points, vtkCellArray* cells,
  vtkUnsignedCharArray* colors, bool cell_colors,  X3DWriter* writer)
{
  vtkstd::vector<float> colorVector;

  if (colors)
    {
    vtkIdType cellId = 0;
    vtkIdType npts = 0;
    vtkIdType *indx = 0;
    for (cells->InitTraversal(); cells->GetNextCell(npts,indx); cellId++)
      {
      for (vtkIdType cc=0; cc < npts; cc++)
        {
        unsigned char color[4];
        if (cell_colors)
          {
          colors->GetTupleValue(cellId, color);
          }
        else
          {
          colors->GetTupleValue(indx[cc], color);
          }

        colorVector.push_back(color[0]/255.0);
        colorVector.push_back(color[1]/255.0);
        colorVector.push_back(color[2]/255.0);
        }
      }
    }

  std::vector<float> vec;

  vec.clear();
  
  double t[3];
  for(int i = 0; i < points->GetData()->GetNumberOfTuples(); i++) {
    points->GetData()->GetTuple(i, t);
	  vec.push_back(t[0]);
	  vec.push_back(t[1]);
	  vec.push_back(t[2]);
  }
  writer->startNode(ID::PointSet);
  writer->startNode(ID::Coordinate);
  writer->setMFVec3f(ID::point, vec);
  writer->endNode();
  if (colors)
    {
    writer->startNode(ID::Color);
	  writer->setMFColor(ID::point, colorVector);
    writer->endNode();
    }
  return true; 
}

static bool vtkX3DExporterWriterRenderPoints(
  vtkPolyData* pd, 
  vtkUnsignedCharArray* colors,
  bool cell_colors
  ,  X3DWriter* writer)
{
  if (pd->GetNumberOfCells() == 0)
    {
    return false;
    }

  vtkstd::vector<float> colorVec;
  vtkstd::vector<float> coordinateVec;

  vtkPoints* points = pd->GetPoints();

  // We render as cells so that even when coloring with cell data, the points
  // are assigned colors correctly.

  if ( (colors !=0) && cell_colors)
    {
    // Cell colors are used, however PointSet element can only have point
    // colors, hence we use this method. Although here we end up with duplicate
    // points, that's exactly what happens in case of OpenGL rendering, so it's
    // okay.
    vtkIdType numCells = pd->GetNumberOfCells();
    vtkSmartPointer<vtkIdList> pointIds = vtkSmartPointer<vtkIdList>::New();
    for (vtkIdType cid =0; cid < numCells; cid++)
      {
      pointIds->Reset();
      pd->GetCellPoints(cid, pointIds);

      // Get the color for this cell.
      unsigned char color[4];
      colors->GetTupleValue(cid, color);
      double dcolor[3];
      dcolor[0] = color[0]/255.0;
      dcolor[1] = color[1]/255.0;
      dcolor[2] = color[2]/255.0;


      for (vtkIdType cc=0; cc < pointIds->GetNumberOfIds(); cc++)
        {
        vtkIdType pid = pointIds->GetId(cc);
        double* point = points->GetPoint(pid);
        coordinateVec.push_back(point[0]);
        coordinateVec.push_back(point[1]);
        coordinateVec.push_back(point[2]);
        colorVec.push_back(dcolor[0]);
        colorVec.push_back(dcolor[1]);
        colorVec.push_back(dcolor[2]);
        }
      }
    }
  else
    {
    // Colors are point colors, simply render all the points and corresponding
    // colors.
    vtkIdType numPoints = points->GetNumberOfPoints();
    for (vtkIdType pid=0; pid < numPoints; pid++)
      {
      double* point = points->GetPoint(pid);
      coordinateVec.push_back(point[0]);
      coordinateVec.push_back(point[1]);
      coordinateVec.push_back(point[2]);

      if (colors)
        {
        unsigned char color[4];
        colors->GetTupleValue(pid, color);
        colorVec.push_back(color[0]/255.0);
        colorVec.push_back(color[1]/255.0);
        colorVec.push_back(color[2]/255.0);
        }
      }
    }

  writer->startNode(ID::PointSet);
  writer->startNode(ID::Coordinate);
  writer->setMFVec3f(ID::point, coordinateVec);
  writer->endNode(); // Coordinate
  if (colors)
    {
    writer->startNode(ID::Color);
	writer->setMFColor(ID::color, colorVec);
    writer->endNode(); // Color
    }
  writer->endNode(); // PointSet
  return true; 
}

