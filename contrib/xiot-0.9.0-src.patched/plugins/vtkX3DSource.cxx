#include "vtkX3DSource.h"

#include "vtkX3DImporter.h"
#include "vtkActor.h"
#include "vtkActorCollection.h"
#include "vtkAppendPolyData.h"
#include "vtkCellData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkMultiBlockDataSet.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkUnsignedCharArray.h"


vtkCxxRevisionMacro(vtkX3DSource, "$Revision: 1.11 $");
vtkStandardNewMacro(vtkX3DSource);

//------------------------------------------------------------------------------
vtkX3DSource::vtkX3DSource()
{
  this->FileName = NULL;
  this->Importer = NULL;
  this->Color = 1;
  this->Append = 1;

  this->SetNumberOfInputPorts(0);
}

//------------------------------------------------------------------------------
vtkX3DSource::~vtkX3DSource()
{
  this->SetFileName(NULL);
  if (this->Importer)
    {
    this->Importer->Delete();
    this->Importer = NULL;
    }
}

//------------------------------------------------------------------------------
int vtkX3DSource::RequestData(
  vtkInformation*, vtkInformationVector**, vtkInformationVector* outputVector)
{
  vtkInformation* info = outputVector->GetInformationObject(0);

  vtkDataObject* doOutput = info->Get(vtkDataObject::DATA_OBJECT());
  vtkMultiBlockDataSet* output = vtkMultiBlockDataSet::SafeDownCast(doOutput);
  if (!output)
    {
    return 0;
    }

  if (this->Importer == NULL)
    {
    this->InitializeImporter();
    }
  this->CopyImporterToOutputs(output);

  return 1;
}

//------------------------------------------------------------------------------
void vtkX3DSource::InitializeImporter()
{
  if (this->Importer)
    {
    this->Importer->Delete();
    this->Importer = NULL;
    }
  this->Importer = vtkX3DImporter::New();
  this->Importer->SetFileName(this->FileName);
  this->Importer->Read();
}

//------------------------------------------------------------------------------
void vtkX3DSource::CopyImporterToOutputs(vtkMultiBlockDataSet* mbOutput)
{
  vtkRenderer* ren;
  vtkActorCollection* actors;
  vtkActor* actor;
  vtkPolyDataMapper* mapper;
  vtkPolyData* input;
  vtkPolyData* output;
  int idx;
  int numArrays, arrayIdx, numPoints, numCells;
  vtkDataArray* array;
  int arrayCount = 0;
  char name[256];
  int ptIdx;
  vtkAppendPolyData* append = NULL;

  if (this->Importer == NULL)
    {
    return;
    }

  if (this->Append)
    {
    append = vtkAppendPolyData::New();
    }

  ren = this->Importer->GetRenderer();
  actors = ren->GetActors();
  actors->InitTraversal();
  idx = 0;
  while ( (actor = actors->GetNextActor()) )
    {
    mapper = vtkPolyDataMapper::SafeDownCast(actor->GetMapper());
    if (mapper)
      {
      input = mapper->GetInput();
      input->Update();

      output = vtkPolyData::New();

      if (!append)
        {
        mbOutput->SetBlock(idx, output);
        }

      vtkTransformPolyDataFilter *tf = vtkTransformPolyDataFilter::New();
      vtkTransform *trans = vtkTransform::New();
      tf->SetInput(input);
      tf->SetTransform(trans);
      trans->SetMatrix(actor->GetMatrix());
      input = tf->GetOutput();
      input->Update();

      output->CopyStructure(input);
      // Only copy well formed arrays.
      numPoints = input->GetNumberOfPoints();
      numArrays = input->GetPointData()->GetNumberOfArrays();
      for ( arrayIdx = 0; arrayIdx < numArrays; ++arrayIdx)
        {
        array = input->GetPointData()->GetArray(arrayIdx);
        if (array->GetNumberOfTuples() == numPoints)
          {
          if (array->GetName() == NULL)
            {
            sprintf(name, "X3DArray%d", ++arrayCount);
            array->SetName(name);
            }
          output->GetPointData()->AddArray(array);
          } 
        }
      // Only copy well formed arrays.
      numCells = input->GetNumberOfCells();
      numArrays = input->GetCellData()->GetNumberOfArrays();
      for ( arrayIdx = 0; arrayIdx < numArrays; ++arrayIdx)
        {
        array = input->GetCellData()->GetArray(arrayIdx);
        if (array->GetNumberOfTuples() == numCells)
          {
          if (array->GetName() == NULL)
            {
            sprintf(name, "X3DArray%d", ++arrayCount);
            array->SetName(name);
            }
          output->GetCellData()->AddArray(array);
          } 
        }
      if (this->Color)
        {
        vtkUnsignedCharArray *colorArray = vtkUnsignedCharArray::New();
        unsigned char r, g, b;
        double* actorColor;
     
        actorColor = actor->GetProperty()->GetColor();
        r = static_cast<unsigned char>(actorColor[0]*255.0);
        g = static_cast<unsigned char>(actorColor[1]*255.0);
        b = static_cast<unsigned char>(actorColor[2]*255.0);
        colorArray->SetName("X3DColor");
        colorArray->SetNumberOfComponents(3);
        for (ptIdx = 0; ptIdx < numPoints; ++ptIdx)
          {
          colorArray->InsertNextValue(r);
          colorArray->InsertNextValue(g);
          colorArray->InsertNextValue(b);
          }
        output->GetPointData()->SetScalars(colorArray);
        colorArray->Delete();
        colorArray = NULL;
        }
      if (append)
        {
        append->AddInput(output);
        }
      output->Delete();
      output = NULL;

      ++idx;
      tf->Delete();
      tf = NULL;
      trans->Delete();
      trans = NULL;
      }
    }

  if (append)
    {
    append->Update();
    vtkPolyData* newOutput = vtkPolyData::New();
    newOutput->ShallowCopy(append->GetOutput());
    mbOutput->SetBlock(0, newOutput);
    newOutput->Delete();
    append->Delete();
    }
}



//------------------------------------------------------------------------------
void vtkX3DSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  if (this->FileName)
    {
    os << indent << "FileName: " << this->FileName << endl;
    }
  os << indent << "Color: " << this->Color << endl;
  os << indent << "Append: " << this->Append << endl;
}

