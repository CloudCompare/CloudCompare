/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: X3DTest.cxx,v $

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "X3DTest.h"
#include "vtkActor.h"
#include "vtkConeSource.h"
#include "vtkDebugLeaks.h"
#include "vtkGlyph3D.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkRegressionTestImage.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"

#include "vtkX3DExporter.h"
#include "vtkX3DExporterGeneric.h"


int X3DTest( int argc, char *argv[] )
{
  vtkRenderer *renderer = vtkRenderer::New();
  vtkRenderWindow *renWin = vtkRenderWindow::New();
    renWin->AddRenderer(renderer);
  vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
    iren->SetRenderWindow(renWin);

  vtkSphereSource *sphere = vtkSphereSource::New();
    sphere->SetThetaResolution(8); sphere->SetPhiResolution(8);
  vtkPolyDataMapper *sphereMapper = vtkPolyDataMapper::New();
    sphereMapper->SetInputConnection(sphere->GetOutputPort());
  vtkActor *sphereActor = vtkActor::New();
    sphereActor->SetMapper(sphereMapper);

  vtkConeSource *cone = vtkConeSource::New();
    cone->SetResolution(6);

  vtkGlyph3D *glyph = vtkGlyph3D::New();
    glyph->SetInputConnection(sphere->GetOutputPort());
    glyph->SetSourceConnection(cone->GetOutputPort());
    glyph->SetVectorModeToUseNormal();
    glyph->SetScaleModeToScaleByVector();
    glyph->SetScaleFactor(0.25);

  vtkPolyDataMapper *spikeMapper = vtkPolyDataMapper::New();
    spikeMapper->SetInputConnection(glyph->GetOutputPort());

  vtkActor *spikeActor = vtkActor::New();
    spikeActor->SetMapper(spikeMapper);

  renderer->AddActor(sphereActor);
  renderer->AddActor(spikeActor);
  renderer->SetBackground(1,1,1);
  renWin->SetSize(300,300);

  // interact with data

  renWin->Render();
  
  vtkX3DExporter *exporter = vtkX3DExporter::New();
  exporter->SetInput(renWin);
  exporter->SetFileName("testX3DExporter.x3d");
  exporter->Update();
  exporter->Write();

  exporter->SetBinary(1);
  exporter->SetFileName("testX3DExporterBinary.x3db");
  exporter->Write();

  vtkX3DExporterGeneric *genericexporter = vtkX3DExporterGeneric::New();
  genericexporter->SetInput(renWin);
  genericexporter->SetBinary(0);
  genericexporter->SetFileName("testX3DExporterGeneric.x3d");
  genericexporter->SetFakeFileName("testX3DExporter.x3d");
  genericexporter->Update();
  genericexporter->Write();

  genericexporter->SetBinary(1);
  genericexporter->SetFileName("testX3DExporterBinaryGeneric.x3db");
  genericexporter->SetFakeFileName("testX3DExporterBinary.x3db");
  genericexporter->Write();


  int retVal = vtkRegressionTestImage( renWin );

  if ( retVal == vtkRegressionTester::DO_INTERACTOR)
    {
    iren->Start();
    }
  // Clean up
  exporter->Delete();
  genericexporter->Delete();
  renderer->Delete();
  renWin->Delete();
  iren->Delete();
  sphere->Delete();
  sphereMapper->Delete();
  sphereActor->Delete();
  cone->Delete();
  glyph->Delete();
  spikeMapper->Delete();
  spikeActor->Delete();

  return !retVal;
}

