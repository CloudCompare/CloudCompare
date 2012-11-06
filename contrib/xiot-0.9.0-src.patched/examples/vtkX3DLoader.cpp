#include "Argument_helper.h"
#include <iostream>
#include <string>
#include <fstream>
#include <xiot/X3DLoader.h>
#include <xiot/X3DDefaultNodeHandler.h>
#include <xiot/X3DAttributes.h>
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkVRMLImporter.h"
#include "vtkX3DImporter.h"
#include "vtkLight.h"
#include "vtkTimerLog.h"
#include "vtkVRMLExporter.h"
#include "vtkX3DExporterGeneric.h"
#include "vtkX3DExporter.h"
#include "vtkSmartPointer.h"

using namespace std;


string input_filename, output_filename;
bool no_normals = false, useOldExporter = false;

void exportFile(vtkRenderWindow *renWin)
  {
  vtkExporter* exporter = NULL;
  string extension = output_filename.substr(output_filename.find_last_of('.') + 1, output_filename.size());
  if (extension == "x3d" || extension == "x3db")
    {
    if (useOldExporter)
      {
      vtkX3DExporter* e = vtkX3DExporter::New();
      e->SetBinary(extension == "x3d" ? 0 : 1);
      e->SetFastest(0);
      e->SetFileName(output_filename.c_str());
      exporter = e;
      }
    else
      {
      vtkX3DExporterGeneric* e = vtkX3DExporterGeneric::New();
      e->SetBinary(extension == "x3d" ? 0 : 1);
      e->SetFastest(0);
      e->SetFileName(output_filename.c_str());
      exporter = e;
      }
    }
  else if (extension == "vrml" || extension == "wrl")
    {
    vtkVRMLExporter* e = vtkVRMLExporter::New();
    e->SetFileName(output_filename.c_str());
    exporter = e;
    }
  else
    {
 		cerr << "Can't detect output filename from extension: " << extension << endl;
		cerr << "Known extensions: wrl, vrml, x3d, x3db" << extension << endl;
    }

  exporter->SetRenderWindow(renWin);
  exporter->Write();
  exporter->Delete();

 
  }

int showScene(vtkImporter* importer)
{
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	vtkSmartPointer<vtkTimerLog> timer = vtkSmartPointer<vtkTimerLog>::New();

	renWin->AddRenderer(renderer);
	iren->SetRenderWindow(renWin);

	timer->StartTimer();

	importer->SetRenderWindow(renWin);
	importer->Read();
	timer->StopTimer();

	renWin->SetSize(800,600);
	renWin->Render();

	if (dsr::verbose)
		cout << "Time to load file: " << timer->GetElapsedTime() << endl;

	iren->Start();

  if (!output_filename.empty())
    {
    timer->StartTimer();
    exportFile(renWin);
    timer->StopTimer();
    if (dsr::verbose)
  		cout << "Time to write file: " << timer->GetElapsedTime() << endl;
    }

	importer->Delete();
	return 0;
}



int loadVRML(string input_filename)
{
	vtkVRMLImporter* importer = vtkVRMLImporter::New();
	importer->SetFileName(input_filename.c_str());
	importer->SetDebug(dsr::VERBOSE ? 1 : 0);
	return showScene(importer);
}

int loadX3D(string input_filename)
{
	vtkX3DImporter* importer = vtkX3DImporter::New();
	importer->SetFileName(input_filename.c_str());
  importer->SetDebug(dsr::VERBOSE ? 1 : 0);
  importer->SetCalculateNormals(no_normals ? 0 : 1);
	return showScene(importer);
}


bool fileExists(const std::string& fileName)
{
	std::fstream fin;
	fin.open(fileName.c_str(),std::ios::in);
	if( fin.is_open() )
	{
		fin.close();
		return true;
	}
	fin.close();
	return false;
}

int main(int argc, char *argv[])
{
	dsr::Argument_helper ah;

	ah.new_string("input_filename", "The name of the input file. Can be either VRML or X3D.", input_filename);
  ah.new_flag('n', "no-normalCalculation", "Switch off the automatic calculation of vertex normals", no_normals);
  ah.new_flag('o', "oldExporter", "Use the old non-XIOT vtkX3DImporter instead of XIOT version", useOldExporter);
  ah.new_optional_string("outFile", "Write scene out to given file", output_filename);
	
	//ARGUMENT_HELPER_BASICS(ah);
	ah.set_description("A demonstrator that allows loading of VRML or X3D scenes. It demonstrates the capabilities of the generic X3D loader library.");
	ah.set_author("Kristian Sons, kristian.sons@actor3d.com");
	ah.set_version(0.9f);
	ah.set_build_date(__DATE__);

	ah.process(argc, argv);


	// Check output string
	if (fileExists(input_filename))
	{
		string extension = input_filename.substr(input_filename.find_last_of('.') + 1, input_filename.size());
		if (extension == "x3d" || extension == "x3db")
		{
			return loadX3D(input_filename);
		}
		else if (extension == "vrml" || extension == "wrl")
		{
			return loadVRML(input_filename);
		}

		cerr << "Can't detect filetype from extension: " << extension << endl;
		cerr << "Known extensions: wrl, vrml, x3d, x3db" << extension << endl;
		return 1;
	}

	cerr << "Input file not found or not readable: " << input_filename << endl;
	return 1;
}
