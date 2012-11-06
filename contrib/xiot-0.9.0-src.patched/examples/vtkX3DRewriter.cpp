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
#include "vtkX3DExporter.h"
#include "vtkX3DExporterGeneric.h"

using namespace std;

#define PERFORMANCE_RUNS 15

string input_filename, output_filename;
bool testPerformance = false;
bool useGeneric = false;

int performTest(vtkImporter* importer)
{
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	importer->SetRenderWindow(renWin);
	importer->Read();
	renWin->Delete();
	importer->Delete();
	return 0;
}

void saveVRML(const std::string &output_filename, vtkRenderWindow *renWin)
{
  vtkVRMLExporter* exporter = vtkVRMLExporter::New();
  exporter->SetFileName(output_filename.c_str());
  exporter->SetRenderWindow(renWin);
  exporter->Update();
  exporter->Delete();
}

void saveX3D(const std::string &output_filename, vtkRenderWindow *renWin, bool binary)
{
  if (useGeneric)
    {
    vtkX3DExporterGeneric* exporter = vtkX3DExporterGeneric::New();
    exporter->SetFileName(output_filename.c_str());
    exporter->SetRenderWindow(renWin);
    exporter->SetBinary(binary ? 1 : 0);
    exporter->Update();
    exporter->Delete();
    }
  else
    {
    vtkX3DExporter* exporter = vtkX3DExporter::New();
    exporter->SetFileName(output_filename.c_str());
    exporter->SetRenderWindow(renWin);
    exporter->SetBinary(binary ? 1 : 0);
    exporter->Update();
    exporter->Delete();
    }
}

void loadVRML(const std::string &input_filename, vtkRenderWindow *renWin)
{
	vtkVRMLImporter* importer = vtkVRMLImporter::New();
	importer->SetFileName(input_filename.c_str());
	importer->SetDebug(dsr::VERBOSE ? 1 : 0);
	importer->SetRenderWindow(renWin);
	importer->Read();
  importer->Delete();
}

void loadX3D(const std::string &input_filename, vtkRenderWindow *renWin)
{
	vtkX3DImporter* importer = vtkX3DImporter::New();
	importer->SetFileName(input_filename.c_str());
	importer->SetDebug(dsr::verbose ? 1 : 0);
  importer->SetRenderWindow(renWin);
	importer->Read();
  importer->Delete();
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
	ah.new_string("output_filename", "The name of the output file. Can be either VRML or X3D.", output_filename);
  ah.new_flag('g', "generic", "Use XIOT based exporter instead of old VTK exporer", useGeneric);
	//ARGUMENT_HELPER_BASICS(ah);
	ah.set_description("A test program imports an X3D or VRML file to VTK and exports it again. It is NOT a converter since you will loose information transforming X3D to the VTK structure.");
	ah.set_author("Kristian Sons, kristian.sons@actor3d.com");
	ah.set_version(0.9f);
	ah.set_build_date(__DATE__);

	ah.process(argc, argv);


	// Check output string
	if (fileExists(input_filename))
	{
		string extension = input_filename.substr(input_filename.find_last_of('.') + 1, input_filename.size());
    vtkRenderWindow* renWin = NULL;
   	vtkTimerLog* timer = vtkTimerLog::New();
	  timer->StartTimer();

		if (extension == "x3d" || extension == "x3db")
		{
      renWin = vtkRenderWindow::New();
      renWin->OffScreenRenderingOn();
			loadX3D(input_filename, renWin);
		}
		else if (extension == "vrml" || extension == "wrl")
		{
      renWin = vtkRenderWindow::New();
      renWin->OffScreenRenderingOn();
			loadVRML(input_filename, renWin);
		}
    else 
    {
  		cerr << "Can't detect filetype from input file extension: " << extension << endl;
	  	cerr << "Known extensions: wrl, vrml, x3d, x3db" << extension << endl;
      return 1;
    }
    timer->StopTimer();
   	if (dsr::verbose)
  		cout << "Time to load file: " << timer->GetElapsedTime() << endl;

    
    renWin->Frame();
    renWin->Render();
        
    extension = output_filename.substr(output_filename.find_last_of('.') + 1, output_filename.size());

    timer->StartTimer();
    if (extension == "x3d" || extension == "x3db")
		{
    	saveX3D(output_filename, renWin, extension == "x3db");
		}
		else if (extension == "vrml" || extension == "wrl")
		{
      saveVRML(output_filename, renWin);
		}
    else 
    {
  		cerr << "Can't detect filetype from input file extension: " << extension << endl;
	  	cerr << "Known extensions: wrl, vrml, x3d, x3db" << extension << endl;
      renWin->Delete();
      return 1;
    }
    timer->StopTimer();
   	if (dsr::verbose)
  		cout << "Time to save file: " << timer->GetElapsedTime() << endl;

    
    renWin->Delete();
    return 0;
	}

	cerr << "Input file not found or not readable: " << input_filename << endl;
	return 1;
}
