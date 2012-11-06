#include "ArgumentHelper.h"
#include <iostream>
#include <string>
#include <fstream>
#include "TutorialApplication.h"
#include "X3DLoader.h"
#include "X3DDefaultNodeHandler.h"
#include "X3DAttributes.h"
#include "X3DParseException.h"


using namespace std;

string input_filename;
string output_filename;

class MyNodeHandler : public X3D::X3DDefaultNodeHandler
{
	virtual int startShape(const X3D::X3DAttributes& attr);
	virtual int endShape();
	
	virtual int startMaterial(const X3D::X3DAttributes& attr);
	
	virtual int startUnhandled(const char* nodeName, const X3D::X3DAttributes& attr);
	virtual int endUnhandled(const char* nodeName);
};

int MyNodeHandler::startShape(const X3D::X3DAttributes &attr)
{
	cout << "Start Shape event" << endl;
	return X3D::CONTINUE;
}

int MyNodeHandler::startMaterial(const X3D::X3DAttributes &attr)
{
	cout << "Start Material event" << endl;
	int index;

	index = attr.getAttributeIndex(X3D::diffuseColor);
	if (index != -1)
	{
		X3D::SFColor diffuseColor = attr.getSFColor(index);
		cout << "Diffuse Color is set as: " << diffuseColor.r << " " << diffuseColor.g << " " << diffuseColor.b << endl;
	}
	return X3D::CONTINUE;
}

int MyNodeHandler::endShape()
{
	cout << "End Shape event" << endl;
	return X3D::CONTINUE;
}

int MyNodeHandler::startUnhandled(const char* nodeName, const X3D::X3DAttributes& attr)
{
	cout << "Unhandled Start Node event: " << nodeName << endl;
	return X3D::CONTINUE;
}

int MyNodeHandler::endUnhandled(const char* nodeName)
{
	cout << "Unhandled End Node event: " << nodeName << endl;
	return X3D::CONTINUE;
}

int start(const string &input_filename)
{
	X3D::X3DLoader loader;
	MyNodeHandler handler;
	loader.setNodeHandler(&handler);
	try {
		loader.load(input_filename.c_str());
	} catch (X3D::X3DParseException& e)
	{	
	  cerr << "Error while parsing file " << input_filename << ":" << endl;
      cerr << e.getMessage() << " (Line: " << e.getLineNumber() << ", Column: " << e.getColumnNumber() << ")" << endl;
      return 1;
	}
	return 0;
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
  dsr::ArgumentHelper ah;

  ah.new_string("input_filename.vtk", "The name of the input file", input_filename);
  //ah.new_string("output_filename", "The name of the output file", output_filename);
  
  //ARGUMENT_HELPER_BASICS(ah);
  ah.set_description("A simple test application for the X3DLoader");
  ah.set_author("Kristian Sons, kristian.sons@actor3d.com");
  ah.set_version(0.9f);
  ah.set_build_date(__DATE__);

  ah.process(argc, argv);


  // Check output string
  if (fileExists(input_filename))
  {
	  TutorialApplication app;
	  app.setX3DFile(input_filename.c_str());

	  try {
		  app.go();
	  } catch( Exception& e ) {
		  fprintf(stderr, "An exception has occurred: %s\n",
			  e.getFullDescription().c_str());
	  }

	  return 0;
  }

  cerr << "Input file not found or not readable: " << input_filename << endl;
  return 1;
}
