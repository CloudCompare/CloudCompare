#include "Argument_helper.h"
#include <iostream>
#include <string>
#include <fstream>
#include <xiot/X3DLoader.h>
#include <xiot/X3DAttributes.h>
#include "X3DLogNodeHandler.h"


using namespace std;

std::string input_filename;
std::string output_filename;

int start(const std::string &input_filename, const std::string &output_filename)
{
	XIOT::X3DLoader loader;
	X3DTypes::initMaps();
	X3DLogNodeHandler* handler = new X3DLogNodeHandler(input_filename);
	loader.setNodeHandler(handler);
  loader.load(input_filename.c_str());
	
	delete handler;
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
  dsr::Argument_helper ah;

  ah.new_string("input_filename.vtk", "The name of the input file", input_filename);
  //ah.new_string("output_filename", "The name of the output file", output_filename);
  
  //ARGUMENT_HELPER_BASICS(ah);
  ah.set_description("An eventLog creation tool.");
  ah.set_author("Kristian Sons, kristian.sons@actor3d.com");
  ah.set_version(0.9f);
  ah.set_build_date(__DATE__);

  ah.process(argc, argv);


  // Check output string
  if (fileExists(input_filename))
  {
	  return start(input_filename, output_filename);
  }
  
  cerr << "Input file not found or not readable: " << input_filename << endl;
  return 1;
}
