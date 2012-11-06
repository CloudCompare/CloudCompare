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

  ah.new_string("input_filename.x3d[b]", "The name of the input file", input_filename);
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
