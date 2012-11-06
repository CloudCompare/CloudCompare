#include <iostream>
#include <string>
#include <fstream>
#include "TutorialApplication.h"


using namespace std;

int main(int argc, char *argv[])
{
  TutorialApplication app;

  try {
	  app.go();
  } catch( Exception& e ) {
	  fprintf(stderr, "An exception has occurred: %s\n",
		  e.getFullDescription().c_str());
  }
  return 0;
}
