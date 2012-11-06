#include "Argument_helper.h"
#include "X3DTest.h"
#include <iostream>
#include <string>
#include <fstream>


int main(int argc, char *argv[])
{
  dsr::Argument_helper ah;
  ah.set_description("Simple X3D Test");
  ah.set_author("Kristian Sons, kristian.sons@actor3d.com");
  ah.set_build_date(__DATE__);

  ah.process(argc, argv);

  X3DTest(argc, argv);
  
}
