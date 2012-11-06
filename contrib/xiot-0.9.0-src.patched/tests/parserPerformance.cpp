#include "Argument_helper.h"
#include <iostream>
#include <string>
#include <fstream>
#include <ctime>
#include <xiot/X3DDefaultNodeHandler.h>
#include <xiot/X3DLoader.h>
#include <xiot/X3DTypes.h>
#include <xiot/X3DAttributes.h>


using namespace std;
using namespace XIOT;

string input_filename;
bool no_attributes, no_attribute_values;

class MyContentHandler : public X3DDefaultNodeHandler
{
	void startDocument()
	{
		cout << "Start Document" << endl;
	}

	void endDocument()
	{
		cout << "End Document" << endl;
	}

  int startCoordinate(const X3DAttributes &attr)
    {
    cout << "Handling: Coordinate" << endl;
    if (!no_attribute_values)
       {
    int index = attr.getAttributeIndex(ID::point);
    if (index != -1)
      {
      MFVec3f value;
      attr.getMFVec3f(index, value);
      cout << "point: " << value.size() << endl;
      }
      }
     return CONTINUE;
    }

  int startColor(const X3DAttributes &attr)
    {
    cout << "Handling: Color" << endl;
    if (!no_attribute_values)
       {
    int index = attr.getAttributeIndex(ID::color);
    if (index != -1)
      {
      MFColor value;
      attr.getMFColor(index, value);
      cout << "color: " << value.size() << endl;
      }
      }
     return CONTINUE;
    }

  int startIndexedFaceSet(const X3DAttributes &attr)
    {
    cout << "Handling: IndexedFaceSet" << endl;
    if (!no_attribute_values)
       {
          int index = attr.getAttributeIndex(ID::coordIndex);
          if (index != -1)
            {
            MFInt32 value;
            attr.getMFInt32(index, value);
            cout << "coordIndex: " << value.size() << endl;
            }
          index = attr.getAttributeIndex(ID::normalIndex);
          if (index != -1)
            {
            MFInt32 value;
            attr.getMFInt32(index, value);
            cout << "normalIndex: " << value.size() << endl;
            }
          index = attr.getAttributeIndex(ID::colorIndex);
          if (index != -1)
            {
            MFInt32 value;
            attr.getMFInt32(index, value);
            cout << "colorIndex: " << value.size() << endl;
            }
      }
     return CONTINUE;
    }

  int startIndexedLineSet(const X3DAttributes &attr)
    {
    cout << "Handling: IndexedLineSet" << endl;
    if (!no_attribute_values)
       {
          int index = attr.getAttributeIndex(ID::coordIndex);
          if (index != -1)
            {
            MFInt32 value;
            attr.getMFInt32(index, value);
            cout << "coordIndex: " << value.size() << endl;
            }
          index = attr.getAttributeIndex(ID::colorIndex);
          if (index != -1)
            {
            MFInt32 value;
            attr.getMFInt32(index, value);
            cout << "colorIndex: " << value.size() << endl;
            }
      }
     return CONTINUE;
    }

  int startUnhandled(const char* nodeName, const X3DAttributes &attr)
    {
    cout << "Handling: " << nodeName << endl;
    if (!no_attributes)
    {
      for(size_t i = 0; i < attr.getLength(); i++)
      {
        cout << " -- " <<  attr.getAttributeName(static_cast<int>(i));
        if (!no_attribute_values)
          {
          std::string s = attr.getAttributeValue(static_cast<int>(i));
          s.append("1"); // Prevent form parser optimization
          }
        cout << endl;
      }
    }
    return CONTINUE;
    }
};


int start(const std::string &filename)
{
  X3DLoader l;
  MyContentHandler handler;
  l.setNodeHandler(&handler);
	
  
	try {
  clock_t start,end;
  start = clock();
  for (int i = 0; i < 10; i++)
    {
    l.load(filename.c_str());
    }
  end = clock();
  double dif = double(end-start) / CLOCKS_PER_SEC;
  printf ("Parsing took an average of %f seconds.\n", dif/10.0);

  }
  catch (std::exception& e)
	{
		cerr << endl << "Parsing failed: " << e.what() << endl;
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
  dsr::Argument_helper ah;

  ah.new_string("input_filename", "The name of the input file", input_filename);
  ah.new_flag('a', "skip-attributes", "Do not process attributes", no_attributes);
  ah.new_flag('b', "skip-attributes-values", "Do not process attribute values", no_attribute_values);

  //ARGUMENT_HELPER_BASICS(ah);
  ah.set_description("A simple test application for the parser performance");
  ah.set_author("Kristian Sons, kristian.sons@actor3d.com");
  ah.set_version(0.9f);
  ah.set_build_date(__DATE__);

  ah.process(argc, argv);


  // Check output string
  if (fileExists(input_filename))
  {
	  return start(input_filename);
  }
  
  cerr << "Input file not found or not readable: " << input_filename << endl;
  return 1;
}
