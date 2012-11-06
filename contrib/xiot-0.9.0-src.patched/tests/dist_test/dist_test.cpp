#include <iostream>
#include <string>
#include <fstream>
#include <cassert>
#include <xiot/X3DLoader.h>
#include <xiot/X3DDefaultNodeHandler.h>
#include <xiot/X3DAttributes.h>
#include <xiot/X3DParseException.h>
#include <xiot/X3DWriterFI.h>
#include <xiot/X3DWriterXML.h>


using namespace std;
using namespace XIOT;

string input_filename;
string output_filename;

class MyNodeHandler : public X3DDefaultNodeHandler
{
public:
	MyNodeHandler() {
		_eventCount = 0;
	}

	void testEvent(int number, const char* eventName)
	{
		_eventCount++;
		if (_eventCount != number)
		{
			cerr << eventName<< ": Not the expected event order. Expected: " << number << " is: " << _eventCount << endl;
			assert(false);
		}
		cout << "Event number " << number << ": " << eventName << endl;
	}

	virtual int startX3D(const X3DAttributes &attr)
	{
		testEvent(1, "start X3D");
		return CONTINUE;		
	}

	virtual int startShape(const X3DAttributes& attr)
	{
		testEvent(9, "start Shape");
		return CONTINUE;		
	}

	virtual int endShape()
	{
		testEvent(16, "end Shape");
		return CONTINUE;		
	}

	virtual int startBox(const X3DAttributes& attr)
	{
		testEvent(14, "start Box");
		int index = attr.getAttributeIndex(ID::size);
		assert(index != -1);
		
		return CONTINUE;		
	}
	
	virtual int endBox()
	{
		testEvent(15, "end Box");
		return CONTINUE;		
	}
	
	virtual int startMaterial(const X3DAttributes& attr)
	{
		testEvent(11, "start Material");
		int index = attr.getAttributeIndex(ID::diffuseColor);
		assert(index != -1);
		
		SFColor diffuseColor;
		attr.getSFColor(index, diffuseColor);
		assert(diffuseColor.r == 1.0);
		assert(diffuseColor.g == 0.0);
		assert(diffuseColor.b == 0.0);
		cout << "Diffuse Color is: " << diffuseColor.r << " " << diffuseColor.g << " " << diffuseColor.b << endl;
		
		index = attr.getAttributeIndex(ID::transparency);
		assert(index != -1);
		float transparency = attr.getSFFloat(index);
		assert(transparency == 0.1f);
		
		return CONTINUE;		
	}

	virtual int startUnhandled(const char* nodeName, const X3DAttributes& attr)
	{
		cout << "Event number " << ++_eventCount << ": unhandled start event " << nodeName << endl;
		return CONTINUE;
	}

	virtual int endUnhandled(const char* nodeName)
	{
		cout << "Event number " << ++_eventCount << ": unhandled end event " << nodeName << endl;
		return CONTINUE;
	}


	int _eventCount;
};





int start()
{
	X3DWriter* writer[2];
	writer[0] = new X3DWriterXML();
	writer[1] = new X3DWriterFI();

	for(int i = 0; i< 2; i++)
	{
		X3DWriter* w = writer[i];
		if (i == 0)
			w->openFile("iotest.x3d");
		else 
			w->openFile("iotest.x3db");
		w->startX3DDocument();
		
		w->startNode(ID::Shape);
		w->startNode(ID::Appearance);
		w->startNode(ID::Material);
		w->setSFVec3f(ID::diffuseColor, 1.0f, 0.0f, 0.0f);
		w->setSFFloat(ID::transparency, 0.1f);
		w->endNode();
		w->endNode(); // Appearance
		w->startNode(ID::Box);
		w->setSFVec3f(ID::size, 0.5f, 0.5f, 0.5f);
		w->endNode(); // Box
		w->endNode();//Shape
		w->endX3DDocument();
		w->closeFile();

		delete w;
	}

	
	for(int i = 0; i< 2; i++)
	{
		X3DLoader loader;
		MyNodeHandler handler;
		loader.setNodeHandler(&handler);
		try {
			if (i == 0)
				loader.load("iotest.x3d");
			else
				loader.load("iotest.x3db");
		} catch (X3DParseException& e)
		{	
		  cerr << "Error while parsing file:" << endl;
		  cerr << e.getMessage() << " (Line: " << e.getLineNumber() << ", Column: " << e.getColumnNumber() << ")" << endl;
		  return 1;
		}
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
  // Check output string
  return start();
}
