#include <xiot/X3DTypes.h>

#include <string>
#include <stdio.h>

#define _CRT_SECURE_NO_DEPRECATE // disable MSVS warning concerning unsafe "fopen"

using namespace std;
using namespace XIOT;

void generateX3DSwitch()
{
	FILE* f = fopen("X3DSwitch.inc", "w");
	fprintf(f, "int X3DSwitch::doStartElement(int id, const X3DAttributes& attr) const\n{\n\tassert(_handler);\n\n");
	fprintf(f, "\tswitch(id)\n\t{\n");
	for (int i = 0; i < ID::X3DELEMENT_COUNT; i++)
	{
		std::string elementStr = X3DTypes::getElementByID(i);
		fprintf(f, "\tcase %s:\n", elementStr.c_str());
		fprintf(f, "\t\treturn _handler->start%s(attr);\n", elementStr.c_str());
		//fprintf(f, "\t\tbreak;\n");
	}
	fprintf(f, "\tcase -1:\n\tdefault:\n\t\treturn _handler->startUnknown(attr);\n\t};\n}\n\n");

	fprintf(f, "int X3DSwitch::doEndElement(int id, const char* nodeName) const\n{\n\n");
	fprintf(f, "\tswitch(id)\n\t{\n");
	for (int i = 0; i < ID::X3DELEMENT_COUNT; i++)
	{
		std::string elementStr = X3DTypes::getElementByID(i);
		fprintf(f, "\tcase %s:\n", elementStr.c_str());
		fprintf(f, "\t\treturn _handler->end%s();\n", elementStr.c_str());
		//fprintf(f, "\t\tbreak;\n");
	}
	fprintf(f, "\tcase -1:\n\tdefault:\n\t\treturn _handler->endUnknown(id, nodeName);\n\t};\n}\n\n");

	fclose(f);
}

void generateNodeHandler()
{
	FILE* f = fopen("X3DNodeHandler.h.inc", "w");

	for (int i = 0; i < ID::X3DELEMENT_COUNT; i++)
	{
		std::string elementStr = X3DTypes::getElementByID(i);
		fprintf(f, "  /// Callbacks for %s Nodes\n",elementStr.c_str()); 
		fprintf(f, "  virtual int start%s(const X3DAttributes& attr) = 0;\n", elementStr.c_str());
		fprintf(f, "  virtual int end%s() = 0;\n\n", elementStr.c_str());
	}
	fprintf(f, "  virtual int startUnknown(int id, const char* nodeName, const X3DAttributes& attr) = 0;\n");
	fprintf(f, "  virtual int endUnknown(int id, const char* nodeName) = 0;\n");
	fclose(f);
}

void generateDefaultNodeHandler()
{
	FILE* f = fopen("X3DDefaultNodeHandler.h.inc", "w");
	for (int i = 0; i < ID::X3DELEMENT_COUNT; i++)
	{
		std::string elementStr = X3DTypes::getElementByID(i);
		
		fprintf(f, "  virtual int start%s(const X3DAttributes &attr);\n", elementStr.c_str());
		fprintf(f, "  virtual int end%s();\n\n", elementStr.c_str());
	}
	fprintf(f, "  virtual int startUnknown(int id, const char* nodeName, const X3DAttributes &attr);\n");
	fprintf(f, "  virtual int endUnknown(int id, const char* nodeName);\n");
	fclose(f);

	f = fopen("X3DDefaultNodeHandler.cpp.inc", "w");
	for (int i = 0; i < ID::X3DELEMENT_COUNT; i++)
	{
		std::string elementStr = X3DTypes::getElementByID(i);
		fprintf(f, "int X3DDefaultNodeHandler::start%s(const X3DAttributes &attr) {\n", elementStr.c_str());
		fprintf(f, "  return startUnhandled(\"%s\", attr);\n}\n\n", elementStr.c_str());
		fprintf(f, "int X3DDefaultNodeHandler::end%s() {\n", elementStr.c_str());
		fprintf(f, "  return endUnhandled(\"%s\");\n}\n\n", elementStr.c_str());
	}
	fprintf(f, "int X3DDefaultNodeHandler::startUnknown(int id, const char* nodeName, const X3DAttributes &attr) {\n");
	fprintf(f, "  return startUnhandled(nodeName ? nodeName : \"Unknown\", attr);\n}\n\n");
	fprintf(f, "int X3DDefaultNodeHandler::endUnknown(int id, const char* nodeName) {\n");
	fprintf(f, "  return endUnhandled(nodeName ? nodeName : \"Unknown\");\n}\n\n");

	fprintf(f, "int X3DDefaultNodeHandler::startUnhandled(const char* nodeName, const X3DAttributes &attr) {\n");
	fprintf(f, "  // do nothing in the default implementation\n  return 1;\n}\n\n");
	fprintf(f, "int X3DDefaultNodeHandler::endUnhandled(const char* nodeName) {\n");
	fprintf(f, "  // do nothing in the default implementation\n  return 1;\n}\n\n");
	fclose(f);

}

void generateLogNodeHandler()
{
	FILE* f = fopen("X3DLogNodeHandler.h.inc", "w");
	for (int i = 0; i < ID::X3DELEMENT_COUNT; i++)
	{
		std::string elementStr = X3DTypes::getElementByID(i);
		
		fprintf(f, "  int start%s(const X3DAttributes &attr);\n", elementStr.c_str());
		fprintf(f, "  int end%s();\n\n", elementStr.c_str());
	}
	fprintf(f, "  int startUnknown(const X3DAttributes &attr);\n");
	fprintf(f, "  int endUnknown(int id, const char* nodeName);\n");
	fprintf(f, "  int iCounter;\n");
	fclose(f);

	f = fopen("X3DLogNodeHandler.cpp.inc", "w");
	for (int i = 0; i < ID::X3DELEMENT_COUNT; i++)
	{
		std::string elementStr = X3DTypes::getElementByID(i);
		fprintf(f, "int X3DLogNodeHandler::start%s(const X3DAttributes &attr) {\n", elementStr.c_str());
		fprintf(f, "  fprintf(fp, \"Event %%4i - Start node %s with %%i attribute(s): %%s\\n\", iCounter++, attr.getLength(), attr.getAttributesAsString().c_str());\n", elementStr.c_str());
		fprintf(f, "  return 1;\n}\n\n");
		fprintf(f, "int X3DLogNodeHandler::end%s() {\n", elementStr.c_str());
		fprintf(f, "  fprintf(fp, \"Event %%4i - End node %s\\n\", iCounter++);\n", elementStr.c_str());
		fprintf(f, "  return 1;\n}\n\n", elementStr.c_str());
	}
	fprintf(f, "int X3DLogNodeHandler::startUnknown(const X3DAttributes &attr) {\n");
	fprintf(f, "  fprintf(fp, \"Event %%4i - Start unknown node %%s with %%i attribute(s): %%s\\n\", iCounter++, attr.getNodeName(), attr.getLength(), attr.getAttributesAsString().c_str());\n");
	fprintf(f, "  return 1;\n}\n\n");
	fprintf(f, "int X3DLogNodeHandler::endUnknown(int id, const char* nodeName) {\n");
	fprintf(f, "  fprintf(fp, \"Event %%4i - End unknown node %%s with %%i attribute(s): %%s\\n\", iCounter++, nodeName);\n");
	fprintf(f, "  return 1;\n}\n\n");

	fclose(f);

}



void generateMaps()
{
	FILE* f = fopen("X3DTypes.cpp.inc", "w");

	

	// Generate elementFromIDMap
	for (int i = 0; i < ID::X3DELEMENT_COUNT; i++)
	{
		std::string elementStr = X3DTypes::getElementByID(i);
		
		fprintf(f, "elementFromIDMap.insert(std::pair<int,std::string>(%d, \"%s\"));\n", i, elementStr.c_str());
	}

	fprintf(f, "\n\n");

	// Generate elementFromStringMap
	for (int i = 0; i < ID::X3DELEMENT_COUNT; i++)
	{
		std::string elementStr = X3DTypes::getElementByID(i);
		
		fprintf(f, "elementFromStringMap.insert(std::pair<std::string,int>(\"%s\", %d));\n", elementStr.c_str(), i);
	}

	fprintf(f, "\n\n");

	// Generate attributeFromIDMap
	for (int i = 0; i < ID::X3DATTRIBUTE_COUNT; i++)
	{
		std::string attributeStr = X3DTypes::getAttributeByID(i);
		
		fprintf(f, "attributeFromIDMap.insert(std::pair<int,std::string>(%d, \"%s\"));\n", i, attributeStr.c_str());
	}

	fprintf(f, "\n\n");

	// Generate attributeFromStringMap
	for (int i = 0; i < ID::X3DATTRIBUTE_COUNT; i++)
	{
		std::string attributeStr = X3DTypes::getAttributeByID(i);
		
		fprintf(f, "attributeFromStringMap.insert(std::pair<std::string,int>(\"%s\", %d));\n", attributeStr.c_str(), i);
	}
	
	fclose(f);
}


int main(int argc, char *argv[])
{
	X3DTypes::initMaps();

	generateX3DSwitch();
	generateNodeHandler();
	generateDefaultNodeHandler();
	generateLogNodeHandler();
	//generateMaps();

	return 0;
}
