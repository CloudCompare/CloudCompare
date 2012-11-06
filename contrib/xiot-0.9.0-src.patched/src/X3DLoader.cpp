#include <xiot/X3DLoader.h>
#include <xiot/X3DXMLLoader.h>
#include <xiot/X3DFILoader.h>
#include <xiot/X3DParseException.h>
#include <xiot/X3DTypes.h>

#include <iostream>
#include <cassert>

using namespace std;

namespace XIOT {

X3DLoader::X3DLoader()
: _handler(NULL)
{
}

X3DLoader::~X3DLoader()
{
}

bool X3DLoader::load(const char* fileStr, bool fileValidation) const
{
	assert(_handler);
	X3DTypes::initMaps();
  std::string fileName(fileStr);
	std::string extensionStr = fileName.substr(fileName.find_last_of('.') + 1, fileName.size());
	if (extensionStr == "x3d")
	{
		X3DXMLLoader xmlLoader;
		xmlLoader.setNodeHandler(_handler);
		return xmlLoader.load(fileStr, fileValidation);
	}
	else if  (extensionStr == "x3db")
	{
		X3DFILoader fiLoader;
		fiLoader.setNodeHandler(_handler);
		return fiLoader.load(fileStr, fileValidation);
	}
	return false;
}

void X3DLoader::setNodeHandler(X3DNodeHandler *handler)
{
	this->_handler = handler;
}


}
