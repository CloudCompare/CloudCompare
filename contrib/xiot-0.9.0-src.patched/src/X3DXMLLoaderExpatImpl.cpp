#include <xiot/X3DXMLLoader.h>
#include <xiot/X3DNodeHandler.h>
#include <xiot/X3DTypes.h>
#include <xiot/X3DSwitch.h>
#include <xiot/X3DXMLAttributes.h>
#include <xiot/X3DParseException.h>

#include XIOT_EXPAT_HEADER 

#include <iostream>
#include <cassert>
#include <fstream>

using namespace std;

namespace XIOT {

class X3DXMLContentHandler {
public:
  X3DXMLContentHandler(X3DNodeHandler* nodeHandler);
  ~X3DXMLContentHandler();

  virtual void startElement(const char* qName, const char** atts);
  virtual void endElement(const char* qName);

  void startDocument();
  void endDocument();

  //void fatalError(const XERCES_CPP_NAMESPACE_QUALIFIER SAXParseException& exception);
  //void error(const XERCES_CPP_NAMESPACE_QUALIFIER SAXParseException& exception);

private:
  X3DNodeHandler* _nodeHandler;
  X3DSwitch _switch;
  int _skipCount;
};

void exp_startElement(void* data, const char* qName, const char** atts)
{
  X3DXMLContentHandler* p = reinterpret_cast<X3DXMLContentHandler*>(data);
  p->startElement(qName, atts);
} // startElement

void exp_endElement(void* data, const char* qName)
{
  X3DXMLContentHandler* p = reinterpret_cast<X3DXMLContentHandler*>(data);
  p->endElement(qName);
} // endElement


class XMLParserImpl 
{     
public:
  XMLParserImpl() : _parser(XML_ParserCreate(NULL)), _handler(NULL) {
	
    XML_SetElementHandler(_parser, exp_startElement, exp_endElement);
	
  };
  ~XMLParserImpl() {
	XML_ParserFree(_parser);
	if(_handler)
		delete _handler;
  }

  void setHandler(X3DXMLContentHandler* handler)
  {
	  X3DXMLContentHandler* oldHandler = _handler;
	  _handler = handler;
	  if(oldHandler)
		  delete oldHandler;
	  XML_SetUserData(_parser, reinterpret_cast<void*>(_handler));
  }

  XML_Parser _parser;
private:
  X3DXMLContentHandler* _handler;
};

X3DXMLContentHandler::X3DXMLContentHandler(X3DNodeHandler* nodeHandler) : _nodeHandler(nodeHandler), _skipCount(0)
{
	_switch.setNodeHandler(nodeHandler);
}

X3DXMLContentHandler::~X3DXMLContentHandler()
{
}

void X3DXMLContentHandler::startDocument()
{
	_nodeHandler->startDocument();
}

void X3DXMLContentHandler::startElement(const char* qName, const char** atts)
{
	if (_skipCount != 0)
	{
		_skipCount++;
		return;
	}
	int id = X3DTypes::getElementID(qName);
	X3DXMLAttributes xmlAttributes(atts);
	int state = _switch.doStartElement(id, xmlAttributes);
	if (state == XIOT::SKIP_CHILDREN)
		_skipCount = 1;

}

void X3DXMLContentHandler::endElement(const char* nodeName)
{
	if (_skipCount != 0)
	{
		_skipCount--;
		return;
	}
	int id = X3DTypes::getElementID(nodeName);
	_switch.doEndElement(id, nodeName);
}

void X3DXMLContentHandler::endDocument()
{
	_nodeHandler->endDocument();
}



X3DXMLLoader::X3DXMLLoader()
{
  _impl = new XMLParserImpl();
}

X3DXMLLoader::~X3DXMLLoader()
{
  delete _impl;
}

bool X3DXMLLoader::load(const char* fileStr, bool ) const
{
  static const int BUFF_SIZE = 2*1024*1024;
	std::fstream fin;
  
  assert(_handler);
  _impl->setHandler(new X3DXMLContentHandler(_handler));
 
  fin.open(fileStr,std::ios::in);
  
  if( !fin.is_open() )
  {
	  cerr << "Could not open file: " << fileStr << endl;
    return false;
  }

  _handler->startDocument();
  while(!fin.eof())
  {
    char* buffer = (char*)XML_GetBuffer(_impl->_parser, BUFF_SIZE);
    if(buffer == NULL)
    {
      cerr << "Could not acquire expat buffer" << endl;
      return false;
    } 

    fin.read(buffer, BUFF_SIZE);
    if(!XML_ParseBuffer(_impl->_parser, static_cast<int>(fin.gcount()), fin.eof()))
    {
      // error
      cerr << XML_ErrorString(XML_GetErrorCode(_impl->_parser)) << endl;
      return false;
    } 
  } 
  _handler->endDocument();
  return true;
}



}
