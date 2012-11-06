#include <xiot/X3DFILoader.h>

#include <xiot/X3DFIAttributes.h>
#include <xiot/FIConstants.h>
#include <xiot/FISAXParser.h>
#include <xiot/FIContentHandler.h>
#include <xiot/FITypes.h>
#include <xiot/X3DParserVocabulary.h>

#include <iostream>
#include <sstream>
#include <cassert>
#include <exception>

namespace XIOT {

class X3DFIContentHandler : public FI::DefaultContentHandler
{
public:
	
	X3DFIContentHandler(X3DNodeHandler* nodeHandler);
	virtual ~X3DFIContentHandler() {};
	
	virtual void startDocument();
	virtual void endDocument();

	virtual void startElement(const FI::ParserVocabulary* vocab, const FI::Element &element, const FI::Attributes &attributes);
	virtual void endElement(const FI::ParserVocabulary* vocab, const FI::Element &element);

private:
  X3DNodeHandler* _nodeHandler;
  X3DSwitch _switch;
  int _skipCount;
};

X3DFIContentHandler::X3DFIContentHandler(X3DNodeHandler* nodeHandler) : _nodeHandler(nodeHandler), _skipCount(0) 
{
	_switch.setNodeHandler(nodeHandler);
}
	
void X3DFIContentHandler::startDocument()
{
	_nodeHandler->startDocument();
}

void X3DFIContentHandler::endDocument()
{
	_nodeHandler->endDocument();
}

void X3DFIContentHandler::startElement(const FI::ParserVocabulary* vocab, const FI::Element &element, const FI::Attributes &attributes)
{
	if (_skipCount != 0)
	{
		_skipCount++;
		return;
	}
	X3DFIAttributes fiAttributes(&attributes, vocab);
	int id = element._qualifiedName._nameSurrogateIndex - 1;
	int state = _switch.doStartElement(id, fiAttributes);
	if (state == XIOT::SKIP_CHILDREN)
		_skipCount = 1;
}

void X3DFIContentHandler::endElement(const FI::ParserVocabulary*, const FI::Element &element)
{
	if (_skipCount != 0)
	{
		_skipCount--;
		return;
	}
	int id = element._qualifiedName._nameSurrogateIndex - 1;
	_switch.doEndElement(id, "");
}


X3DFILoader::X3DFILoader()
{
}

X3DFILoader::~X3DFILoader()
{
	file.close();
}

bool X3DFILoader::load(const char* fileStr, bool )
{
  assert(_handler);
  x3dswitch.setNodeHandler(_handler);

  FI::ContentHandler* handler = new X3DFIContentHandler(_handler);
  FI::SAXParser parser;
  
  std::ifstream fs(fileStr,  std::istream::binary |  std::istream::in);
  
  parser.setStream(&fs);
  parser.setContentHandler(handler);

  FI::ParserVocabulary* vocabulary = new XIOT::X3DParserVocabulary();
  parser.addExternalVocabularies(vocabulary->getExternalVocabularyURI(), vocabulary);

  try {
	parser.parse();
  } catch (std::exception& e)
  {
	std::cerr << std::endl << "Parsing failed: " << e.what() << std::endl;
	delete handler;
	return false;
  }
  delete handler;
  return true;  
}


}
