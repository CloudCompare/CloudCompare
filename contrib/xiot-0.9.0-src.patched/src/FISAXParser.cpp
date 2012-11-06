#include <xiot/FISAXParser.h>
#include <xiot/FIConstants.h>
#include <xiot/FIParserVocabulary.h>
#include <xiot/FIContentHandler.h>

namespace FI {


SAXParser::~SAXParser()
{
}

void SAXParser::setContentHandler(ContentHandler* handler)
{
	_contentHandler = handler;
}

void SAXParser::parse()
{
	_terminated = _doubleTerminated = false;
	if (!detectFIDocument())
		throw std::runtime_error("Input is not a Fast Infoset document.");
	processDocument();
}

void SAXParser::processDocument()
{
	_contentHandler->startDocument();
	processDocumentProperties();
	
	// Process children
	while(!_terminated)
	{
		_b = static_cast<unsigned char>(_stream->get());
		if(!checkBit(_b, 1)) { // 0 padding announcing element
			processElement();
		}
	}

	_contentHandler->endDocument();
}

void SAXParser::processElement()
{
	Element element;
	// C.3.3 If the optional component attributes is present, then the bit '1' (presence) is appended to the bit stream;
	// otherwise, the bit '0' (absence) is appended.
	bool hasAttributes = checkBit(_b, 2) != 0;
	
	// C.3.4 If the optional component namespace-attributes is present, it is encoded as described in the three
	// following subclauses.
	
	// C.3.4.1 The four bits '1110' (presence) and the two bits '00' (padding) are appended to the bit stream.
	// check for namespace attributes
	if((_b & Constants::ELEMENT_NAMESPACE_ATTRIBUTES_MASK) == Constants::ELEMENT_NAMESPACE_ATTRIBUTES_FLAG)
	{
		throw std::runtime_error("No namespace support yet");
	}

	// C.3.5 The value of the component qualified-name is encoded as described in C.18.
	getQualifiedNameOrIndex3(element._qualifiedName);
	
	if(hasAttributes) 
		processAttributes();

	_contentHandler->startElement(_vocab, element, _attributes);
	_attributes.clear();

	while(!_terminated) {
		_b = static_cast<unsigned char>(_stream->get());
		if(!checkBit(_b, 1)) { // 0 padding announcing element
			processElement();
		}
		else if((_b & Constants::TWO_BITS) == Constants::ELEMENT_CHARACTER_CHUNK)
		{
			processCharacterChunk();
		}
		else if (_b == Constants::TERMINATOR_SINGLE ||
				 _b == Constants::TERMINATOR_DOUBLE)
		{
			_terminated = true;
			_doubleTerminated = _b == Constants::TERMINATOR_DOUBLE;
		}
		else
			throw std::runtime_error("message.decodingEIIs");
	}
	_contentHandler->endElement(_vocab, element);
	
	_terminated = _doubleTerminated;
	_doubleTerminated = false;
}

void SAXParser::processAttributes()
{
	do {
		_b = static_cast<unsigned char>(_stream->get());
		if (!checkBit(_b,1))
		{
			FI::Attribute attribute;
			getAttribute(attribute);
			_attributes.push_back(attribute);
		}
		else if (_b == Constants::TERMINATOR_SINGLE ||
				 _b == Constants::TERMINATOR_DOUBLE)
		{
			_terminated = true;
			_doubleTerminated = _b == Constants::TERMINATOR_DOUBLE;
		}
		else
			throw std::runtime_error("message.decodingAIIs");

	} while(!_terminated);

	_terminated = _doubleTerminated;
	_doubleTerminated = false;
}

void SAXParser::processCharacterChunk()
{
	FI::CharacterChunk chunk;
	getCharacterChunk(chunk);
	_contentHandler->characters(_vocab, chunk);
}


} // namespace FI
