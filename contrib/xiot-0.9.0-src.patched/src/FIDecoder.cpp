#include <xiot/FIDecoder.h>

#include <xiot/X3DFIAttributes.h>
#include <xiot/FIConstants.h>
#include <xiot/FIParserVocabulary.h>

#include <iostream>
#include <sstream>
#include <cassert>

namespace FI {

Decoder::Decoder()
{
	_vocab = new DefaultParserVocabulary();
}


Decoder::~Decoder()
{
	if (_vocab != NULL)
	{
		delete _vocab;
	}
}

void Decoder::addExternalVocabularies(const std::string &name, ParserVocabulary* parserVocabulary)
{
	_externalVocabularies[name] = parserVocabulary;
}


void Decoder::setStream(std::istream* stream)
{
	_stream = stream;
}



// C.1
bool Decoder::detectFIDocument()
{
	_b = static_cast<unsigned char>(_stream->get());
	// C.1.3 A fast infoset document may begin either with an XML declaration (see 12.3) followed by:
	
	// a) the sixteen bits '1110000000000000' (identification); followed by
	if(_b != Constants::IDENT1) // equals 11100000
		return false;
	
	_b = static_cast<unsigned char>(_stream->get());
	if(_b != Constants::IDENT2) // equals 00000000
		return false;

	// b) the sixteen bits '0000000000000001' (version number); followed by
	_b = static_cast<unsigned char>(_stream->get());
	if(_b != Constants::VERSION1) // equals 00000000
		return false;

	_b = static_cast<unsigned char>(_stream->get());
	if(_b != Constants::VERSION2) // equals 00000001
		return false;

	// c) the bit '0' (padding)
	_b = static_cast<unsigned char>(_stream->get());
	return checkBit(_b, 1) == 0;
}

// C.2 Encoding of the Document type
void Decoder::processDocumentProperties()
{
	unsigned char tableBits = _b;
	// C.2.3 For each of the seven optional components additional-data, initial-vocabulary, notations,
	// unparsed-entities, character-encoding-scheme, standalone, and version (in this order), if the component
	// is present, then the bit '1' (presence) is appended to the bit stream; otherwise, the bit '0' (absence) is appended.
	if(checkBit(tableBits, 2))
		decodeAdditionalData();
	if(checkBit(tableBits, 3))
		decodeInitialVocabulary();
	if(checkBit(tableBits, 4))
		decodeNotations();
	if(checkBit(tableBits, 5))
		decodeUnparsedEntities();
	if(checkBit(tableBits, 6))
		decodeCharacterEncodingScheme();
	if(checkBit(tableBits, 7))
		decodeStandalone();
	if(checkBit(tableBits, 8))
		decodeVersion();
}

void Decoder::getDocument(FI::Document &)
{
	processDocumentProperties();
	 // read Children
	if(!readChildren())
	  std::cerr << "Children not valid!\n";

}

bool Decoder::readChildren()
{
	
	do {
		_b = static_cast<unsigned char>(_stream->get());
		if(!checkBit(_b, 1)) { // 0 padding announcing element
			Element element;
			getElement(element);
		}
	} while(_b != 0xF0 && _b != 0xFF);

	//// children could also be 
	//processing-instruction ProcessingInstruction,
	//comment Comment,
	//document-type-declaration DocumentTypeDeclaration

	return true;
}


void Decoder::getCharacterChunk(FI::CharacterChunk& chunk)
{
	getNonIdentifyingStringOrIndex3(chunk.characterCodes);
	if (chunk.characterCodes._addToTable)
		_vocab->addCharacterChunk(_vocab->resolveCharacterChunk(chunk.characterCodes));

}

bool Decoder::readAttributes(FI::Element& element)
{
	if(_b == 0xF0)
	{
		_stream->get();
		return false; // Termination detected (no more attributes)
	}
	else if(_b == 0xFF)
	{
		return false;
	}

	if(checkBit(_b, 1))
		return false; // '0' padding expected

	FI::Attribute attribute;
	getAttribute(attribute);

	// add attribute to list	
	element._attributes.push_back(attribute);
	return true;
}

void Decoder::getElement(FI::Element &element)
{
	if((_b & Constants::FOUR_BITS) == 0xF0)
		return; // Termination detected

	if(checkBit(_b, 1))  
		return; // expect '0' padding

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
		readAttributes(element);
		
	//std::cout << "startElement: " << X3DTypes::getElementByID(id) << " -> Attributes: " << attr.getAttributesAsString() << std::endl;

	//_processor->processElementStart(element);

	do {
		_b = static_cast<unsigned char>(_stream->get());

		if(!checkBit(_b, 1)) // 0 padding announcing element
		{
			Element element;
			getElement(element);
		}
		else if((_b & Constants::TWO_BITS) == Constants::ELEMENT_CHARACTER_CHUNK)
		{
			FI::CharacterChunk chunk;
			getCharacterChunk(chunk);
		}
	} while(_b != 0xF0 && _b != 0xFF);

	//_processor->processElementEnd(element);

	//std::cout << "endElement: " << X3DTypes::getElementByID(id) << std::endl;

	if((_b == 0xF0) || ((_b == 0xFF) && hasAttributes)) // Termination handling
		_stream->get();

}

// C.4
void Decoder::getAttribute(FI::Attribute &attribute)
{
	//C.4.3 The value of qualified-name is encoded as described in C.17.
	getQualifiedNameOrIndex2(attribute._qualifiedName);
	
	// NOTE – C.17 always ends on the eighth bit of the same or another octet.
	_b = static_cast<unsigned char>(_stream->get()); // Get next byte
	
	//C.4.4 The value of normalized-value is encoded as described in C.14.
	getNonIdentifyingStringOrIndex1(attribute._normalizedValue);

	if (attribute._normalizedValue._addToTable)
		_vocab->addAttributeValue(_vocab->resolveAttributeValue(attribute._normalizedValue));
}

// C.17: Check whether we've got a literal QNAME or surrogate
void Decoder::getQualifiedNameOrIndex2(FI::QualifiedNameOrIndex& name)
{
	// C.17.3 If the alternative literal-qualified-name is present, then the four bits '1111' (identification) and the bit
	// '0' (padding) are appended to the bit stream
	if((_b & Constants::ATTRIBUTE_LITERAL_QNAME_FLAG) == Constants::ATTRIBUTE_LITERAL_QNAME_FLAG)
	{
		// C.17.3.1 For each of the optional components prefix and namespace-name (in this order), if the component is
		// present, then the bit '1' (presence) is appended to the bit stream; otherwise, the bit '0' (absence) is appended
		bool isPrefixPresent = checkBit(_b, 7) != 0;
		bool isNamespaceNamePresent = checkBit(_b, 8) != 0;
		
		_b = static_cast<unsigned char>(_stream->get()); // next byte

		// C.17.3.2 If the optional component prefix is present, it is encoded as described in C.13.
		if(isPrefixPresent)
			getIdentifyingStringOrIndex(name._prefix);

		// C.17.3.3 If the optional component namespace-name is present, it is encoded as described in C.13.
		if(isNamespaceNamePresent)
			getIdentifyingStringOrIndex(name._namespaceName);

		// C.17.3.4 The component local-name is encoded as described in C.13.
		getIdentifyingStringOrIndex(name._localName);
	}
	else 
	{
		// C.17.4 If the alternative name-surrogate-index is present, it is encoded as described in C.25.
		name._nameSurrogateIndex = getInteger2(); 
	}
}

// C.18: Check whether we've got a literal QNAME or surrogate
void Decoder::getQualifiedNameOrIndex3(FI::QualifiedNameOrIndex& name)
{
	// C.18.3 If the alternative literal-qualified-name is present, then the four bits '1111' (identification) are
	// appended to the bit stream
	if((_b & Constants::ELEMENT_LITERAL_QNAME_FLAG) == Constants::ELEMENT_LITERAL_QNAME_FLAG)
	{
		// C.18.3.1 For each of the optional components prefix and namespace-name (in this order), if the component is
		// present, then the bit '1' (presence) is appended to the bit stream; otherwise, the bit '0' (absence) is appended.
		bool isPrefixPresent = checkBit(_b, 7) != 0;
		bool isNamespaceNamePresent = checkBit(_b, 8) != 0;

		_b = static_cast<unsigned char>(_stream->get()); // next byte
		
		// C.18.3.2 If the optional component prefix is present, it is encoded as described in C.13.
		if(isPrefixPresent)
			getIdentifyingStringOrIndex(name._prefix);
		
		// C.18.3.3 If the optional component namespace-name is present, it is encoded as described in C.13.
		if(isNamespaceNamePresent)
			getIdentifyingStringOrIndex(name._namespaceName);
		// C.18.3.4 The component local-name is encoded as described in C.13.
		getIdentifyingStringOrIndex(name._localName);
	}
	else
	{
		// C.18.4 If the alternative name-surrogate-index is present, it is encoded as described in C.27.
		name._nameSurrogateIndex = getInteger3(); 
	}
}

// C.13
void Decoder::getIdentifyingStringOrIndex(FI::IdentifyingStringOrIndex &value)
{
	// C.13.3 If the alternative literal-character-string is present, then the bit '0' (discriminant) is appended to the
	// bit stream, and the literal-character-string is encoded as described in C.22.
	if(!checkBit(_b, 1))
	{
		// C.22
		getNonEmptyOctetString2(value._literalCharacterString);
	}
	// C.13.4 If the alternative string-index is present, then the bit '1' (discriminant) is appended to the bit stream, and
	// the string-index is encoded as described in C.25
	else
	{
		value._stringIndex = getInteger2();		
	}

}

// C.14
void Decoder::getNonIdentifyingStringOrIndex1(FI::NonIdentifyingStringOrIndex& value)
{
	//C.14.3 If the alternative literal-character-string is present, then the bit '0' (discriminant) is appended to the
	//bit stream, and the literal-character-string is encoded as described in the two following subclauses.
	if(!checkBit(_b, 1)) {
		// C.14.3.1 If the value of the component add-to-table is TRUE, then the bit '1' is appended to the bit stream;
		// otherwise, the bit '0' is appended.
		value._addToTable = checkBit(_b, 2) != 0;
		// C.14.3.2 The value of the component character-string is encoded as described in C.19.
		getEncodedCharacterString3(value._characterString);
	}
	else
	{
		// If the alternative string-index is present, then the bit '1' (discriminant) is appended to the bit stream, and
		// the string-index is encoded as described in C.26
		value._stringIndex = getInteger2();
	}
}

// C.15 
void Decoder::getNonIdentifyingStringOrIndex3(FI::NonIdentifyingStringOrIndex& value)
{
	// C.15.3 If the alternative literal-character-string is present, then the bit '0' (discriminant) is appended to the
	// bit stream, and the literal-character-string is encoded as described in the two following subclauses.
	if(!checkBit(_b, 3)) {
		// C.15.3.1 If the value of the component add-to-table is TRUE, then the bit '1' is appended to the bit stream, otherwise
		// the bit '0' is appended.
		value._addToTable = checkBit(_b,4) != 0;
		// C.15.3.2 The value of the component character-string is encoded as described in C.20.
		getEncodedCharacterString5(value._characterString);
	}
	else
	{
		// C.15.4 If the alternative string-index is present, then the bit '1' (discriminant) is appended to the bit stream, and
		// the string-index is encoded as described in C.28.
		value._stringIndex = getInteger4();
	}
}

// C.19
void Decoder::getEncodedCharacterString3(FI::EncodedCharacterString& value)
{
	// C.19.3 The value of the component encoding-format is encoded as described in the four following subclauses.
	switch(_b & Constants::ENCODED_CHARACTER_STRING_3RD_MASK)
	{
	// C.19.3.1 If the alternative utf-8 is present, then the two bits '00' (discriminant)
	// are appended to the bit stream.
	case Constants::ENCODED_CHARACTER_STRING_3RD_UTF8:
		value._encodingFormat = FI::ENCODINGFORMAT_UTF8;
		break;
	// C.19.3.2 If the alternative utf-16 is present, then the two bits '01' 
	// (discriminant) are appended to the bit stream.
	case Constants::ENCODED_CHARACTER_STRING_3RD_UTF16:
		value._encodingFormat = FI::ENCODINGFORMAT_UTF16;
		break;
	// C.19.3.3 If the alternative restricted-alphabet is present, then the two bits '10' 
	// (discriminant) are appended to the bit stream, and the restricted-alphabet is encoded as described in C.29.
	case Constants::ENCODED_CHARACTER_STRING_3RD_RESTRICTED_ALPHABET:
		value._encodingFormat = FI::ENCODINGFORMAT_RESTRICTED_ALPHABET;
		value._restrictedAlphabet = getSmallInteger5();
		break;
	case Constants::ENCODED_CHARACTER_STRING_3RD_ENCODING_ALGORITHM:
		value._encodingFormat = FI::ENCODINGFORMAT_ENCODING_ALGORITHM;
		value._encodingAlgorithm = getSmallInteger5();
		break;
	default:
		throw std::invalid_argument("Unknown encoding format.");
	}
	getNonEmptyOctetString5(value._octets);
}

// C.20 Encoding of the EncodedCharacterString type starting on the fifth bit of an octet
void Decoder::getEncodedCharacterString5(FI::EncodedCharacterString& value)
{
	switch(_b & Constants::ENCODED_CHARACTER_STRING_5TH_MASK)
	{
	case Constants::ENCODED_CHARACTER_STRING_5TH_UTF8:
		value._encodingFormat = FI::ENCODINGFORMAT_UTF8;
		break;
	case Constants::ENCODED_CHARACTER_STRING_5TH_UTF16:
		value._encodingFormat = FI::ENCODINGFORMAT_UTF16;
		break;
	case Constants::ENCODED_CHARACTER_STRING_5TH_RESTRICTED_ALPHABET:
		value._encodingFormat = FI::ENCODINGFORMAT_RESTRICTED_ALPHABET;
		value._restrictedAlphabet = getSmallInteger7();
		break;
	case Constants::ENCODED_CHARACTER_STRING_5TH_ENCODING_ALGORITHM:
		value._encodingFormat = FI::ENCODINGFORMAT_ENCODING_ALGORITHM;
		value._encodingAlgorithm = getSmallInteger7();
		break;
	default:
		throw std::invalid_argument("Unknown encoding format.");

	}
	getNonEmptyOctetString7(value._octets);	
}

// C.22
void Decoder::getNonEmptyOctetString2(FI::NonEmptyOctetString &value)
{
	int iLength = 0;

	if(!checkBit(_b, 2)) // small
	{
		iLength = (_b & Constants::LAST_SIX_BITS) + 1;
	}
	else if(_b == Constants::NON_EMPTY_OCTET_STRING_2ND_MEDIUM) // medium
	{
		iLength = static_cast<unsigned int>(_stream->get()) + 65;
	}
	else if(_b == Constants::NON_EMPTY_OCTET_STRING_2ND_LARGE)
	{
		iLength = (_stream->get() << 24) + (_stream->get() << 16) + (_stream->get() << 8) + _stream->get() + 321;
	}
	else
		throw std::runtime_error("Illegal Octet length encoding"); // Failure

	value.reserve(iLength);
	for(int i = 0; i < iLength; i++)
	{
		value.push_back(static_cast<const char>(_stream->get()));
	}

}

// C.23
void Decoder::getNonEmptyOctetString5(FI::NonEmptyOctetString &value)
{
	int iLength = 0;
	
	// Determine length of the string
	if(!checkBit(_b, 5)) // small
	{
		iLength = (_b & Constants::LAST_THREE_BITS) + 1;
	}
	else switch(_b & Constants::LAST_FOUR_BITS)
	{
	case Constants::NON_EMPTY_OCTET_STRING_5TH_MEDIUM:
		iLength = _stream->get() + 9;
		break;
	case Constants::NON_EMPTY_OCTET_STRING_5TH_LARGE:
		iLength = (_stream->get() << 24) + (_stream->get() << 16) + (_stream->get()<< 8) + _stream->get() + 265;
		break;
	default:
		throw std::runtime_error("Illegal Octet length encoding");
	}
	
	value.reserve(iLength);
	for(int i = 0; i < iLength; i++)
		value.push_back(static_cast<const char>(_stream->get()));

}

// C.24
void Decoder::getNonEmptyOctetString7(FI::NonEmptyOctetString &value)
{
	size_t iLength;

	// Determine length of the string
	if(!checkBit(_b, 7)) // small
	{
		iLength = (_b & 0x01) + 1;
	}
	else switch(_b & Constants::LAST_TWO_BITS)
	{
	case Constants::NON_EMPTY_OCTET_STRING_7TH_MEDIUM:
		iLength = _stream->get() + 3;
		break;
	case Constants::NON_EMPTY_OCTET_STRING_7TH_LARGE:
		_stream->read((char*)&iLength, 4); 
		iLength += 265;
		break;
	default:
		throw std::runtime_error("Illegal Octet length encoding");
	}
	
	value.reserve(iLength);
	for(size_t i = 0; i < iLength; i++)
		value.push_back(static_cast<const char>(_stream->get()));
	
}

// C.25 & C.26
unsigned int Decoder::getInteger2()
{
	unsigned int iValue;
	char buf[2];

	if((_b & Constants::LAST_SEVEN_BITS) == Constants::LAST_SEVEN_BITS)
		return 0;

	if(!checkBit(_b,2)) // small, '0' padding
	{
		iValue = (_b & Constants::LAST_SIX_BITS) + 1;
	}
	else switch(_b & Constants::INTEGER_2ND_LENGTH_MASK)
	{
		case Constants::INTEGER_2ND_LENGTH_MEDIUM:
			iValue = ((_b & Constants::LAST_FIVE_BITS) << 8) + _stream->get() + 65; 
			break;
		case Constants::INTEGER_3RD_LENGTH_LARGE:
			_stream->read(buf, 2);
			iValue = ((_b & Constants::LAST_FOUR_BITS) << 16) + (buf[0] << 8) + buf[1] + 8257;
			break;
		default:
			throw std::runtime_error("Illegal Integer length encoding");
	}

	return iValue;
}

// C.27
unsigned int Decoder::getInteger3()
{
	unsigned int iValue;
	char buf[3];

	if(!checkBit(_b,3)) // tiny, '0' padding
	{
		iValue = (_b & Constants::LAST_FIVE_BITS) + 1;
	}
	else switch(_b & Constants::INTEGER_3RD_LENGTH_MASK)
	{
		case Constants::INTEGER_3RD_LENGTH_SMALL:
			iValue = ((_b & Constants::LAST_THREE_BITS) << 8) + _stream->get() + 33; 
			break;
		case Constants::INTEGER_3RD_LENGTH_MEDIUM:
			_stream->read(buf, 2);
			iValue = ((_b & Constants::LAST_THREE_BITS) << 16) + (buf[0] << 8) + buf[1] + 2081; 
			break;
		case Constants::INTEGER_3RD_LENGTH_LARGE:
			if((_b & Constants::LAST_THREE_BITS) != 0) // padding
				return 0;
			_stream->read(buf, 3);
			if((_b & Constants::FOUR_BITS) != 0) // padding
				return 0;
			iValue = ((buf[0] & Constants::LAST_FOUR_BITS) << 16) + (buf[1] << 8) + buf[2] + 526369;
			break;
		default:
			return 0;
	}

	return iValue;
}

// C.28
unsigned int Decoder::getInteger4()
{
	unsigned int iValue;
	char buf[3];

	if(!checkBit(_b,4)) // tiny, '0' padding
	{
		iValue = (_b & Constants::LAST_FOUR_BITS) + 1;
	}
	else switch(_b & Constants::INTEGER_4TH_LENGTH_MASK)
	{
		case Constants::INTEGER_4TH_LENGTH_SMALL:
			iValue = ((_b & Constants::LAST_TWO_BITS) << 8) + _stream->get() + 17; 
			break;
		case Constants::INTEGER_4TH_LENGTH_MEDIUM:
			_stream->read(buf, 2);
			iValue = ((_b & Constants::LAST_TWO_BITS) << 16) + (buf[0] << 8) + buf[1] + 1041; 
			break;
		case Constants::INTEGER_4TH_LENGTH_LARGE:
			if((_b & Constants::LAST_THREE_BITS) != 0) // padding
				return 0;
			_stream->read(buf, 3);
			if((_b & Constants::FOUR_BITS) != 0) // padding
				return 0;
			iValue = ((buf[0] & Constants::LAST_FOUR_BITS) << 16) + (buf[1] << 8) + buf[2] + 263185;
			break;
		default:
			return 0;
	}

	return iValue;
}

// C.29
unsigned int Decoder::getSmallInteger5()
{
	unsigned int result = (_b & Constants::LAST_FOUR_BITS) << 4;
	_b = static_cast<unsigned char>(_stream->get()); // next byte
	return result + ((_b & Constants::FOUR_BITS) >> 4) + 1;
}

// C.29
unsigned int Decoder::getSmallInteger7()
{
	return (((_b & Constants::LAST_TWO_BITS) << 6) + (((_b = static_cast<char>(_stream->get())) & Constants::SIX_BITS) >> 2) + 1);
}

int Decoder::checkBit(unsigned char c, unsigned char iPos)
{
	switch(iPos) // evtl. schneller als der vergleich mit 2^(8-iPos)
	{
	case 1:
		return (c & 0x80);
	case 2:
		return (c & 0x40);
	case 3:
		return (c & 0x20);
	case 4:
		return (c & 0x10);
	case 5:
		return (c & 0x08);
	case 6:
		return (c & 0x04);
	case 7:
		return (c & 0x02);
	case 8:
		return (c & 0x01);
	default:
		throw std::range_error("checkBit position must be between 1-8");
	}
}

// C.2.4
void Decoder::decodeAdditionalData()
{
  /* C.2.4 If the optional component additional-data is present, then the number of additional-datum
	components is encoded as described in C.21, and each of the additional-datum components is encoded as described
	in the two following subclauses.
	C.2.4.1 The bit '0' (padding) is appended to the bit stream and the id component is encoded as described in C.22.
	C.2.4.2 The bit '0' (padding) is appended to the bit stream and the data component is encoded as described in C.22.
  */
  throw std::runtime_error("Decoding of additional data is not yet supported.");
}

// C.2.5
void Decoder::decodeInitialVocabulary()
{

	//C.2.5 If the optional component initial-vocabulary is present, then the three bits '000' (padding) are appended
	//to the bit stream, and the component is encoded as described in the five following subclauses.
	//C.2.5.1 For each of the thirteen optional components of initial-vocabulary (in textual order), if the component is
	//present, then the bit '1' (presence) is appended to the bit stream; otherwise, the bit '0' (absence) is appended.
	
	
	//C.2.5.3 For each of the components restricted-alphabets, encoding-algorithms, prefixes, namespacenames,
	//local-names, other-ncnames, and other-uris (in this order) which is present, the number of
	//NonEmptyOctetString items in the component is encoded as described in C.21, and then each item is encoded (in
	//order) as follows: The bit '0' (padding) is appended to the bit stream, and the NonEmptyOctetString is encoded as
	//described in C.22.
	//C.2.5.4 For each of the components attribute-values, content-character-chunks, and other-strings (in
	//this order) which is present, the number of EncodedCharacterString items in the component is encoded as described
	//in C.21, and then each item is encoded (in order) as follows: The two bits '00' (padding) are appended to the bit stream,
	//and the EncodedCharacterString is encoded as described in C.19.
	//C.2.5.5 For each of the components element-name-surrogates and attribute-name-surrogates (in this
	//order) which is present, the number of NameSurrogate items in the component is encoded as described in C.21, and
	//then each item is encoded (in order) as follows: The six bits '000000' (padding) are appended to the bit stream, and the
	//NameSurrogate is encoded as described in C.16.
	char b, b2;
	_stream->get(b);
	_stream->get(b2);

	if (
		b == 0x10  // 00010000
		&& b2 == 0 // 00000000
		)
	{
		_b = static_cast<unsigned char>(_stream->get()); // next byte
		decodeExternalVocabularyURI();
		return;
	}
	throw std::runtime_error("Only external-vocabulary (as initial-vocabulary) is supported (yet).");

}

void Decoder::decodeExternalVocabularyURI()
{
	//C.2.5.2 If the optional component external-vocabulary of initial-vocabulary is present, then the bit '0'
	//(padding) is appended to the bit stream and the component is encoded as described in C.22.
	NonEmptyOctetString externalVocabularyURI;
	getNonEmptyOctetString2(externalVocabularyURI);

	std::map<std::string, ParserVocabulary*>::iterator I = _externalVocabularies.find(std::string(externalVocabularyURI.begin(), externalVocabularyURI.end()));
	if (I == _externalVocabularies.end())
		throw std::runtime_error("externalVocabularyNotRegistered!");

	// Replace default vocabulary by external
	if (_vocab)
		delete _vocab;
	_vocab = (*I).second;
}

// C.2.6
void Decoder::decodeNotations()
{
	//	C.2.6 If the optional component notations is present, it is encoded as described in the two following subclauses.
	//C.2.6.1 Each item of notations (in order) is encoded as follows: The six bits '110000' (identification) are appended
	//to the bit stream, and the Notation is encoded as described in C.11.
	//C.2.6.2 The four bits '1111' (termination) and the four bits '0000' (padding) are appended to the bit stream.
	throw std::runtime_error("Decoding of notations is not yet supported.");
}

void Decoder::decodeUnparsedEntities()
{
	//C.2.7 If the optional component unparsed-entities is present, it is encoded as described in the two following
	//subclauses.
	//C.2.7.1 Each item of unparsed-entities (in order) is encoded as follows: The seven bits '1101000' (identification)
	//are appended to the bit stream, and the UnparsedEntity is encoded as described in C.10.
	//C.2.7.2 The four bits '1111' (termination) and the four bits '0000' (padding) are appended to the bit stream.
	throw std::runtime_error("Decoding of unparsed-entities is not yet supported.");
}

void Decoder::decodeCharacterEncodingScheme()
{
	//C.2.8 If the optional component character-encoding-scheme is present, then the bit '0' (padding) is appended
	//to the bit stream, and the NonEmptyOctetString is encoded as described in C.22.
	throw std::runtime_error("Decoding of character-encoding-scheme is not yet supported.");
}

void Decoder::decodeStandalone()
{
	//C.2.9 If the optional component standalone is present, it is encoded as follows: The seven bits '0000000'
	//(padding) are appended to the bit stream. If the value of standalone is TRUE, then the bit '1' is appended to the bit
	//stream, otherwise the bit '0' is appended.
	
	//bool standalone = _stream->get() > 0;
}
void Decoder::decodeVersion()
{
	// C.2.10 If the optional component version is present, then its value is encoded as described in C.14.
	NonIdentifyingStringOrIndex version;
	getNonIdentifyingStringOrIndex1(version);
}

} // end namespace FI

