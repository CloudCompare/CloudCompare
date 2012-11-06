#include <xiot/FIParserVocabulary.h>
#include <xiot/FIConstants.h>
#include <iostream>
#include <cassert>

#define THROW(s) { \
std::stringstream ss; \
ss << s; \
throw std::runtime_error(ss.str());\
}

#define OCTETS2STRING(octets, str) { if ((octets).size())  (str).insert((str).begin(), (octets).begin(), (octets).end()); else (str).clear(); }

namespace FI {


QualifiedName ParserVocabulary::resolveElementName(const QualifiedNameOrIndex &nameOrIndex) const
{
	unsigned int surrogateIndex = nameOrIndex._nameSurrogateIndex;
	return (surrogateIndex == INDEX_NOT_SET) ?
		resolveQualifiedName(nameOrIndex) : getElementName(surrogateIndex);
}

QualifiedName ParserVocabulary::resolveAttributeName(const QualifiedNameOrIndex &nameOrIndex) const
{
	unsigned int surrogateIndex = nameOrIndex._nameSurrogateIndex;
	return (surrogateIndex == INDEX_NOT_SET) ?
		resolveQualifiedName(nameOrIndex) : getAttributeName(surrogateIndex);
}

QualifiedName ParserVocabulary::resolveQualifiedName(const QualifiedNameOrIndex &nameOrIndex) const
{
	assert(nameOrIndex._nameSurrogateIndex == INDEX_NOT_SET);
	std::string prefix, namespaceName, localName;
		
		if (nameOrIndex._prefix._stringIndex == INDEX_NOT_SET) {
			OCTETS2STRING(nameOrIndex._prefix._literalCharacterString, prefix);
		} else {
			prefix = getPrefix(nameOrIndex._prefix._stringIndex);
		}
		
		if (nameOrIndex._namespaceName._stringIndex == INDEX_NOT_SET) {
			OCTETS2STRING(nameOrIndex._namespaceName._literalCharacterString, namespaceName);
		} else {
			namespaceName = getNamespaceName(nameOrIndex._namespaceName._stringIndex);
		}

		if (nameOrIndex._localName._stringIndex == INDEX_NOT_SET) {
			OCTETS2STRING(nameOrIndex._localName._literalCharacterString, localName);
		} else {
			localName = getLocalName(nameOrIndex._localName._stringIndex);
		}
		
		return QualifiedName(prefix, namespaceName, localName);
}

std::string ParserVocabulary::resolveAttributeValue(const NonIdentifyingStringOrIndex &input) const
{
	unsigned int stringIndex = input._stringIndex;
	if (stringIndex == INDEX_NOT_SET)
	{
		return decodeCharacterString(input._characterString);
	}
	return getAttributeValue(stringIndex);
}

std::string	ParserVocabulary::resolveCharacterChunk(const NonIdentifyingStringOrIndex &input) const
{
	unsigned int stringIndex = input._stringIndex;
	if (stringIndex == INDEX_NOT_SET)
	{
		return decodeCharacterString(input._characterString);
	}
	return getCharacterChunk(stringIndex);
}


std::string	ParserVocabulary::decodeCharacterString(const EncodedCharacterString &input) const
{
	switch (input._encodingFormat)
	{
		case ENCODINGFORMAT_UTF8:
			return std::string(input._octets.begin(), input._octets.end());
		case ENCODINGFORMAT_UTF16:
			throw std::runtime_error("UTF-16 encoding is not supported (yet)");
		case ENCODINGFORMAT_ENCODING_ALGORITHM:
			return getEncodingAlgorithm(input._encodingAlgorithm)->decodeToString(input._octets);
		case ENCODINGFORMAT_RESTRICTED_ALPHABET:
			return "RESTRICTED";
		default:
			throw std::runtime_error("Unknown encoding format.");
	}
}

// -- DefaultParserVocabulary


DefaultParserVocabulary::DefaultParserVocabulary()
{
	initEncodingAlgorithms();
}


DefaultParserVocabulary::DefaultParserVocabulary(const char* uri) : _externalVocabularyURI(uri) 
{
	initEncodingAlgorithms();
}

void DefaultParserVocabulary::initEncodingAlgorithms()
{
  _encodingAlgorithms.insert(_encodingAlgorithms.begin(), Constants::ENCODING_ALGORITHM_BUILTIN_END, NULL);
  _encodingAlgorithms[IntEncodingAlgorithm::ALGORITHM_ID] = &_intEncodingAlgorithm;
  _encodingAlgorithms[FloatEncodingAlgorithm::ALGORITHM_ID] = &_floatEncodingAlgorithm;

}

IEncodingAlgorithm* DefaultParserVocabulary::getEncodingAlgorithm(unsigned int index) const
{
	if (index < Constants::ENCODING_ALGORITHM_BUILTIN_END)
	{
		IEncodingAlgorithm* result = _encodingAlgorithms[index];
		if (result)
			return result;

		if (!result)
		{
			THROW("Built-in encoding algorithm not implemented (yet) " << index);
		}
		return result;
	}
	if (index >= Constants::ENCODING_ALGORITHM_APPLICATION_START)
	{
		try {
			IEncodingAlgorithm* result = _encodingAlgorithms.at(index);
			if (!result) throw std::out_of_range("");
			return result;
		} catch (std::out_of_range&)
		{
			THROW("URI not present for encoding algorithm identifier " << index);
		}
	}
	THROW("Encoding algorithm index 11-31 are reserved for future versions of FastInfoSet");
}




void DefaultParserVocabulary::addAttributeValue(std::string value)
{
	_attributeValues.push_back(value);
}

void DefaultParserVocabulary::addCharacterChunk(std::string value)
{
	_characterChunks.push_back(value);
}

void DefaultParserVocabulary::addEncodingAlgorithm(IEncodingAlgorithm* algorithm)
{
	if (_encodingAlgorithms.size() <= Constants::ENCODING_ALGORITHM_APPLICATION_START)
		_encodingAlgorithms.resize(Constants::ENCODING_ALGORITHM_APPLICATION_START+1, NULL);
	//std::cout << "Adding alogrithm to table pos: " << _encodingAlgorithms.size() << std::endl;
	_encodingAlgorithms.push_back(algorithm);
}

} // namespace FI
