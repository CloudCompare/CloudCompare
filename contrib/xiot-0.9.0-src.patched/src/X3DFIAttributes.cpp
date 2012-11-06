#include <xiot/X3DFIAttributes.h>

#include <iostream>

#include <xiot/FITypes.h>
#include <xiot/FIConstants.h>
#include <xiot/X3DParserVocabulary.h>
#include <xiot/X3DParseException.h>
#include <xiot/X3DFIEncodingAlgorithms.h>
#include <xiot/X3DFICompressionTools.h>

#define getValueAt(index) _impl->_attributes->at((index))._normalizedValue

namespace XIOT {

class FIAttributeImpl
{
public:
	FI::Attributes* _attributes;
	FI::ParserVocabulary* _vocab;
};

X3DFIAttributes::X3DFIAttributes(const void *const attributes, const FI::ParserVocabulary* vocab) : 
_impl(new FIAttributeImpl())
{
	_impl->_attributes = (FI::Attributes*) attributes;
	_impl->_vocab = (FI::ParserVocabulary*) vocab;
}

X3DFIAttributes::~X3DFIAttributes()
{
	delete _impl;
}


int X3DFIAttributes::getAttributeIndex(int attributeID) const{
	int i = 0;
	for(std::vector<FI::Attribute>::const_iterator I = _impl->_attributes->begin();
		I != _impl->_attributes->end(); I++, i++)
	{
		if ((*I)._qualifiedName._nameSurrogateIndex == static_cast<unsigned int>(attributeID+1))
			return i;
	}
	return -1;
}

size_t X3DFIAttributes::getLength() const {
	return _impl->_attributes->size();
}

std::string X3DFIAttributes::getAttributeValue(int id) const {
  return _impl->_vocab->resolveAttributeValue(_impl->_attributes->at(id)._normalizedValue);
}

std::string X3DFIAttributes::getAttributeName(int id) const {
  return _impl->_vocab->resolveAttributeName(_impl->_attributes->at(id)._qualifiedName)._localName;
}

// Single fields
bool X3DFIAttributes::getSFBool(int index) const{
	FI::NonIdentifyingStringOrIndex value = getValueAt(index);
	if(value._stringIndex == X3DParserVocabulary::ATTRIBUT_VALUE_TRUE_INDEX)
		return true;
	if(value._stringIndex == X3DParserVocabulary::ATTRIBUT_VALUE_FALSE_INDEX)
		return false;
	if(value._stringIndex == FI::INDEX_NOT_SET)
		return X3DDataTypeFactory::getSFBoolFromString(_impl->_vocab->resolveAttributeValue(value)); 
	
	throw X3DParseException("Unknown SFBool encoding");
}

float X3DFIAttributes::getSFFloat(int index) const {
	std::vector<float> result;
	getFloatArray(getValueAt(index), result);
	if (result.size() == 1)
	{
		return result[0];
	}
	else
		throw X3DParseException("Wrong size for SFFloat");
}
int X3DFIAttributes::getSFInt32(int index) const {
	std::vector<int> result;
	getIntArray(getValueAt(index), result);
	if (result.size() == 1)
	{
		return result[0];
	}
	else
		throw X3DParseException("Wrong size for SFInt32");
}

void X3DFIAttributes::getSFVec3f(int index, SFVec3f &value) const {
	std::vector<float> result;
	getFloatArray(getValueAt(index), result);
	if (result.size() == 3)
	{
		value.x = result[0];
		value.y = result[1];
		value.z = result[2];
	}
	else
		throw X3DParseException("Wrong size for SFVec3f");
}

void X3DFIAttributes::getSFVec2f(int index, SFVec2f &value) const {
	std::vector<float> result;
	getFloatArray(getValueAt(index), result);
	if (result.size() == 2)
	{
		value.x = result[0];
		value.y = result[1];
	}
	else
		throw X3DParseException("Wrong size for SFVec2f");
}

void X3DFIAttributes::getSFRotation(int index, SFRotation &value) const {
	std::vector<float> result;
	getFloatArray(getValueAt(index), result);
	if (result.size() == 4)
	{
		value.x = result[0];
		value.y = result[1];
		value.z = result[2];
		value.angle = result[3];
	}
	else
		throw X3DParseException("Wrong size for SFRotation");
}

void X3DFIAttributes::getSFString(int index, SFString& value) const {
	value.assign(_impl->_vocab->resolveAttributeValue(getValueAt(index))); 
}

void X3DFIAttributes::getSFColor(int index, SFColor &value) const {
	std::vector<float> result;
	getFloatArray(getValueAt(index), result);
	if (result.size() == 3)
	{
		value.r = result[0];
		value.g = result[1];
		value.b = result[2];
	}
	else
		throw X3DParseException("Wrong size for SFColor");
}

void X3DFIAttributes::getSFColorRGBA(int index, SFColorRGBA &value) const {
	std::vector<float> result;
	getFloatArray(getValueAt(index), result);
	if (result.size() == 4)
	{
		value.r = result[0];
		value.g = result[1];
		value.b = result[2];
		value.a = result[3];
	}
	else
		throw X3DParseException("Wrong size for SFColorRGBA");
}

void X3DFIAttributes::getSFImage(int index, SFImage& value) const {
  MFInt32 signedVector;
  SFImage result;
	getIntArray(getValueAt(index), signedVector);
  for(MFInt32::const_iterator I = signedVector.begin(); I != signedVector.end(); I++)
  {
    result.push_back(static_cast<unsigned int>(*I));
  }
  std::swap(result, value);
} 

// Multi Field
void X3DFIAttributes::getMFFloat(int index, MFFloat &value) const {
	getFloatArray(getValueAt(index), value);
}
void X3DFIAttributes::getMFInt32(int index, MFInt32 &value) const {
	getIntArray(getValueAt(index), value);
}

void X3DFIAttributes::getMFVec3f(int index, MFVec3f &value) const {
	std::vector<float> fArray;
	getFloatArray(getValueAt(index), fArray);
	if (fArray.size() % 3 == 0)
	{
		float* pPos = &fArray.front();
		MFVec3f result;
		for(size_t i = 0; i < fArray.size() / 3; i++, pPos+=3)
		{
			SFVec3f c(pPos);
			result.push_back(c);
		}
		std::swap(result, value);
	}
	else
		throw X3DParseException("Wrong size for MFVec3f");
}

void X3DFIAttributes::getMFVec2f(int index, MFVec2f &value) const {
	std::vector<float> fArray;
	getFloatArray(getValueAt(index), fArray);
	if (fArray.size() % 2 == 0)
	{
		float* pPos = &fArray.front();
		MFVec2f result;
		for(size_t i = 0; i < fArray.size() / 2; i++, pPos+=2)
		{
			SFVec2f c(pPos);
			result.push_back(c);
		}
		std::swap(result, value);
	}
	else
		throw X3DParseException("Wrong size for MFVec2f");
}

void X3DFIAttributes::getMFRotation(int index, MFRotation &value) const  {
	std::vector<float> fArray;
	getFloatArray(getValueAt(index), fArray);
	if (fArray.size() % 4 == 0)
	{
		float* pPos = &fArray.front();
		MFRotation result;
		for(size_t i = 0; i < fArray.size() / 4; i++, pPos+=4)
		{
			SFRotation c(pPos);
			result.push_back(c);
		}
		std::swap(result, value);
	}
	else
		throw X3DParseException("Wrong size for MFRotation");
}

void X3DFIAttributes::getMFString(int index, MFString &value) const {
	return X3DDataTypeFactory::getMFStringFromString(_impl->_vocab->resolveAttributeValue(getValueAt(index)), value); 
}

void X3DFIAttributes::getMFColor(int index, MFColor &value) const {
	std::vector<float> fArray;
	getFloatArray(getValueAt(index), fArray);
	if (fArray.size() % 3 == 0)
	{
		float* pPos = &fArray.front();
		MFColor result;
		for(size_t i = 0; i < fArray.size() / 3; i++, pPos+=3)
		{
			SFColor c(pPos);
			result.push_back(c);
		}
		std::swap(result, value);
	}
	else
		throw X3DParseException("Wrong size for MFColor");
}

void X3DFIAttributes::getMFColorRGBA(int index, MFColorRGBA &value) const {
	std::vector<float> fArray;
	getFloatArray(getValueAt(index), fArray);
	if (fArray.size() % 4 == 0)
	{
		float* pPos = &fArray.front();
		MFColorRGBA result;
		for(size_t i = 0; i < fArray.size() / 4; i++, pPos+=4)
		{
			SFColorRGBA c(pPos);
			result.push_back(c);
		}
		std::swap(result, value);
	}
	else
		throw X3DParseException("Wrong size for MFColorRGBA");
}

void X3DFIAttributes::getFloatArray(const FI::NonIdentifyingStringOrIndex &value, std::vector<float> &vec) const
{
	if (value._stringIndex == FI::INDEX_NOT_SET
		&& 	value._characterString._encodingFormat == FI::ENCODINGFORMAT_ENCODING_ALGORITHM)
	{
		switch (value._characterString._encodingAlgorithm)
		{
			case QuantizedzlibFloatArrayAlgorithm::ALGORITHM_ID:
				{
				QuantizedzlibFloatArrayAlgorithm::decodeToFloatArray(value._characterString._octets, vec);
				break;
				}
			case FI::FloatEncodingAlgorithm::ALGORITHM_ID:
				{
				FI::FloatEncodingAlgorithm::decodeToFloatArray(value._characterString._octets, vec);
				break;
				}
			default:
				{
				std::stringstream ss;
				ss << "Encoding Algortihm with id <" << value._characterString._encodingAlgorithm << "> is not known for encoding of float arrays." << std::endl;
				throw X3DParseException(ss.str());
				}
		}
		return;
	}
	// This is for not algorithm encoded values
	X3DDataTypeFactory::getMFFloatFromString(_impl->_vocab->resolveAttributeValue(value), vec);
}

void X3DFIAttributes::getIntArray(const FI::NonIdentifyingStringOrIndex &value, std::vector<int> &vec) const
{
	if (value._stringIndex == FI::INDEX_NOT_SET
		&& 	value._characterString._encodingFormat == FI::ENCODINGFORMAT_ENCODING_ALGORITHM)
	{
		switch(value._characterString._encodingAlgorithm)
		{
			case DeltazlibIntArrayAlgorithm::ALGORITHM_ID:
				{
				DeltazlibIntArrayAlgorithm::decodeToIntArray(value._characterString._octets, vec);
				break;
				}
			case FI::IntEncodingAlgorithm::ALGORITHM_ID:
				{
				FI::IntEncodingAlgorithm::decodeToIntArray(value._characterString._octets, vec);
				break;
				}
			default:
				{
				std::stringstream ss;
				ss << "Encoding Algortihm with id <" << value._characterString._encodingAlgorithm << "> is not known for encoding of int arrays." << std::endl;
				throw X3DParseException(ss.str());
				}
		}
		return;
	}
	// This is for not algorithm encoded values
	X3DDataTypeFactory::getMFInt32FromString(_impl->_vocab->resolveAttributeValue(value), vec);
}


}

