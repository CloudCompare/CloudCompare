#include <xiot/X3DXMLAttributes.h>
#include <xercesc/sax2/Attributes.hpp>
#include <xercesc/util/XMLString.hpp>

XERCES_CPP_NAMESPACE_USE

namespace XIOT {

class XMLAttributeImpl
{
public:
	XERCES_CPP_NAMESPACE_QUALIFIER Attributes *_attributes;
};

X3DXMLAttributes::X3DXMLAttributes(const void *const attributes) : 
_impl(new XMLAttributeImpl)
{
	_impl->_attributes = (XERCES_CPP_NAMESPACE_QUALIFIER Attributes *)attributes;
}

X3DXMLAttributes::~X3DXMLAttributes()
{
  delete _impl;
}


/*!
 * <b>getAttributeIndex</b> returns the attribute's index while it takes the attribute's ID.
 * 
 * @param attributeID The id of the node as specified in 
 * @link{http://www.web3d.org/x3d/specifications/ISO-IEC-FCD-19776-3.2-X3DEncodings-CompressedBinary/Part03/tables.html}
 */
int X3DXMLAttributes::getAttributeIndex(int attributeID) const{
	
	const char* sAttribute = X3DTypes::getAttributeByID(attributeID);
	XMLCh *xmlChar = XMLString::transcode(sAttribute);
	int result = _impl->_attributes->getIndex(xmlChar);
	XMLString::release(&xmlChar);
	return result;
}

size_t X3DXMLAttributes::getLength() const {
	return static_cast<size_t>(_impl->_attributes->getLength());
}

std::string X3DXMLAttributes::getAttributeValue(int id) const {
  char* sValue = XMLString::transcode(_impl->_attributes->getValue(id));
  std::string result(sValue);
	XMLString::release(&sValue);
	return result; 
}

std::string X3DXMLAttributes::getAttributeName(int id) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getQName(id));
  std::string result(sValue);
	XMLString::release(&sValue);
	return result; 
}

// Single fields
bool X3DXMLAttributes::getSFBool(int index) const{
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	bool result = X3DDataTypeFactory::getSFBoolFromString(sValue);
	XMLString::release(&sValue);
	return result; 
}

float X3DXMLAttributes::getSFFloat(int index) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	float result = X3DDataTypeFactory::getSFFloatFromString(sValue);
	XMLString::release(&sValue);
	return result; 
}
int X3DXMLAttributes::getSFInt32(int index) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	int result = X3DDataTypeFactory::getSFInt32FromString(sValue);
	XMLString::release(&sValue);
	return result; 
}

void X3DXMLAttributes::getSFVec3f(int index, SFVec3f &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getSFVec3fFromString(sValue, value);
	XMLString::release(&sValue);
}

void X3DXMLAttributes::getSFVec2f(int index, SFVec2f &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getSFVec2fFromString(sValue, value);
	XMLString::release(&sValue);
}

void X3DXMLAttributes::getSFRotation(int index, SFRotation &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getSFRotationFromString(sValue, value);
	XMLString::release(&sValue);
}

void X3DXMLAttributes::getSFString(int index, SFString &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
  value.assign(sValue);
	XMLString::release(&sValue);
}

void X3DXMLAttributes::getSFColor(int index, SFColor &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getSFColorFromString(sValue, value);
	XMLString::release(&sValue);
}

void X3DXMLAttributes::getSFColorRGBA(int index, SFColorRGBA &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getSFColorRGBAFromString(sValue, value);
	XMLString::release(&sValue);
}

void X3DXMLAttributes::getSFImage(int index, SFImage &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getSFImageFromString(sValue, value);
	XMLString::release(&sValue);
} 

// Multi Field
void X3DXMLAttributes::getMFFloat(int index, MFFloat &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getMFFloatFromString(sValue, value);
	XMLString::release(&sValue);
}
void X3DXMLAttributes::getMFInt32(int index, MFInt32 &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getMFInt32FromString(sValue, value);
	XMLString::release(&sValue);
}

void X3DXMLAttributes::getMFVec3f(int index, MFVec3f &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getMFVec3fFromString(sValue, value);
	XMLString::release(&sValue);
}

void X3DXMLAttributes::getMFVec2f(int index, MFVec2f &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getMFVec2fFromString(sValue, value);
	XMLString::release(&sValue);
}

void X3DXMLAttributes::getMFRotation(int index, MFRotation &value) const  {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getMFRotationFromString(sValue, value);
	XMLString::release(&sValue);
}

void X3DXMLAttributes::getMFString(int index, MFString &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getMFStringFromString(sValue, value);
	XMLString::release(&sValue);
}

void X3DXMLAttributes::getMFColor(int index, MFColor &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getMFColorFromString(sValue, value);
	XMLString::release(&sValue);
}

void X3DXMLAttributes::getMFColorRGBA(int index, MFColorRGBA &value) const {
	char* sValue = XMLString::transcode(_impl->_attributes->getValue(index));
	X3DDataTypeFactory::getMFColorRGBAFromString(sValue, value);
	XMLString::release(&sValue);
}
}

