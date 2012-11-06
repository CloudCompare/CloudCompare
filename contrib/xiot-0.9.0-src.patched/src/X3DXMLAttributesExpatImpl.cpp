#include <xiot/X3DXMLAttributes.h>

#include XIOT_EXPAT_HEADER
#include <cstring>

namespace XIOT {

struct ExpatAttribute {
	ExpatAttribute(const char* name, const char* value) : _name(name), _value(value) {};
	const char* _name;
	const char* _value;
};

class XMLAttributeImpl
{
public:
	XMLAttributeImpl(const void *const va) {
		if (va != NULL)
		{
			const char** a1 = (const char**)va;
			while(*a1 != 0)
			{
				const char* attrName = *a1++;
				const char* attrValue = *a1++;
				_attributes.push_back(ExpatAttribute(attrName, attrValue));
			}
		}
	}
	std::vector<ExpatAttribute> _attributes;
};



X3DXMLAttributes::X3DXMLAttributes(const void *const va) : 
_impl(new XMLAttributeImpl(va))
{
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
	std::vector<ExpatAttribute>::iterator I = _impl->_attributes.begin();
	while(I!=_impl->_attributes.end())
	{
		if(!strcmp(sAttribute, (*I)._name))
			return static_cast<int>(I -  _impl->_attributes.begin());
		I++;
	}
	return -1;
}

size_t X3DXMLAttributes::getLength() const {
	return _impl->_attributes.size();
}

std::string X3DXMLAttributes::getAttributeValue(int id) const {
  return _impl->_attributes[id]._value;
}

std::string X3DXMLAttributes::getAttributeName(int id) const {
  return _impl->_attributes[id]._name;
}

// Single fields
bool X3DXMLAttributes::getSFBool(int index) const{
	const char* sValue = _impl->_attributes.at(index)._value;
	return X3DDataTypeFactory::getSFBoolFromString(sValue);
}

float X3DXMLAttributes::getSFFloat(int index) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	return X3DDataTypeFactory::getSFFloatFromString(sValue);
}

int X3DXMLAttributes::getSFInt32(int index) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	return X3DDataTypeFactory::getSFInt32FromString(sValue);
}

void X3DXMLAttributes::getSFVec3f(int index, SFVec3f& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getSFVec3fFromString(sValue, value);
}

void X3DXMLAttributes::getSFVec2f(int index, SFVec2f& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getSFVec2fFromString(sValue, value);
}

void X3DXMLAttributes::getSFRotation(int index, SFRotation& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getSFRotationFromString(sValue, value);
}
void X3DXMLAttributes::getSFString(int index, SFString& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	value.assign(sValue);
}

void X3DXMLAttributes::getSFColor(int index, SFColor& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getSFColorFromString(sValue, value);
}

void X3DXMLAttributes::getSFColorRGBA(int index, SFColorRGBA& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getSFColorRGBAFromString(sValue, value);
}

void X3DXMLAttributes::getSFImage(int index, SFImage& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getSFImageFromString(sValue, value);
} 

// Multi Field
void X3DXMLAttributes::getMFFloat(int index, MFFloat& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getMFFloatFromString(sValue, value);
}
void X3DXMLAttributes::getMFInt32(int index, MFInt32& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getMFInt32FromString(sValue, value);
}

void X3DXMLAttributes::getMFVec3f(int index, MFVec3f& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getMFVec3fFromString(sValue, value);
}
void X3DXMLAttributes::getMFVec2f(int index, MFVec2f& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getMFVec2fFromString(sValue, value);
}
void X3DXMLAttributes::getMFRotation(int index, MFRotation& value) const  {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getMFRotationFromString(sValue, value);
}

void X3DXMLAttributes::getMFString(int index, MFString& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getMFStringFromString(sValue, value);
}

void X3DXMLAttributes::getMFColor(int index, MFColor& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getMFColorFromString(sValue, value);
}

void X3DXMLAttributes::getMFColorRGBA(int index, MFColorRGBA& value) const {
	const char* sValue = _impl->_attributes.at(index)._value;
	X3DDataTypeFactory::getMFColorRGBAFromString(sValue, value);
}
}

