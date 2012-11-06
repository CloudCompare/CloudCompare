#include <xiot/X3DAttributes.h>

namespace XIOT {

bool X3DAttributes::isDEF() const
{
	return getAttributeIndex(ID::DEF) != ATTRIBUTE_NOT_FOUND;
}

bool X3DAttributes::isUSE() const
{
	return getAttributeIndex(ID::USE) != ATTRIBUTE_NOT_FOUND;
}

std::string X3DAttributes::getDEF() const
{
	std::string def;
	int index = getAttributeIndex(ID::DEF);
	if (index == ATTRIBUTE_NOT_FOUND)
		return "";
	getSFString(index, def);
	return def;
}

std::string X3DAttributes::getUSE() const
{
	std::string use;
	int index = getAttributeIndex(ID::USE);
	if (index == ATTRIBUTE_NOT_FOUND)
		return "";
	getSFString(index, use);
	return use;
}

} // namespace X3D

