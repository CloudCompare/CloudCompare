#include <xiot/X3DParserVocabulary.h>
#include <xiot/X3DTypes.h>

namespace XIOT {

X3DParserVocabulary::X3DParserVocabulary() :  DefaultParserVocabulary() 
{
	_externalVocabularyURI = "urn:external-vocabulary";
	X3DTypes::initMaps();

	// Init X3D Element name table
	std::map<int, std::string>::iterator I = X3DTypes::elementFromIDMap.begin();
	while(I != X3DTypes::elementFromIDMap.end())
	{
		_elementNames.push_back(FI::QualifiedName((*I).second));
		I++;
	}

	// Init X3D Attribute name table
	std::map<int, std::string>::iterator A = X3DTypes::attributeFromIDMap.begin();
	while(A != X3DTypes::attributeFromIDMap.end())
	{
		_attributeNames.push_back(FI::QualifiedName((*A).second));
		A++;
	}

	// Init Attribute value table
	_attributeValues.push_back("false"); // Index 1
	_attributeValues.push_back("true"); // Index 2

	addEncodingAlgorithm(NULL);
	addEncodingAlgorithm(&_deltazlibIntArrayAlgorithm);
	addEncodingAlgorithm(&_quantizedzlibFloatArrayAlgorithm);
};






} // namespace FI
