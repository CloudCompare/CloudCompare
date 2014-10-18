//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccMaterialSet.h"

//Local
#include "ccGenericGLDisplay.h"

//Qt
#include <QDataStream>
#include <QStringList>
#include <QString>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QMap>

//System
#include <string.h>
#include <assert.h>
#include <set>

ccMaterialSet::ccMaterialSet(QString name/*=QString()*/)
	: std::vector<ccMaterial>()
	, CCShareable()
	, ccHObject(name)
	, m_display(0)
{
	setFlagState(CC_LOCKED,true);
}

ccMaterialSet::~ccMaterialSet()
{
	associateTo(0);
}

int ccMaterialSet::findMaterial(QString mtlName)
{
	unsigned i = 0;
	for (ccMaterialSet::const_iterator it = begin(); it != end(); ++it,++i)
		if (it->name == mtlName)
			return static_cast<int>(i);

	return -1;
}

bool ccMaterialSet::addMaterial(const ccMaterial& mat)
{
	if (findMaterial(mat.name) >= 0)
		return false;

	try
	{
		push_back(mat);
	}
	catch(std::bad_alloc)
	{
		//not enough memory
		return false;
	}

	return true;
}

//MTL PARSER INSPIRED FROM KIXOR.NET "objloader" (http://www.kixor.net/dev/objloader/)
bool ccMaterialSet::ParseMTL(QString path, const QString& filename, ccMaterialSet &materials, QStringList& errors)
{
	//open mtl file
	QString fullPathFilename = path + QString('/') + filename;
	QFile file(fullPathFilename);
	if (!file.open(QFile::ReadOnly))
	{
		errors << QString("Error reading file: %1").arg(filename);
		return false;
	}

	//update path (if the input filename has already a relative path)
	path = QFileInfo(fullPathFilename).absolutePath();
	
	QTextStream stream(&file);

	QString currentLine = stream.readLine();
	unsigned currentLineIndex = 0;
	int currentMatIndex = -1;
	ccMaterial currentMaterial;
	while( !currentLine.isNull() )
	{
		++currentLineIndex;

		QStringList tokens = currentLine.split(QRegExp("\\s+"),QString::SkipEmptyParts);

		//skip comments & empty lines
		if( tokens.empty() || tokens.front().startsWith('/',Qt::CaseInsensitive) || tokens.front().startsWith('#',Qt::CaseInsensitive) )
		{
			currentLine = stream.readLine();
			continue;
		}

		//start material
		if (tokens.front() == "newmtl")
		{
			//push the previous material
			if (currentMatIndex >= 0)
				materials.addMaterial(currentMaterial);

			++currentMatIndex;
			currentMaterial = ccMaterial();
			//materials.resize(materials.size()+1);
			// get the name
			currentMaterial.name = (tokens.size()>1 ? tokens[1] : "undefined");

		}
		else if (currentMatIndex >= 0) //we already have a "current" material
		{
			//ambient
			if (tokens.front() == "Ka")
			{
				if (tokens.size() > 3)
				{
					currentMaterial.ambient[0] = tokens[1].toFloat();
					currentMaterial.ambient[1] = tokens[2].toFloat();
					currentMaterial.ambient[2] = tokens[3].toFloat();
				}
			}

			//diff
			else if (tokens.front() == "Kd")
			{
				if (tokens.size() > 3)
				{
					currentMaterial.diffuseFront[0] = tokens[1].toFloat();
					currentMaterial.diffuseFront[1] = tokens[2].toFloat();
					currentMaterial.diffuseFront[2] = tokens[3].toFloat();
					//duplicate
					memcpy(currentMaterial.diffuseBack,currentMaterial.diffuseFront,sizeof(float)*3);
				}
			}

			//specular
			else if (tokens.front() == "Ks")
			{
				if (tokens.size() > 3)
				{
					currentMaterial.specular[0] = tokens[1].toFloat();
					currentMaterial.specular[1] = tokens[2].toFloat();
					currentMaterial.specular[2] = tokens[3].toFloat();
				}
			}
			//shiny
			else if (tokens.front() == "Ns")
			{
				if (tokens.size() > 1)
					currentMaterial.setShininess(tokens[1].toFloat());
			}
			//transparent
			else if (tokens.front() == "d" || tokens.front() == "Tr")
			{
				if (tokens.size() > 1)
					currentMaterial.setTransparency(tokens[1].toFloat());
			}
			//reflection
			else if (tokens.front() == "r")
			{
				//ignored
				//if (tokens.size() > 1)
				//	currentMaterial.reflect = tokens[1].toFloat();
			}
			//glossy
			else if (tokens.front() == "sharpness")
			{
				//ignored
				//if (tokens.size() > 1)
				//	currentMaterial.glossy = tokens[1].toFloat();
			}
			//refract index
			else if (tokens.front() == "Ni")
			{
				//ignored
				//if (tokens.size() > 1)
				//	currentMaterial.refract_index = tokens[1].toFloat();
			}
			// illumination type
			else if (tokens.front() == "illum")
			{
				//ignored
			}
			// texture map
			else if (tokens.front() == "map_Ka"
					|| tokens.front() == "map_Kd"
					|| tokens.front() == "map_Ks")
			{
				//DGM: in case there's hidden or space characters at the beginning of the line...
				int shift = currentLine.indexOf("map_K",0);
				QString textureFilename = (shift + 7 < currentLine.size() ? currentLine.mid(shift+7).trimmed() : QString());
				QString fullTexName = path + QString('/') + textureFilename;
				if (!currentMaterial.setTexture(fullTexName))
				{
					errors << QString("Failed to load texture file: %1").arg(fullTexName);
				}
			}
			else
			{
				errors << QString("Unknown command '%1' at line %2").arg(tokens.front()).arg(currentLineIndex);
			}
		}

		currentLine = stream.readLine();
	}

	file.close();

	//push the last material
	if (currentMatIndex >= 0)
		materials.addMaterial(currentMaterial);

	return true;
}

bool ccMaterialSet::saveAsMTL(QString path, const QString& baseFilename, QStringList& errors) const
{
	//open mtl file
	QString filename = path+QString('/')+baseFilename+QString(".mtl");
	QFile file(filename);
	if (!file.open(QFile::WriteOnly))
	{
		errors << QString("Error writing file: %1").arg(filename);
		return false;
	}
	QTextStream stream(&file);

	stream << "# Generated by CloudCompare" << endl;

	//texture filenames already used
	QMap<QString,QString> filenamesSaved;

	unsigned texIndex = 0;
	for (ccMaterialSet::const_iterator it=begin(); it!=end(); ++it)
	{
		stream << endl << "newmtl " << it->name << endl;

		stream << "Ka " << it->ambient[0]		<< " " << it->ambient[1]		<< " " << it->ambient[2]		<< endl;
		stream << "Kd " << it->diffuseFront[0]	<< " " << it->diffuseFront[1]	<< " " << it->diffuseFront[2]	<< endl;
		stream << "Ks " << it->specular[0]		<< " " << it->specular[1]		<< " " << it->specular[2]		<< endl;
		stream << "Tr " << it->ambient[3]		<< endl; //we take the ambient's by default
		stream << "illum 1"						<< endl;
		stream << "Ns " << it->shininessFront	<< endl; //we take the front's by default

		if (it->hasTexture())
		{
			QString absFilename = it->getAbsoluteFilename();

			//file has not already been saved?
			if (!filenamesSaved.contains(absFilename))
			{
				QFileInfo fileInfo(absFilename);

				QString texName = fileInfo.fileName();
				if (fileInfo.suffix().isEmpty())
					texName += QString(".jpg");

				//new absolute filemane
				filenamesSaved[absFilename] = path + QString('/') + texName;
			}

			assert(!filenamesSaved[absFilename].isEmpty());
			absFilename = filenamesSaved[absFilename];
			if (it->getTexture().mirrored().save(absFilename)) //mirrored: see ccMaterial
			{
				stream << "map_Kd " << QFileInfo(absFilename).fileName() << endl;
			}
			else
			{
				errors << QString("Failed to save texture #%1 to file '%2'!").arg(texIndex).arg(absFilename);
			}
			++texIndex;
		}
	}

	file.close();


	return true;
}

//Number of times a texture is 'used'
typedef QMap< /*key=*/unsigned, /*usageCount=*/unsigned > TextureUsageMap;
//Shared version of TextureUsageMap
typedef QSharedPointer<TextureUsageMap> _TextureUsageMap;
//Association between a display and a 'texture usage map'
static QMap< ccGenericGLDisplay*, _TextureUsageMap > s_textureToDisplayAssociation;

void ccMaterialSet::associateTo(ccGenericGLDisplay* display)
{
	if (m_display == display)
		return;

	for (ccMaterialSet::iterator it = begin(); it!=end(); ++it)
	{
		if (it->hasTexture())
		{
			//we release texture from old display (if any)
			if (it->texID != 0 && m_display)
			{
				//check if the texture is declared in the 'texture usage map'
				//of the former display
				bool releaseTexture = false;
				_TextureUsageMap& usageMap = s_textureToDisplayAssociation[m_display];
				assert(usageMap); //should already exist!
				if (usageMap)
				{
					unsigned usageCount = usageMap->value(it->texID);
					assert(usageCount != 0);
					if (usageCount < 2)
					{
						//we can release it!
						usageMap->remove(it->texID);
						releaseTexture = true;

						//shall we clean everything or leave empty shells?
						if (usageMap->size() == 0)
							s_textureToDisplayAssociation.remove(m_display);
					}
					else
					{
						//we only decrease its usage count
						(*usageMap)[it->texID]--;
					}
				}

				if (releaseTexture)
					m_display->releaseTexture(it->texID);
			}

			//we register texture in the new one (if any)
			it->texID = (display ? display->getTexture(it->getTexture()) : 0);

			//Register new texID
			if (it->texID)
			{
				_TextureUsageMap& usageMap = s_textureToDisplayAssociation[display];
				//Create map for the new display if necessary
				if (!usageMap)
					usageMap = _TextureUsageMap(new TextureUsageMap);
				//increase this texture ID usage count
				(*usageMap)[it->texID]++;
			}
		}
	}

	m_display = display;
}

const ccGenericGLDisplay* ccMaterialSet::getAssociatedDisplay()
{
	return m_display;
}

bool ccMaterialSet::append(const ccMaterialSet& source)
{
	try
	{
		reserve(size() + source.size());
	}
	catch(.../*const std::bad_alloc&*/) //out of memory
	{
		ccLog::Warning("[ccMaterialSet::append] Not enough memory");
		return false;
	}
	
	for (ccMaterialSet::const_iterator it=source.begin(); it!=source.end(); ++it)
	{
		push_back(*it);
		//back().getTexture().detach(); //FIXME: was in the old version (non shared images)... still meaningful? or even necessary?
	}

	return true;
}


ccMaterialSet* ccMaterialSet::clone() const
{
	ccMaterialSet* cloneSet = new ccMaterialSet(getName());
	if (!cloneSet->append(*this))
	{
		ccLog::Warning("[ccMaterialSet::clone] Not enough memory");
		cloneSet->release();
		cloneSet = 0;
	}

	return cloneSet;
}

bool ccMaterialSet::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//Materials count (dataVersion>=20)
	uint32_t count = (uint32_t)size();
	if (out.write((const char*)&count,4) < 0)
		return WriteError();

	//texture filenames
	std::set<QString> texFilenames;

	QDataStream outStream(&out);

	//Write each material
	for (ccMaterialSet::const_iterator it = begin(); it!=end(); ++it)
	{
		//material name (dataVersion>=20)
		outStream << it->name;
		//texture (dataVersion>=20)
		outStream << it->textureFilename;
		texFilenames.insert(it->textureFilename);
		//material colors (dataVersion>=20)
		//we don't use QByteArray here as it has its own versions!
		if (out.write((const char*)it->diffuseFront,sizeof(float)*4) < 0) 
			return WriteError();
		if (out.write((const char*)it->diffuseBack,sizeof(float)*4) < 0) 
			return WriteError();
		if (out.write((const char*)it->ambient,sizeof(float)*4) < 0) 
			return WriteError();
		if (out.write((const char*)it->specular,sizeof(float)*4) < 0) 
			return WriteError();
		if (out.write((const char*)it->emission,sizeof(float)*4) < 0) 
			return WriteError();
		//material shininess (dataVersion>=20)
		outStream << it->shininessFront;
		outStream << it->shininessBack;
	}

	//now save the number of textures (dataVersion>=37)
	outStream << static_cast<uint32_t>(texFilenames.size());
	//and save the textures (dataVersion>=37)
	{
		for (std::set<QString>::const_iterator it=texFilenames.begin(); it!=texFilenames.end(); ++it)
		{
			outStream << *it; //name
			outStream << ccMaterial::GetTexture(*it); //then image
		}
	}

	return true;
}

bool ccMaterialSet::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//Materials count (dataVersion>=20)
	uint32_t count = 0;;
	if (in.read((char*)&count,4) < 0)
		return ReadError();
	if (count == 0)
		return true;

	//Read each material
	try
	{
		resize(count);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return MemoryError();
	}

	QDataStream inStream(&in);
	
	for (ccMaterialSet::iterator it = begin(); it!=end(); ++it)
	{

		//material name (dataVersion>=20)
		inStream >> it->name;
		if (dataVersion < 37)
		{
			//texture (dataVersion>=20)
			QImage texture;
			inStream >> texture;
			it->setTexture(texture,QString(),false);
		}
		else
		{
			//texture 'filename' (dataVersion>=37)
			inStream >> it->textureFilename;
		}
		//material colors (dataVersion>=20)
		if (in.read((char*)it->diffuseFront,sizeof(float)*4) < 0) 
			return ReadError();
		if (in.read((char*)it->diffuseBack,sizeof(float)*4) < 0) 
			return ReadError();
		if (in.read((char*)it->ambient,sizeof(float)*4) < 0) 
			return ReadError();
		if (in.read((char*)it->specular,sizeof(float)*4) < 0) 
			return ReadError();
		if (in.read((char*)it->emission,sizeof(float)*4) < 0) 
			return ReadError();
		//material shininess (dataVersion>=20)
		inStream >> it->shininessFront;
		inStream >> it->shininessBack;
	}

	if (dataVersion >= 37)
	{
		//now load the number of textures (dataVersion>=37)
		uint32_t texCount = 0;
		inStream >> texCount;
		//and load the textures (dataVersion>=37)
		{
			for (uint32_t i=0; i<texCount; ++i)
			{
				QString filename;
				inStream >> filename;
				QImage image;
				inStream >> image;
				ccMaterial::AddTexture(image,filename);
			}
		}
	}

	return true;
}
