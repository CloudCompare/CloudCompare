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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#include "ccMaterialSet.h"

//Local
#include "ccGenericGLDisplay.h"

//Qt
#include <QDataStream>
#include <QStringList>
#include <QString>

//System
#include <assert.h>

ccMaterialSet::ccMaterialSet(QString name/*=QString()*/)
	: std::vector<ccMaterial>()
	, CCShareable()
	, ccHObject(name)
	, m_display(0)
{
	setFlagState(CC_LOCKED,true);
	setFlagState(CC_FATHER_DEPENDANT,false);
}

ccMaterialSet::~ccMaterialSet()
{
	associateTo(0);
}

int ccMaterialSet::findMaterial(QString mtlName)
{
	unsigned i=0;
	ccMaterialSet::const_iterator it = begin();
	for (; it != end(); ++it,++i)
		if (it->name == mtlName)
			return (int)i;

	return -1;
}

bool ccMaterialSet::addMaterial(const ccMaterial& mat)
{
	if (findMaterial(mat.name)>=0)
		return false;

	push_back(mat);
	return  true;
}

#define MTL_LOADER_WHITESPACE " \t\n\r"

//! Max number of characters per line in an ASCII file
const int MAX_ASCII_FILE_LINE_LENGTH	=	4096;

//MTL PARSER INSPIRED FROM KIXOR.NET "objloader" (http://www.kixor.net/dev/objloader/)
bool ccMaterialSet::ParseMTL(QString path, const QString& filename, ccMaterialSet &materials, QStringList& errors)
{
	char current_line[MAX_ASCII_FILE_LINE_LENGTH];
	unsigned current_line_index = 0;
	int current_mtl_index = -1;
	ccMaterial currentMaterial;

	// open scene
	QString fullName = path+QString('/')+filename;
	FILE *fp = fopen(qPrintable(fullName), "r");
	if(fp == 0)
	{
		errors << QString("Error reading file: %1").arg(filename);
		return false;
	}

	while( fgets(current_line, MAX_ASCII_FILE_LINE_LENGTH, fp) )
	{
		++current_line_index;

		QStringList tokens = QString(current_line).split(QRegExp("\\s+"),QString::SkipEmptyParts);

		//skip comments & empty lines
		if( tokens.empty() || tokens.front().startsWith('/',Qt::CaseInsensitive) || tokens.front().startsWith('#',Qt::CaseInsensitive) )
			continue;

		//start material
		if (tokens.front() == "newmtl")
		{
			//push the precedent material
			if (current_mtl_index>=0)
				materials.addMaterial(currentMaterial);

			++current_mtl_index;
			currentMaterial = ccMaterial();
			materials.resize(materials.size()+1);
			// get the name
			currentMaterial.name = (tokens.size()>1 ? tokens[1] : "undefined");

		}
		else if (current_mtl_index>=0) //we already have a "current" material
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
				QString texture_filename = QString(current_line+7).trimmed();
				QString fullTexName = path+QString('/')+texture_filename;
				QImage image;
				image.load(fullTexName);
				if (!image.load(fullTexName))
				{
					errors << QString("Failed to load texture file: %1").arg(fullTexName);
				}
				else
				{
					currentMaterial.texture = image.mirrored(); //mirrored = WTF?!
				}
			}
			else
			{
				errors << QString("Unknown command '%1' at line %2").arg(tokens.front()).arg(current_line_index);
			}
		}
	}

	//push the last material
	if (current_mtl_index>=0)
		materials.addMaterial(currentMaterial);

	fclose(fp);
	fp=0;

	return true;
}

void ccMaterialSet::associateTo(ccGenericGLDisplay* display)
{
	if (m_display == display)
		return;

	ccMaterialSet::iterator it = begin();
	for (;it!=end();++it)
		if (!it->texture.isNull())
		{
			//we release texture from old display (if any)
			if (m_display)
				m_display->releaseTexture(it->texID);
			//we register texture in the new one (if any)
			//it->texID = 0;
			it->texID = (display ? display->getTexture(it->texture) : 0);
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
		reserve(size()+source.size());
	}
	catch(.../*const std::bad_alloc&*/) //out of memory
	{
		ccLog::Warning("[ccMaterialSet::append] Not enough memory");
		return false;
	}
	
	for (ccMaterialSet::const_iterator it = source.begin(); it!=source.end(); ++it)
	{
		push_back(*it);
		back().texture.detach();
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
		cloneSet=0;
	}

	return cloneSet;
}

bool ccMaterialSet::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//Materials count (dataVersion>=20)
	uint32_t count = (uint32_t)size();
	if (out.write((const char*)&count,4)<0)
		return WriteError();

	//Write each material
	for (ccMaterialSet::const_iterator it = begin();it!=end();++it)
	{
		//material name (dataVersion>=20)
		QDataStream outStream(&out);
		outStream << it->name;
		//texture (dataVersion>=20)
		outStream << it->texture;
		//material colors (dataVersion>=20)
		//we don't use QByteArray here as it has its own versions!
		if (out.write((const char*)it->diffuseFront,sizeof(float)*4)<0) 
			return WriteError();
		if (out.write((const char*)it->diffuseBack,sizeof(float)*4)<0) 
			return WriteError();
		if (out.write((const char*)it->ambient,sizeof(float)*4)<0) 
			return WriteError();
		if (out.write((const char*)it->specular,sizeof(float)*4)<0) 
			return WriteError();
		if (out.write((const char*)it->emission,sizeof(float)*4)<0) 
			return WriteError();
		//material shininess (dataVersion>=20)
		outStream << it->shininessFront;
		outStream << it->shininessBack;
	}

	return true;
}

bool ccMaterialSet::fromFile_MeOnly(QFile& in, short dataVersion)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion))
		return false;

	//Materials count (dataVersion>=20)
	uint32_t count = 0;;
	if (in.read((char*)&count,4)<0)
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
	for (ccMaterialSet::iterator it = begin();it!=end();++it)
	{

		//material name (dataVersion>=20)
		QDataStream inStream(&in);
		inStream >> it->name;
		//texture (dataVersion>=20)
		inStream >> it->texture;
		//material colors (dataVersion>=20)
		if (in.read((char*)it->diffuseFront,sizeof(float)*4)<0) 
			return ReadError();
		if (in.read((char*)it->diffuseBack,sizeof(float)*4)<0) 
			return ReadError();
		if (in.read((char*)it->ambient,sizeof(float)*4)<0) 
			return ReadError();
		if (in.read((char*)it->specular,sizeof(float)*4)<0) 
			return ReadError();
		if (in.read((char*)it->emission,sizeof(float)*4)<0) 
			return ReadError();
		//material shininess (dataVersion>=20)
		inStream >> it->shininessFront;
		inStream >> it->shininessBack;
	}

	return true;
}
