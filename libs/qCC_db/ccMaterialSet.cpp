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

//System
#include <assert.h>

ccMaterialSet::ccMaterialSet(const char* name/*=0*/)
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
		const char* current_token = strtok( current_line, MTL_LOADER_WHITESPACE);

		//skip comments
		if (!current_token || current_token[0]=='/' || current_token[0]=='#')
			continue;

		//start material
		if (strcmp(current_token, "newmtl")==0)
		{
			//push the precedent material
			if (current_mtl_index>=0)
				materials.addMaterial(currentMaterial);

			++current_mtl_index;
			currentMaterial = ccMaterial();
			materials.resize(materials.size()+1);
			// get the name
			currentMaterial.name = QString(strtok(NULL, MTL_LOADER_WHITESPACE));

		}
		else if (current_mtl_index>=0) //we already have a "current" material
		{
			//ambient
			if (strcmp(current_token, "Ka")==0)
			{
				currentMaterial.ambient[0] = (float)atof( strtok(NULL, MTL_LOADER_WHITESPACE));
				currentMaterial.ambient[1] = (float)atof( strtok(NULL, MTL_LOADER_WHITESPACE));
				currentMaterial.ambient[2] = (float)atof( strtok(NULL, MTL_LOADER_WHITESPACE));
			}

			//diff
			else if (strcmp(current_token, "Kd")==0)
			{
				currentMaterial.diffuseFront[0] = (float)atof( strtok(NULL, MTL_LOADER_WHITESPACE));
				currentMaterial.diffuseFront[1] = (float)atof( strtok(NULL, MTL_LOADER_WHITESPACE));
				currentMaterial.diffuseFront[2] = (float)atof( strtok(NULL, MTL_LOADER_WHITESPACE));
				memcpy(currentMaterial.diffuseBack,currentMaterial.diffuseFront,sizeof(float)*3);
			}

			//specular
			else if (strcmp(current_token, "Ks")==0)
			{
				currentMaterial.specular[0] = (float)atof( strtok(NULL, MTL_LOADER_WHITESPACE));
				currentMaterial.specular[1] = (float)atof( strtok(NULL, MTL_LOADER_WHITESPACE));
				currentMaterial.specular[2] = (float)atof( strtok(NULL, MTL_LOADER_WHITESPACE));
			}
			//shiny
			else if (strcmp(current_token, "Ns")==0)
			{
				double shiny = atof( strtok(NULL, MTL_LOADER_WHITESPACE));
				currentMaterial.setShininess((float)shiny);
			}
			//transparent
			else if (strcmp(current_token, "d")==0 || strcmp(current_token, "Tr")==0)
			{
				float trans = (float)atof( strtok(NULL, MTL_LOADER_WHITESPACE));
				currentMaterial.setTransparency(trans);
			}
			//reflection
			else if (strcmp(current_token, "r")==0)
			{
				//ignored
				//currentMaterial.reflect = (float)atof( strtok(NULL, MTL_LOADER_WHITESPACE));
			}
			//glossy
			else if (strcmp(current_token, "sharpness")==0)
			{
				//ignored
				//currentMaterial.glossy = (float)atof( strtok(NULL, MTL_LOADER_WHITESPACE));
			}
			//refract index
			else if (strcmp(current_token, "Ni")==0)
			{
				//ignored
				//currentMaterial.refract_index = (float)atof( strtok(NULL, MTL_LOADER_WHITESPACE));
			}
			// illumination type
			else if (strcmp(current_token, "illum")==0)
			{
				//ignored
			}
			// texture map
			else if (strcmp(current_token, "map_Ka")==0
				|| strcmp(current_token, "map_Kd")==0
				|| strcmp(current_token, "map_Ks")==0
				)
			{
				QString texture_filename=QString(strtok(NULL, "\t\n\r"));
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
				errors << QString("Unknown command '%1' at line %2").arg(current_token).arg(current_line_index);
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

void ccMaterialSet::clone(ccMaterialSet& dest) const
{
	dest.reserve(size());
	ccMaterialSet::const_iterator it = begin();
	for (;it!=end();++it)
	{
		dest.push_back(*it);
		dest.back().texture.detach();
	}
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
