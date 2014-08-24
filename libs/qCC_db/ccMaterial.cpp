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

//Always first
#include "ccIncludeGL.h"

//Local
#include "ccMaterial.h"
#include "ccBasicTypes.h"
#include "ccLog.h"

//Qt
#include <QMap>
#include <QUuid>

//System
#include <string.h>
#include <assert.h>

//Textures DB
QMap<QString, QImage> s_textureDB;

ccMaterial::ccMaterial(QString _name)
	: name(_name)
	, texID(0)
{
	memcpy(diffuseFront,ccColor::bright,sizeof(float)*4);
	memcpy(diffuseBack,ccColor::bright,sizeof(float)*4);
	memcpy(ambient,ccColor::night,sizeof(float)*4);
	memcpy(specular,ccColor::night,sizeof(float)*4);
	memcpy(emission,ccColor::night,sizeof(float)*4);
	setShininess(50.0);
};

ccMaterial::ccMaterial(const ccMaterial& mtl)
	: name(mtl.name)
	, textureFilename(mtl.textureFilename)
	, shininessFront(mtl.shininessFront)
	, shininessBack(mtl.shininessFront)
	, texID(0)
{
	memcpy(diffuseFront,mtl.diffuseFront,sizeof(float)*4);
	memcpy(diffuseBack,mtl.diffuseBack,sizeof(float)*4);
	memcpy(ambient,mtl.ambient,sizeof(float)*4);
	memcpy(specular,mtl.specular,sizeof(float)*4);
	memcpy(emission,mtl.emission,sizeof(float)*4);
}

void ccMaterial::setDiffuse(const float color[4])
{
	memcpy(diffuseFront,color,sizeof(float)*4);
	memcpy(diffuseBack,color,sizeof(float)*4);
}

void ccMaterial::setShininess(float val)
{
	shininessFront = val;
	shininessBack = 0.8f * val;
}

void ccMaterial::setTransparency(float val)
{
	diffuseFront[3] = val;
	diffuseBack[3] = val;
	ambient[3] = val;
	specular[3] = val;
	emission[3] = val;
}

void ccMaterial::applyGL(bool lightEnabled, bool skipDiffuse) const
{
	if (lightEnabled)
	{
		if (!skipDiffuse)
		{
			glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseFront);
			glMaterialfv(GL_BACK, GL_DIFFUSE, diffuseBack);
		}
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,ambient);
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,specular);
		glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,emission);
		glMaterialf(GL_FRONT, GL_SHININESS, shininessFront);
		glMaterialf(GL_BACK, GL_SHININESS, shininessBack);
	}
	else
	{
		glColor4fv(diffuseFront);
	}
}

bool ccMaterial::setTexture(QString absoluteFilename)
{
	if (!s_textureDB.contains(absoluteFilename))
	{
		//try to load the corresponding file
		QImage image(absoluteFilename);
		if (image.isNull())
		{
			ccLog::Warning(QString("[ccMaterial] Failed to load image '%1'").arg(absoluteFilename));
			return false;
		}

		s_textureDB[absoluteFilename] = image.mirrored();
	}

	textureFilename = absoluteFilename;

	return true;
}

void ccMaterial::setTexture(QImage image, QString absoluteFilename/*=QString()*/, bool mirrorImage/*=true*/)
{
	if (absoluteFilename.isEmpty())
	{
		absoluteFilename = QString("tex_") + QUuid::createUuid().toString();
		assert(!s_textureDB.contains(absoluteFilename));
	}
	else
	{
		if (s_textureDB.contains(absoluteFilename))
		{
			if (s_textureDB[absoluteFilename].size() != image.size())
			{
				ccLog::Warning(QString("[ccMaterial] A texture with the same name (%1) but with a different size has already been loaded!").arg(absoluteFilename));
			}
			textureFilename = absoluteFilename;
			return;
		}
	}

	textureFilename = absoluteFilename;

	//insert image into DB if necessary
	s_textureDB[absoluteFilename] = mirrorImage ? image.mirrored() : image;
}

const QImage ccMaterial::getTexture() const
{
	return s_textureDB[textureFilename];
}

bool ccMaterial::hasTexture() const
{
	if (textureFilename.isEmpty())
		return false;

	return !s_textureDB[textureFilename].isNull();
}

void ccMaterial::MakeLightsNeutral()
{
	GLint maxLightCount;
	glGetIntegerv(GL_MAX_LIGHTS,&maxLightCount);
	
	for (int i=0; i<maxLightCount; ++i)
	{
		if (glIsEnabled(GL_LIGHT0+i))
		{
			float diffuse[4];
			float ambiant[4];
			float specular[4];

			glGetLightfv(GL_LIGHT0+i,GL_DIFFUSE,diffuse);
			glGetLightfv(GL_LIGHT0+i,GL_AMBIENT,ambiant);
			glGetLightfv(GL_LIGHT0+i,GL_SPECULAR,specular);

			 diffuse[0] =  diffuse[1] =  diffuse[2] = ( diffuse[0] +  diffuse[1] +  diffuse[2]) / 3.0f;	//'mean' (gray) value
			 ambiant[0] =  ambiant[1] =  ambiant[2] = ( ambiant[0] +  ambiant[1] +  ambiant[2]) / 3.0f;	//'mean' (gray) value
			specular[0] = specular[1] = specular[2] = (specular[0] + specular[1] + specular[2]) / 3.0f;	//'mean' (gray) value

			glLightfv(GL_LIGHT0+i, GL_DIFFUSE, diffuse);
			glLightfv(GL_LIGHT0+i, GL_AMBIENT, ambiant);
			glLightfv(GL_LIGHT0+i,GL_SPECULAR,specular);
		}
	}
}

QImage ccMaterial::GetTexture(QString absoluteFilename)
{
	return s_textureDB[absoluteFilename];
}

void ccMaterial::AddTexture(QImage image, QString absoluteFilename)
{
	s_textureDB[absoluteFilename] = image;
}
