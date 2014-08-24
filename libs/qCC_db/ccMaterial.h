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

#ifndef CC_MATERIAL_HEADER
#define CC_MATERIAL_HEADER

//Local
#include "qCC_db.h"

//Qt
#include <QImage>
#include <QString>
#include <QStringList>

//! Mesh (triangle) material
struct QCC_DB_LIB_API ccMaterial
{
	QString name;
	QString textureFilename;
	//QImage texture;
	float diffuseFront[4];
	float diffuseBack[4];
	float ambient[4];
	float specular[4];
	float emission[4];
	float shininessFront;
	float shininessBack;

	unsigned texID;

	/*float reflect;
	float refract;
	float trans;
	float glossy;
	float refract_index;
	//*/

	//! Default constructor
	ccMaterial(QString name = QString("default"));

	//! Copy constructor
	ccMaterial(const ccMaterial& mtl);

	//! Sets diffuse color (both front and back)
	void setDiffuse(const float color[4]);

	//! Sets shininess (both front - 100% - and back - 80%)
	void setShininess(float val);

	//! Sets transparency (all colors)
	void setTransparency(float val);

	//! Apply parameters (OpenGL)
	void applyGL(bool lightEnabled, bool skipDiffuse) const;

	//! Returns whether the material has an associated texture or not
	bool hasTexture() const;

	//! Sets texture
	/** If no filename is provided, a random one will be generated.
	**/
	void setTexture(QImage image, QString absoluteFilename =  QString(), bool mirrorImage = true);

	//! Sets texture
	/** If the filename is not already in DB, the corresponding file will be loaded.
		\return whether the file could be loaded (or is already in DB)
	**/
	bool setTexture(QString absoluteFilename);

	//! Returns the texture absolute filename (if any)
	QString getAbsoluteFilename() const { return textureFilename; }

	//! Returns the texture (if any)
	const QImage getTexture() const;

	//! Helper: makes all active GL light sources neutral (i.e. 'gray')
	/** WARNING: an OpenGL context must be active!
	**/
	static void MakeLightsNeutral();

	//! Returns the texture image associated to a given name
	static QImage GetTexture(QString absoluteFilename);

	//! Adds a texture to the global texture DB
	static void AddTexture(QImage image, QString absoluteFilename);

};

#endif //CC_MATERIAL_HEADER