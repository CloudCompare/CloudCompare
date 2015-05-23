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
#include "ccSerializableObject.h"
#include "ccColorTypes.h"

//Qt
#include <QImage>
#include <QString>
#include <QStringList>
#include <QSharedPointer>

//! Mesh (triangle) material
class QCC_DB_LIB_API ccMaterial : public ccSerializableObject
{
public:
	//! Const + Shared type
	typedef QSharedPointer<const ccMaterial> CShared;
	//! Shared type
	typedef QSharedPointer<ccMaterial> Shared;

	//! Default constructor
	ccMaterial(QString name = QString("default"));

	//! Copy constructor
	ccMaterial(const ccMaterial& mtl);

	//! Returns the material name
	inline const QString& getName() const { return m_name; }
	//! Returns the texture filename (if any)
	inline const QString& getTextureFilename() const { return m_textureFilename; }
	//! Sets the material name
	inline void setName(QString name) { m_name = name; }

	//! Sets diffuse color (both front and back)
	void setDiffuse(const ccColor::Rgbaf& color);
	//! Sets diffuse color (front)
	inline void setDiffuseFront(const ccColor::Rgbaf& color) { m_diffuseFront = color; }
	//! Sets diffuse color (back)
	inline void setDiffuseBack(const ccColor::Rgbaf& color) { m_diffuseBack = color; }
	//! Returns front diffuse color
	inline const ccColor::Rgbaf& getDiffuseFront() const { return m_diffuseFront; }
	//! Returns back diffuse color
	inline const ccColor::Rgbaf& getDiffuseBack() const { return m_diffuseBack; }

	//! Sets ambient color
	inline void setAmbient(const ccColor::Rgbaf& color) { m_ambient = color; }
	//! Returns ambient color
	inline const ccColor::Rgbaf& getAmbient() const { return m_ambient; }

	//! Sets specular color
	inline void setSpecular(const ccColor::Rgbaf& color) { m_specular = color; }
	//! Returns specular color
	inline const ccColor::Rgbaf& getSpecular() const { return m_specular; }

	//! Sets emission color
	inline void setEmission(const ccColor::Rgbaf& color) { m_emission = color; }
	//! Returns emission color
	inline const ccColor::Rgbaf& getEmission() const { return m_emission; }

	//! Sets shininess (both front - 100% - and back - 80%)
	void setShininess(float val);
	//! Sets shininess (front)
	inline void setShininessFront(float val) { m_shininessFront = val; }
	//! Sets shininess (back)
	inline void setShininessBack(float val) { m_shininessBack = val; }
	//! Returns front shininess
	inline float getShininessFront() const { return m_shininessFront; }
	//! Returns back shininess
	inline float getShininessBack() const { return m_shininessBack; }

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

	//! Loads texture from file (and set it if successful)
	/** If the filename is not already in DB, the corresponding file will be loaded.
		\return whether the file could be loaded (or is already in DB) or not
	**/
	bool loadAndSetTexture(QString absoluteFilename);

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

	//! Compares this material with another one
	/** \return true if both materials are equivalent or false otherwise
	**/
	bool compare(const ccMaterial& mtl) const;

	//inherited from ccSerializableObject
	virtual bool isSerializable() const { return true; }
	/** \warning Doesn't save the texture image!
	**/
	virtual bool toFile(QFile& out) const;
	virtual bool fromFile(QFile& in, short dataVersion, int flags);

	//! Returns unique identifier (UUID)
	inline QString getUniqueIdentifier() const { return m_uniqueID; }

protected:
	QString m_name;
	QString m_textureFilename;
	QString m_uniqueID;

	ccColor::Rgbaf m_diffuseFront;
	ccColor::Rgbaf m_diffuseBack;
	ccColor::Rgbaf m_ambient;
	ccColor::Rgbaf m_specular;
	ccColor::Rgbaf m_emission;
	float m_shininessFront;
	float m_shininessBack;

	//float m_reflect;
	//float m_refract;
	//float m_trans;
	//float m_glossy;
	//float m_refract_index;
};

#endif //CC_MATERIAL_HEADER