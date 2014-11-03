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

	//! Sets diffuse color (both front and back)
	void setDiffuse(const float color[4]);
	//! Sets diffuse color (front)
	void setDiffuseFront(const float color[4]);
	//! Sets diffuse color (back)
	void setDiffuseBack(const float color[4]);
	//! Returns front diffuse color
	inline const float* getDiffuseFront() const { return m_diffuseFront; }
	//! Returns back diffuse color
	inline const float* getDiffuseBack() const { return m_diffuseBack; }

	//! Sets ambient color
	void setAmbient(const float color[4]);
	//! Returns ambient color
	inline const float* getAmbient() const { return m_ambient; }

	//! Sets specular color
	void setSpecular(const float color[4]);
	//! Returns specular color
	inline const float* getSpecular() const { return m_specular; }

	//! Sets emission color
	void setEmission(const float color[4]);
	//! Returns emission color
	inline const float* getEmission() const { return m_emission; }

	//! Sets shininess (both front - 100% - and back - 80%)
	void setShininess(float val);
	//! Sets shininess (front)
	void setShininessFront(float val);
	//! Sets shininess (back)
	void setShininessBack(float val);
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

	float m_diffuseFront[4];
	float m_diffuseBack[4];
	float m_ambient[4];
	float m_specular[4];
	float m_emission[4];
	float m_shininessFront;
	float m_shininessBack;

	//float m_reflect;
	//float m_refract;
	//float m_trans;
	//float m_glossy;
	//float m_refract_index;
};

#endif //CC_MATERIAL_HEADER