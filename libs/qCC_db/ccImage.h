//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_IMAGE_HEADER
#define CC_IMAGE_HEADER

//Local
#include "ccHObject.h"

//Qt
#include <QImage>

class ccCameraSensor;
class QGLBuffer;

//! Generic image
class QCC_DB_LIB_API ccImage : public ccHObject
{
public:

	enum IMAGE_DISPLAY_TYPE
	{
		IMAGE_DISPLAY_2D,	// 2d as foreground
		IMAGE_DISPLAY_2P5D,	// fake 3D
		IMAGE_DISPLAY_3D,	// real 3D
	};

	//! Default constructor
	ccImage();

	//! Constructor from QImage
	ccImage(const QImage& image, const QString& name = QString("unknown"));

	~ccImage();

	//inherited methods (ccHObject)
	virtual bool isSerializable() const override { return true; }

	//! Returns unique class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::IMAGE; }

	//! Loads image from file
	/** \param filename image filename
		\param error a human readable description of what went wrong (if method fails)
		\return success
	**/
	bool load(const QString& filename, QString& error, bool fake = false);

	bool loadWithWidthHeight(const QString& filename, int width, int height, QString& error);

	//! Returns image data
	//inline QImage& data();
	//! Returns image data (const version)
	//inline const QImage& data() const;
	QImage ccImage::data() const;

	//! Sets image data
	void setData(const QImage& image, bool fake = false);

	//! Returns image width
	inline unsigned getW() const { return m_width; }

	//! Returns image height
	inline unsigned getH() const { return m_height; }

	//! Sets image texture transparency
	void setAlpha(float value);

	//! Returns image texture transparency
	inline float getAlpha() const { return m_texAlpha; }

	//! Manually sets aspect ratio
	void setAspectRatio(float ar) { m_aspectRatio = ar; }

	//! Returns aspect ratio
	inline float getAspectRatio() const { return m_aspectRatio; }

	//! Sets associated sensor
	void setAssociatedSensor(ccCameraSensor* sensor);

	//! Returns associated sensor
	ccCameraSensor* getAssociatedSensor() { return m_associatedSensor; }

	//! Returns associated sensor (const version)
	const ccCameraSensor* getAssociatedSensor() const { return m_associatedSensor; }

	void removeFromDisplay(const ccGenericGLDisplay* win) override; //for proper VBO release

	void setDisplay(ccGenericGLDisplay* win) override;

	QString getImagePath() { return m_file_name; }
	//inherited from ccHObject
	virtual ccBBox getOwnFitBB(ccGLMatrix& trans) override;

	void setDisplayType(IMAGE_DISPLAY_TYPE type) { m_display_type = type; }

protected:

	//inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;
	virtual void onDeletionOf(const ccHObject* obj) override;
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags) override;

	//! Updates aspect ratio
	void updateAspectRatio();

	//! Image width (in pixels)
	unsigned m_width;
	//! Image height (in pixels)
	unsigned m_height;

	//! Aspect ratio w/h
	/** Default is m_width/m_height.
		Should be changed if pixels are not square.
	**/
	float m_aspectRatio;

	//! Texture transparency
	float m_texAlpha;

	//! Image data
	QImage m_image;

	//! Associated sensor
	ccCameraSensor* m_associatedSensor;

	QString m_file_name;

	IMAGE_DISPLAY_TYPE m_display_type;

// vbo
	QGLBuffer* m_vbo;
	GLuint m_texture_id;
	int m_vbo_state;// 0-new, 1-initialized, 2-failed
	int vboInit(int count);
	bool updateVBO(const CC_DRAW_CONTEXT & context);
	void releaseVBO();
public:
	bool updateTexBuffer();

};

#endif //CC_IMAGE_HEADER
