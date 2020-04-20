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

//Always first
#include "ccIncludeGL.h"

#include "ccImage.h"

//Local
#include "ccCameraSensor.h"

//Qt
#include <QFileInfo>
#include <QImageReader>
#include <QOpenGLTexture>

ccImage::ccImage()
	: ccHObject("Not loaded")
	, m_width(0)
	, m_height(0)
	, m_aspectRatio(1.0f)
	, m_texAlpha(1.0f)
	, m_associatedSensor(nullptr)
{
	setVisible(true);
	lockVisibility(false);
	setSelectionBehavior(SELECTION_FIT_BBOX); //same as the camera sensors that are (generally) associated to images
	setEnabled(false);
}

ccImage::ccImage(const QImage& image, const QString& name)
	: ccHObject(name)
	, m_width(image.width())
	, m_height(image.height())
	, m_aspectRatio(1.0f)
	, m_texAlpha(1.0f)
	, m_image(image)
	, m_associatedSensor(nullptr)
{
	updateAspectRatio();
	setVisible(true);
	lockVisibility(false);
	setEnabled(true);
}

bool ccImage::load(const QString& filename, QString& error)
{
	QImageReader reader(filename);
	//m_image = QImage(filename);
	QImage image = reader.read();
	if (image.isNull())
	{
		error = reader.errorString();
		return false;
	}

	setData(image);

	setName(QFileInfo(filename).fileName());
	setEnabled(true);

	return true;
}

void ccImage::setData(const QImage& image)
{
	m_image = image;
	m_width = m_image.width();
	m_height = m_image.height();
	updateAspectRatio();
}

void ccImage::updateAspectRatio()
{
	setAspectRatio(m_height != 0 ? static_cast<float>(m_width) / m_height : 1.0f);
}

void ccImage::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (m_image.isNull())
		return;

	if (!MACRO_Draw2D(context) || !MACRO_Foreground(context))
		return;
		
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert( glFunc != nullptr );
	
	if ( glFunc == nullptr )
		return;

	glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
	glFunc->glEnable(GL_BLEND);
	glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glFunc->glPushAttrib(GL_ENABLE_BIT);
	glFunc->glEnable(GL_TEXTURE_2D);

	QOpenGLTexture texture(m_image);
	texture.bind();
	{
		//we make the texture fit inside viewport
		int realWidth = static_cast<int>(m_height * m_aspectRatio); //take aspect ratio into account!
		GLfloat cw = static_cast<GLfloat>(context.glW) /realWidth;
		GLfloat ch = static_cast<GLfloat>(context.glH) /m_height;
		GLfloat zoomFactor = (cw > ch ? ch : cw) / 2;
		GLfloat dX = realWidth*zoomFactor;
		GLfloat dY = m_height*zoomFactor;

		glFunc->glColor4f(1, 1, 1, m_texAlpha);
		glFunc->glBegin(GL_QUADS);
		glFunc->glTexCoord2f(0, 1); glFunc->glVertex2f(-dX, -dY);
		glFunc->glTexCoord2f(1, 1); glFunc->glVertex2f( dX, -dY);
		glFunc->glTexCoord2f(1, 0); glFunc->glVertex2f( dX,  dY);
		glFunc->glTexCoord2f(0, 0); glFunc->glVertex2f(-dX,  dY);
		glFunc->glEnd();
	}
	texture.release();

	glFunc->glPopAttrib();
	glFunc->glPopAttrib();	
}

void ccImage::setAlpha(float value)
{
	if (value <= 0)
		m_texAlpha = 0;
	else if (value > 1.0f)
		m_texAlpha = 1.0f;
	else
		m_texAlpha = value;
}

void ccImage::setAssociatedSensor(ccCameraSensor* sensor)
{
	m_associatedSensor = sensor;

	if (m_associatedSensor)
		m_associatedSensor->addDependency(this,DP_NOTIFY_OTHER_ON_DELETE);
}

void ccImage::onDeletionOf(const ccHObject* obj)
{
	if (obj == m_associatedSensor)
		setAssociatedSensor(nullptr);

	ccHObject::onDeletionOf(obj);
}

bool ccImage::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//we can't save the associated sensor here (as it may be shared by multiple images)
	//so instead we save it's unique ID (dataVersion>=38)
	//WARNING: the sensor must be saved in the same BIN file! (responsibility of the caller)
	uint32_t sensorUniqueID = (m_associatedSensor ? (uint32_t)m_associatedSensor->getUniqueID() : 0);
	if (out.write((const char*)&sensorUniqueID, 4) < 0)
		return WriteError();

	//for backward compatibility
	float texU = 1.0f;
	float texV = 1.0f;

	QDataStream outStream(&out);
	outStream << m_width;
	outStream << m_height;
	outStream << m_aspectRatio;
	outStream << texU;
	outStream << texV;
	outStream << m_texAlpha;
	outStream << m_image;
	QString fakeString;
	outStream << fakeString; //formerly: 'complete filename'

	return true;
}

bool ccImage::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//as the associated sensor can't be saved directly (as it may be shared by multiple images)
	//we only store its unique ID (dataVersion >= 38) --> we hope we will find it at loading time (i.e. this
	//is the responsibility of the caller to make sure that all dependencies are saved together)
	uint32_t sensorUniqueID = 0;
	if (in.read((char*)&sensorUniqueID,4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the vertices unique ID in the 'm_associatedCloud' pointer!!!
	*(uint32_t*)(&m_associatedSensor) = sensorUniqueID;

	float texU = 1.0f;
	float texV = 1.0f;

	QDataStream inStream(&in);
	inStream >> m_width;
	inStream >> m_height;
	inStream >> m_aspectRatio;
	inStream >> texU;
	inStream >> texV;
	inStream >> m_texAlpha;
	inStream >> m_image;
	QString fakeString;
	inStream >> fakeString; //formerly: 'complete filename'

	return true;
}

ccBBox ccImage::getOwnFitBB(ccGLMatrix& trans)
{
	ccHObject::Container sensors;
	filterChildren(sensors, false, CC_TYPES::SENSOR, false, m_currentDisplay);

	//if we have exactly one sensor child, we use its own bounding-box
	if (sensors.size() == 1)
	{
		return sensors.front()->getOwnFitBB(trans);
	}

	//otherwise we stick to the default behavior
	return ccHObject::getOwnFitBB(trans);
}
