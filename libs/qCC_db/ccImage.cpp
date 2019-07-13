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
#include <QGLBuffer>
#include <QGL>

ccImage::ccImage()
	: ccHObject("Not loaded")
	, m_width(0)
	, m_height(0)
	, m_aspectRatio(1.0f)
	, m_texAlpha(1.0f)
	, m_associatedSensor(0)
	, m_display_type(IMAGE_DISPLAY_2D)
	, m_vbo(nullptr)
	, m_vbo_state(0)
	, m_texture_id(0)
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
	, m_associatedSensor(0)
	, m_display_type(IMAGE_DISPLAY_2D)
	, m_vbo(nullptr)
	, m_vbo_state(0)
	, m_texture_id(0)
{
	updateAspectRatio();
	setVisible(true);
	lockVisibility(false);
	setEnabled(true);
}

ccImage::~ccImage()
{
	releaseVBO();
}

bool ccImage::load(const QString& filename, QString& error, bool fake)
{
	QImageReader reader(filename);
	//m_image = QImage(filename);
	QImage image = reader.read();
	if (image.isNull())
	{
		error = reader.errorString();
		return false;
	}

	setData(image, fake);
	m_file_name = filename;

	setName(QFileInfo(filename).fileName());
	setEnabled(true);

	return true;
}

bool ccImage::loadWithWidthHeight(const QString & filename, int width, int height, QString & error)
{
	m_file_name = filename;
	setName(QFileInfo(filename).fileName());
	setEnabled(true);

	m_width = width;
	m_height = height;
	updateAspectRatio();
	return true;
}

// inline QImage & ccImage::data()
// {
// 	if (m_image.isNull()) {
// 		QImageReader reader(m_file_name);
// 		return reader.read();
// 	}
// 	else {
// 		return m_image;
// 	}
// }
QImage ccImage::data() const
//inline const QImage & ccImage::data() const
{
	if (m_image.isNull()) {
		QImageReader reader(m_file_name);
		return reader.read();
	}
	else {
		return m_image;
	}
}

void ccImage::setData(const QImage& image, bool fake)
{
	if (!fake) {
		m_image = image;
	}	
	m_width = image.width();
	m_height = image.height();
	updateAspectRatio();
}

void ccImage::updateAspectRatio()
{
	setAspectRatio(m_height != 0 ? static_cast<float>(m_width) / m_height : 1.0f);
}

int ccImage::vboInit(int count)
{
	if (!m_vbo->isCreated()) {
		if (!m_vbo->create()) {
			return false;
		}
		m_vbo->setUsagePattern(QGLBuffer::DynamicDraw);
	}
	if (!m_vbo->bind()) {
		m_vbo->destroy();
		return false;
	}
	int totalSizeBytes = count * sizeof(CCVector3);
	totalSizeBytes += count * sizeof(CCVector2);
	if (totalSizeBytes != m_vbo->size()) {
		m_vbo->allocate(totalSizeBytes);
		if (m_vbo->size() != totalSizeBytes) {
			m_vbo->release();
			m_vbo->destroy();
			return false;
		}
	}
	m_vbo->release();
	return totalSizeBytes;
}

static CCVector3 s_points[6];
static CCVector2 s_uvs[6];

bool ccImage::updateVBO(const CC_DRAW_CONTEXT & context)
{
	if (m_vbo_state == 2) {return false;}
	if (m_vbo_state == 1) {return true;} // if no update
	
	if (!m_vbo)	{
		m_vbo = new QGLBuffer;
	}
	int vboSizeBytes = vboInit(6); // 2 triangles
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if (glFunc) {
		if (glFunc->glGetError() != 0) {
			return false;
		}
	}

	if (vboSizeBytes > 0) {
		m_vbo->bind();
		// write vertices
		{
			CCVector3* v = s_points;
			*v++ = CCVector3(0, 0, 0);
			*v++ = CCVector3(m_width, 0, 0);
			*v++ = CCVector3(m_width, m_height, 0);

			*v++ = CCVector3(m_width, m_height, 0);
			*v++ = CCVector3(0, m_height, 0);
			*v++ = CCVector3(0, 0, 0);
		}
		m_vbo->write(0, s_points, sizeof(CCVector3) * 2 * 3);

		// write texture index
		{
			CCVector2* v = s_uvs;
			*v++ = CCVector2(0, 0);
			*v++ = CCVector2(1, 0);
			*v++ = CCVector2(1, 1);

			*v++ = CCVector2(1, 1);
			*v++ = CCVector2(0, 1);
			*v++ = CCVector2(0, 0);
		}
		m_vbo->write(sizeof(CCVector3) * 2 * 3, s_uvs, sizeof(CCVector2) * 2 * 3);
		m_vbo->release();
	}

	if (vboSizeBytes < 0) {
		m_vbo->destroy();
		delete m_vbo;
		m_vbo = nullptr;

		m_vbo_state = 2;
		return false;
	}
	if (glFunc) {
		GLenum er = glFunc->glGetError();
		if (glFunc->glGetError() != 0) {
			m_vbo_state = 2;
			return false;
		}
	}
	
	if (!updateTexBuffer()) {
		m_vbo_state = 2;
		return false;
	}

	m_vbo_state = 1;

	return true;
}

bool ccImage::updateTexBuffer()
{
	if (m_texture_id) {
		glDeleteTextures(1, &m_texture_id);
	}
	if (m_image.isNull()) {
		return false;
	}
	glGenTextures(1, /*(GLuint*)*/&m_texture_id);
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	QImage img = QGLWidget::convertToGLFormat(m_image);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, img.width(), img.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, img.bits());
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glBindTexture(GL_TEXTURE_2D, 0);
	return true;
}

void ccImage::releaseVBO()
{
	if (m_texture_id) {
		glDeleteTextures(1, &m_texture_id);
	}

	if (m_vbo_state == 0) {
		return;
	}

	if (m_vbo) {
		m_vbo->destroy();
		delete m_vbo;
		m_vbo = nullptr;
	}

	return;
}

void ccImage::drawMeOnly(CC_DRAW_CONTEXT& context)
{	
	switch (m_display_type)
	{
	case ccImage::IMAGE_DISPLAY_2D:
	{
		if (!MACRO_Draw2D(context) || !MACRO_Foreground(context))
			return;

		if (m_image.isNull()) return;

		//get the set of OpenGL functions (version 2.1)
		QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
		assert(glFunc != nullptr);

		if (glFunc == nullptr)
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
			GLfloat cw = static_cast<GLfloat>(context.glW) / realWidth;
			GLfloat ch = static_cast<GLfloat>(context.glH) / m_height;
			GLfloat zoomFactor = (cw > ch ? ch : cw) / 2;
			GLfloat dX = realWidth * zoomFactor;
			GLfloat dY = m_height * zoomFactor;

			glFunc->glColor4f(1, 1, 1, m_texAlpha);
			glFunc->glBegin(GL_QUADS);
			glFunc->glTexCoord2f(0, 1); glFunc->glVertex2f(-dX, -dY);
			glFunc->glTexCoord2f(1, 1); glFunc->glVertex2f(dX, -dY);
			glFunc->glTexCoord2f(1, 0); glFunc->glVertex2f(dX, dY);
			glFunc->glTexCoord2f(0, 0); glFunc->glVertex2f(-dX, dY);
			glFunc->glEnd();
		}
		texture.release();

		glFunc->glPopAttrib();
		glFunc->glPopAttrib();

		break;
	}		
	case ccImage::IMAGE_DISPLAY_2P5D:
	{
		if (!MACRO_Draw3D(context))
			return;

		//get the set of OpenGL functions (version 2.1)
		QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
		assert(glFunc != nullptr);

		if (glFunc == nullptr)
			return;

		glFunc->glMatrixMode(GL_MODELVIEW);
		glFunc->glPushMatrix();

		glFunc->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		glFunc->glBegin(GL_LINE_LOOP);
		ccGL::Vertex3(glFunc, 0.f, 0.f, 0.f);
		ccGL::Vertex3(glFunc, m_width, 0.f, 0.f);
		ccGL::Vertex3(glFunc, m_width, m_height, 0.f);
		ccGL::Vertex3(glFunc, 0.f, m_height, 0.f);
		glFunc->glEnd();

		glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
		glFunc->glEnable(GL_BLEND);
		glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glFunc->glPushAttrib(GL_ENABLE_BIT);
		glFunc->glEnable(GL_TEXTURE_2D);

		bool useVBO = false;
		if (context.useVBOs) {
			useVBO = updateVBO(context);
		}

		if (useVBO) {
			glFunc->glColor4f(1, 1, 1, m_texAlpha);
			glFunc->glEnableClientState(GL_VERTEX_ARRAY);
			glFunc->glEnableClientState(GL_TEXTURE_COORD_ARRAY);
 			if (m_vbo->bind()) {
 				glFunc->glVertexPointer(3, GL_FLOAT, 0, nullptr);

				const GLbyte* start = nullptr;
				int Shift = sizeof(CCVector3) * 2 * 3;
 				glFunc->glTexCoordPointer(2, GL_FLOAT, 0, static_cast<const GLvoid*>(start + Shift));
 				m_vbo->release(); 				
 			}

			glBindTexture(GL_TEXTURE_2D, m_texture_id);
			glDrawArrays(GL_TRIANGLES, 0, 6);
			glBindTexture(GL_TEXTURE_2D, 0);

			glFunc->glDisableClientState(GL_VERTEX_ARRAY);
			glFunc->glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		}
		else if (!m_image.isNull()) {			 
			QOpenGLTexture texture(m_image); // did not transfer to gl format, so the vertex tex coord is upside down
			texture.bind();
			{
				// glBindTexture(GL_TEXTURE_2D, m_texture_id);
				glFunc->glColor4f(1, 1, 1, m_texAlpha);
				glFunc->glBegin(GL_QUADS);
				glFunc->glTexCoord2f(0, 1); glFunc->glVertex3f(0, 0, 0);
				glFunc->glTexCoord2f(1, 1); glFunc->glVertex3f(m_width, 0, 0);
				glFunc->glTexCoord2f(1, 0); glFunc->glVertex3f(m_width, m_height, 0);
				glFunc->glTexCoord2f(0, 0); glFunc->glVertex3f(0, m_height, 0);
				glFunc->glEnd();
				// glBindTexture(GL_TEXTURE_2D, 0);
			}
			texture.release();			
		}

		glFunc->glPopAttrib();
		glFunc->glPopAttrib();

		glFunc->glPopMatrix();

		break;
	}
	case ccImage::IMAGE_DISPLAY_3D:
	{
		if (!m_associatedSensor) {
			return;
		}
		//! now it is displayed in camera sensor
		break;
	}
	default:
		break;
	}
	
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
	if (obj == m_associatedSensor) {
		setAssociatedSensor(0);
	}
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
	float texU = 1.0f, texV = 1.0f;

	QDataStream outStream(&out);
	outStream << m_width;
	outStream << m_height;
	outStream << m_aspectRatio;
	outStream << texU;
	outStream << texV;
	outStream << m_texAlpha;
	outStream << m_image;
	outStream << m_file_name; //formerly: 'complete filename'

	return true;
}

bool ccImage::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//as the associated sensor can't be saved directly (as it may be shared by multiple images)
	//we only store its unique ID (dataVersion >= 38) --> we hope we will find it at loading time (i.e. this
	//is the responsibility of the caller to make sure that all dependencies are saved together)
	uint32_t sensorUniqueID = 0;
	if (in.read((char*)&sensorUniqueID,4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the vertices unique ID in the 'm_associatedCloud' pointer!!!
	*(uint32_t*)(&m_associatedSensor) = sensorUniqueID;

	float texU, texV;

	QDataStream inStream(&in);
	inStream >> m_width;
	inStream >> m_height;
	inStream >> m_aspectRatio;
	inStream >> texU;
	inStream >> texV;
	inStream >> m_texAlpha;
	inStream >> m_image;
	inStream >> m_file_name; //formerly: 'complete filename'

	return true;
}

void ccImage::removeFromDisplay(const ccGenericGLDisplay * win)
{
	if (win == m_currentDisplay)
	{
		releaseVBO();
	}

	//call parent's method
	ccHObject::removeFromDisplay(win);
}

void ccImage::setDisplay(ccGenericGLDisplay * win)
{
	if (m_currentDisplay && win != m_currentDisplay) {
		releaseVBO();
	}
	ccHObject::setDisplay(win);
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
