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

#include "ccIncludeGL.h"

//Local
#include "cc2DLabel.h"
#include "ccBasicTypes.h"
#include "ccGenericPointCloud.h"
#include "ccPointCloud.h"
#include "ccSphere.h"
#include "ccGenericGLDisplay.h"
#include "ccScalarField.h"

//Qt
#include <QSharedPointer>

//System
#include <string.h>
#include <assert.h>

cc2DLabel::cc2DLabel(QString name/*=QString()*/)
	: ccHObject(name.isEmpty() ? "label" : name)
	, m_showFullBody(true)
	, m_dispIn3D(true)
	, m_dispIn2D(true)
{
	m_screenPos[0] = m_screenPos[1] = 0.05f;

	clear(false);

	lockVisibility(false);
	setEnabled(true);
}

QString cc2DLabel::getName() const
{
	QString processedName = m_name;

	size_t count = m_points.size();
	if (count > 0)
	{
		processedName.replace(QString("pt_0_idx"),QString::number(m_points[0].index));
		if (count > 1)
		{
			processedName.replace(QString("pt_1_idx"),QString::number(m_points[1].index));
			if (m_points[0].cloud)
				processedName.replace(QString("pt_0_cloud_id"),QString::number(m_points[0].cloud->getUniqueID()));
			if (m_points[1].cloud)
				processedName.replace(QString("pt_1_cloud_id"),QString::number(m_points[1].cloud->getUniqueID()));
			if (count > 2)
			{
				processedName.replace(QString("pt_2_idx"),QString::number(m_points[2].index));
				if (m_points[2].cloud)
					processedName.replace(QString("pt_2_cloud_id"),QString::number(m_points[2].cloud->getUniqueID()));
			}
		}
	}

	return processedName;
}

void cc2DLabel::setPosition(float x, float y)
{
	m_screenPos[0] = x;
	m_screenPos[1] = y;
}

bool cc2DLabel::move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight)
{
	assert(screenHeight > 0 && screenWidth > 0);
	
	m_screenPos[0] += static_cast<float>(dx)/static_cast<float>(screenWidth);
	m_screenPos[1] += static_cast<float>(dy)/static_cast<float>(screenHeight);

	return true;
}

void cc2DLabel::clear(bool ignoreDependencies)
{
	if (ignoreDependencies)
	{
		m_points.clear();
	}
	else
	{
		//remove all dependencies first!
		while (!m_points.empty())
		{
			m_points.back().cloud->removeDependencyWith(this);
			m_points.pop_back();
		}
	}

	m_lastScreenPos[0] = m_lastScreenPos[1] = -1;
	memset(m_labelROI,0,sizeof(int)*4);
	setVisible(false);
	setName("Label");
}

void cc2DLabel::onDeletionOf(const ccHObject* obj)
{
	ccHObject::onDeletionOf(obj); //remove dependencies, etc.

	//check that associated clouds are not about to be deleted!
	size_t pointsToRemove = 0;
	{
		for (size_t i=0; i<m_points.size(); ++i)
			if (m_points[i].cloud == obj)
				++pointsToRemove;
	}

	if (pointsToRemove == 0)
		return;

	if (pointsToRemove == m_points.size())
	{
		clear(true); //don't call clear as we don't want/need to update input object's dependencies!
	}
	else
	{
		//remove only the necessary points
		size_t j=0;
		for (size_t i=0; i<m_points.size(); ++i)
		{
			if (m_points[i].cloud != obj)
			{
				if (i != j)
					std::swap(m_points[i],m_points[j]);
				j++;
			}
		}
		assert(j != 0);
		m_points.resize(j);
	}

	updateName();
}

void cc2DLabel::updateName()
{
	switch(m_points.size())
	{
	case 0:
		setName("Label");
		break;
	case 1:
		setName("Point #pt_0_idx");
		break;
	case 2:
		if (m_points[0].cloud == m_points[1].cloud)
			setName("Vector #pt_0_idx - #pt_1_idx");
		else
			setName("Vector #pt_0_idx(@pt_0_cloud_id) - #pt_1_idx@(@pt_1_cloud_id)");
		break;
	case 3:
		if (m_points[0].cloud == m_points[2].cloud && m_points[1].cloud == m_points[2].cloud)
			setName("Triplet #pt_0_idx - #pt_1_idx - #pt_2_idx");
		else
			setName("Triplet #pt_0_idx(@pt_0_cloud_id) - #pt_1_idx(@pt_1_cloud_id) - #pt_2_idx(@pt_2_cloud_id)");
		break;
	}
}

bool cc2DLabel::addPoint(ccGenericPointCloud* cloud, unsigned pointIndex)
{
	assert(cloud && cloud->size()>pointIndex);

	if (m_points.size() == 3)
		return false;

	try
	{
		m_points.resize(m_points.size()+1);
	}
	catch(std::bad_alloc)
	{
		//not enough memory
		return false;
	}

	m_points.back().cloud = cloud;
	m_points.back().index = pointIndex;

	updateName();

	//we want to be notified whenever an associated cloud is deleted (in which case
	//we'll automatically clear the label)
	cloud->addDependency(this,DP_NOTIFY_OTHER_ON_DELETE);
	//we must also warn the cloud whenever we delete this label
	//addDependency(cloud,DP_NOTIFY_OTHER_ON_DELETE); //DGM: automatically done by the previous call to addDependency!

	return true;
}

bool cc2DLabel::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//points count (dataVersion >= 20)
	uint32_t count = (uint32_t)m_points.size();
	if (out.write((const char*)&count,4) < 0)
		return WriteError();

	//points & associated cloud ID (dataVersion >= 20)
	for (std::vector<PickedPoint>::const_iterator it=m_points.begin(); it!=m_points.end(); ++it)
	{
		//point index
		uint32_t index = (uint32_t)it->index;
		if (out.write((const char*)&index,4) < 0)
			return WriteError();
		//cloud ID (will be retrieved later --> make sure that the cloud is saved alongside!)
		uint32_t cloudID = (uint32_t)it->cloud->getUniqueID();
		if (out.write((const char*)&cloudID,4) < 0)
			return WriteError();
	}

	//Relative screen position (dataVersion >= 20)
	if (out.write((const char*)m_screenPos,sizeof(float)*2) < 0)
		return WriteError();

	//Collapsed state (dataVersion >= 20)
	if (out.write((const char*)&m_showFullBody,sizeof(bool)) < 0)
		return WriteError();

	//Show in 2D boolean (dataVersion >= 21)
	if (out.write((const char*)&m_dispIn2D,sizeof(bool)) < 0)
		return WriteError();

	//Show in 3D boolean (dataVersion >= 21)
	if (out.write((const char*)&m_dispIn3D,sizeof(bool)) < 0)
		return WriteError();

	return true;
}

bool cc2DLabel::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//points count (dataVersion >= 20)
	uint32_t count = 0;
	if (in.read((char*)&count,4) < 0)
		return ReadError();

	//points & associated cloud ID (dataVersion >= 20)
	assert(m_points.empty());
	for (uint32_t i=0; i<count; ++i)
	{
		//point index
		uint32_t index = 0;
		if (in.read((char*)&index,4) < 0)
			return ReadError();
		//cloud ID (will be retrieved later --> make sure that the cloud is saved alongside!)
		uint32_t cloudID = 0;
		if (in.read((char*)&cloudID,4) < 0)
			return ReadError();

		//[DIRTY] WARNING: temporarily, we set the cloud unique ID in the 'PickedPoint::cloud' pointer!!!
		PickedPoint pp;
		pp.index = (unsigned)index;
		*(uint32_t*)(&pp.cloud) = cloudID;
		m_points.push_back(pp);
		if (m_points.size() != i+1)
			return MemoryError();
	}

	//Relative screen position (dataVersion >= 20)
	if (in.read((char*)m_screenPos,sizeof(float)*2) < 0)
		return ReadError();

	//Collapsed state (dataVersion >= 20)
	if (in.read((char*)&m_showFullBody,sizeof(bool)) < 0)
		return ReadError();

	if (dataVersion > 20)
	{
		//Show in 2D boolean (dataVersion >= 21)
		if (in.read((char*)&m_dispIn2D,sizeof(bool)) < 0)
			return ReadError();

		//Show in 3D boolean (dataVersion >= 21)
		if (in.read((char*)&m_dispIn3D,sizeof(bool)) < 0)
			return ReadError();
	}

	return true;
}

//return angle between two vectors (in degrees)
//warning: vectors will be normalized by default
double GetAngle_deg(CCVector3& AB, CCVector3& AC)
{
	AB.normalize();
	AC.normalize();
	double dotprod = AB.dot(AC);
	if (dotprod <= -1.0)
		return 180.0;
	else if (dotprod > 1.0)
		return 0.0;
	return 180.0*acos(dotprod)/M_PI;
}

void AddPointCoordinates(QStringList& body, unsigned pointIndex, ccGenericPointCloud* cloud, int precision)
{
	assert(cloud);
	const CCVector3* P = cloud->getPointPersistentPtr(pointIndex);
	bool isShifted = cloud->isShifted();

	QString coordStr = QString("P#%0:").arg(pointIndex);
	if (isShifted)
	{
		body << coordStr;
		coordStr = QString("  [shifted]");
	}
	
	coordStr += QString(" (%1;%2;%3)").arg(P->x,0,'f',precision).arg(P->y,0,'f',precision).arg(P->z,0,'f',precision);
	body << coordStr;
	
	if (isShifted)
	{
		CCVector3d Pg = cloud->toGlobal3d(*P);
		QString globCoordStr = QString("  [original] (%1;%2;%3)").arg(Pg.x,0,'f',precision).arg(Pg.y,0,'f',precision).arg(Pg.z,0,'f',precision);
		body << globCoordStr;
	}
}

QStringList cc2DLabel::getLabelContent(int precision)
{
	QStringList body;

	switch(m_points.size())
	{
	case 0:
		//can happen if the associated cloud(s) has(ve) been deleted!
		body << "Deprecated";
		break;

	case 1: //point
		{
			//init title
			/*title = m_title;
			//automatically elide the title
			title = titleFontMetrics.elidedText(title,Qt::ElideRight,dx);
			//*/

			//coordinates
			ccGenericPointCloud* cloud = m_points[0].cloud;
			const unsigned& pointIndex = m_points[0].index;
			AddPointCoordinates(body,pointIndex,cloud,precision);

			//normal
			if (cloud->hasNormals())
			{
				const CCVector3& N = cloud->getPointNormal(pointIndex);
				QString normStr = QString("Normal: (%1;%2;%3)").arg(N.x,0,'f',precision).arg(N.y,0,'f',precision).arg(N.z,0,'f',precision);
				body << normStr;
			}
			//color
			if (cloud->hasColors())
			{
				const colorType* C = cloud->getPointColor(pointIndex);
				assert(C);
				QString colorStr = QString("Color: (%1;%2;%3)").arg(C[0]).arg(C[1]).arg(C[2]);
				body << colorStr;
			}
			//scalar field
			if (cloud->hasDisplayedScalarField())
			{
				ScalarType D = cloud->getPointScalarValue(pointIndex);
				QString source("Scalar");
				//fetch the real scalar field name if possible
				if (cloud->isA(CC_TYPES::POINT_CLOUD))
				{
					ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
					if (pc->getCurrentDisplayedScalarField())
						source = QString(pc->getCurrentDisplayedScalarField()->getName());
				}
				QString sfStr = QString("%1 = %2").arg(source).arg(D,0,'f',precision);
				body << sfStr;
			}
		}
		break;

	case 2: //vector
		{
			//1st point
			ccGenericPointCloud* cloud1 = m_points[0].cloud;
			const unsigned& pointIndex1 = m_points[0].index;
			const CCVector3* P1 = cloud1->getPointPersistentPtr(pointIndex1);
			//2nd point
			ccGenericPointCloud* cloud2 = m_points[1].cloud;
			const unsigned& pointIndex2 = m_points[1].index;
			const CCVector3* P2 = cloud2->getPointPersistentPtr(pointIndex2);

			PointCoordinateType d = (*P1-*P2).norm();
			QString distStr = QString("Distance = %1").arg(d,0,'f',precision);
			body << distStr;

			CCVector3 V = *P2-*P1;
			QString vecStr = QString("Vec: (%1;%2;%3)").arg(V.x,0,'f',precision).arg(V.y,0,'f',precision).arg(V.z,0,'f',precision);
			body << vecStr;

			AddPointCoordinates(body,pointIndex1,cloud1,precision);
			AddPointCoordinates(body,pointIndex2,cloud2,precision);
		}
		break;

	case 3: //triangle/plane
		{
			//1st point
			ccGenericPointCloud* cloud1 = m_points[0].cloud;
			const unsigned& pointIndex1 = m_points[0].index;
			const CCVector3* P1 = cloud1->getPointPersistentPtr(pointIndex1);
			//2nd point
			ccGenericPointCloud* cloud2 = m_points[1].cloud;
			const unsigned& pointIndex2 = m_points[1].index;
			const CCVector3* P2 = cloud2->getPointPersistentPtr(pointIndex2);
			//3rd point
			ccGenericPointCloud* cloud3 = m_points[2].cloud;
			const unsigned& pointIndex3 = m_points[2].index;
			const CCVector3* P3 = cloud3->getPointPersistentPtr(pointIndex3);

			//area
			CCVector3 P1P2 = *P2-*P1;
			CCVector3 P1P3 = *P3-*P1;
			CCVector3 N = P1P2.cross(P1P3); //N=ABxAC
			PointCoordinateType area = N.norm()*(PointCoordinateType)0.5;
			QString areaStr = QString("Area = %1").arg(area,0,'f',precision);
			body << areaStr;

			//coordinates
			AddPointCoordinates(body,pointIndex1,cloud1,precision);
			AddPointCoordinates(body,pointIndex2,cloud2,precision);
			AddPointCoordinates(body,pointIndex3,cloud3,precision);

			//normal
			N.normalize();
			QString normStr = QString("Normal: (%1;%2;%3)").arg(N.x,0,'f',precision).arg(N.y,0,'f',precision).arg(N.z,0,'f',precision);
			body << normStr;

			//angle
			CCVector3 P2P3 = *P3-*P2;

			//negatives
			CCVector3 _P1P2 = -P1P2;
			CCVector3 _P1P3 = -P1P3;
			CCVector3 _P2P3 = -P2P3;

			double angleAtP1 = GetAngle_deg(P1P2,P1P3);
			double angleAtP2 = GetAngle_deg(P2P3,_P1P2);
			double angleAtP3 = GetAngle_deg(_P1P3,_P2P3); //should be equal to 180-a1-a2!
			QString angleStr = QString("Angles: A=%1 - B=%3 - C=%5 deg.").arg(angleAtP1,0,'f',precision).arg(angleAtP2,0,'f',precision).arg(angleAtP3,0,'f',precision);
			body << angleStr;
		}
		break;

	default:
		assert(false);
		break;
	}

	return body;
}

bool cc2DLabel::acceptClick(int x, int y, Qt::MouseButton button)
{
	if (button == Qt::RightButton)
	{
		if (	x >= m_lastScreenPos[0]+m_labelROI[0] && x <= m_lastScreenPos[0]+m_labelROI[2]
			&&	y >= m_lastScreenPos[1]-m_labelROI[3] && y <= m_lastScreenPos[1]-m_labelROI[1])
			{
				//toggle collapse state
				m_showFullBody = !m_showFullBody;
				return true;
			}
	}

	return false;
}

void cc2DLabel::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (m_points.empty())
		return;

	//2D foreground only
	if (!MACRO_Foreground(context))
		return;

	//Not compatible with virtual transformation (see ccDrawableObject::enableGLTransformation)
	if (MACRO_VirtualTransEnabled(context))
		return;

	if (MACRO_Draw3D(context))
		drawMeOnly3D(context);
	else if (MACRO_Draw2D(context))
		drawMeOnly2D(context);
}

//unit point marker
static QSharedPointer<ccSphere> c_unitPointMarker(0);

void cc2DLabel::drawMeOnly3D(CC_DRAW_CONTEXT& context)
{
	assert(!m_points.empty());

	//standard case: list names pushing
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
	{
		//not particularily fast
		if (MACRO_DrawFastNamesOnly(context))
			return;
		glPushName(getUniqueIDForDisplay());
	}

	const float c_sizeFactor = 4.0f;
	bool loop = false;

	size_t count = m_points.size();
	switch (count)
	{
	case 3:
		{
			glPushAttrib(GL_COLOR_BUFFER_BIT);
			glEnable(GL_BLEND);

			//we draw the triangle
			glColor4ub(255,255,0,128);
			glBegin(GL_TRIANGLES);
			for (unsigned i=0; i<count; ++i)
				ccGL::Vertex3v(m_points[i].cloud->getPoint(m_points[i].index)->u);
			glEnd();

			glPopAttrib();
			loop=true;
		}
	case 2:
		{
			//segment width
			glPushAttrib(GL_LINE_BIT);
			glLineWidth(c_sizeFactor);

			//we draw the segments
			if (isSelected())
				glColor3ubv(ccColor::red);
			else
				glColor3ubv(ccColor::green);
			glBegin(GL_LINES);
			for (unsigned i=0; i<count; i++)
			{
				if (i+1<count || loop)
				{
					ccGL::Vertex3v(m_points[i].cloud->getPoint(m_points[i].index)->u);
					ccGL::Vertex3v(m_points[(i+1)%count].cloud->getPoint(m_points[(i+1)%count].index)->u);
				}
			}
			glEnd();
			glPopAttrib();
		}

	case 1:
		{
			//display point marker as spheres
			{
				if (!c_unitPointMarker)
				{
					c_unitPointMarker = QSharedPointer<ccSphere>(new ccSphere(1.0f,0,"PointMarker",12));
					c_unitPointMarker->showColors(true);
					c_unitPointMarker->setVisible(true);
					c_unitPointMarker->setEnabled(true);
				}
			
				//build-up point maker own 'context'
				CC_DRAW_CONTEXT markerContext = context;
				markerContext.flags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the sphere doesn't push its own!
				markerContext._win = 0;

				if (isSelected() && !pushName)
					c_unitPointMarker->setTempColor(ccColor::red);
				else
					c_unitPointMarker->setTempColor(ccColor::magenta);

				for (unsigned i=0; i<count; i++)
				{
					glMatrixMode(GL_MODELVIEW);
					glPushMatrix();
					const CCVector3* P = m_points[i].cloud->getPoint(m_points[i].index);
					ccGL::Translate(P->x,P->y,P->z);
					glScalef(context.pickedPointsRadius,context.pickedPointsRadius,context.pickedPointsRadius);
					c_unitPointMarker->draw(markerContext);
					glPopMatrix();
				}
			}

			if (m_dispIn3D && !pushName) //no need to display label in point picking mode
			{
				QFont font(context._win->getTextDisplayFont()); //takes rendering zoom into account!
				//font.setPointSize(font.pointSize()+2);
				font.setBold(true);

				//draw their name
				glPushAttrib(GL_DEPTH_BUFFER_BIT);
				glDisable(GL_DEPTH_TEST);
				for (unsigned j=0; j<count; j++)
				{
					const CCVector3* P = m_points[j].cloud->getPoint(m_points[j].index);
					QString title = (count == 1 ? getName() : QString("P#%0").arg(m_points[j].index));
					context._win->display3DLabel(	title,
													*P + CCVector3(	context.pickedPointsTextShift,
																	context.pickedPointsTextShift,
																	context.pickedPointsTextShift),
													ccColor::magenta,
													font );
				}
				glPopAttrib();
			}
		}
	}

	if (pushName)
		glPopName();
}

//display parameters
static const int c_margin = 5;
static const int c_arrowBaseSize = 3;
//static const int c_buttonSize = 10;

void cc2DLabel::drawMeOnly2D(CC_DRAW_CONTEXT& context)
{
	if (!m_dispIn2D)
		return;

	assert(!m_points.empty());

	//standard case: list names pushing
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
		glPushName(getUniqueID());

	//we should already be in orthoprojective & centered omde
	//glOrtho(-halfW,halfW,-halfH,halfH,-maxS,maxS);

	int strHeight = 0;
	int titleHeight = 0;
	QString title(getName());
	QStringList body;
	GLdouble arrowDestX=-1.0,arrowDestY=-1.0;
	QFont bodyFont,titleFont;
	if (!pushName)
	{
		/*** line from 2D point to label ***/

		//compute arrow head position
		CCVector3 arrowDest;
		m_points[0].cloud->getPoint(m_points[0].index,arrowDest);
		for (unsigned i=1; i<m_points.size(); ++i)
			arrowDest += *m_points[i].cloud->getPointPersistentPtr(m_points[i].index);
		arrowDest /= (PointCoordinateType)m_points.size();

		//project it in 2D screen coordinates
		int VP[4];
		context._win->getViewportArray(VP);
		const double* MM = context._win->getModelViewMatd(); //viewMat
		const double* MP = context._win->getProjectionMatd(); //projMat
		GLdouble zp;
		gluProject(arrowDest.x,arrowDest.y,arrowDest.z,MM,MP,VP,&arrowDestX,&arrowDestY,&zp);

		/*** label border ***/
		bodyFont = context._win->getTextDisplayFont(); //takes rendering zoom into account!
		titleFont = QFont(context._win->getTextDisplayFont()); //takes rendering zoom into account!
		titleFont.setBold(true);
		QFontMetrics titleFontMetrics(titleFont);
		QFontMetrics bodyFontMetrics(bodyFont);

		strHeight = bodyFontMetrics.height();
		titleHeight = titleFontMetrics.height();

		if (m_showFullBody)
			body = getLabelContent(context.dispNumberPrecision);

		//base box dimension
		int dx = 150;
		dx = std::max(dx,titleFontMetrics.width(title));

		int dy = c_margin;	//top vertical margin
		dy += titleHeight;	//title
		if (!body.empty())
		{
			dy += c_margin;	//vertical margin above separator
			for (int j=0; j<body.size(); ++j)
			{
				dx = std::max(dx,bodyFontMetrics.width(body[j]));
				dy += strHeight; //body line height
			}
			dy += c_margin;	//vertical margin below text
		}
		else
		{
			dy += c_margin;	// vertical margin (purely for aesthetics)
		}
		dy += c_margin;		// bottom vertical margin
		dx += c_margin*2;	// horizontal margins

		//main rectangle
		m_labelROI[0]=0;
		m_labelROI[1]=0;
		m_labelROI[2]=dx;
		m_labelROI[3]=dy;

		//close button
		/*m_closeButtonROI[2]=dx-c_margin;
		m_closeButtonROI[0]=m_closeButtonROI[2]-c_buttonSize;
		m_closeButtonROI[3]=c_margin;
		m_closeButtonROI[1]=m_closeButtonROI[3]+c_buttonSize;
		//*/

		//automatically elide the title
		//title = titleFontMetrics.elidedText(title,Qt::ElideRight,m_closeButtonROI[0]-2*c_margin);
	}

	int halfW = (context.glW >> 1);
	int halfH = (context.glH >> 1);

	//draw label rectangle
	int xStart = m_lastScreenPos[0] = (int)((float)context.glW * m_screenPos[0]);
	int yStart = m_lastScreenPos[1] = (int)((float)context.glH * (1.0f-m_screenPos[1]));

	//colors
	bool highlighted = (!pushName && isSelected());
	//default background color
	colorType defaultBkgColor[4];
	memcpy(defaultBkgColor,context.labelDefaultCol,sizeof(colorType)*3);
	defaultBkgColor[3] = (colorType)((float)context.labelsTransparency*(float)MAX_COLOR_COMP/100.0f);
	//default border color (mustn't be totally transparent!)
	colorType defaultBorderColor[4];
	if (highlighted)
		memcpy(defaultBorderColor,ccColor::red,sizeof(colorType)*3);
	else
		memcpy(defaultBorderColor,context.labelDefaultCol,sizeof(colorType)*3);
	defaultBorderColor[3] = (colorType)((float)(50+context.labelsTransparency/2)*(float)MAX_COLOR_COMP/100.0f);

	glPushAttrib(GL_COLOR_BUFFER_BIT);
	glEnable(GL_BLEND);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glTranslatef(static_cast<GLfloat>(-halfW+xStart),static_cast<GLfloat>(-halfH+yStart),0);

	if (!pushName)
	{
		//compute arrow base position relatively to the label rectangle (for 0 to 8)
		int arrowBaseConfig = 0;
		int iArrowDestX = (int)arrowDestX-xStart;
		int iArrowDestY = (int)arrowDestY-yStart;
		{
			if (iArrowDestX < m_labelROI[0]) //left
				arrowBaseConfig += 0;
			else if (iArrowDestX > m_labelROI[2]) //Right
				arrowBaseConfig += 2;
			else  //Middle
				arrowBaseConfig += 1;

			if (iArrowDestY > -m_labelROI[1]) //Top
				arrowBaseConfig += 0;
			else if (iArrowDestY < -m_labelROI[3]) //Bottom
				arrowBaseConfig += 6;
			else  //Middle
				arrowBaseConfig += 3;
		}

		//we make the arrow base start from the nearest corner
		if (arrowBaseConfig != 4) //4 = label above point!
		{
			glColor4ubv(defaultBorderColor);
			glBegin(GL_TRIANGLE_FAN);
			glVertex2d(arrowDestX-xStart,arrowDestY-yStart);
			switch(arrowBaseConfig)
			{
			case 0: //top-left corner
				glVertex2i(m_labelROI[0], -m_labelROI[1]-2*c_arrowBaseSize);
				glVertex2i(m_labelROI[0], -m_labelROI[1]);
				glVertex2i(m_labelROI[0]+2*c_arrowBaseSize, -m_labelROI[1]);
				break;
			case 1: //top-middle edge
				glVertex2i(std::max(m_labelROI[0],iArrowDestX-c_arrowBaseSize), -m_labelROI[1]);
				glVertex2i(std::min(m_labelROI[2],iArrowDestX+c_arrowBaseSize), -m_labelROI[1]);
				break;
			case 2: //top-right corner
				glVertex2i(m_labelROI[2], -m_labelROI[1]-2*c_arrowBaseSize);
				glVertex2i(m_labelROI[2], -m_labelROI[1]);
				glVertex2i(m_labelROI[2]-2*c_arrowBaseSize, -m_labelROI[1]);
				break;
			case 3: //middle-left edge
				glVertex2i(m_labelROI[0], std::min(-m_labelROI[1],iArrowDestY+c_arrowBaseSize));
				glVertex2i(m_labelROI[0], std::max(-m_labelROI[3],iArrowDestY-c_arrowBaseSize));
				break;
			case 4: //middle of rectangle!
				break;
			case 5: //middle-right edge
				glVertex2i(m_labelROI[2], std::min(-m_labelROI[1],iArrowDestY+c_arrowBaseSize));
				glVertex2i(m_labelROI[2], std::max(-m_labelROI[3],iArrowDestY-c_arrowBaseSize));
				break;
			case 6: //bottom-left corner
				glVertex2i(m_labelROI[0], -m_labelROI[3]+2*c_arrowBaseSize);
				glVertex2i(m_labelROI[0], -m_labelROI[3]);
				glVertex2i(m_labelROI[0]+2*c_arrowBaseSize, -m_labelROI[3]);
				break;
			case 7: //bottom-middle edge
				glVertex2i(std::max(m_labelROI[0],iArrowDestX-c_arrowBaseSize), -m_labelROI[3]);
				glVertex2i(std::min(m_labelROI[2],iArrowDestX+c_arrowBaseSize), -m_labelROI[3]);
				break;
			case 8: //bottom-right corner
				glVertex2i(m_labelROI[2], -m_labelROI[3]+2*c_arrowBaseSize);
				glVertex2i(m_labelROI[2], -m_labelROI[3]);
				glVertex2i(m_labelROI[2]-2*c_arrowBaseSize, -m_labelROI[3]);
				break;
			}
			glEnd();
		}
	}

	//main rectangle
	glColor4ubv(defaultBkgColor);
	glBegin(GL_QUADS);
	glVertex2i(m_labelROI[0], -m_labelROI[1]);
	glVertex2i(m_labelROI[0], -m_labelROI[3]);
	glVertex2i(m_labelROI[2], -m_labelROI[3]);
	glVertex2i(m_labelROI[2], -m_labelROI[1]);
	glEnd();

	//if (highlighted)
	{
		glPushAttrib(GL_LINE_BIT);
		glLineWidth(3.0f);
		glColor4ubv(defaultBorderColor);
		glBegin(GL_LINE_LOOP);
		glVertex2i(m_labelROI[0], -m_labelROI[1]);
		glVertex2i(m_labelROI[0], -m_labelROI[3]);
		glVertex2i(m_labelROI[2], -m_labelROI[3]);
		glVertex2i(m_labelROI[2], -m_labelROI[1]);
		glEnd();
		glPopAttrib();
	}

	//draw close button
	/*glColor3ubv(ccColor::black);
	glBegin(GL_LINE_LOOP);
	glVertex2i(m_closeButtonROI[0],-m_closeButtonROI[1]);
	glVertex2i(m_closeButtonROI[0],-m_closeButtonROI[3]);
	glVertex2i(m_closeButtonROI[2],-m_closeButtonROI[3]);
	glVertex2i(m_closeButtonROI[2],-m_closeButtonROI[1]);
	glEnd();
	glBegin(GL_LINES);
	glVertex2i(m_closeButtonROI[0]+2,-m_closeButtonROI[1]+2);
	glVertex2i(m_closeButtonROI[2]-2,-m_closeButtonROI[3]-2);
	glVertex2i(m_closeButtonROI[2]-2,-m_closeButtonROI[1]+2);
	glVertex2i(m_closeButtonROI[0]+2,-m_closeButtonROI[3]-2);
	glEnd();
	//*/

	//display text
	if (!pushName)
	{
		int xStartRel = c_margin;
		int yStartRel = -c_margin;
		yStartRel -= titleHeight;

		const colorType* defaultTextColor = (context.labelsTransparency<40 ? context.textDefaultCol : ccColor::darkBlue);

		context._win->displayText(title,xStart+xStartRel,yStart+yStartRel,ccGenericGLDisplay::ALIGN_DEFAULT,0,defaultTextColor,&titleFont);
		yStartRel -= c_margin;

		if (!body.empty())
		{
			//line separation
			glColor4ubv(defaultBorderColor);
			glBegin(GL_LINES);
			glVertex2i(xStartRel,yStartRel);
			glVertex2i(xStartRel+m_labelROI[2]-m_labelROI[0]-2*c_margin,yStartRel);
			glEnd();

			//display body
			yStartRel -= c_margin;
			for (int i=0; i<body.size(); ++i)
			{
				yStartRel -= strHeight;
				context._win->displayText(body[i],xStart+xStartRel,yStart+yStartRel,ccGenericGLDisplay::ALIGN_DEFAULT,0,defaultTextColor,&bodyFont);
			}
		}
	}

	glPopAttrib();

	glPopMatrix();

	if (pushName)
		glPopName();
}
