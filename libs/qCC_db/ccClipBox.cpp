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

#include "ccClipBox.h"

//Local
#include "ccGenericPointCloud.h"
#include "ccCylinder.h"
#include "ccCone.h"
#include "ccSphere.h"
#include "ccTorus.h"
#include "ccHObjectCaster.h"

//system
#include <assert.h>

//Components geometry
static QSharedPointer<ccCylinder> c_arrowShaft(0);
static QSharedPointer<ccCone> c_arrowHead(0);
static QSharedPointer<ccSphere> c_centralSphere(0);
static QSharedPointer<ccTorus> c_torus(0);

void DrawUnitArrow(int ID, const CCVector3& start, const CCVector3& direction, PointCoordinateType scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	if (ID > 0)
		glLoadName(ID);
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	ccGL::Translate(start.x,start.y,start.z);
	ccGL::Scale(scale,scale,scale);

	//we compute scalar prod between the two vectors
	CCVector3 Z(0.0,0.0,1.0);
	PointCoordinateType ps = Z.dot(direction);
	
	if (ps < 1)
	{
		CCVector3 axis(1,0,0);
		PointCoordinateType angle_deg = static_cast<PointCoordinateType>(180.0);
		
		if (ps > -1)
		{
			//we deduce angle from scalar prod
			angle_deg = acos(ps) * static_cast<PointCoordinateType>(CC_RAD_TO_DEG);

			//we compute rotation axis with scalar prod
			axis = Z.cross(direction);
		}
		
		ccGL::Rotate(angle_deg, axis.x, axis.y, axis.z);
	}

	if (!c_arrowShaft)
		c_arrowShaft = QSharedPointer<ccCylinder>(new ccCylinder(0.15f,0.6f,0,"ArrowShaft",12));
	if (!c_arrowHead)
		c_arrowHead = QSharedPointer<ccCone>(new ccCone(0.3f,0,0.4f,0,0,0,"ArrowHead",24));

	glTranslatef(0,0,0.3f);
	c_arrowShaft->setTempColor(col);
	c_arrowShaft->draw(context);
	glTranslatef(0,0,0.3f+0.2f);
	c_arrowHead->setTempColor(col);
	c_arrowHead->draw(context);

	glPopMatrix();
}

static void DrawUnitTorus(int ID, const CCVector3& center, const CCVector3& direction, PointCoordinateType scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	if (ID > 0)
		glLoadName(ID);
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	ccGL::Translate(center.x,center.y,center.z);
	ccGL::Scale(scale,scale,scale);

	//we compute scalar prod between the two vectors
	CCVector3 Z(0,0,1);
	PointCoordinateType ps = Z.dot(direction);
	
	if (ps < 1)
	{
		CCVector3 axis(1,0,0);
		PointCoordinateType angle_deg = 180;
		
		if (ps > -1)
		{
			//we deduce angle from scalar prod
			angle_deg = acos(ps) * static_cast<PointCoordinateType>(CC_RAD_TO_DEG);

			//we compute rotation axis with scalar prod
			axis = Z.cross(direction);
		}
		
		ccGL::Rotate(angle_deg, axis.x, axis.y, axis.z);
	}

	if (!c_torus)
		c_torus = QSharedPointer<ccTorus>(new ccTorus(0.2f,0.4f,2.0*M_PI,false,0,0,"Torus",12));

	glTranslatef(0,0,0.3f);
	c_torus->setTempColor(col);
	c_torus->draw(context);

	glPopMatrix();
}

// Unused function
/*static void DrawUnitSphere(int ID, const CCVector3& center, PointCoordinateType radius, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	if (ID > 0)
		glLoadName(ID);
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	ccGL::Translate(center.x,center.y,center.z);
	ccGL::Scale(radius,radius,radius);

	if (!c_centralSphere)
		c_centralSphere = QSharedPointer<ccSphere>(new ccSphere(1,0,"CentralSphere",24));
	
	c_centralSphere->setTempColor(col);
	c_centralSphere->draw(context);

	glPopMatrix();
}
//*/

static void DrawUnitCross(int ID, const CCVector3& center, PointCoordinateType scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	if (ID > 0)
		glLoadName(ID);
	
	scale /= 2;
	DrawUnitArrow(0, center, CCVector3(-1, 0, 0), scale, col, context);
	DrawUnitArrow(0, center, CCVector3( 1, 0, 0), scale, col, context);
	DrawUnitArrow(0, center, CCVector3( 0,-1, 0), scale, col, context);
	DrawUnitArrow(0, center, CCVector3( 0, 1, 0), scale, col, context);
	DrawUnitArrow(0, center, CCVector3( 0, 0,-1), scale, col, context);
	DrawUnitArrow(0, center, CCVector3( 0, 0, 1), scale, col, context);
}

ccClipBox::ccClipBox(ccHObject* associatedEntity, QString name/*= QString("clipping box")*/)
	: ccHObject(name)
	, m_associatedEntity(0)
	, m_activeComponent(NONE)
{
	setSelectionBehavior(SELECTION_IGNORED);

	setAssociatedEntity(associatedEntity);
}

ccClipBox::~ccClipBox()
{
	setAssociatedEntity(0);
}

void ccClipBox::reset()
{
	m_box.clear();
	resetGLTransformation();

	if (m_associatedEntity)
	{
		m_box = m_associatedEntity->getOwnBB();
	}

	update();

	//send 'modified' signal
	emit boxModified(&m_box);
}

bool ccClipBox::setAssociatedEntity(ccHObject* entity)
{
	//release previous one
	if (m_associatedEntity)
	{
		ccGenericPointCloud* points = ccHObjectCaster::ToGenericPointCloud(m_associatedEntity);
		if (points)
			points->unallocateVisibilityArray();
	}
	m_associatedEntity = 0;

	//try to initialize new one
	if (entity)
	{
		if (!entity->isKindOf(CC_TYPES::POINT_CLOUD) && !entity->isKindOf(CC_TYPES::MESH))
		{
			ccLog::Warning("[Clipping box] Unhandled type of entity");
			return false;
		}
		else
		{
			ccGenericPointCloud* points = ccHObjectCaster::ToGenericPointCloud(entity);
			if (points)
			{
				if (points->resetVisibilityArray())
				{
					m_associatedEntity = entity;
				}
				else
				{
					ccLog::Warning("[Clipping box] Not enough memory");
					return false;
				}
			}
		}
	}

	reset();

	return true;
}

void ccClipBox::setActiveComponent(int id)
{
	switch(id)
	{
	case 1:
		m_activeComponent = X_MINUS_ARROW;
		break;
	case 2:
		m_activeComponent = X_PLUS_ARROW;
		break;
	case 3:
		m_activeComponent = Y_MINUS_ARROW;
		break;
	case 4:
		m_activeComponent = Y_PLUS_ARROW;
		break;
	case 5:
		m_activeComponent = Z_MINUS_ARROW;
		break;
	case 6:
		m_activeComponent = Z_PLUS_ARROW;
		break;
	case 7:
		m_activeComponent = CROSS;
		break;
	case 8:
		m_activeComponent = SPHERE;
		break;
	case 9:
		m_activeComponent = X_MINUS_TORUS;
		break;
	case 10:
		m_activeComponent = Y_MINUS_TORUS;
		break;
	case 11:
		m_activeComponent = Z_MINUS_TORUS;
		break;
	case 12:
		m_activeComponent = X_PLUS_TORUS;
		break;
	case 13:
		m_activeComponent = Y_PLUS_TORUS;
		break;
	case 14:
		m_activeComponent = Z_PLUS_TORUS;
		break;
	default:
		m_activeComponent = NONE;
	}
}

static CCVector3d PointToVector(int x, int y, int screenWidth, int screenHeight)
{
	//convert mouse position to vector (screen-centered)
	CCVector3d v(	static_cast<double>(2 * std::max(std::min(x,screenWidth-1),-screenWidth+1) - screenWidth) / static_cast<double>(screenWidth),
					static_cast<double>(screenHeight - 2 * std::max(std::min(y,screenHeight-1),-screenHeight+1)) / static_cast<double>(screenHeight),
					0 );

	//square 'radius'
	double d2 = v.x*v.x + v.y*v.y;

	//projection on the unit sphere
	if (d2 > 1)
	{
		double d = sqrt(d2);
		v.x /= d;
		v.y /= d;
	}
	else
	{
		v.z = sqrt(1 - d2);
	}

	return v;
}

bool ccClipBox::move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight)
{
	if (m_activeComponent != SPHERE || !m_box.isValid())
		return false;

	//convert mouse position to vector (screen-centered)
	CCVector3d currentOrientation = PointToVector(x,y,screenWidth,screenHeight);

	ccGLMatrixd rotMat = ccGLMatrixd::FromToRotation(m_lastOrientation,currentOrientation);

	CCVector3 C = m_box.getCenter();

	ccGLMatrixd transMat;
	transMat.setTranslation(-C);
	transMat = rotMat * transMat;
	transMat.setTranslation(transMat.getTranslationAsVec3D() + CCVector3d::fromArray(C.u));

	//rotateGL(transMat);
	m_glTrans = ccGLMatrix(transMat.inverse().data()) * m_glTrans;
	enableGLTransformation(true);

	m_lastOrientation = currentOrientation;

	update();

	return true;
}

void ccClipBox::setClickedPoint(int x, int y, int screenWidth, int screenHeight, const ccGLMatrixd& viewMatrix)
{
	m_lastOrientation = PointToVector(x,y,screenWidth,screenHeight);
	m_viewMatrix = viewMatrix;
}

bool ccClipBox::move3D(const CCVector3d& uInput)
{
	if (m_activeComponent == NONE || !m_box.isValid())
		return false;

	CCVector3d u = uInput;

	//Arrows
	if (m_activeComponent >= X_MINUS_ARROW && m_activeComponent <= CROSS)
	{
		if (m_glTransEnabled)
			m_glTrans.inverse().applyRotation(u);

		switch(m_activeComponent)
		{
		case X_MINUS_ARROW:
			m_box.minCorner().x += static_cast<PointCoordinateType>(u.x);
			if (m_box.minCorner().x > m_box.maxCorner().x)
				m_box.minCorner().x = m_box.maxCorner().x;
			break;
		case X_PLUS_ARROW:
			m_box.maxCorner().x += static_cast<PointCoordinateType>(u.x);
			if (m_box.minCorner().x > m_box.maxCorner().x)
				m_box.maxCorner().x = m_box.minCorner().x;
			break;
		case Y_MINUS_ARROW:
			m_box.minCorner().y += static_cast<PointCoordinateType>(u.y);
			if (m_box.minCorner().y > m_box.maxCorner().y)
				m_box.minCorner().y = m_box.maxCorner().y;
			break;
		case Y_PLUS_ARROW:
			m_box.maxCorner().y += static_cast<PointCoordinateType>(u.y);
			if (m_box.minCorner().y > m_box.maxCorner().y)
				m_box.maxCorner().y = m_box.minCorner().y;
			break;
		case Z_MINUS_ARROW:
			m_box.minCorner().z += static_cast<PointCoordinateType>(u.z);
			if (m_box.minCorner().z > m_box.maxCorner().z)
				m_box.minCorner().z = m_box.maxCorner().z;
			break;
		case Z_PLUS_ARROW:
			m_box.maxCorner().z += static_cast<PointCoordinateType>(u.z);
			if (m_box.minCorner().z > m_box.maxCorner().z)
				m_box.maxCorner().z = m_box.minCorner().z;
			break;
		case CROSS:
			m_box += CCVector3::fromArray(u.u);
			break;
		default:
			assert(false);
			return false;
		}
		
		//send 'modified' signal
		emit boxModified(&m_box);
	}
	else if (m_activeComponent == SPHERE)
	{
		//handled by move2D!
		return false;
	}
	else if (m_activeComponent >= X_MINUS_TORUS && m_activeComponent <= Z_PLUS_TORUS)
	{
		//we guess the rotation order by comparing the current screen 'normal'
		//and the vector prod of u and the current rotation axis
		CCVector3d Rb(0,0,0);
		switch(m_activeComponent)
		{
		case X_MINUS_TORUS:
			Rb.x = -1;
			break;
		case X_PLUS_TORUS:
			Rb.x = 1;
			break;
		case Y_MINUS_TORUS:
			Rb.y = -1;
			break;
		case Y_PLUS_TORUS:
			Rb.y = 1;
			break;
		case Z_MINUS_TORUS:
			Rb.z = -1;
			break;
		case Z_PLUS_TORUS:
			Rb.z = 1;
			break;
		default:
			assert(false);
			return false;
		}
		
		CCVector3d R = Rb;
		if (m_glTransEnabled)
			m_glTrans.applyRotation(R);

		CCVector3d RxU = R.cross(u);

		//look for the most parallel dimension
		double maxDot = m_viewMatrix.getColumnAsVec3D(0).dot(RxU);
		for (int i=1; i<3; ++i)
		{
			double dot = m_viewMatrix.getColumnAsVec3D(i).dot(RxU);
			if (fabs(dot) > fabs(maxDot))
			{
				maxDot = dot;
			}
		}

		//angle is proportional to absolute displacement
		double angle_rad = u.norm()/m_box.getDiagNorm() * M_PI;
		if (maxDot < 0.0)
			angle_rad = -angle_rad;

		ccGLMatrixd rotMat;
		rotMat.initFromParameters(angle_rad,Rb,CCVector3d(0,0,0));

		CCVector3 C = m_box.getCenter();
		ccGLMatrixd transMat;
		transMat.setTranslation(-C);
		transMat = rotMat * transMat;
		transMat.setTranslation(transMat.getTranslationAsVec3D() + CCVector3d::fromArray(C.u));

		m_glTrans = m_glTrans * ccGLMatrix(transMat.inverse().data());
		enableGLTransformation(true);
	}
	else
	{
		assert(false);
		return false;
	}

	update();

	return true;
}

void ccClipBox::setBox(const ccBBox& box)
{
	m_box = box;

	update();

	//send 'modified' signal
	emit boxModified(&m_box);
}

void ccClipBox::shift(const CCVector3& v)
{
	m_box.minCorner() += v;
	m_box.maxCorner() += v;
		
	update();

	//send 'modified' signal
	emit boxModified(&m_box);
}

void ccClipBox::update(bool shrink/*=false*/)
{
	ccGenericPointCloud* cloud = m_associatedEntity ? ccHObjectCaster::ToGenericPointCloud(m_associatedEntity) : 0;
	if (!cloud)
		return;

	unsigned count = cloud->size();
	if (count == 0 || !cloud->isVisibilityTableInstantiated())
	{
		assert(false);
		return;
	}

	ccGenericPointCloud::VisibilityTableType* visTable = cloud->getTheVisibilityArray();

	if (m_glTransEnabled)
	{
		ccGLMatrix transMat = m_glTrans.inverse();

		for (unsigned i=0; i<count; ++i)
		{
			if (!shrink || visTable->getValue(i) == POINT_VISIBLE)
			{
				CCVector3 P = *cloud->getPoint(i);
				transMat.apply(P);
				visTable->setValue(i,m_box.contains(P) ? POINT_VISIBLE : POINT_HIDDEN);
			}
		}
	}
	else
	{
		for (unsigned i=0; i<count; ++i)
		{
			if (!shrink || visTable->getValue(i) == POINT_VISIBLE)
			{
				const CCVector3* P = cloud->getPoint(i);
				visTable->setValue(i,m_box.contains(*P) ? POINT_VISIBLE : POINT_HIDDEN);
			}
		}
	}
}

ccBBox ccClipBox::getOwnBB(bool withGLFeatures/*=false*/)
{
	ccBBox bbox = m_box;

	if (withGLFeatures)
	{
		PointCoordinateType scale = computeArrowsScale();
		bbox.minCorner() -= CCVector3(scale,scale,scale);
		bbox.maxCorner() += CCVector3(scale,scale,scale);
	}

	return bbox;
}

PointCoordinateType ccClipBox::computeArrowsScale() const
{
	PointCoordinateType scale = m_box.getDiagNorm()/10;

	if (m_associatedEntity)
	{
		scale = std::max<PointCoordinateType>(scale,m_associatedEntity->getOwnBB().getDiagNorm()/100);
	}

	return scale;
}

const ColorCompType c_lightComp = ccColor::MAX/2;
const ccColor::Rgb c_lightRed  (ccColor::MAX, c_lightComp , c_lightComp);
const ccColor::Rgb c_lightGreen(c_lightComp,  ccColor::MAX, c_lightComp);
const ccColor::Rgb c_lightBlue (c_lightComp,  c_lightComp , ccColor::MAX);

void ccClipBox::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!MACRO_Draw3D(context))
		return;

	if (!m_box.isValid())
		return;

	//m_box.draw(m_selected ? context.bbDefaultCol : ccColor::magenta);
	m_box.draw(ccColor::yellow);
	
	//standard case: list names pushing
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
		glPushName(getUniqueIDForDisplay());

	if (m_selected)
	{
		//draw the interactors
		const CCVector3& minC = m_box.minCorner();
		const CCVector3& maxC = m_box.maxCorner();
		CCVector3 center = m_box.getCenter();
	
		PointCoordinateType scale = computeArrowsScale();

		//custom arrow 'context'
		CC_DRAW_CONTEXT componentContext = context;
		componentContext.flags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the arows don't push their own!
		componentContext._win = 0;

		//1 if names shall be pushed, 0 otherwise
		if (pushName)
			glPushName(0); //fake ID, will be replaced by the arrows one if any

		DrawUnitArrow(X_MINUS_ARROW*pushName,CCVector3(minC.x,center.y,center.z),CCVector3(-1.0, 0.0, 0.0),scale,ccColor::red,componentContext);
		DrawUnitArrow(X_PLUS_ARROW*pushName,CCVector3(maxC.x,center.y,center.z),CCVector3( 1.0, 0.0, 0.0),scale,ccColor::red,componentContext);
		DrawUnitArrow(Y_MINUS_ARROW*pushName,CCVector3(center.x,minC.y,center.z),CCVector3( 0.0,-1.0, 0.0),scale,ccColor::green,componentContext);
		DrawUnitArrow(Y_PLUS_ARROW*pushName,CCVector3(center.x,maxC.y,center.z),CCVector3( 0.0, 1.0, 0.0),scale,ccColor::green,componentContext);
		DrawUnitArrow(Z_MINUS_ARROW*pushName,CCVector3(center.x,center.y,minC.z),CCVector3( 0.0, 0.0,-1.0),scale,ccColor::blue,componentContext);
		DrawUnitArrow(Z_PLUS_ARROW*pushName,CCVector3(center.x,center.y,maxC.z),CCVector3( 0.0, 0.0, 1.0),scale,ccColor::blue,componentContext);
		DrawUnitCross(CROSS*pushName,minC-CCVector3(scale,scale,scale)/2.0,scale,ccColor::yellow,componentContext);
		//DrawUnitSphere(SPHERE*pushName,maxC+CCVector3(scale,scale,scale)/2.0,scale/2.0,ccColor::yellow,componentContext);
		DrawUnitTorus(X_MINUS_TORUS*pushName,CCVector3(minC.x,center.y,center.z),CCVector3(-1.0, 0.0, 0.0),scale,c_lightRed,componentContext);
		DrawUnitTorus(Y_MINUS_TORUS*pushName,CCVector3(center.x,minC.y,center.z),CCVector3( 0.0,-1.0, 0.0),scale,c_lightGreen,componentContext);
		DrawUnitTorus(Z_MINUS_TORUS*pushName,CCVector3(center.x,center.y,minC.z),CCVector3( 0.0, 0.0,-1.0),scale,c_lightBlue,componentContext);
		DrawUnitTorus(X_PLUS_TORUS*pushName,CCVector3(maxC.x,center.y,center.z),CCVector3( 1.0, 0.0, 0.0),scale,c_lightRed,componentContext);
		DrawUnitTorus(Y_PLUS_TORUS*pushName,CCVector3(center.x,maxC.y,center.z),CCVector3( 0.0, 1.0, 0.0),scale,c_lightGreen,componentContext);
		DrawUnitTorus(Z_PLUS_TORUS*pushName,CCVector3(center.x,center.y,maxC.z),CCVector3( 0.0, 0.0, 1.0),scale,c_lightBlue,componentContext);

		if (pushName)
			glPopName();
	}

	if (pushName)
		glPopName();
}
