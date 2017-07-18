//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: ccCompass                      #
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
//#                     COPYRIGHT: Sam Thiele  2017                        #
//#                                                                        #
//##########################################################################

#include "ccLineation.h"

//static sphere for drawing with
static QSharedPointer<ccSphere> c_unitPointMarker(nullptr);
static QSharedPointer<ccCylinder> c_bodyMarker(nullptr);

static QSharedPointer<ccCone> c_headMarker(nullptr);

//ctor
ccLineation::ccLineation(ccPointCloud* associatedCloud) 
	: ccPolyline(associatedCloud)
{
	init();
}

ccLineation::ccLineation(ccPolyline* obj)
	: ccPolyline(obj->getAssociatedCloud())
{
	init();

	//load points
	for (int i = 0; i < obj->size(); i++)
	{
		int pId = obj->getPointGlobalIndex(i); //get global point ID
		addPointIndex(pId); //add point to this polyline
	}
	updateMetadata();
	setName(obj->getName());
}

void ccLineation::init()
{
	//add metadata tag defining the ccCompass class type
	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "Lineation");
	setMetaData(*map, true);
}

void ccLineation::updateMetadata()
{
	//calculate trace orientation (trend/plunge)
	CCVector3f dir = getDirection(); dir.normalize();
	float trend, plunge;
	//special case: dir is vertical
	if (dir.z > 0.9999999) //vector = 0,0,1
	{
		trend = 0;
		if (dir.z < 0)
			plunge = 90;
		else
			plunge = -90;
	}
	else //normal cases...
	{
		CCVector3f hzComp = CCVector3f(dir.x, dir.y, 0); hzComp.normalize();

		//calculate plunge: plunge = angle between vector & vector projected onto horizontal (x,y) plane
		plunge = std::acos(dir.dot(hzComp)) * (180 / M_PI); //plunge measured from horizontal (in degrees)
		if (dir.z > 0) //lineations pointing towards the sky have negative plunges
			plunge *= -1;

		//calculate trend (N.B. I have very little idea how exactly this code work, it's kinda magic)
		//[c.f. http://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors ]
		CCVector3f N(0, 1, 0); //north vector
		float dot = hzComp.dot(N);
		float det = CCVector3f(0, 0, 1).dot(hzComp.cross(N));
		trend = std::atan2(det, dot) * (180 / M_PI); //heading measured clockwise from north (in degrees)
		if (trend < 0)
			trend += 360;
	}

	//store trend and plunge info
	QVariantMap* map = new QVariantMap();
	CCVector3 s = *getPoint(0); //start point
	CCVector3 e = *getPoint(1); //end point
	float length = (s - e).norm();

	map->insert("Sx", s.x); map->insert("Sy", s.y); map->insert("Sz", s.z);
	map->insert("Ex", e.x); map->insert("Ey", e.y); map->insert("Ez", e.z);
	map->insert("Trend", trend); map->insert("Plunge", plunge); map->insert("Length", length);
	setMetaData(*map, true);

	//rename
	QString lengthstr = QString("").asprintf("%.1f on ", length);
	QString trendAndPlungeStr = QString("%2->%3").arg((int)plunge, 2, 10, QChar('0')).arg((int)trend, 3, 10, QChar('0'));
	QString namestr = lengthstr + trendAndPlungeStr;

	setName(namestr);
}

//overidden from ccHObject
void ccLineation::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!MACRO_Foreground(context)) //2D foreground only
		return; //do nothing

	if (MACRO_Draw3D(context))
	{
		if (size() == 0) //no points -> bail!
			return;

		//get the set of OpenGL functions (version 2.1)
		QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
		if (glFunc == nullptr) {
			assert(false);
			return;
		}

		//check sphere exists
		if (!c_unitPointMarker)
		{
			c_unitPointMarker = QSharedPointer<ccSphere>(new ccSphere(1.0f, 0, "PointMarker", 6));

			c_unitPointMarker->showColors(true);
			c_unitPointMarker->setVisible(true);
			c_unitPointMarker->setEnabled(true);
		}

		//check arrow parts exist
		if (!c_bodyMarker)
		{
			c_bodyMarker = QSharedPointer<ccCylinder>(new ccCylinder(0.02f, 0.9f, 0, "UnitNormal", 12));
			c_bodyMarker->showColors(true);
			c_bodyMarker->setVisible(true);
			c_bodyMarker->setEnabled(true);
			c_bodyMarker->setTempColor(ccColor::green);
			c_bodyMarker->showNormals(false);
		}
		if (!c_headMarker)
		{
			c_headMarker = QSharedPointer<ccCone>(new ccCone(0.05f, 0.0f, 0.1f, 0, 0, 0, "UnitNormalHead", 12));
			c_headMarker->showColors(true);
			c_headMarker->setVisible(true);
			c_headMarker->setEnabled(true);
			c_headMarker->setTempColor(ccColor::green);
			c_headMarker->showNormals(false);
		}

		//not sure what this does, but it looks like fun
		CC_DRAW_CONTEXT markerContext = context; //build-up point maker own 'context'
		markerContext.drawingFlags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the sphere doesn't push its own!
		markerContext.display = 0;

		//get camera info
		ccGLCameraParameters camera;
		glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
		glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
		glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());

		//set draw colour
		c_unitPointMarker->setTempColor(getMeasurementColour());

		//draw points
		const ccViewportParameters& viewportParams = context.display->getViewportParameters();
		for (unsigned i = 0; i < size(); i++)
		{
			const CCVector3* P = getPoint(i);
			glFunc->glMatrixMode(GL_MODELVIEW);
			glFunc->glPushMatrix();
			ccGL::Translate(glFunc, P->x, P->y, P->z);
			float scale = context.labelMarkerSize * m_relMarkerScale * 0.15;
			if (viewportParams.perspectiveView && viewportParams.zFar > 0)
			{
				//in perspective view, the actual scale depends on the distance to the camera!
				const double* M = camera.modelViewMat.data();
				double d = (camera.modelViewMat * CCVector3d::fromArray(P->u)).norm();
				double unitD = viewportParams.zFar / 2; //we consider that the 'standard' scale is at half the depth
				scale = static_cast<float>(scale * sqrt(d / unitD)); //sqrt = empirical (probably because the marker size is already partly compensated by ccGLWindow::computeActualPixelSize())
			}
			glFunc->glScalef(scale, scale, scale);
			c_unitPointMarker->draw(markerContext);
			glFunc->glPopMatrix();
		}

		//draw arrow
		c_bodyMarker->setTempColor(getMeasurementColour());
		c_headMarker->setTempColor(getMeasurementColour());
		if (size() == 2) //two points
		{
			const CCVector3 start = *getPoint(0);
			const CCVector3 end = *getPoint(1);

			CCVector3 disp = end - start;
			float length = disp.norm();
			float width = length / 3; //round to nearest order of magnitude (works for any unit down to mm)
			CCVector3 dir = disp / length;

			//transform into coord space with origin at start and arrow head at 0,0,1
			//(unashamedly pilfered from ccPlanarEntityInterface::glDrawNormal(...)
			glFunc->glMatrixMode(GL_MODELVIEW);
			glFunc->glPushMatrix();
			ccGL::Translate(glFunc, start.x, start.y, start.z);
			ccGLMatrix mat = ccGLMatrix::FromToRotation(CCVector3(0, 0, PC_ONE), dir);
			glFunc->glMultMatrixf(mat.data());
			ccGL::Scale(glFunc, width, width, length);

			//draw arrow body
			glFunc->glTranslatef(0, 0, 0.45f);
			c_bodyMarker->draw(markerContext);

			//draw arrow head
			glFunc->glTranslatef(0, 0, 0.45f);
			c_headMarker->draw(markerContext);
			glFunc->glPopMatrix();
		}
	}
}

//returns true if object is a lineation
bool ccLineation::isLineation(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("Lineation");
	}
	return false;
	/*return object->isKindOf(CC_TYPES::POLY_LINE) //lineations are polylines
	&& object->hasMetaData("Sx") //ensure polyline has correct metadata for lineation
	&& object->hasMetaData("Sy")
	&& object->hasMetaData("Sz")
	&& object->hasMetaData("Ex")
	&& object->hasMetaData("Ey")
	&& object->hasMetaData("Ez")
	&& object->hasMetaData("Trend")
	&& object->hasMetaData("Plunge");*/
}