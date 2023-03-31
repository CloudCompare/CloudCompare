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

#include "ccPointPair.h"

//static sphere for drawing with
static QSharedPointer<ccSphere> c_unitPointMarker(nullptr);
static QSharedPointer<ccCylinder> c_bodyMarker(nullptr);
static QSharedPointer<ccCone> c_headMarker(nullptr);

//ctor
ccPointPair::ccPointPair(ccPointCloud* associatedCloud) 
	: ccPolyline(associatedCloud)
{
	//do nothing
}

ccPointPair::ccPointPair(ccPolyline* obj)
	: ccPolyline(obj->getAssociatedCloud())
{
	//load points
	for (unsigned i = 0; i < obj->size(); i++)
	{
		int pId = obj->getPointGlobalIndex(i); //get global point ID
		addPointIndex(pId); //add point to this polyline
	}

	//copy name
	setName(obj->getName());
}

CCVector3 ccPointPair::getDirection()
{
	if (size() != 2)
	{
		return CCVector3(); //null vector
	}
	else
	{
		const CCVector3 start = *getPoint(0);
		const CCVector3 end = *getPoint(1);
		return end - start;
	}
}

//overidden from ccHObject
void ccPointPair::drawMeOnly(CC_DRAW_CONTEXT& context)
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

		//color-based entity picking
		bool entityPickingMode = MACRO_EntityPicking(context);
		ccColor::Rgb pickingColor;
		if (entityPickingMode)
		{
			//not fast at all!
			if (MACRO_FastEntityPicking(context))
			{
				return;
			}

			pickingColor = context.entityPicking.registerEntity(this);
		}

		//check sphere exists
		if (!c_unitPointMarker)
		{
			c_unitPointMarker.reset(new ccSphere(1.0f, nullptr, "PointMarker", 6));

			c_unitPointMarker->showColors(true);
			c_unitPointMarker->setVisible(true);
			c_unitPointMarker->setEnabled(true);
			c_unitPointMarker->showNormals(true);
		}

		//check arrow parts exist
		if (!c_bodyMarker)
		{
			c_bodyMarker.reset(new ccCylinder(1.0f, 0.9f, nullptr, "UnitNormal", 12));
			c_bodyMarker->showColors(true);
			c_bodyMarker->setVisible(true);
			c_bodyMarker->setEnabled(true);
			c_bodyMarker->setTempColor(ccColor::green);
			c_bodyMarker->showNormals(false);
		}
		if (!c_headMarker)
		{
			c_headMarker.reset(new ccCone(2.5f, 0.0f, 0.1f, 0, 0, nullptr, "UnitNormalHead", 12));
			c_headMarker->showColors(true);
			c_headMarker->setVisible(true);
			c_headMarker->setEnabled(true);
			c_headMarker->setTempColor(ccColor::green);
			c_headMarker->showNormals(false);
		}

		//not sure what this does, but it looks like fun
		//Daniel: it's dark magic!
		CC_DRAW_CONTEXT markerContext = context; //build-up point maker own 'context'
		markerContext.drawingFlags &= (~CC_ENTITY_PICKING); //we must remove the 'entity picking flag' so that the sphere doesn't override the picking color!
		markerContext.display = nullptr;

		//get camera info
		ccGLCameraParameters camera;
		glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
		glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
		glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());

		ccColor::Rgb color = entityPickingMode ? pickingColor : getMeasurementColour();

		//set draw colour
		c_unitPointMarker->setTempColor(color);

		//get point size for drawing
		float pSize = 1.0f;
		glFunc->glGetFloatv(GL_POINT_SIZE, &pSize);

		//draw points
		const ccViewportParameters& viewportParams = context.display->getViewportParameters();
		for (unsigned i = 0; i < size(); i++)
		{
			const CCVector3* P = getPoint(i);
			glFunc->glMatrixMode(GL_MODELVIEW);
			glFunc->glPushMatrix();
			ccGL::Translate(glFunc, P->x, P->y, P->z);
			float scale = context.labelMarkerSize * m_relMarkerScale * 0.2 * fmin(pSize, 4);
			if (viewportParams.perspectiveView && viewportParams.zFar > 0)
			{
				//in perspective view, the actual scale depends on the distance to the camera!
				const double* M = camera.modelViewMat.data();
				double d = (camera.modelViewMat * (*P)).norm();
				double unitD = viewportParams.zFar / 2; //we consider that the 'standard' scale is at half the depth
				scale = static_cast<float>(scale * sqrt(d / unitD)); //sqrt = empirical (probably because the marker size is already partly compensated by ccGLWindowInterface::computeActualPixelSize())
			}
			glFunc->glScalef(scale, scale, scale);
			c_unitPointMarker->showNormals(!entityPickingMode);
			c_unitPointMarker->draw(markerContext);
			glFunc->glPopMatrix();
		}

		//draw arrow
		c_bodyMarker->setTempColor(color);
		c_headMarker->setTempColor(color);
		if (size() == 2) //two points
		{
			const CCVector3 start = *getPoint(0);
			const CCVector3 end = *getPoint(1);

			CCVector3 disp = end - start;
			float length = disp.norm();
			float width = context.labelMarkerSize * m_relMarkerScale * 0.05 * std::fmin(pSize, 5);
			CCVector3 dir = disp / length;

			//transform into coord space with origin at start and arrow head at 0,0,1
			//(unashamedly pilfered from ccPlanarEntityInterface::glDrawNormal(...)
			glFunc->glMatrixMode(GL_MODELVIEW);
			glFunc->glPushMatrix();
			ccGL::Translate(glFunc, start.x, start.y, start.z); //start = 0,0,0
			ccGLMatrix mat = ccGLMatrix::FromToRotation(CCVector3(0, 0, CCCoreLib::PC_ONE), dir); //end = 0,0,1
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


//returns true if object is a pointPair
bool ccPointPair::isPointPair(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("PointPair") |
			object->getMetaData("ccCompassType").toString().contains("Lineation") |
			object->getMetaData("ccCompassType").toString().contains("Thickness") |
			object->getMetaData("ccCompassType").toString().contains("PinchNode") |
			object->getMetaData("ccCompassType").toString().contains("Relationship");
	}
	return false;
}
