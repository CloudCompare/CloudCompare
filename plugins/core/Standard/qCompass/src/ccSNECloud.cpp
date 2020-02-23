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

#include "ccSNECloud.h"
#include <ccScalarField.h>
#include <ccColorRampShader.h>
//pass ctors straight to ccPointCloud
ccSNECloud::ccSNECloud()
	: ccPointCloud()
{ 
	updateMetadata();
}

ccSNECloud::ccSNECloud(ccPointCloud* obj)
	: ccPointCloud()
{ 
	//copy points, normals and scalar fields from obj.
	*this += obj;

	//set name
	setName(obj->getName());

	//update metadata
	updateMetadata();
}

void ccSNECloud::updateMetadata()
{
	//add metadata tag defining the ccCompass class type
	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "SNECloud");
	setMetaData(*map, true);
}

//returns true if object is a lineation
bool ccSNECloud::isSNECloud(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("SNECloud");
	}
	return false;
}

void ccSNECloud::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!MACRO_Foreground(context)) //2D foreground only
		return; //do nothing

	//draw point cloud
	ccPointCloud::drawMeOnly(context);

	//draw normal vectors
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

		//get camera info
		ccGLCameraParameters camera;
		glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
		glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
		glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());

		const ccViewportParameters& viewportParams = context.display->getViewportParameters();
		
		//get point size for drawing
		float pSize = 0.0f;
		glFunc->glGetFloatv(GL_POINT_SIZE, &pSize);

		//setup
		if (pSize != 0.0f)
		{
			glFunc->glPushAttrib(GL_LINE_BIT);
			glFunc->glLineWidth(static_cast<GLfloat>(pSize));
		}

		glFunc->glMatrixMode(GL_MODELVIEW);
		glFunc->glPushMatrix();

		glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
		glFunc->glEnable(GL_BLEND);

		//get normal vector properties
		int thickID = getScalarFieldIndexByName("Thickness");
		if (thickID == -1) //if no thickness defined, try for "spacing"
		{
			thickID = getScalarFieldIndexByName("Spacing");
		}

		//draw normals
		glFunc->glBegin(GL_LINES);
		for (unsigned p = 0; p < size(); p++)
		{

			//skip out-of-range points
			if (m_currentDisplayedScalarField != nullptr)
			{
				if (!m_currentDisplayedScalarField->areNaNValuesShownInGrey()) {
					if (!m_currentDisplayedScalarField->displayRange().isInRange(m_currentDisplayedScalarField->getValue(p))) {
						continue;
					}
				}
			}

			//skip hidden points
			if (isVisibilityTableInstantiated())
			{
				if (m_pointsVisibility[p] != POINT_VISIBLE && !m_pointsVisibility.empty())
				{
					continue; //skip this point
				}
			}

			//push colour
			if (m_currentDisplayedScalarField != nullptr)
			{
				const ccColor::Rgb* col = m_currentDisplayedScalarField->getColor(m_currentDisplayedScalarField->getValue(p));
				const ccColor::Rgba col4(col->r, col->g, col->b, 200);
				glFunc->glColor4ubv(col4.rgba);
			}
			else
			{
				const ccColor::Rgba col4(200, 200, 200, 200);
				glFunc->glColor4ubv(col4.rgba);
			}

			//get length from thickness (if defined)
			float length = 1.0f;
			if (thickID != -1)
			{
				length = getScalarField(thickID)->getValue(p);
			}


			//calculate start and end points of normal vector
			const CCVector3 start = *getPoint(p);
			CCVector3 end = start + (getPointNormal(p)*length);

			//push line to opengl
			ccGL::Vertex3v(glFunc, start.u);
			ccGL::Vertex3v(glFunc, end.u);
		}
		glFunc->glEnd();
			
		glFunc->glPopAttrib(); //GL_COLOR_BUFFER_BIT

		//cleanup
		if (pSize != 0.0f)
		{
			glFunc->glPopAttrib(); //GL_LINE_BIT
		}
		glFunc->glPopMatrix();
	}
}
