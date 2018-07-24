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

		//glDrawParams glParams;
		//getDrawingParameters(glParams);

		//get camera info
		ccGLCameraParameters camera;
		glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
		glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
		glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());

		const ccViewportParameters& viewportParams = context.display->getViewportParameters();
		
		//get point size for drawing
		float pSize;
		glFunc->glGetFloatv(GL_POINT_SIZE, &pSize);

		//draw normal vectors if highlighted
		//if ((m_isHighlighted | m_isAlternate | m_isActive))
		//{
			//setup
			if (pSize != 0)
			{
				glFunc->glPushAttrib(GL_LINE_BIT);
				glFunc->glLineWidth(static_cast<GLfloat>(pSize));
			}

			glFunc->glMatrixMode(GL_MODELVIEW);
			glFunc->glPushMatrix();
			glFunc->glEnable(GL_BLEND);

			//get normal vector properties
			int thickID = getScalarFieldIndexByName("Thickness");
			//int weightID = getScalarFieldIndexByName("Weight");
			//float weight;
			//float maxWeight = getScalarField( weightID )->getMax();

			//draw normals
			glFunc->glBegin(GL_LINES);
			for (unsigned p = 0; p < size(); p++)
			{
				//get weight
				//weight = getScalarField(weightID)->getValue(p);
				//weight /= maxWeight;

				//push colour
				const ccColor::Rgb* col = m_currentDisplayedScalarField->getColor(m_currentDisplayedScalarField->getValue(p));
				const ccColor::Rgba col4(col->r, col->g, col->b,200);
				glFunc->glColor4ubv(col4.rgba);

				//get length from thickness (if defined)
				float length = 1.0;
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
			
			//cleanup
			if (pSize != 0) {
				glFunc->glPopAttrib();
			}
			glFunc->glPopMatrix();
	}
}
