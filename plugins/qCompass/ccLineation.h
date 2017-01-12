#ifndef CC_LINEATION_HEADER
#define CC_LINEATION_HEADER

#include <ccPolyline.h>
#include <ccSphere.h>
#include <ccCylinder.h>
#include <ccCone.h>
#include <GenericIndexedCloudPersist.h>
#include <ccPointCloud.h>

//static sphere for drawing withs
static QSharedPointer<ccSphere> c_unitPointMarker(0);
static QSharedPointer<ccCylinder> c_bodyMarker(0);
static QSharedPointer<ccCone> c_headMarker(0);

class ccLineation : public ccPolyline
{
public:
	ccLineation(ccPointCloud* associatedCloud)
		: ccPolyline(associatedCloud)
	{ }

	virtual ~ccLineation() {}

	//inherited from ccHObject
	inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::POLY_LINE; }

	//get the direction of this lineation (not normalized) 
	CCVector3 getDirection()
	{
		if (size() != 2)
		{
			return CCVector3();
		} else
		{
			const CCVector3 start = *getPoint(0);
			const CCVector3 end = *getPoint(1);
			return end - start;
		}
	}

	ccColor::Rgba colour = ccColor::green;

protected:
	float m_relMarkerScale = 5.0;

	//overidden from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override
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
			c_unitPointMarker->setTempColor(colour);

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
			if (size() == 2) //two points
			{
				const CCVector3 start = *getPoint(0);
				const CCVector3 end = *getPoint(1);

				CCVector3 disp = end - start;
				float length = disp.norm();
				float width = length/3; //round to nearest order of magnitude (works for any unit down to mm)
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
};

#endif
