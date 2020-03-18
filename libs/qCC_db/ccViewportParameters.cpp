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

#include "ccViewportParameters.h"

//CCLib
#include <CCConst.h>

ccViewportParameters::ccViewportParameters()
	: pixelSize(1.0f)
	, zoom(1.0f)
	, defaultPointSize(1)
	, defaultLineWidth(1)
	, perspectiveView(false)
	, objectCenteredView(true)
	, zNearCoef(0.005)
	, zNear(0)
	, zFar(0)
	, pivotPoint(0, 0, 0)
	, cameraCenter(0, 0, 0)
	, fov(30.0f)
	, perspectiveAspectRatio(1.0f)
	, orthoAspectRatio(1.0f)
{
	viewMat.toIdentity();
}

ccViewportParameters::ccViewportParameters(const ccViewportParameters& params)
	: pixelSize(params.pixelSize)
	, zoom(params.zoom)
	, viewMat(params.viewMat)
	, defaultPointSize(params.defaultPointSize)
	, defaultLineWidth(params.defaultLineWidth)
	, perspectiveView(params.perspectiveView)
	, objectCenteredView(params.objectCenteredView)
	, zNearCoef(params.zNearCoef)
	, zNear(params.zNear)
	, zFar(params.zFar)
	, pivotPoint(params.pivotPoint)
	, cameraCenter(params.cameraCenter)
	, fov(params.fov)
	, perspectiveAspectRatio(params.perspectiveAspectRatio)
	, orthoAspectRatio(params.orthoAspectRatio)
{
}

bool ccViewportParameters::toFile(QFile& out) const
{
	//base modelview matrix (dataVersion>=20)
	if (!viewMat.toFile(out))
		return false;

	//other parameters (dataVersion>=20)
	QDataStream outStream(&out);
	outStream << pixelSize;
	outStream << zoom;
	outStream << defaultPointSize;
	outStream << defaultLineWidth;
	outStream << perspectiveView;
	outStream << objectCenteredView;
	outStream << pivotPoint.x;
	outStream << pivotPoint.y;
	outStream << pivotPoint.z;
	outStream << cameraCenter.x;
	outStream << cameraCenter.y;
	outStream << cameraCenter.z;
	outStream << fov;
	outStream << perspectiveAspectRatio;
	//ortho mode aspect ratio (dataVersion>=30)
	outStream << orthoAspectRatio;

	return true;
}

bool ccViewportParameters::fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	//base modelview matrix (dataVersion>=20)
	if (dataVersion >= 36) //we now save the camera matrix in double precision
	{
		if (!viewMat.fromFile(in, dataVersion, flags, oldToNewIDMap))
			return false;
	}
	else
	{
		//camera matrix was saved in standard (float) precision
		ccGLMatrix _viewMat;
		if (!_viewMat.fromFile(in, dataVersion, flags, oldToNewIDMap))
			return false;
		viewMat = ccGLMatrixd(_viewMat.data());
	}

	//other parameters (dataVersion>=20)
	QDataStream inStream(&in);
	inStream >> pixelSize;
	//before version 25, we were saving the inverse of 'pixelSize' ('globalZoom')
	if (dataVersion < 25)
		pixelSize = (pixelSize> ZERO_TOLERANCE ? 1.0f/pixelSize : 1.0f);
	inStream >> zoom;
	inStream >> defaultPointSize;
	inStream >> defaultLineWidth;
	inStream >> perspectiveView;
	inStream >> objectCenteredView;
	if (dataVersion >= 36) //we now save the camera center and pivot point in double precision
	{
		inStream >> pivotPoint.x;
		inStream >> pivotPoint.y;
		inStream >> pivotPoint.z;
		inStream >> cameraCenter.x;
		inStream >> cameraCenter.y;
		inStream >> cameraCenter.z;
	}
	else
	{
		CCVector3 _pivotPoint;
		ccSerializationHelper::CoordsFromDataStream(inStream,flags,_pivotPoint.u,3);
		pivotPoint = CCVector3d::fromArray(_pivotPoint.u);
		if (dataVersion >= 25) //after version 25 the camera center is saved as a separate point!
		{
			CCVector3 _cameraCenter;
			ccSerializationHelper::CoordsFromDataStream(inStream,flags,_cameraCenter.u,3);
			cameraCenter = CCVector3d::fromArray(_cameraCenter.u);
		}
		else
		{
			//FIXME: doesn't work in object-centered perspective!
			cameraCenter = pivotPoint;
		}
	}
	inStream >> fov;
	inStream >> perspectiveAspectRatio;
	if (dataVersion < 25) //screenPan has been replaced by cameraCenter(x,y) in object centered mode!
	{
		float screenPan[2];
		inStream >> screenPan[0];
		inStream >> screenPan[1];

		if (objectCenteredView)
		{
			cameraCenter.x += static_cast<double>(screenPan[0]);
			cameraCenter.y += static_cast<double>(screenPan[1]);
		}
	}

	//ortho mode aspect ratio (dataVersion>=30)
	if (dataVersion >= 30)
	{
		inStream >> orthoAspectRatio;
	}
	else
	{
		orthoAspectRatio = 1.0f;
	}

	return true;
}

int ccViewportParameters::ZNearCoefToIncrement(double coef, int iMax)
{
	assert(coef >= 0 && coef <= 1.0);
	double id = -(iMax / 3.0) * log10(coef);
	int i = static_cast<int>(id);
	//cope with numerical inaccuracies
	if (fabs(id - i) > fabs(id - (i + 1)))
	{
		++i;
	}
	assert(i >= 0 && i <= iMax);
	return iMax - i;
}

ccGLMatrixd ccViewportParameters::computeViewMatrix(const CCVector3d& cameraCenter) const
{
	ccGLMatrixd viewMatd;
	viewMatd.toIdentity();

	//apply current camera parameters (see trunk/doc/rendering_pipeline.doc)
	if (objectCenteredView)
	{
		//place origin on pivot point
		viewMatd.setTranslation(/*viewMatd.getTranslationAsVec3D()*/ -pivotPoint);

		//rotation (viewMat is simply a rotation matrix around the pivot here!)
		viewMatd = viewMat * viewMatd;

		//go back to initial origin
		//then place origin on camera center
		viewMatd.setTranslation(viewMatd.getTranslationAsVec3D() + pivotPoint - cameraCenter);
	}
	else
	{
		//place origin on camera center
		viewMatd.setTranslation(/*viewMatd.getTranslationAsVec3D()*/ -cameraCenter);

		//rotation (viewMat is the rotation around the camera center here - no pivot)
		viewMatd = viewMat * viewMatd;
	}

	return viewMatd;
}

ccGLMatrixd ccViewportParameters::computeScaleMatrix(const QRect& glViewport) const
{
	ccGLMatrixd scaleMatd;
	scaleMatd.toIdentity();
	if (perspectiveView) //perspective mode
	{
		//for proper aspect ratio handling
		if (glViewport.height() != 0)
		{
			double ar = static_cast<double>(glViewport.width() / (glViewport.height() * perspectiveAspectRatio));
			if (ar < 1.0)
			{
				//glScalef(ar, ar, 1.0);
				scaleMatd.data()[0] = ar;
				scaleMatd.data()[5] = ar;
			}
		}
	}
	else //ortho. mode
	{
		//apply zoom
		double totalZoom = zoom / pixelSize;
		//glScalef(totalZoom,totalZoom,totalZoom);
		scaleMatd.data()[0] = totalZoom;
		scaleMatd.data()[5] = totalZoom;
		scaleMatd.data()[10] = totalZoom;
	}

	return scaleMatd;
}

CCVector3d ccViewportParameters::getRealCameraCenter(const ccBBox& visibleObjectsBBox) const
{
	if (perspectiveView)
	{
		//the camera center is always defined in perspective mode
		return cameraCenter;
	}
	else
	{
		//in orthographic mode, we put the camera at the center of the
		//visible objects (along the viewing direction)
		return CCVector3d(	cameraCenter.x,
							cameraCenter.y,
							visibleObjectsBBox.isValid() ? visibleObjectsBBox.getCenter().z : 0.0);
	}
}

CCVector3d ccViewportParameters::getViewDir() const
{
	//view direction is (the opposite of) the 3rd line of the current view matrix
	const double* M = viewMat.data();
	CCVector3d axis(-M[2], -M[6], -M[10]);
	axis.normalize();

	return axis;
}

CCVector3d ccViewportParameters::getUpDir() const
{
	//up direction is the 2nd line of the current view matrix
	const double* M = viewMat.data();
	CCVector3d axis(M[1], M[5], M[9]);
	axis.normalize();

	return axis;
}