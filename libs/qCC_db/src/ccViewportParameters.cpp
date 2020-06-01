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

//CCCoreLib
#include <CCConst.h>

//Qt
#include <QRect>

ccViewportParameters::ccViewportParameters()
	: defaultPointSize(1)
	, defaultLineWidth(1)
	, perspectiveView(false)
	, objectCenteredView(true)
	, zNearCoef(0.005)
	, zNear(0)
	, zFar(0)
	, fov_deg(50.0f)
	, cameraAspectRatio(1.0f)
	, focalDistance(1.0)
	, pivotPoint(0, 0, 0)
	, cameraCenter(0, 0, focalDistance)
{
	viewMat.toIdentity();
}

ccViewportParameters::ccViewportParameters(const ccViewportParameters& params)
	: viewMat(params.viewMat)
	, defaultPointSize(params.defaultPointSize)
	, defaultLineWidth(params.defaultLineWidth)
	, perspectiveView(params.perspectiveView)
	, objectCenteredView(params.objectCenteredView)
	, zNearCoef(params.zNearCoef)
	, zNear(params.zNear)
	, zFar(params.zFar)
	, fov_deg(params.fov_deg)
	, cameraAspectRatio(params.cameraAspectRatio)
	, focalDistance(params.focalDistance)
	, pivotPoint(params.pivotPoint)
	, cameraCenter(params.cameraCenter)
{
}

bool ccViewportParameters::toFile(QFile& out) const
{
	//base modelview matrix (dataVersion>=20)
	if (!viewMat.toFile(out))
		return false;

	//other parameters (dataVersion>=20)
	QDataStream outStream(&out);
	outStream << focalDistance;
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
	outStream << fov_deg;
	outStream << cameraAspectRatio;

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
	float zoom = 1.0f;
	float pixelSize = 0.0f;
	if (dataVersion < 51)
	{
		//we read these values for backward compatibility only: we don't handle them this way anymore
		inStream >> pixelSize;
		inStream >> zoom;
	}
	else
	{
		inStream >> focalDistance;
	}

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
		ccSerializationHelper::CoordsFromDataStream(inStream, flags, _pivotPoint.u, 3);
		pivotPoint = CCVector3d::fromArray(_pivotPoint.u);
		if (dataVersion >= 25) //after version 25 the camera center is saved as a separate point!
		{
			CCVector3 _cameraCenter;
			ccSerializationHelper::CoordsFromDataStream(inStream, flags, _cameraCenter.u, 3);
			cameraCenter = CCVector3d::fromArray(_cameraCenter.u);
		}
		else
		{
			//FIXME: doesn't work in object-centered perspective!
			cameraCenter = pivotPoint;
		}
	}

	inStream >> fov_deg;
	inStream >> cameraAspectRatio;
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

	if (dataVersion >= 30 && dataVersion < 51)
	{
		//ortho mode aspect ratio (30 >= dataVersion < 51)
		float orthoAspectRatio = 0.0f;
		inStream >> orthoAspectRatio;
	}

	//for older version, deduce the focal distance from the old paramters (pixelSize and zoom)
	if (dataVersion < 51 && zoom != 1.0f)
	{
		if (perspectiveView)
		{
			focalDistance = (cameraCenter - pivotPoint).normd();
		}
		else
		{
			focalDistance = pixelSize * 2048 / computeDistanceToWidthRatio(); //2048 = average screen size? Sadly we don't have this information
		}
		setFocalDistance(focalDistance / zoom);
		ccLog::Warning("[ccViewportParameters] Approximate focal distance (sorry, the parameters of viewport objects have changed!)");
	}

	return true;
}

double ccViewportParameters::IncrementToZNearCoef(int i, int iMax)
{
	assert(i >= 0 && i <= iMax);
	return pow(10, -static_cast<double>((iMax - i) * 3) / iMax); //between 10^-3 and 1
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

const CCVector3d& ccViewportParameters::getRotationCenter() const
{
	return (objectCenteredView ? pivotPoint : cameraCenter);
}

ccGLMatrixd ccViewportParameters::computeViewMatrix() const
{
	ccGLMatrixd viewMatd;
	viewMatd.toIdentity();

	const CCVector3d& rotationCenter = getRotationCenter();

	//place origin on rotation center
	viewMatd.setTranslation(/*viewMatd.getTranslationAsVec3D()*/ -rotationCenter); // viewMatd.getTranslationAsVec3D() = (0, 0, 0)

	//rotation (viewMat is simply a rotation matrix)
	viewMatd = viewMat * viewMatd;

	//go back to initial origin, then place origin on camera center
	viewMatd.setTranslation(viewMatd.getTranslationAsVec3D() + rotationCenter - cameraCenter);

	return viewMatd;
}

ccGLMatrixd ccViewportParameters::computeScaleMatrix(const QRect& glViewport) const
{
	ccGLMatrixd scaleMatd;
	scaleMatd.toIdentity();

	//for proper aspect ratio handling
	if (glViewport.height() != 0)
	{
		double ar = static_cast<double>(glViewport.width() / (glViewport.height() * cameraAspectRatio));
		if (ar < 1.0)
		{
			//glScalef(ar, ar, 1.0);
			scaleMatd.data()[0] = ar;
			scaleMatd.data()[5] = ar;
		}
	}

	return scaleMatd;
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

void ccViewportParameters::setPivotPoint(const CCVector3d& P, bool autoUpdateFocal)
{
	pivotPoint = P;
	if (autoUpdateFocal && objectCenteredView)
	{
		//update focal distance accordingly
		focalDistance = cameraCenter.z - pivotPoint.z;
	}
}

void ccViewportParameters::setCameraCenter(const CCVector3d& C, bool autoUpdateFocal)
{
	cameraCenter = C;
	if (autoUpdateFocal && objectCenteredView)
	{
		//update focal distance accordingly
		focalDistance = cameraCenter.z - pivotPoint.z;
	}
}

void ccViewportParameters::setFocalDistance(double distance)
{
	focalDistance = distance;

	if (objectCenteredView)
	{
		cameraCenter.z = pivotPoint.z + focalDistance;
	}
}

double ccViewportParameters::computeDistanceToHalfWidthRatio() const
{
	return std::tan(CCCoreLib::DegreesToRadians(fov_deg / 2.0));
}

double ccViewportParameters::computeDistanceToWidthRatio() const
{
	return 2.0 * computeDistanceToHalfWidthRatio();
}

double ccViewportParameters::computeWidthAtFocalDist() const
{
	return getFocalDistance() * computeDistanceToWidthRatio();
}

double ccViewportParameters::computePixelSize(int glWidth) const
{
	return (glWidth > 0 ? computeWidthAtFocalDist() / glWidth : 1.0);
}
