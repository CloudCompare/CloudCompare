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
	, nearClippingDepth(std::numeric_limits<double>::quiet_NaN())
	, farClippingDepth(std::numeric_limits<double>::quiet_NaN())
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
	, nearClippingDepth(params.nearClippingDepth)
	, farClippingDepth(params.farClippingDepth)
	, zNear(params.zNear)
	, zFar(params.zFar)
	, fov_deg(params.fov_deg)
	, cameraAspectRatio(params.cameraAspectRatio)
	, focalDistance(params.focalDistance)
	, pivotPoint(params.pivotPoint)
	, cameraCenter(params.cameraCenter)
{
}

bool ccViewportParameters::toFile(QFile& out, short dataVersion) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));
	if (dataVersion < 51)
	{
		assert(false);
		return false;
	}

	//base modelview matrix (dataVersion>=20)
	if (!viewMat.toFile(out, dataVersion))
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

	if (dataVersion >= 53)
	{
		outStream << nearClippingDepth;
		outStream << farClippingDepth;
	}

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
		pivotPoint = _pivotPoint;
		if (dataVersion >= 25) //after version 25 the camera center is saved as a separate point!
		{
			CCVector3 _cameraCenter;
			ccSerializationHelper::CoordsFromDataStream(inStream, flags, _cameraCenter.u, 3);
			cameraCenter = _cameraCenter;
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

	//for older version, deduce the focal distance from the old parameters (pixelSize and zoom)
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

	// clipping depths
	if (dataVersion < 53)
	{
		nearClippingDepth = farClippingDepth = std::numeric_limits<double>::quiet_NaN();
	}
	else
	{
		inStream >> nearClippingDepth;
		inStream >> farClippingDepth;
	}

	return true;
}

short ccViewportParameters::minimumFileVersion() const
{
	// we need verison 53 to save a non-NaN near and far clipping depths
	short minVersion = (std::isnan(nearClippingDepth) && std::isnan(farClippingDepth) ? 51 : 53);

	return std::max(minVersion, viewMat.minimumFileVersion());
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

void ccViewportParameters::log() const
{
	ccLog::Print("View Matrix");
	ccLog::Print(viewMat.toString());
	ccLog::Print(QString("Default point size: %1").arg(defaultPointSize));
	ccLog::Print(QString("Default line width: %1").arg(defaultLineWidth));
	ccLog::Print(QString("Perspective view: %1").arg(perspectiveView ? "yes" : "no"));
	ccLog::Print(QString("Object-centered view: %1").arg(objectCenteredView ? "yes" : "no"));
	ccLog::Print(QString("zNearCoef: %1").arg(zNearCoef));
	ccLog::Print(QString("nearClippingDepth: %1").arg(nearClippingDepth));
	ccLog::Print(QString("farClippingDepth: %1").arg(farClippingDepth));
	ccLog::Print(QString("zNear: %1").arg(zNear));
	ccLog::Print(QString("zFar: %1").arg(zFar));
	ccLog::Print(QString("fov: %1 deg").arg(fov_deg));
	ccLog::Print(QString("camera a.r.: %1").arg(cameraAspectRatio));
	ccLog::Print(QString("focal distance: %1").arg(getFocalDistance()));
	ccLog::Print(QString("pivot point:(%1 ; %2; %3)").arg(pivotPoint.x).arg(pivotPoint.y).arg(pivotPoint.z));
	ccLog::Print(QString("camera center:(%1 ; %2; %3)").arg(cameraCenter.x).arg(cameraCenter.y).arg(cameraCenter.z));
}
