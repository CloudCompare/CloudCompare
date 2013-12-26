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

#include "ccGenericGLDisplay.h"

//CCLib
#include <CCConst.h>

ccViewportParameters::ccViewportParameters()
	: pixelSize(1.0f)
	, zoom(1.0f)
	, defaultPointSize(1)
	, defaultLineWidth(1)
	, perspectiveView(false)
	, objectCenteredView(true)
	, pivotPoint(0.0f)
	, cameraCenter(0.0f)
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

bool ccViewportParameters::fromFile(QFile& in, short dataVersion, int flags)
{
	//base modelview matrix (dataVersion>=20)
	if (!viewMat.fromFile(in, dataVersion, flags))
		return false;

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
	inStream >> pivotPoint.x;
	inStream >> pivotPoint.y;
	inStream >> pivotPoint.z;
	if (dataVersion >= 25) //we now save the camera center as a separate point!
	{
		inStream >> cameraCenter.x;
		inStream >> cameraCenter.y;
		inStream >> cameraCenter.z;
	}
	else
	{
		//FIXME: doesn't work in object-centered perspective!
		cameraCenter = pivotPoint;
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
			cameraCenter.x += screenPan[0];
			cameraCenter.y += screenPan[1];
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
