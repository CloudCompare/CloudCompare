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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#include "ccGenericGLDisplay.h"

ccViewportParameters::ccViewportParameters()
	: globalZoom(1.0f)
	, zoom(1.0f)
	, defaultPointSize(1)
	, defaultLineWidth(1)
	, perspectiveView(false)
	, objectCenteredPerspective(true)
	, pivotPoint(0.0f)
	, fov(50.0f)
	, aspectRatio(1.0f)

{
	screenPan[0] = screenPan[1] = 0.0f;
	baseViewMat.toIdentity();
}

ccViewportParameters::ccViewportParameters(const ccViewportParameters& params)
	: globalZoom(params.globalZoom)
	, zoom(params.zoom)
	, baseViewMat(params.baseViewMat)
	, defaultPointSize(params.defaultPointSize)
	, defaultLineWidth(params.defaultLineWidth)
	, perspectiveView(params.perspectiveView)
	, objectCenteredPerspective(params.objectCenteredPerspective)
	, pivotPoint(params.pivotPoint)
	, fov(params.fov)
	, aspectRatio(params.aspectRatio)
{
	screenPan[0] = params.screenPan[0];
	screenPan[1] = params.screenPan[1];
}

bool ccViewportParameters::toFile(QFile& out) const
{
	//base modelview matrix (dataVersion>=20)
	if (!baseViewMat.toFile(out))
		return false;

	//other parameters (dataVersion>=20)
	QDataStream outStream(&out);
	outStream << globalZoom;
    outStream << zoom;
    outStream << defaultPointSize;
    outStream << defaultLineWidth;
	outStream << perspectiveView;
	outStream << objectCenteredPerspective;
	outStream << pivotPoint.x;
	outStream << pivotPoint.y;
	outStream << pivotPoint.z;
	outStream << fov;
	outStream << aspectRatio;
	outStream << screenPan[0];
	outStream << screenPan[1];

	return true;
}

bool ccViewportParameters::fromFile(QFile& in, short dataVersion)
{
	//base modelview matrix (dataVersion>=20)
	if (!baseViewMat.fromFile(in,dataVersion))
		return false;

	//other parameters (dataVersion>=20)
	QDataStream inStream(&in);
	inStream >> globalZoom;
    inStream >> zoom;
    inStream >> defaultPointSize;
    inStream >> defaultLineWidth;
	inStream >> perspectiveView;
	inStream >> objectCenteredPerspective;
	inStream >> pivotPoint.x;
	inStream >> pivotPoint.y;
	inStream >> pivotPoint.z;
	inStream >> fov;
	inStream >> aspectRatio;
	inStream >> screenPan[0];
	inStream >> screenPan[1];

	return true;
}
