//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qAnimation                      #
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
//#             COPYRIGHT: Ryan Wicks, 2G Robotics Inc., 2015              #
//#                                                                        #
//##########################################################################

#include "ViewInterpolate.h"
#include <math.h>

void
poly_create(double x[4], double y[4], double result[4])
{
	double a1 = 2*(x[2]-x[0]);
	double b1 = 2*(x[2]-x[1]);
	double c1 = (6 / (double)(x[2]-x[1])) * (y[2] - y[1]) + (6 / (double)(x[1]-x[0])) * (y[0] - y[1]);
	double a2 = 2*(x[2]-x[1]);
	double b2 = 2*(x[3]-x[1]);
	double c2 = (6 / (double)(x[3]-x[2])) * (y[3] - y[2]) + (6 / (double)(x[2]-x[1])) * (y[1] - y[2]);

	double q = (a2*c1 - a1*c2) / (a2*b1 - a1*b2);
	double p = (c2 - b2*q) / a2;

	double a = (p/(6*(x[2]-x[1])));
	double b = (q/(6*(x[2]-x[1])));
	double c = (y[1]/(x[2]-x[1]) - (p*(x[2]-x[1]))/6);
	double d = (y[2]/(x[2]-x[1]) - (q*(x[2]-x[1]))/6);

	result[0] = a;
	result[1] = b;
	result[2] = c;
	result[3] = d;

	if (x[0] > x[1] || x[1] > x[2] || x[2] > x[3]) {
		result[0] = 0;
		result[1] = 0;
		result[2] = 0;
		result[3] = 0;
	}
}

double
poly_interpolate(double t[4], double coeffs[4], double r)
{
	double x = t[1] + (t[2] - t[1])*r;
	assert(x >= t[1]);
	assert(x <= t[2]);
	double a = coeffs[0];
	double b = coeffs[1];
	double c = coeffs[2];
	double d = coeffs[3];
	double dx1 = x - t[1];
	double dx2 = t[2] - x;
	return a*pow(dx2, 3) + b*pow(dx1, 3) + c*dx2 + d*dx1;
}

ViewInterpolate::ViewInterpolate()
    : m_view0 (nullptr)
    , m_view1 (nullptr)
    , m_view2 (nullptr)
    , m_view3 (nullptr)
    , m_totalSteps ( 0 )
    , m_currentStep ( 0 )
    , m_doCubic ( 0 )
{
}

ViewInterpolate::ViewInterpolate(cc2DViewportObject * viewPort0, cc2DViewportObject * viewPort1,  cc2DViewportObject * viewPort2,  cc2DViewportObject * viewPort3, double dt0, double dt1, double dt2, double dt3, bool doCubic, unsigned int stepCount )
    : m_view0 ( viewPort0 )
    , m_view1 ( viewPort1 )
    , m_view2 ( viewPort2 )
    , m_view3 ( viewPort3 )
    , m_totalSteps ( stepCount )
    , m_currentStep ( 0 )
    , m_doCubic ( doCubic )
{
	if (not doCubic) return;

	m_ts[0] = 0;
	m_ts[1] = dt0;
	m_ts[2] = dt0+dt1;
	m_ts[3] = dt0+dt1+dt2;

	const ccViewportParameters vp[4] = {
		m_view0->getParameters(), m_view1->getParameters(),
		m_view2->getParameters(), m_view3->getParameters()};

	double Y[4];
	for (int i = 0; i < 4; i++) Y[i] = vp[i].cameraCenter.x;
	poly_create(m_ts, Y, m_cam_cntr_x);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].cameraCenter.y;
	poly_create(m_ts, Y, m_cam_cntr_y);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].cameraCenter.z;
	poly_create(m_ts, Y, m_cam_cntr_z);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].pivotPoint.x;
	poly_create(m_ts, Y, m_pvt_pt_x);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].pivotPoint.y;
	poly_create(m_ts, Y, m_pvt_pt_y);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].pivotPoint.z;
	poly_create(m_ts, Y, m_pvt_pt_z);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].pixelSize;
	poly_create(m_ts, Y, m_pixelSize);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].zoom;
	poly_create(m_ts, Y, m_zoom);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].defaultPointSize;
	poly_create(m_ts, Y, m_defaultPointSize);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].defaultLineWidth;
	poly_create(m_ts, Y, m_defaultLineWidth);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].zNearCoef;
	poly_create(m_ts, Y, m_zNearCoef);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].zNear;
	poly_create(m_ts, Y, m_zNear);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].zFar;
	poly_create(m_ts, Y, m_zFar);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].fov;
	poly_create(m_ts, Y, m_fov);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].perspectiveAspectRatio;
	poly_create(m_ts, Y, m_perspectiveAspectRatio);
	for (int i = 0; i < 4; i++) Y[i] = vp[i].orthoAspectRatio;
	poly_create(m_ts, Y, m_orthoAspectRatio);

	double phi[4], theta[4], psi[4];
	Vector3Tpl<double> tr[4];

	for (int i = 0; i < 4; i++)
		vp[i].viewMat.getParameters(phi[i], theta[i], psi[i], tr[i]);

	for (int i = 0; i < 4; i++) Y[i] = phi[i];
	poly_create(m_ts, Y, m_vm_phi);
	for (int i = 0; i < 4; i++) Y[i] = theta[i];
	poly_create(m_ts, Y, m_vm_theta);
	for (int i = 0; i < 4; i++) Y[i] = psi[i];
	poly_create(m_ts, Y, m_vm_psi);
	for (int i = 0; i < 4; i++) Y[i] = tr[i].x;
	poly_create(m_ts, Y, m_vm_translation_x);
	for (int i = 0; i < 4; i++) Y[i] = tr[i].y;
	poly_create(m_ts, Y, m_vm_translation_y);
	for (int i = 0; i < 4; i++) Y[i] = tr[i].z;
	poly_create(m_ts, Y, m_vm_translation_z);
}

//helper function for interpolating between simple numerical types
template <class T> T InterpolateNumber( T start, T end, double interpolationFraction )
{
	return static_cast < T > ( static_cast<double>(start) + (static_cast<double>(end) - static_cast<double>(start)) * interpolationFraction );
}

bool ViewInterpolate::nextView ( cc2DViewportObject& outViewport )
{
	if (m_currentStep >= m_totalSteps
	    ||	m_view0 == nullptr
	    ||	m_view1 == nullptr
	    ||	m_view2 == nullptr
	    ||	m_view3 == nullptr)
	{
		return false;
	}

	//initial and final views
	const ccViewportParameters& viewParams0 = m_view0->getParameters();
	const ccViewportParameters& viewParams1 = m_view1->getParameters();
	const ccViewportParameters& viewParams2 = m_view2->getParameters();
	const ccViewportParameters& viewParams3 = m_view3->getParameters();
	ccViewportParameters interpView = m_view1->getParameters();

	//interpolation fraction
	double interpolate_fraction = static_cast <double>(m_currentStep) / m_totalSteps;

	if (not m_doCubic) {
		interpView.pixelSize              = InterpolateNumber ( viewParams1.pixelSize, viewParams2.pixelSize, interpolate_fraction );
		interpView.zoom                   = InterpolateNumber ( viewParams1.zoom, viewParams2.zoom, interpolate_fraction );
		interpView.defaultPointSize       = InterpolateNumber ( viewParams1.defaultPointSize, viewParams2.defaultPointSize, interpolate_fraction );
		interpView.defaultLineWidth       = InterpolateNumber ( viewParams1.defaultLineWidth, viewParams2.defaultLineWidth, interpolate_fraction );
		interpView.zNearCoef              = InterpolateNumber ( viewParams1.zNearCoef, viewParams2.zNearCoef, interpolate_fraction );
		interpView.zNear                  = InterpolateNumber ( viewParams1.zNear, viewParams2.zNear, interpolate_fraction );
		interpView.zFar                   = InterpolateNumber ( viewParams1.zFar, viewParams2.zFar, interpolate_fraction );
		interpView.fov                    = InterpolateNumber ( viewParams1.fov, viewParams2.fov, interpolate_fraction );
		interpView.perspectiveAspectRatio = InterpolateNumber ( viewParams1.perspectiveAspectRatio, viewParams2.perspectiveAspectRatio, interpolate_fraction );
		interpView.orthoAspectRatio       = InterpolateNumber ( viewParams1.orthoAspectRatio, viewParams2.orthoAspectRatio, interpolate_fraction );
		interpView.viewMat                = ccGLMatrixd::Interpolate(interpolate_fraction, viewParams1.viewMat, viewParams2.viewMat);
		interpView.pivotPoint             = viewParams1.pivotPoint + (viewParams2.pivotPoint - viewParams1.pivotPoint) * interpolate_fraction;
		interpView.cameraCenter           = viewParams1.cameraCenter + (viewParams2.cameraCenter - viewParams1.cameraCenter) * interpolate_fraction;
	} else {
		interpView.pixelSize              = poly_interpolate(m_ts, m_pixelSize, interpolate_fraction);
		interpView.zoom                   = poly_interpolate(m_ts, m_zoom, interpolate_fraction);
		interpView.defaultPointSize       = poly_interpolate(m_ts, m_defaultPointSize, interpolate_fraction);
		interpView.defaultLineWidth       = poly_interpolate(m_ts, m_defaultLineWidth, interpolate_fraction);
		interpView.zNearCoef              = poly_interpolate(m_ts, m_zNearCoef, interpolate_fraction);
		interpView.zNear                  = poly_interpolate(m_ts, m_zNear, interpolate_fraction);
		interpView.zFar                   = poly_interpolate(m_ts, m_zFar, interpolate_fraction);
		interpView.fov                    = poly_interpolate(m_ts, m_fov, interpolate_fraction);
		interpView.perspectiveAspectRatio = poly_interpolate(m_ts, m_perspectiveAspectRatio, interpolate_fraction);
		interpView.orthoAspectRatio       = poly_interpolate(m_ts, m_orthoAspectRatio, interpolate_fraction);
		interpView.pivotPoint.x           = poly_interpolate(m_ts, m_pvt_pt_x, interpolate_fraction);
		interpView.pivotPoint.y           = poly_interpolate(m_ts, m_pvt_pt_y, interpolate_fraction);
		interpView.pivotPoint.z           = poly_interpolate(m_ts, m_pvt_pt_z, interpolate_fraction);
		interpView.cameraCenter.x         = poly_interpolate(m_ts, m_cam_cntr_x, interpolate_fraction);
		interpView.cameraCenter.y         = poly_interpolate(m_ts, m_cam_cntr_y, interpolate_fraction);
		interpView.cameraCenter.z         = poly_interpolate(m_ts, m_cam_cntr_z, interpolate_fraction);

		/* Using Euler angles is less than ideal. But easy to implement... */

		Vector3Tpl<double> tr;
		double phi, theta, psi;

		phi                               = poly_interpolate(m_ts, m_vm_phi, interpolate_fraction);
		theta                             = poly_interpolate(m_ts, m_vm_theta, interpolate_fraction);
		psi                               = poly_interpolate(m_ts, m_vm_psi, interpolate_fraction);
		tr.x                              = poly_interpolate(m_ts, m_vm_translation_x, interpolate_fraction);
		tr.y                              = poly_interpolate(m_ts, m_vm_translation_y, interpolate_fraction);
		tr.z                              = poly_interpolate(m_ts, m_vm_translation_z, interpolate_fraction);
		interpView.viewMat.initFromParameters(phi, theta, psi, tr);
	}

	outViewport.setParameters( interpView );

	++m_currentStep;

	return true;
}
