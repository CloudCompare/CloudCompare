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
//$Author:: dgm                                                            $
//$Rev:: 2219                                                              $
//$LastChangedDate:: 2012-07-20 18:03:24 +0200 (ven., 20 juil. 2012)       $
//**************************************************************************
//

#include "ccCalibratedImage.h"
#include "ccPointCloud.h"

#if defined(_WIN32) || defined(WIN32) //DGM: I've tested levmar only on Windows yet!
//#define USE_LEVMAR
#endif

#ifdef USE_LEVMAR
#include <levmar.h>
#else
#include <ConjugateGradient.h>
#endif
#include <CCConst.h>

//Qt
#include <QDir>
#include <QColor>
#include <QFile>
#include <QTextStream>

ccCalibratedImage::ccCalibratedImage()
	: ccImage()
	, m_fov_deg(45.0f)
	, m_focal_pix(1.0f)
	, m_pos(0.0f)
	, m_trans()
	, m_k1(0)
	, m_k2(0)
{
	m_trans.toIdentity();
	setAlpha(0.5f);
};

void ccCalibratedImage::setFocal(float focal_pix, bool deduceFOV/*=true*/)
{
	m_focal_pix = focal_pix;
	if (deduceFOV)
		setFovFromFocal(m_focal_pix);
}

void ccCalibratedImage::setFovFromFocal(float focal_pix)
{
	setFov(CC_RAD_TO_DEG * 2.0 * atan((float)m_height * 0.5f / m_focal_pix));
}

void ccCalibratedImage::setCameraMatrix(const ccGLMatrix& mat)
{
	m_trans = mat;
}

const ccGLMatrix& ccCalibratedImage::getCameraMatrix() const
{
	return m_trans;
}

void  ccCalibratedImage::setCameraMatrix(const CCVector3& axis, float angle_rad, const CCVector3& t)
{
	m_trans.initFromParameters(angle_rad,axis,t);
	m_trans.invert();
}

//see http://opencv.willowgarage.com/documentation/cpp/camera_calibration_and_3d_reconstruction.html
bool ccCalibratedImage::undistort()
{
	if (m_image.isNull())
		return false;

	//nothing to do
	if (m_k1==0 && m_k2==0)
		return true;

	float f2 = m_focal_pix*m_focal_pix;
	float cx = 0.5f*(float)m_width;
	float cy = 0.5f*(float)m_height;

	//look for optimal enlargement coeff
	float enlargeCoef = 1.0f;
	//{
	//	float lastrp = 0;
	//	const float c_stepSize = 0.01f;
	//	unsigned step=0;
	//	while (step<50)
	//	{
	//		enlargeCoef = 1.0f+c_stepSize*(float)step;
	//		float maxcx = 0.5f*enlargeCoef*(float)m_width;
	//		float maxcy = 0.5f*enlargeCoef*(float)m_height;
	//		float maxp2 = (maxcx*maxcx+maxcy*maxcy)/f2;
	//		float maxrp = 1.0f+maxp2*(m_k1+m_k2*maxp2);
	//		float eqx = maxrp * maxcx;
	//		float eqy = maxrp * maxcy;
	//		if ((int)eqx>=cx && (int)eqy>=cy)
	//			break;
	//		//something wrong!
	//		if (maxrp<lastrp)
	//		{
	//			if (step<2)
	//				return false;
	//			else
	//			{
	//				//step back
	//				enlargeCoef = 1.0f+c_stepSize*(float)(step-1);
	//				break;
	//			}
	//		}
	//		lastrp=maxrp;
	//		++step;
	//	}
	//}

	unsigned newWidth = (unsigned)(enlargeCoef*(float)m_width);
	unsigned newHeight = (unsigned)(enlargeCoef*(float)m_height);
	float newCx = 0.5f*(float)newWidth;
	float newCy = 0.5f*(float)newHeight;
	QImage image(QSize(newWidth,newHeight),m_image.format());
	image.fill(0);

	//image undistortion
	{
		for (unsigned i=0;i<newWidth;++i)
		{
			float x = (float)(i-newCx);
			float x2 = x*x;
			for (unsigned j=0;j<newHeight;++j)
			{
				float y = (float)(j-newCy);
				float y2 = y*y;

				float p2 = (x2+y2)/f2; //p = pix/f
				float rp = 1.0f+p2*(m_k1+p2*m_k2); //r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4
				float eqx = rp * x + cx;
				float eqy = rp * y + cy;

				int pixx=(int)eqx;
				int pixy=(int)eqy;
				if (pixx>=0 && pixx<(int)m_width && pixy>=0 && pixy<(int)m_height)
					image.setPixel(i,j,m_image.pixel(pixx,pixy));
			}
		}
	}

	//update image parameters
	m_image = image;
	m_width = newWidth;
	m_height = newHeight;
	//m_aspectRatio = (aspect ratio is not changed)
	m_k1 = m_k2 = 0;

	setName(getName()+QString("_undistort"));

	return true;
}

#ifdef USE_LEVMAR
static unsigned s_funcStepIteration = 0;
static double s_mean_e=0.0, s_std_e=0.0;
static void func_step(double *X0, double *x, int m, int n, void *data)
{
	//we extract parameters
	const double& a0 = X0[0];
	const double& a1 = X0[1];
	const double& a2 = X0[2];
	const double& b0 = X0[3];
	const double& b1 = X0[4];
	const double& b2 = X0[5];
	const double& c1 = X0[6];
	const double& c2 = X0[7];

	//global error
	s_mean_e=0.0;
	s_std_e=0.0;

	double* _A = (double*)data;
	const unsigned count = (n>>1);
	double* _x=x;
	for(unsigned k=0; k<count; ++k)
	{
		const double& xi = _A[0];
		const double& yi = _A[1];
		const double& Xi = _A[2];
		const double& Yi = _A[3];

		double d = 1.0+c1*xi+c2*yi;
		double u = (a0+a1*xi+a2*yi)-d*Xi;

		//error
		*_x = u*u;
		s_mean_e += *_x;
		s_std_e += (*_x) * (*_x);

		double v = (b0+b1*xi+b2*yi)-d*Yi;
		++_x;

		//error
		*_x = v*v;
		s_mean_e += *_x;
		s_std_e += (*_x) * (*_x);
		++_x;

		_A += 4;
	}

	s_mean_e /= (double)n;
	s_std_e = sqrt(s_std_e/(double)n - s_mean_e*s_mean_e);

	++s_funcStepIteration;
}
#endif

bool ccCalibratedImage::computeOrthoRectificationParams(CCLib::GenericIndexedCloud* keypoints3D, std::vector<KeyPoint>& keypointsImage, double a_out[], double b_out[], double c_out[]) const
{
	if (!keypoints3D)
		return false;

	unsigned count = keypointsImage.size();
	if (count<4)
		return false;

	//first guess for X (a0 a1 a2 b0 b1 b2 c1 c2)
	double norm = (double)std::max(m_width,m_height);
	double X0[8];
	X0[0] = 1.0/sqrt(norm);
	X0[1] = 1.0/norm;
	X0[2] = 1.0/norm;
	X0[3] = 1.0/sqrt(norm);
	X0[4] = 1.0/norm;
	X0[5] = 1.0/norm;
	X0[6] = 1.0/norm;
	X0[7] = 1.0/norm;

#ifndef USE_LEVMAR

	//compute the A matrix and b vector
	unsigned Neq = 2*count; //number of equations
	double *A = new double[8*Neq]; // 8 coefficients: a0 a1 a2 b0 b1 b2 c1 c2
	double *b = new double[Neq];

    //for all points
	{
		double* _A=A;
		double* _b=b;
		for (unsigned i=0;i<count;++i)
		{
			const KeyPoint& kp = keypointsImage[i];
			double kpx = (double)kp.x;
			double kpy = (double)kp.y;
			const CCVector3* P = keypoints3D->getPoint(kp.index);

			*_A++ = 1.0;
			*_A++ = kpx;
			*_A++ = kpy;
			*_A++ = 0.0;
			*_A++ = 0.0;
			*_A++ = 0.0;
			*_A++ = -kpx * (double)P->x;
			*_A++ = -kpy * (double)P->x;
			*_b++ = (double)P->x;

			*_A++ = 0.0;
			*_A++ = 0.0;
			*_A++ = 0.0;
			*_A++ = 1.0;
			*_A++ = kpx;
			*_A++ = kpy;
			*_A++ = -kpx * (double)P->y;
			*_A++ = -kpy * (double)P->y;
			*_b++ = (double)P->y;
		}
	}

	//conjugate gradient initialization
	//we solve tA.A.X=tA.b
	CCLib::ConjugateGradient<8,double> cg;
	CCLib::SquareMatrixd& tAA = cg.A();
	double* tAb = cg.b();

	//compute tA.A and tA.b
	{
		for (unsigned i=0; i<8; ++i)
		{
			//tA.A part
			for (unsigned j=i; j<8; ++j)
			{
				double sum_prod = 0;
				const double* _Ai = A+i;
				const double* _Aj = A+j;
				for (unsigned k=0; k<Neq; ++k)
				{
					//sum_prod += A[(8*2*k)+i]*A[(8*2*k)+j];
					sum_prod += (*_Ai) * (*_Aj);
					_Ai += 8;
					_Aj += 8;
				}
				tAA.m_values[j][i] = tAA.m_values[i][j] = sum_prod;
			}

			//tA.b part
			{
				double sum_prod = 0;
				const double* _Ai = A+i;
				const double* _b = b;
				for (unsigned k=0; k<Neq; ++k)
				{
					//sum_prod += A[(8*2*k)+i]*b[k];
					sum_prod += (*_Ai) * (*_b++);
					_Ai += 8;
				}
				tAb[i] = sum_prod;
			}
		}
	}

	//init. conjugate gradient
	cg.initConjugateGradient(X0);

    //conjugate gradient iterations
	{
		double convergenceThreshold = 1e-8/* * norm*/;  //max. error for convergence
		for (unsigned i=0; i<1500; ++i)
		{
			double lastError = cg.iterConjugateGradient(X0);
			if (lastError < convergenceThreshold) //converged
			{
				#ifdef _DEBUG
				printf("[computeOrthoRectificationParams] Convergence reached in %i iterations (error: %g)\n",i+1,lastError);
				#endif
				break;
			}
		}
	}

	//inverse normalization
	//for (unsigned i=0;i<8;++i)
	//	X0[i] *= norm;


	delete[] A;
	A=0;
	delete[] b;
	b=0;

#else

	double* A = new double[count*4];
	double* _A=A;
	for (unsigned i=0;i<count;++i)
	{
		const KeyPoint& kp = keypointsImage[i];
		CCVector3 P = *keypoints3D->getPoint(kp.index);
		//m_trans.apply(P);

		*_A++ = kp.x;
		*_A++ = kp.y;
		*_A++ = P.x;
		*_A++ = P.y;
	}

	//LM minimization
	double info[LM_INFO_SZ];
	s_funcStepIteration=0;
	dlevmar_dif(func_step, X0, NULL, 8, count*2, 5000, 0, info, 0, 0, A); //numerical derivatives

	delete[] A;
	A=0;

#endif

	a_out[0] = X0[0];
	a_out[1] = X0[1];
	a_out[2] = X0[2];
	b_out[0] = X0[3];
	b_out[1] = X0[4];
	b_out[2] = X0[5];
	c_out[0] = 1.0;
	c_out[1] = X0[6];
	c_out[2] = X0[7];

	return true;
}

ccImage* ccCalibratedImage::orthoRectifyAsImage(CCLib::GenericIndexedCloud* keypoints3D,
												std::vector<KeyPoint>& keypointsImage,
												double& pixelSize,
												double* minCorner/*=0*/,
												double* maxCorner/*=0*/,
												double* realCorners/*=0*/) const
{
	double a[3],b[3],c[3];

	if (!computeOrthoRectificationParams(keypoints3D,keypointsImage,a,b,c))
		return 0;

	const double& a0 = a[0];
	const double& a1 = a[1];
	const double& a2 = a[2];
	const double& b0 = b[0];
	const double& b1 = b[1];
	const double& b2 = b[2];
	//const double& c0 = c[0];
	const double& c1 = c[1];
	const double& c2 = c[2];

	//first, we compute the ortho-rectified image corners
	double corners[8];
	double xi,yi,qi;

	double halfWidth = (double)m_width/2.0;
	double halfHeight = (double)m_height/2.0;

	//top-left
	xi = -halfWidth;
	yi = -halfHeight;
	qi = 1.0+c1*xi+c2*yi;
	corners[0] = (a0+a1*xi+a2*yi)/qi;
	corners[1] = (b0+b1*xi+b2*yi)/qi;

	//top-right
	xi =  halfWidth;
	yi = -halfHeight;
	qi = 1.0+c1*xi+c2*yi;
	corners[2] = (a0+a1*xi+a2*yi)/qi;
	corners[3] = (b0+b1*xi+b2*yi)/qi;

	//bottom-right
	xi = halfWidth;
	yi = halfHeight;
	qi = 1.0+c1*xi+c2*yi;
	corners[4] = (a0+a1*xi+a2*yi)/qi;
	corners[5] = (b0+b1*xi+b2*yi)/qi;

	//bottom-left
	xi = -halfWidth;
	yi =  halfHeight;
	qi = 1.0+c1*xi+c2*yi;
	corners[6] = (a0+a1*xi+a2*yi)/qi;
	corners[7] = (b0+b1*xi+b2*yi)/qi;

	if (realCorners)
		memcpy(realCorners,corners,8*sizeof(double));

	//we look for min and max bounding box
	double minC[2] = {corners[0],corners[1]};
	double maxC[2] = {corners[0],corners[1]};

	for (unsigned k=1;k<4;++k)
	{
		const double* C = corners+2*k;
		if (minC[0] > C[0])
			minC[0] = C[0];
		else if (maxC[0] < C[0])
			maxC[0] = C[0];

		if (minC[1] > C[1])
			minC[1] = C[1];
		else if (maxC[1] < C[1])
			maxC[1] = C[1];
	}

	//output 3D boundaries (optional)
	if (minCorner)
	{
		minCorner[0]=minC[0];
		minCorner[1]=minC[1];
	}
	if (maxCorner)
	{
		maxCorner[0]=maxC[0];
		maxCorner[1]=maxC[1];
	}

	double dx = maxC[0]-minC[0];
	double dy = maxC[1]-minC[1];

	double _pixelSize = pixelSize;
	if (_pixelSize<=0.0)
	{
		unsigned maxSize = std::max(m_width,m_height);
		_pixelSize = std::max(dx,dy)/(double)maxSize;
	}
	unsigned w = (unsigned)((double)dx/_pixelSize);
	unsigned h = (unsigned)((double)dy/_pixelSize);

	QImage orthoImage(w,h,QImage::Format_ARGB32);
	if (orthoImage.isNull()) //not enough memory!
		return 0;

	QColor color;
	int blackValue = color.black();

	for (unsigned i=0;i<w;++i)
	{
		double xip = minC[0]+(double)i*_pixelSize;
		for (unsigned j=0;j<h;++j)
		{
			double yip = minC[1]+(double)j*_pixelSize;
			double q = (c2*xip-a2)*(c1*yip-b1)-(c2*yip-b2)*(c1*xip-a1);
			double p = (a0-xip)*(c1*yip-b1)-(b0-yip)*(c1*xip-a1);
			double yi = p/q;
			yi += halfHeight;
			int y = (int)yi;

			if (y>=0 && y<(int)m_height)
			{
				q = (c1*xip-a1)*(c2*yip-b2)-(c1*yip-b1)*(c2*xip-a2);
				p = (a0-xip)*(c2*yip-b2)-(b0-yip)*(c2*xip-a2);
				double  xi = p/q;
				xi += halfWidth;
				int x = (int)xi;

				if (x>=0 && x<(int)m_width)
				{
					QRgb rgb = m_image.pixel(x,y);
					//pure black pixels are treated as transparent ones!
					if (rgb != blackValue)
						orthoImage.setPixel(i,h-1-j,rgb);
					else
						orthoImage.setPixel(i,h-1-j,qRgba(qRed(rgb),qGreen(rgb),qBlue(rgb),0));
				}
				else
				{
					orthoImage.setPixel(i,h-1-j,qRgba(0,0,0,0)); //black by default
				}
			}
			else
			{
				orthoImage.setPixel(i,h-1-j,qRgba(0,0,0,0)); //black by default
			}
		}
	}

	//output pixel size (auto)
	pixelSize = _pixelSize;

	return new ccImage(orthoImage,getName());
}

bool ccCalibratedImage::OrthoRectifyAsImages(std::vector<ccCalibratedImage*> images, 
											double a[], double b[], double c[],
											unsigned maxSize,
											QDir* outputDir/*=0*/,
											std::vector<ccImage*>* result/*=0*/,
											std::vector<std::pair<double,double> >* relativePos/*=0*/)
{
	unsigned count = images.size();
	if (count==0)
		return false;

	//min & max corners for each images
	double* minCorners = new double[2*count];
	double* maxCorners = new double[2*count];
	//max dimension of all (ortho-rectified) images, horizontally or vertically
	double maxDimAllImages = 0.0;
	//corners for the global set
	double globalCorners[4] = { 0, 0, 0, 0};

	//compute output corners and max dimension for all images
	for (unsigned k=0;k<count;++k)
	{
		const double& a0 = a[k*3];
		const double& a1 = a[k*3+1];
		const double& a2 = a[k*3+2];
		const double& b0 = b[k*3];
		const double& b1 = b[k*3+1];
		const double& b2 = b[k*3+2];
		//const double& c0 = c[k*3];
		const double& c1 = c[k*3+1];
		const double& c2 = c[k*3+2];

		//first, we compute the ortho-rectified image corners
		double corners[8];
		double xi,yi,qi;

		unsigned width = images[k]->getW();
		unsigned height = images[k]->getH();

		//top-left
		xi = -0.5*(double)width;
		yi = -0.5*(double)height;
		qi = 1.0+c1*xi+c2*yi;
		corners[0] = (a0+a1*xi+a2*yi)/qi;
		corners[1] = (b0+b1*xi+b2*yi)/qi;

		//top-right
		xi =  0.5*(double)width;
		//yi = -0.5*(double)height;
		qi = 1.0+c1*xi+c2*yi;
		corners[2] = (a0+a1*xi+a2*yi)/qi;
		corners[3] = (b0+b1*xi+b2*yi)/qi;

		//bottom-right
		//xi =  0.5*(double)width;
		yi = 0.5*(double)height;
		qi = 1.0+c1*xi+c2*yi;
		corners[4] = (a0+a1*xi+a2*yi)/qi;
		corners[5] = (b0+b1*xi+b2*yi)/qi;

		//bottom-left
		xi =  -0.5*(double)width;
		//yi = 0.5*(double)height;
		qi = 1.0+c1*xi+c2*yi;
		corners[6] = (a0+a1*xi+a2*yi)/qi;
		corners[7] = (b0+b1*xi+b2*yi)/qi;

		//we look for min and max bounding box
		double* minC = minCorners+2*k;
		double* maxC = maxCorners+2*k;
		maxC[0]=minC[0]=corners[0];
		maxC[1]=minC[1]=corners[1];
		for (unsigned k=1;k<4;++k)
		{
			const double* C = corners+2*k;
			//dimension: X
			if (minC[0] > C[0])
				minC[0] = C[0];
			else if (maxC[0] < C[0])
				maxC[0] = C[0];

			if (globalCorners[0] > minC[0])
				globalCorners[0] = minC[0];
			if (globalCorners[2] < maxC[0])
				globalCorners[2] = maxC[0];

			//dimension: Y
			if (minC[1] > C[1])
				minC[1] = C[1];
			else if (maxC[1] < C[1])
				maxC[1] = C[1];

			if (globalCorners[1] > minC[1])
				globalCorners[1] = minC[1];
			if (globalCorners[3] < maxC[1])
				globalCorners[3] = maxC[1];
		}

		double dx = maxC[0]-minC[0];
		double dy = maxC[1]-minC[1];
		double maxd = std::max(dx,dy);
		if (maxd>maxDimAllImages)
			maxDimAllImages=maxd;
	}

	//deduce pixel size
	double pixelSize = maxDimAllImages/(double)maxSize;

	if (outputDir)
	{
		//write header
		QFile f(outputDir->absoluteFilePath("ortho_rectification_log.txt"));
		if (f.open(QIODevice::WriteOnly | QIODevice::Text))
		{
			QTextStream stream(&f);
			stream.setRealNumberPrecision(6);
			stream << "PixelSize" << ' ' << pixelSize << endl;
			stream << "Global3DBBox" << ' ' << globalCorners[0] << ' ' << globalCorners[1] << ' ' << globalCorners[2] << ' ' << globalCorners[3] << endl;
			int globalWidth = (int)ceil((globalCorners[2]-globalCorners[0])/pixelSize);
			int globalHeight = (int)ceil((globalCorners[3]-globalCorners[1])/pixelSize);
			stream << "Global2DBBox" << ' ' << 0 << ' ' << 0 << ' ' << globalWidth-1 << ' ' << globalHeight-1 << endl;
		}
	}

	//projet each image accordingly
	for (unsigned k=0;k<count;++k)
	{
		double* minC = minCorners+2*k;
		double* maxC = maxCorners+2*k;
		double dx = maxC[0]-minC[0];
		double dy = maxC[1]-minC[1];

		ccCalibratedImage* image = images[k];
		unsigned width = images[k]->getW();
		unsigned height = images[k]->getH();
		unsigned w = (unsigned)ceil(dx/pixelSize);
		unsigned h = (unsigned)ceil(dy/pixelSize);

		QImage orthoImage(w,h,QImage::Format_ARGB32);
		if (orthoImage.isNull()) //not enough memory!
		{
			//clear mem.
			if (result)
				while (!result->empty())
				{
					delete result->back();
					result->pop_back();
				}
			delete[] minCorners;
			delete[] maxCorners;
			return false;
		}

		//ortho rectification parameters
		const double& a0 = a[k*3];
		const double& a1 = a[k*3+1];
		const double& a2 = a[k*3+2];
		const double& b0 = b[k*3];
		const double& b1 = b[k*3+1];
		const double& b2 = b[k*3+2];
		//const double& c0 = c[k*3];
		const double& c1 = c[k*3+1];
		const double& c2 = c[k*3+2];

		for (unsigned i=0;i<w;++i)
		{
			double xip = minC[0]+(double)i*pixelSize;
			for (unsigned j=0;j<h;++j)
			{
				double yip = minC[1]+(double)j*pixelSize;
				double q = (c2*xip-a2)*(c1*yip-b1)-(c2*yip-b2)*(c1*xip-a1);
				double p = (a0-xip)*(c1*yip-b1)-(b0-yip)*(c1*xip-a1);
				double yi = p/q;

				q = (c1*xip-a1)*(c2*yip-b2)-(c1*yip-b1)*(c2*xip-a2);
				p = (a0-xip)*(c2*yip-b2)-(b0-yip)*(c2*xip-a2);
				double  xi = p/q;

				xi += 0.5*(double)width;
				yi += 0.5*(double)height;

				int x = (int)xi;
				int y = (int)yi;
				if (x>=0 && x<(int)width && y>=0 && y<(int)height)
				{
					QRgb rgb = image->data().pixel(x,y);
					//pure black pixels are treated as transparent ones!
					if (qRed(rgb)+qGreen(rgb)+qBlue(rgb)>0)
						orthoImage.setPixel(i,h-1-j,rgb);
					else
						orthoImage.setPixel(i,h-1-j,qRgba(qRed(rgb),qGreen(rgb),qBlue(rgb),0));
				}
				else
					orthoImage.setPixel(i,h-1-j,qRgba(255,0,255,0));
			}
		}

		//eventually compute relative pos
		if (relativePos)
		{
			double xShift = (minC[0]-minCorners[0])/pixelSize;
			double yShift = (minC[1]-minCorners[1])/pixelSize;
			relativePos->push_back(std::pair<double,double>(xShift,yShift));
		}

		if (outputDir)
		{
			//export image
			QString exportFilename = QString("ortho_rectified_%1.png").arg(image->getName());
			orthoImage.save(outputDir->absoluteFilePath(exportFilename));

			//export meta-data
			QFile f(outputDir->absoluteFilePath("ortho_rectification_log.txt"));
			if (f.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) //always append
			{
				double xShiftGlobal = (minC[0]-globalCorners[0])/pixelSize;
				double yShiftGlobal = (minC[1]-globalCorners[1])/pixelSize;
				QTextStream stream(&f);
				stream.setRealNumberPrecision(6);
				stream << "Image" << ' ' << exportFilename  << ' ';
				stream << "Local3DBBox" << ' ' << minC[0] << ' ' << minC[1] << ' ' << maxC[0] << ' ' << maxC[1] << ' ';
				stream << "Local2DBBox" << ' ' << xShiftGlobal << ' ' << yShiftGlobal <<  ' ' << xShiftGlobal+(double)(w-1) << ' ' << yShiftGlobal+(double)(h-1) << endl;
				f.close();
			}
		}

		if (result)
			result->push_back(new ccImage(orthoImage,image->getName()));
	}

	delete[] minCorners;
	delete[] maxCorners;

	return true;
}


ccPointCloud* ccCalibratedImage::orthoRectifyAsCloud(CCLib::GenericIndexedCloud* keypoints3D, std::vector<KeyPoint>& keypointsImage) const
{
	double a[3],b[3],c[3];

	if (!computeOrthoRectificationParams(keypoints3D,keypointsImage,a,b,c))
		return 0;

	const double& a0 = a[0];
	const double& a1 = a[1];
	const double& a2 = a[2];
	const double& b0 = b[0];
	const double& b1 = b[1];
	const double& b2 = b[2];
	//const double& c0 = c[0];
	const double& c1 = c[1];
	const double& c2 = c[2];

	PointCoordinateType defaultZ = 0.0;

	ccPointCloud* proj = new ccPointCloud(getName()+QString(".ortho-rectified"));
	if (!proj->reserve(m_width*m_height))
	{
		delete proj;
		return 0;
	}

	if (!proj->reserveTheRGBTable())
	{
		delete proj;
		return 0;
	}
	proj->showColors(true);
	colorType C[3];

	unsigned realCount=0;

	//ortho rectification
	{
		for (unsigned pi = 0; pi<(unsigned)m_width; ++pi)
		{
			double xi = (double)pi-0.5*(double)m_width;
			for (unsigned pj = 0; pj<(unsigned)m_height; ++pj)
			{
				double yi = (double)pj-0.5*(double)m_height;
				double qi = (1.0+c1*xi+c2*yi);
				CCVector3 P((a0+a1*xi+a2*yi)/qi,
							(b0+b1*xi+b2*yi)/qi,
							defaultZ);

				//and color?
				QRgb rgb = m_image.pixel(pi,pj);
				C[0]=qRed(rgb);
				C[1]=qGreen(rgb);
				C[2]=qBlue(rgb);
				if ((int)C[0]+(int)C[1]+(int)C[2]>0)
				{
					//add point
					proj->addPoint(P);
					//and color
					proj->addRGBColor(C);
					++realCount;
				}
			}
		}
	}

	if (realCount==0)
	{
		delete proj;
		proj=0;
	}
	else
	{
		proj->resize(realCount);
	}

	return proj;
}

void ccCalibratedImage::setDistortionCoefficients(float k1, float k2)
{
	m_k1 = k1;
	m_k2 = k2;
}

void ccCalibratedImage::getDistortionCoefficients(float& k1, float& k2) const
{
	k1 = m_k1;
	k2 = m_k2;
}
