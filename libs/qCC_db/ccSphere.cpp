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
//$Rev:: 1856                                                              $
//$LastChangedDate:: 2011-05-21 21:34:24 +0200 (sam., 21 mai 2011)         $
//**************************************************************************
//

#include "ccSphere.h"

#include "ccPointCloud.h"
#include "ccNormalVectors.h"

ccSphere::ccSphere(PointCoordinateType radius,
				   const ccGLMatrix* transMat/*=0*/,
				   QString name/*=QString("Sphere")*/,
				   unsigned precision/*=24*/)
	: ccGenericPrimitive(name,transMat)
	, m_radius(radius)
{
	setDrawingPrecision(std::max<unsigned>(precision,4));  //automatically calls buildUp + 	applyTransformationToVertices
}

ccSphere::ccSphere(QString name/*=QString("Sphere")*/)
	: ccGenericPrimitive(name)
	, m_radius(0)
{
}

ccGenericPrimitive* ccSphere::clone() const
{
	return finishCloneJob(new ccSphere(m_radius,&m_transformation,getName(),m_drawPrecision));
}

bool ccSphere::buildUp()
{
	if (m_drawPrecision<4)
		return false;

	const unsigned steps = m_drawPrecision;

	//vertices
	ccPointCloud* verts = vertices();
	assert(verts);

	//vertices
	unsigned count = steps*(steps-1)+2;
	//faces
	unsigned faces = steps*((steps-2)*2+2);

	if (!init(count,true,faces,0))
	{
		ccLog::Error("[ccSphere::buildUp] Not enough memory");
		return false;
	}

	//2 first points: poles
	verts->addPoint(CCVector3(0.0,0.0,m_radius));
	verts->addNorm(0.0,0.0,1.0);

	verts->addPoint(CCVector3(0.0,0.0,-m_radius));
	verts->addNorm(0.0,0.0,-1.0);

	//then, angular sweep
	float angle_rad_step = M_PI/(float)steps;
	CCVector3 N0,N,P;
	{
		for (unsigned j=1;j<steps;++j)
		{
			float theta = (float)j * angle_rad_step;
			float cos_theta = cos(theta);
			float sin_theta = sin(theta);

			N0.x = sin_theta;
			N0.y = 0;
			N0.z = cos_theta;
		
			for (unsigned i=0;i<steps;++i)
			{
				float phi = (float)i * 2.0f * angle_rad_step;
				float cos_phi = cos(phi);
				float sin_phi = sin(phi);

				N.x = N0.x*cos_phi;
				N.y = N0.x*sin_phi;
				N.z = N0.z;
				N.normalize();

				P = N * m_radius;

				verts->addPoint(P);
				verts->addNorm(N.u);
			}
		}
	}

	//faces
	{
		assert(m_triIndexes);

		//north pole
		{
			for (unsigned i=0;i<steps;++i)
			{
				unsigned A = 2+i;
				unsigned B = (i+1<steps ? A+1 : 2);
				addTriangle(A,B,0);
			}
		}

		//slices
		for (unsigned j=1;j+1<steps;++j)
		{
			unsigned shift = 2+(j-1)*steps;		
			for (unsigned i=0;i<steps;++i)
			{
				unsigned A = shift+i;
				unsigned B = (i+1<steps ? A+1 : shift);
				assert(B<count);
				addTriangle(A,A+steps,B);
				addTriangle(B+steps,B,A+steps);
			}
		}

		//south pole
		{
			unsigned shift = 2+(steps-2)*steps;
			for (unsigned i=0;i<steps;++i)
			{
				unsigned A = shift+i;
				unsigned B = (i+1<steps ? A+1 : shift);
				assert(B<count);
				addTriangle(A,1,B);
			}
		}
	}

	updateModificationTime();
	showNormals(true);

	return true;
}

bool ccSphere::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	//parameters (dataVersion>=21)
	QDataStream outStream(&out);
	outStream << m_radius;

	return true;
}

bool ccSphere::fromFile_MeOnly(QFile& in, short dataVersion)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion))
		return false;

	//parameters (dataVersion>=21)
	QDataStream inStream(&in);
	inStream >> m_radius;

	return true;
}
