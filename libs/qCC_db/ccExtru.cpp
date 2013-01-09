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

#include "ccExtru.h"

//Triangle Lib
#include <triangle.h>

//qCC_db
#include "ccPointCloud.h"
#include "ccNormalVectors.h"

//CCLib
#include <SimpleCloud.h>
#include <Polyline.h>
#include <ManualSegmentationTools.h>

ccExtru::ccExtru(const std::vector<CCVector2>& profile,
				 PointCoordinateType height,
				 const ccGLMatrix* transMat /*= 0*/,
				 QString name/*="Extrusion"*/)
	: ccGenericPrimitive(name,transMat)
	, m_height(height)
{
	m_profile = profile;
	assert(m_profile.size()>2);

	buildUp();
	applyTransformationToVertices();
}

ccExtru::ccExtru(QString name/*="Plane"*/)
	: ccGenericPrimitive(name)
	, m_height(0)
{
}

ccGenericPrimitive* ccExtru::clone() const
{
	return finishCloneJob(new ccExtru(m_profile,m_height,&m_transformation,getName()));
}

bool ccExtru::buildUp()
{
	unsigned count = m_profile.size();
	if (count<3)
		return false;

	//let's try to triangulate the profile first
	//we use the external library 'Triangle'
	triangulateio in;
	memset(&in,0,sizeof(triangulateio));

	in.numberofpoints = m_profile.size();
	in.pointlist = (REAL*)(&m_profile[0]);

	//DGM: we check that last vertex is different from the first one!
	//(yes it happens ;)
	if (m_profile.back().x == m_profile.front().x && 
		m_profile.back().y == m_profile.front().y)
	{
		--in.numberofpoints;
		if (in.numberofpoints<3)
			return false;
	}

	try
	{
		triangulate ( "zQN", &in, &in, 0 );
	}
	catch (...)
	{
		ccLog::Error("[ccPlane::buildUp] Profile triangulation failed");
		return false;
	}
	//Console::print("Nombre de triangles : %i\n",in.numberoftriangles);

	unsigned numberOfTriangles = in.numberoftriangles;
	int* triIndexes = (int*)in.trianglelist;

	if (!triIndexes)
		return false;

	//we must first remove triangles out of the profile! ('Triangle' triangulates the convex hull)
	{
		//build corresponding polyline
		CCLib::SimpleCloud polyCloud;
		if (!polyCloud.reserve(count))
		{
			delete[] triIndexes;
			ccLog::Error("[ccPlane::buildUp] Not enough memory");
			return false;
		}
		else
		{
			for (unsigned i=0;i<count;++i)
				polyCloud.addPoint(CCVector3(m_profile[i].x,m_profile[i].y,0));
		}
		CCLib::Polyline poly(&polyCloud);
		poly.addPointIndex(0,count);
		poly.setClosingState(true);

		//test each triangle center
		const int* _triIndexes = triIndexes;
		unsigned lastValidIndex=0;
		for (unsigned i=0;i<numberOfTriangles;++i,_triIndexes+=3)
		{
			const CCVector2& A = m_profile[_triIndexes[0]];
			const CCVector2& B = m_profile[_triIndexes[1]];
			const CCVector2& C = m_profile[_triIndexes[2]];
			CCVector2 G = CCVector2((A.x+B.x+C.x)/3.0,
									(A.y+B.y+C.y)/3.0);

			//i fG is inside polygon
			if (CCLib::ManualSegmentationTools::isPointInsidePoly(G,&poly))
			{
				if (lastValidIndex<i)
					memcpy(triIndexes+3*lastValidIndex,triIndexes+3*i,3*sizeof(int));
				++lastValidIndex;
			}
		}

		numberOfTriangles = lastValidIndex;
	}

	if (numberOfTriangles==0)
	{
		delete[] triIndexes;
		return false;
	}

	//vertices
	unsigned vertCount = 2*count;
	//faces
	unsigned faceCount = 2*numberOfTriangles+2*count;
	//faces normals
	unsigned faceNormCount = 2+count;

	if (!init(vertCount,false,faceCount,faceNormCount))
	{
		delete[] triIndexes;
		ccLog::Error("[ccPlane::buildUp] Not enough memory");
		return false;
	}

	ccPointCloud* verts = vertices();
	assert(verts);
	assert(m_triNormals);

	//bottom & top faces normals
	m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0.0,0.0,-1.0).u));
	m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0.0,0.0,1.0).u));

	//add profile vertices & normals
	for (unsigned i=0;i<count;++i)
	{
		const CCVector2& P = m_profile[i];
		verts->addPoint(CCVector3(P.x,P.y,0));
		verts->addPoint(CCVector3(P.x,P.y,m_height));

		const CCVector2& PNext = m_profile[(i+1)%count];
		CCVector2 N(-(PNext.y-P.y),PNext.x-P.x);
		N.normalize();
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(N.x,N.y,0.0).u));
	}

	//add faces
	{
		//side faces
		{
			const int* _triIndexes = triIndexes;
			for (unsigned i=0;i<numberOfTriangles;++i,_triIndexes+=3)
			{
				addTriangle(_triIndexes[0]*2,_triIndexes[2]*2,_triIndexes[1]*2);
				addTriangleNormalIndexes(0,0,0);
				addTriangle(_triIndexes[0]*2+1,_triIndexes[1]*2+1,_triIndexes[2]*2+1);
				addTriangleNormalIndexes(1,1,1);
			}
		}

		//thickness
		{
			for (unsigned i=0;i<count;++i)
			{
				unsigned iNext = ((i+1)%count);
				addTriangle(i*2,i*2+1,iNext*2);
				addTriangleNormalIndexes(2+i,2+i,2+i);
				addTriangle(iNext*2,i*2+1,iNext*2+1);
				addTriangleNormalIndexes(2+i,2+i,2+i);
			}
		}
	}

	if (triIndexes)
		delete[] triIndexes;
	triIndexes=0;

	return true;
}

bool ccExtru::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	//parameters (dataVersion>=21)
	QDataStream outStream(&out);
	outStream << m_height;
	//profile size
	outStream << (qint32)m_profile.size();
	//profile points (2D)
	for (unsigned i=0;i<m_profile.size();++i)
	{
		outStream << m_profile[i].x;
		outStream << m_profile[i].y;
	}

	return true;
}

bool ccExtru::fromFile_MeOnly(QFile& in, short dataVersion)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion))
		return false;

	//parameters (dataVersion>=21)
	QDataStream inStream(&in);
	inStream >> m_height;
	//profile size
	qint32 vertCount;
	inStream >> vertCount;
	if (vertCount)
	{
		m_profile.resize(vertCount);
		//profile points (2D)
		for (unsigned i=0;i<m_profile.size();++i)
		{
			inStream >> m_profile[i].x;
			inStream >> m_profile[i].y;
		}
	}
	else
	{
		return false;
	}

	return true;
}
