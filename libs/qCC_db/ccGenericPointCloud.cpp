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
//$Rev:: 2225                                                              $
//$LastChangedDate:: 2012-07-25 23:26:33 +0200 (mer., 25 juil. 2012)       $
//**************************************************************************
//

#include "ccGenericPointCloud.h"

//CCLib
#include <Neighbourhood.h>
#include <DistanceComputationTools.h>

#include "ccOctree.h"
#include "ccSensor.h"
#include "ccPlane.h"

ccGenericPointCloud::ccGenericPointCloud(QString name)
	: ccHObject(name)
	, m_visibilityArray(0)
{
    setVisible(true);
    lockVisibility(false);
	setOriginalShift(0,0,0);
}

ccGenericPointCloud::~ccGenericPointCloud()
{
    clear();
}

void ccGenericPointCloud::clear()
{
	unallocateVisibilityArray();
    deleteOctree();
    enableTempColor(false);
}

bool ccGenericPointCloud::razVisibilityArray()
{
	if (!m_visibilityArray)
	{
		m_visibilityArray = new VisibilityTableType();
		m_visibilityArray->link();
	}

	if (!m_visibilityArray->resize(size()))
	{
		unallocateVisibilityArray();
        return false;
	}

	m_visibilityArray->fill(1); //by default, all points are visible

	return true;
}

void ccGenericPointCloud::unallocateVisibilityArray()
{
	if (m_visibilityArray)
		m_visibilityArray->release();
	m_visibilityArray=0;
}

CC_VISIBILITY_TYPE ccGenericPointCloud::testVisibility(const CCVector3& P)
{
    unsigned i=0,childNum=getChildrenNumber();

    CC_VISIBILITY_TYPE nvt, vt = ALL;

    while (i<childNum && vt==VIEWED)
    {
        if (m_children[i]->isKindOf(CC_SENSOR))
        {
            nvt = static_cast<ccSensor*>(m_children[i])->checkVisibility(P);
            vt = ccMin(vt,nvt);
        }
        ++i;
    }

	return (vt==ALL ? VIEWED : vt);
}

void ccGenericPointCloud::deleteOctree()
{
    ccOctree* oct = getOctree();
	if (oct)
        removeChild(oct);
}

ccOctree* ccGenericPointCloud::getOctree()
{
    for (unsigned i=0;i<m_children.size();++i)
    {
        if (m_children[i]->isA(CC_POINT_OCTREE))
            return static_cast<ccOctree*>(m_children[i]);
    }

    return NULL;
}

ccOctree* ccGenericPointCloud::computeOctree(CCLib::GenericProgressCallback* progressCb)
{
    deleteOctree();
    ccOctree* octree = new ccOctree(this);
    if (octree->build(progressCb)>0)
    {
        octree->setDisplay(getDisplay());
        addChild(octree);
    }
    else
    {
        delete octree;
        octree=NULL;
    }

    return octree;
}


ccGenericPointCloud::VisibilityTableType* ccGenericPointCloud::getTheVisibilityArray()
{
    return m_visibilityArray;
}

CCLib::ReferenceCloud* ccGenericPointCloud::getTheVisiblePoints()
{
    if (!m_visibilityArray || m_visibilityArray->currentSize()<size())
        return 0;

	unsigned i,count = size();
	assert(count == m_visibilityArray->currentSize());

    //we create an entity with the 'visible' vertices only
    CCLib::ReferenceCloud* rc = new CCLib::ReferenceCloud(this);

    for (i=0;i<count;++i)
        if (m_visibilityArray->getValue(i) > 0)
            rc->addPointIndex(i);

    return rc;
}

ccBBox ccGenericPointCloud::getMyOwnBB()
{
    ccBBox emptyBox;
	//specific case: empty cloud
	if (size())
	{
    getBoundingBox(emptyBox.minCorner().u, emptyBox.maxCorner().u);
    emptyBox.setValidity(true);
	}
    return emptyBox;
}

bool ccGenericPointCloud::isVisibilityTableInstantiated() const
{
    return m_visibilityArray && m_visibilityArray->isAllocated();
}

ccPlane* ccGenericPointCloud::fitPlane(double* rms /*= 0*/)
{
	//number of points
	unsigned count = size();
	if (count<3)
		return 0;

	CCLib::Neighbourhood Yk(this);

	//we determine plane normal by computing the smallest eigen value of M = 1/n * S[(p-µ)*(p-µ)']
	CCLib::SquareMatrixd eig = Yk.computeCovarianceMatrix().computeJacobianEigenValuesAndVectors();

	//invalid matrix?
	if (!eig.isValid())
	{
		//ccConsole::Warning(QString("[ccPointCloud::fitPlane] Failed to compute plane/normal for cloud '%1'").arg(getName()));
		return 0;
	}
	eig.sortEigenValuesAndVectors();

	//plane equation
	PointCoordinateType theLSQPlane[4];

	//the smallest eigen vector corresponds to the "least square best fitting plane" normal
	double vec[3];
	eig.getEigenValueAndVector(2,vec);
	//PointCoordinateType sign = (vec[2] < 0.0 ? -1.0 : 1.0);  //plane normal (always with a positive 'Z' by default)
	for (unsigned i=0;i<3;++i)
		theLSQPlane[i]=/*sign*/(PointCoordinateType)vec[i];
	CCVector3 N(theLSQPlane);

	//we also get centroid
	const CCVector3* G = Yk.getGravityCenter();
	assert(G);

	//eventually we just have to compute 'constant' coefficient a3
	//we use the fact that the plane pass through the centroid --> GM.N = 0 (scalar prod)
	//i.e. a0*G[0]+a1*G[1]+a2*G[2]=a3
	theLSQPlane[3] =  G->dot(N);

	//least-square fitting RMS
	if (rms)
	{
		placeIteratorAtBegining();
		*rms = 0.0;
		for (unsigned k=0;k<count;++k)
		{
			double d = (double)CCLib::DistanceComputationTools::computePoint2PlaneDistance(getNextPoint(),theLSQPlane);
			*rms += d*d;
		}
		*rms = sqrt(*rms)/(double)count;
	}

	//we has a plane primitive to the cloud
	eig.getEigenValueAndVector(0,vec); //main direction
	CCVector3 X(vec[0],vec[1],vec[2]); //plane normal
	//eig.getEigenValueAndVector(1,vec); //intermediate direction
	//CCVector3 Y(vec[0],vec[1],vec[2]); //plane normal
	CCVector3 Y = N * X;

	//we eventually check for plane extents
	PointCoordinateType minX=0.0,maxX=0.0,minY=0.0,maxY=0.0;
	placeIteratorAtBegining();
	for (unsigned k=0;k<count;++k)
	{
		//projetion into local 2D plane ref.
		CCVector3 P = *getNextPoint() - *G;
		PointCoordinateType x2D = P.dot(X);
		PointCoordinateType y2D = P.dot(Y);

		if (k!=0)
		{
			if (minX<x2D)
				minX=x2D;
			else if (maxX>x2D)
				maxX=x2D;
			if (minY<y2D)
				minY=y2D;
			else if (maxY>y2D)
				maxY=y2D;
		}
		else
		{
			minX=maxX=x2D;
			minY=maxY=y2D;
		}
	}

	//we recenter plane (as it is not always the case!)
	float dX = maxX-minX;
	float dY = maxY-minY;
	CCVector3 Gt = *G + X * (minX+dX*0.5);
	Gt += Y * (minY+dY*0.5);
	ccGLMatrix glMat(X,Y,N,Gt);

	return new ccPlane(dX,dY,&glMat);
}

void ccGenericPointCloud::setOriginalShift(double x, double y, double z)
{
	m_originalShift[0]=x;
	m_originalShift[1]=y;
	m_originalShift[2]=z;
}

bool ccGenericPointCloud::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//'coordinates shift' (dataVersion>=20)
	if (out.write((const char*)m_originalShift,sizeof(double)*3)<0)
		return WriteError();

	//'visibility' array (dataVersion>=20)
	bool hasVisibilityArray = isVisibilityTableInstantiated();
	if (out.write((const char*)&hasVisibilityArray,sizeof(bool))<0)
		return WriteError();
	if (hasVisibilityArray)
	{
		assert(m_visibilityArray);
		if (!ccSerializationHelper::GenericArrayToFile(*m_visibilityArray,out))
			return false;
	}

	return true;
}

bool ccGenericPointCloud::fromFile_MeOnly(QFile& in, short dataVersion)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion))
		return false;

	if (dataVersion<20)
		return CorruptError();

	//'coordinates shift' (dataVersion>=20)
	if (in.read((char*)m_originalShift,sizeof(double)*3)<0)
		return ReadError();

	//'visibility' array (dataVersion>=20)
	bool hasVisibilityArray = false;
	if (in.read((char*)&hasVisibilityArray,sizeof(bool))<0)
		return ReadError();
	if (hasVisibilityArray)
	{
		if (!m_visibilityArray)
		{
			m_visibilityArray = new VisibilityTableType();
			m_visibilityArray->link();
		}
		if (!ccSerializationHelper::GenericArrayFromFile(*m_visibilityArray,in,dataVersion))
		{
			unallocateVisibilityArray();
			return false;
		}
	}

	return true;
}

