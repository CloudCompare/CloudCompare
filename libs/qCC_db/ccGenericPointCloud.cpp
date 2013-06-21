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

#include "ccGenericPointCloud.h"

//CCLib
#include <Neighbourhood.h>
#include <DistanceComputationTools.h>

#include "ccOctree.h"
#include "ccSensor.h"
#include "ccPlane.h"

ccGenericPointCloud::ccGenericPointCloud(QString name)
	: ccHObject(name)
	, m_pointsVisibility(0)
	, m_pointSize(0)
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
	if (!m_pointsVisibility)
	{
		m_pointsVisibility = new VisibilityTableType();
		m_pointsVisibility->link();
	}

	if (!m_pointsVisibility->resize(size()))
	{
		unallocateVisibilityArray();
        return false;
	}

	m_pointsVisibility->fill(POINT_VISIBLE); //by default, all points are visible

	return true;
}

void ccGenericPointCloud::unallocateVisibilityArray()
{
	if (m_pointsVisibility)
		m_pointsVisibility->release();
	m_pointsVisibility=0;
}

bool ccGenericPointCloud::isVisibilityTableInstantiated() const
{
    return m_pointsVisibility && m_pointsVisibility->isAllocated();
}

uchar ccGenericPointCloud::testVisibility(const CCVector3& P)
{
    uchar bestVisibility = 255; //impossible value

	for (ccHObject::Container::iterator it = m_children.begin(); it != m_children.end(); ++it)
	{
        if ((*it)->isKindOf(CC_SENSOR))
        {
            uchar visibility = static_cast<ccSensor*>(*it)->checkVisibility(P);

			if (visibility == POINT_VISIBLE)
				return POINT_VISIBLE; //shortcut
            
			bestVisibility = std::min<uchar>(visibility,bestVisibility);
        }
    }

	return (bestVisibility == 255 ? POINT_VISIBLE : bestVisibility);
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
    return m_pointsVisibility;
}

CCLib::ReferenceCloud* ccGenericPointCloud::getTheVisiblePoints() const
{
	unsigned count = size();
	assert(count == m_pointsVisibility->currentSize());

    if (!m_pointsVisibility || m_pointsVisibility->currentSize() != count)
        return 0;

	//count the number of points to copy
	unsigned pointCount = 0;
	{
		for (unsigned i=0; i<count; ++i)
			if (m_pointsVisibility->getValue(i) == POINT_VISIBLE)
				++pointCount;
	}

	if (pointCount == 0)
	{
		ccLog::Error("[ccGenericPointCloud::getTheVisiblePoints] No point in selection!");
		return 0;
	}

    //we create an entity with the 'visible' vertices only
    CCLib::ReferenceCloud* rc = new CCLib::ReferenceCloud(const_cast<ccGenericPointCloud*>(this));
	if (rc->reserve(pointCount))
	{
		for (unsigned i=0; i<count; ++i)
			if (m_pointsVisibility->getValue(i) == POINT_VISIBLE)
				rc->addPointIndex(i); //can't fail (see above)
	}
	else
	{
		delete rc;
		rc=0;
		ccLog::Error("[ccGenericPointCloud::getTheVisiblePoints] Not enough memory!");
	}

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

ccPlane* ccGenericPointCloud::fitPlane(double* rms/*= 0*/)
{
	//number of points
	unsigned count = size();
	if (count < 3)
	{
		ccLog::Warning("[ccGenericPointCloud::fitPlane] Not enough points to fit a plane!");
		return 0;
	}

	CCLib::Neighbourhood Yk(this);

	//we determine plane normal by computing the smallest eigen value of M = 1/n * S[(p-µ)*(p-µ)']
	CCLib::SquareMatrixd eig = Yk.computeCovarianceMatrix().computeJacobianEigenValuesAndVectors();

	//invalid matrix?
	if (!eig.isValid())
	{
		//ccConsole::Warning(QString("[ccPointCloud::fitPlane] Failed to compute plane/normal for cloud '%1'").arg(getName()));
		return 0;
	}
	eig.sortEigenValuesAndVectors(); //from the biggest to the smallest eigen value

	//plane equation
	PointCoordinateType theLSQPlane[4];

	//the smallest eigen vector corresponds to the "least square best fitting plane" normal
	double vec[3];
	eig.getEigenValueAndVector(2,vec);
	//PointCoordinateType sign = (vec[2] < 0.0 ? -1.0 : 1.0);  //plane normal (always with a positive 'Z' by default)
	for (unsigned i=0;i<3;++i)
		theLSQPlane[i]=/*sign*/(PointCoordinateType)vec[i];
	CCVector3 N(theLSQPlane);

	//we also get the centroid
	const CCVector3* G = Yk.getGravityCenter();
	assert(G);

	//eventually we just have to compute 'constant' coefficient a3
	//we use the fact that the plane pass through the centroid --> GM.N = 0 (scalar prod)
	//i.e. a0*G[0]+a1*G[1]+a2*G[2]=a3
	theLSQPlane[3] =  G->dot(N);

	//least-square fitting RMS
	if (rms)
	{
		*rms = CCLib::DistanceComputationTools::computeCloud2PlaneDistanceRMS(this, theLSQPlane);
	}

	//we add a plane primitive to the cloud
	eig.getEigenValueAndVector(0,vec); //main direction
	CCVector3 X(vec[0],vec[1],vec[2]);
	CCVector3 Y = N * X;

	//we eventually check for plane extents
	PointCoordinateType minX=0,maxX=0,minY=0,maxY=0;
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
		assert(m_pointsVisibility);
		if (!ccSerializationHelper::GenericArrayToFile(*m_pointsVisibility,out))
			return false;
	}

	//'point size' (dataVersion>=24)
	if (out.write((const char*)&m_pointSize,1)<0)
		return WriteError();

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
		if (!m_pointsVisibility)
		{
			m_pointsVisibility = new VisibilityTableType();
			m_pointsVisibility->link();
		}
		if (!ccSerializationHelper::GenericArrayFromFile(*m_pointsVisibility,in,dataVersion))
		{
			unallocateVisibilityArray();
			return false;
		}
	}

	//'point size' (dataVersion>=24)
	if (dataVersion >= 24)
	{
		if (in.read((char*)&m_pointSize,1)<0)
			return WriteError();
	}
	else
	{
		m_pointSize = 0; //= follows default setting
	}

	return true;
}

