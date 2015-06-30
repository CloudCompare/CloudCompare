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

//Always on top!
#include "ccIncludeGL.h"

#include "ccPlane.h"

//qCC_db
#include "ccPointCloud.h"
#include "ccNormalVectors.h"
#include "ccMaterialSet.h"

//CCLIB
#include "DistanceComputationTools.h"

ccPlane::ccPlane(PointCoordinateType xWidth, PointCoordinateType yWidth, const ccGLMatrix* transMat/*=0*/, QString name/*=QString("Plane")*/)
	: ccGenericPrimitive(name,transMat)
	, m_xWidth(xWidth)
	, m_yWidth(yWidth)
{
	updateRepresentation();
}

ccPlane::ccPlane(QString name /*=QString("Plane")*/)
	: ccGenericPrimitive(name)
	, m_xWidth(0)
	, m_yWidth(0)
{
}

bool ccPlane::buildUp()
{
	if (!init(4,false,2,1))
	{
		ccLog::Error("[ccPlane::buildUp] Not enough memory");
		return false;
	}

	ccPointCloud* verts = vertices();
	assert(verts);
	assert(m_triNormals);

	verts->addPoint(CCVector3(-m_xWidth/2,-m_yWidth/2, 0));
	verts->addPoint(CCVector3(-m_xWidth/2, m_yWidth/2, 0));
	verts->addPoint(CCVector3( m_xWidth/2, m_yWidth/2, 0));
	verts->addPoint(CCVector3( m_xWidth/2,-m_yWidth/2, 0));

	m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0,0,1).u));

	addTriangle(0,2,1);
	addTriangleNormalIndexes(0,0,0);
	addTriangle(0,3,2);
	addTriangleNormalIndexes(0,0,0);

	return true;
}

ccGenericPrimitive* ccPlane::clone() const
{
	return finishCloneJob(new ccPlane(m_xWidth,m_yWidth,&m_transformation,getName()));
}

ccPlane* ccPlane::Fit(CCLib::GenericIndexedCloudPersist *cloud, double* rms/*=0*/)
{
	//number of points
	unsigned count = cloud->size();
	if (count < 3)
	{
		ccLog::Warning("[ccPlane::Fit] Not enough points in input cloud to fit a plane!");
		return 0;
	}

	CCLib::Neighbourhood Yk(cloud);

	//plane equation
	const PointCoordinateType* theLSPlane = Yk.getLSPlane();
	if (!theLSPlane)
	{
		ccLog::Warning("[ccPlane::Fit] Not enough points to fit a plane!");
		return 0;
	}

	//get the centroid
	const CCVector3* G = Yk.getGravityCenter();
	assert(G);

	//and a local base
	CCVector3 N(theLSPlane);
	const CCVector3* X = Yk.getLSPlaneX(); //main direction
	assert(X);
	CCVector3 Y = N * (*X);

	//compute bounding box in 2D plane
	CCVector2 minXY(0,0), maxXY(0,0);
	cloud->placeIteratorAtBegining();
	for (unsigned k=0; k<count; ++k)
	{
		//projection into local 2D plane ref.
		CCVector3 P = *(cloud->getNextPoint()) - *G;

		CCVector2 P2D( P.dot(*X), P.dot(Y) );

		if (k != 0)
		{
			if (minXY.x > P2D.x)
				minXY.x = P2D.x;
			else if (maxXY.x < P2D.x)
				maxXY.x = P2D.x;
			if (minXY.y > P2D.y)
				minXY.y = P2D.y;
			else if (maxXY.y < P2D.y)
				maxXY.y = P2D.y;
		}
		else
		{
			minXY = maxXY = P2D;
		}
	}

	//we recenter the plane
	PointCoordinateType dX = maxXY.x-minXY.x;
	PointCoordinateType dY = maxXY.y-minXY.y;
	CCVector3 Gt = *G + *X * (minXY.x + dX / 2) + Y * (minXY.y + dY / 2);
	ccGLMatrix glMat(*X,Y,N,Gt);

	ccPlane* plane = new ccPlane(dX, dY, &glMat);

	//compute least-square fitting RMS if requested
	if (rms)
	{
		*rms = CCLib::DistanceComputationTools::computeCloud2PlaneDistanceRMS(cloud, theLSPlane);
		plane->setMetaData(QString("RMS"),QVariant(*rms));
	}


	return plane;
}

bool ccPlane::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	//parameters (dataVersion>=21)
	QDataStream outStream(&out);
	outStream << m_xWidth;
	outStream << m_yWidth;

	return true;
}

bool ccPlane::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//parameters (dataVersion>=21)
	QDataStream inStream(&in);
	ccSerializationHelper::CoordsFromDataStream(inStream,flags,&m_xWidth,1);
	ccSerializationHelper::CoordsFromDataStream(inStream,flags,&m_yWidth,1);

	return true;
}

ccBBox ccPlane::getFitBB(ccGLMatrix& trans)
{
	trans = m_transformation;
	return ccBBox( CCVector3(-m_xWidth/2,-m_yWidth/2, 0), CCVector3(m_xWidth/2,m_yWidth/2, 0) );
}

bool ccPlane::setAsTexture(QImage image)
{
	if (image.isNull())
	{
		ccLog::Warning("[ccPlane::setAsTexture] Invalid texture image!");
		return false;
	}

	//texture coordinates
	TextureCoordsContainer* texCoords = getTexCoordinatesTable();
	if (!texCoords)
	{
		texCoords = new TextureCoordsContainer();
		if (!texCoords->reserve(4))
		{
			//not enough memory
			ccLog::Warning("[ccPlane::setAsTexture] Not enough memory!");
			delete texCoords;
			return false;
		}

		//create default texture coordinates
		float TA[2] = { 0.0f, 0.0f };
		float TB[2] = { 0.0f, 1.0f };
		float TC[2] = { 1.0f, 1.0f };
		float TD[2] = { 1.0f, 0.0f };
		texCoords->addElement(TA);
		texCoords->addElement(TB);
		texCoords->addElement(TC);
		texCoords->addElement(TD);

		setTexCoordinatesTable(texCoords);
	}

	if (!hasPerTriangleTexCoordIndexes())
	{
		if (!reservePerTriangleTexCoordIndexes())
		{
			//not enough memory
			ccLog::Warning("[ccPlane::setAsTexture] Not enough memory!");
			setTexCoordinatesTable(0);
			removePerTriangleMtlIndexes();
			return false;
		}
		
		//set default texture indexes
		addTriangleTexCoordIndexes(0,2,1);
		addTriangleTexCoordIndexes(0,3,2);
	}
	
	if (!hasPerTriangleMtlIndexes())
	{
		if (!reservePerTriangleMtlIndexes())
		{
			//not enough memory
			ccLog::Warning("[ccPlane::setAsTexture] Not enough memory!");
			setTexCoordinatesTable(0);
			removePerTriangleTexCoordIndexes();
			return false;
		}

		//set default material indexes
		addTriangleMtlIndex(0);
		addTriangleMtlIndex(0);
	}

	//set material
	if (!getMaterialSet())
		setMaterialSet(new ccMaterialSet());
	ccMaterialSet* materialSet = const_cast<ccMaterialSet*>(getMaterialSet());
	assert(materialSet);
	//remove old material (if any)
	materialSet->clear();
	//add new material
	{
		ccMaterial::Shared material(new ccMaterial("texture"));
		material->setTexture(image,QString(),false);
		materialSet->addMaterial(material);
	}

	showMaterials(true);

	return true;
}
