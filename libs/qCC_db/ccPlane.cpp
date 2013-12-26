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

#include <ccIncludeGL.h>

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
	buildUp();
	applyTransformationToVertices();
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

	verts->addPoint(CCVector3(-m_xWidth*0.5f,-m_yWidth*0.5f, 0));
	verts->addPoint(CCVector3(-m_xWidth*0.5f, m_yWidth*0.5f, 0));
	verts->addPoint(CCVector3( m_xWidth*0.5f, m_yWidth*0.5f, 0));
	verts->addPoint(CCVector3( m_xWidth*0.5f,-m_yWidth*0.5f, 0));

	m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0.0,0.0,1.0).u));

	addTriangle(0,2,1);
	addTriangleNormalIndexes(0,0,0);
	addTriangle(0,3,2);
	addTriangleNormalIndexes(0,0,0);

	return true;
}

ccPlane::ccPlane(QString name /*=QString("Plane")*/)
	: ccGenericPrimitive(name)
	, m_xWidth(0)
	, m_yWidth(0)
{
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
        ccLog::Warning("[ccPlane::fitTo] Not enough points in input cloud to fit a plane!");
        return 0;
    }

    CCLib::Neighbourhood Yk(cloud);

    //plane equation
    const PointCoordinateType* theLSQPlane = Yk.getLSQPlane();
    if (!theLSQPlane)
    {
        ccLog::Warning("[ccGenericPointCloud::fitPlane] Not enough points to fit a plane!");
        return 0;
    }

    //compute least-square fitting RMS if requested
    if (rms)
    {
        *rms = CCLib::DistanceComputationTools::computeCloud2PlaneDistanceRMS(cloud, theLSQPlane);
    }

    //get the centroid
    const CCVector3* G = Yk.getGravityCenter();
    assert(G);

    //and a local base
    CCVector3 N(theLSQPlane);
    const CCVector3* X = Yk.getLSQPlaneX(); //main direction
    assert(X);
    CCVector3 Y = N * (*X);

    PointCoordinateType minX=0,maxX=0,minY=0,maxY=0;
    cloud->placeIteratorAtBegining();
    for (unsigned k=0; k<count; ++k)
    {
        //projetion into local 2D plane ref.
        CCVector3 P = *(cloud->getNextPoint()) - *G;
        
		PointCoordinateType x2D = P.dot(*X);
        PointCoordinateType y2D = P.dot(Y);

        if (k!=0)
        {
            if (minX < x2D)
                minX = x2D;
            else if (maxX > x2D)
                maxX = x2D;
            if (minY < y2D)
                minY = y2D;
            else if (maxY > y2D)
                maxY = y2D;
        }
        else
        {
            minX = maxX = x2D;
            minY = maxY = y2D;
        }
    }

    //we recenter the plane
    PointCoordinateType dX = maxX-minX;
    PointCoordinateType dY = maxY-minY;
    CCVector3 Gt = *G + *X * (minX + dX*static_cast<PointCoordinateType>(0.5));
    Gt += Y * (minY + dY*static_cast<PointCoordinateType>(0.5));
    ccGLMatrix glMat(*X,Y,N,Gt);

    ccPlane* plane = new ccPlane(dX, dY, &glMat);

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
	inStream >> m_xWidth;
	inStream >> m_yWidth;

	return true;
}

ccBBox ccPlane::getFitBB(ccGLMatrix& trans)
{
	trans = m_transformation;
	return ccBBox(CCVector3(-m_xWidth*0.5f,-m_yWidth*0.5f, 0),CCVector3(m_xWidth*0.5f,m_yWidth*0.5f, 0));
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
		float TA[2]={0.0f,0.0f};
		float TB[2]={0.0f,1.0f};
		float TC[2]={1.0f,1.0f};
		float TD[2]={1.0f,0.0f};
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
		ccMaterial material("texture");
		material.texture = image;
		materialSet->addMaterial(material);
		//dirty trick: reset material association so that texture will be refreshed!
		materialSet->associateTo(0);
		if (m_currentDisplay)
			materialSet->associateTo(m_currentDisplay);
	}

	showMaterials(true);

	return true;
}
