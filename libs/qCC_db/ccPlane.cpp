//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//Always on top!
#include "ccIncludeGL.h"

#include "ccPlane.h"

//qCC_db
#include "ccPointCloud.h"
#include "ccMaterialSet.h"

//CCLIB
#include "DistanceComputationTools.h"

ccPlane::ccPlane(PointCoordinateType xWidth, PointCoordinateType yWidth, const ccGLMatrix* transMat/*=0*/, QString name/*=QString("Plane")*/)
	: ccGenericPrimitive(name, transMat)
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
	if (!init(4, false, 2, 1))
	{
		ccLog::Error("[ccPlane::buildUp] Not enough memory");
		return false;
	}

	ccPointCloud* verts = vertices();
	assert(verts);
	assert(m_triNormals);

	// B ------ C
	// |        |
	// A ------ D
	verts->addPoint(CCVector3(-m_xWidth / 2, -m_yWidth / 2, 0));
	verts->addPoint(CCVector3(-m_xWidth / 2,  m_yWidth / 2, 0));
	verts->addPoint(CCVector3( m_xWidth / 2,  m_yWidth / 2, 0));
	verts->addPoint(CCVector3( m_xWidth / 2, -m_yWidth / 2, 0));

	m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0, 0, 1)));

	addTriangle(0, 2, 1); //A C B
	addTriangleNormalIndexes(0, 0, 0);
	addTriangle(0, 3, 2); //A D C
	addTriangleNormalIndexes(0, 0, 0);

	return true;
}

ccGenericPrimitive* ccPlane::clone() const
{
	return finishCloneJob(new ccPlane(m_xWidth, m_yWidth, &m_transformation, getName()));
}

void ccPlane::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	//call parent method
	ccGenericPrimitive::drawMeOnly(context);

	//show normal vector
	if (MACRO_Draw3D(context) && normalVectorIsShown())
	{
		PointCoordinateType scale = sqrt(m_xWidth * m_yWidth) / 2; //DGM: highly empirical ;)
		glDrawNormal(context, m_transformation.getTranslationAsVec3D(), scale);
	}
}

void ccPlane::getEquation(CCVector3& N, PointCoordinateType& constVal) const
{
	N = CCVector3(0, 0, 1);
	m_transformation.applyRotation(N);

	constVal = m_transformation.getTranslationAsVec3D().dot(N);
}

const PointCoordinateType* ccPlane::getEquation()
{
	CCVector3 N = getNormal();
	m_PlaneEquation[0] = N.x;
	m_PlaneEquation[1] = N.y;
	m_PlaneEquation[2] = N.z;
	m_PlaneEquation[3] = getCenter().dot(N); //a point on the plane dot the plane normal
	return m_PlaneEquation;
}

ccPlane* ccPlane::Fit(CCLib::GenericIndexedCloudPersist *cloud, double* rms/*=0*/)
{
	//number of points
	unsigned count = cloud->size();
	if (count < 3)
	{
		ccLog::Warning("[ccPlane::Fit] Not enough points in input cloud to fit a plane!");
		return nullptr;
	}

	CCLib::Neighbourhood Yk(cloud);

	//plane equation
	const PointCoordinateType* theLSPlane = Yk.getLSPlane();
	if (!theLSPlane)
	{
		ccLog::Warning("[ccPlane::Fit] Not enough points to fit a plane!");
		return nullptr;
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
	CCVector2 minXY(0, 0);
	CCVector2 maxXY(0, 0);
	cloud->placeIteratorAtBeginning();
	for (unsigned k = 0; k < count; ++k)
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
	PointCoordinateType dX = maxXY.x - minXY.x;
	PointCoordinateType dY = maxXY.y - minXY.y;
	CCVector3 Gt = *G + *X * (minXY.x + dX / 2) + Y * (minXY.y + dY / 2);
	ccGLMatrix glMat(*X, Y, N, Gt);

	ccPlane* plane = new ccPlane(dX, dY, &glMat);

	//compute least-square fitting RMS if requested
	if (rms)
	{
		*rms = CCLib::DistanceComputationTools::computeCloud2PlaneDistanceRMS(cloud, theLSPlane);
		plane->setMetaData(QString("RMS"), QVariant(*rms));
	}

	return plane;
}

bool ccPlane::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	//parameters (dataVersion >= 21)
	QDataStream outStream(&out);
	outStream << m_xWidth;
	outStream << m_yWidth;

	return true;
}

bool ccPlane::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//parameters (dataVersion>=21)
	QDataStream inStream(&in);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_xWidth, 1);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_yWidth, 1);

	return true;
}

ccBBox ccPlane::getOwnFitBB(ccGLMatrix& trans)
{
	trans = m_transformation;
	return ccBBox(CCVector3(-m_xWidth / 2, -m_yWidth / 2, 0), CCVector3(m_xWidth / 2, m_yWidth / 2, 0));
}

ccMaterial::Shared ccPlane::setAsTexture(QImage image, QString imageFilename/*=QString()*/)
{
	return SetQuadTexture(this, image, imageFilename);
}

ccMaterial::Shared ccPlane::SetQuadTexture(ccMesh* quadMesh, QImage image, QString imageFilename/*=QString()*/)
{
	if (	!quadMesh
		||	quadMesh->size() > 2 //they may not be reserved yet?
		||	!quadMesh->getAssociatedCloud()
		||	quadMesh->getAssociatedCloud()->size() > 4) //they may not be reserved yet?
	{
		ccLog::Warning("[ccPlane::SetQuadTexture] Invalid input quad");
	}
	if (image.isNull())
	{
		ccLog::Warning("[ccPlane::SetQuadTexture] Invalid texture image!");
		return ccMaterial::Shared(nullptr);
	}

	//texture coordinates
	TextureCoordsContainer* texCoords = quadMesh->getTexCoordinatesTable();
	if (!texCoords)
	{
		texCoords = new TextureCoordsContainer();
		if (!texCoords->reserveSafe(4))
		{
			//not enough memory
			ccLog::Warning("[ccPlane::setAsTexture] Not enough memory!");
			delete texCoords;
			return ccMaterial::Shared(nullptr);
		}

		//create default texture coordinates
		TexCoords2D TA (0.0f, 0.0f);
		TexCoords2D TB (0.0f, 1.0f);
		TexCoords2D TC (1.0f, 1.0f);
		TexCoords2D TD (1.0f, 0.0f);
		texCoords->emplace_back(TA);
		texCoords->emplace_back(TB);
		texCoords->emplace_back(TC);
		texCoords->emplace_back(TD);

		quadMesh->setTexCoordinatesTable(texCoords);
	}

	if (!quadMesh->hasPerTriangleTexCoordIndexes())
	{
		if (!quadMesh->reservePerTriangleTexCoordIndexes())
		{
			//not enough memory
			ccLog::Warning("[ccPlane::setAsTexture] Not enough memory!");
			quadMesh->setTexCoordinatesTable(nullptr);
			quadMesh->removePerTriangleMtlIndexes();
			return ccMaterial::Shared(nullptr);
		}
		
		//set default texture indexes
		quadMesh->addTriangleTexCoordIndexes(0, 2, 1);
		quadMesh->addTriangleTexCoordIndexes(0, 3, 2);
	}
	
	if (!quadMesh->hasPerTriangleMtlIndexes())
	{
		if (!quadMesh->reservePerTriangleMtlIndexes())
		{
			//not enough memory
			ccLog::Warning("[ccPlane::setAsTexture] Not enough memory!");
			quadMesh->setTexCoordinatesTable(nullptr);
			quadMesh->removePerTriangleTexCoordIndexes();
			return ccMaterial::Shared(nullptr);
		}

		//set default material indexes
		quadMesh->addTriangleMtlIndex(0);
		quadMesh->addTriangleMtlIndex(0);
	}

	//set material
	if (!quadMesh->getMaterialSet())
	{
		quadMesh->setMaterialSet(new ccMaterialSet());
	}
	ccMaterialSet* materialSet = const_cast<ccMaterialSet*>(quadMesh->getMaterialSet());
	assert(materialSet);
	//remove old materials (if any)
	materialSet->clear();
	//add new material
	ccMaterial::Shared material(new ccMaterial("texture"));
	material->setTexture(image, imageFilename, false);
	materialSet->addMaterial(material);

	quadMesh->showMaterials(true);

	return material;
}

void ccPlane::flip()
{
	ccGLMatrix reverseMat;
	reverseMat.initFromParameters(static_cast<PointCoordinateType>(M_PI), CCVector3(1, 0, 0), CCVector3(0, 0, 0));

	m_transformation = m_transformation * reverseMat;
	updateRepresentation();
}