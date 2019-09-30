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
	if (isVisible()) {
		//call parent method
		ccGenericPrimitive::drawMeOnly(context);

		//show normal vector
		if (MACRO_Draw3D(context) && (normalVectorIsShown() || getNormalEditState()))
		{
			PointCoordinateType scale = sqrt(m_xWidth * m_yWidth) / 2; //DGM: highly empirical ;)
			glDrawNormal(context, getUniqueIDForDisplay(), m_transformation.getTranslationAsVec3D(), scale, nullptr);
		}
	}	
	
	// draw selected profile
	if (((getParent() && getParent()->isSelected()) || isSelected()) 
		&& MACRO_Draw3D(context) && !MACRO_DRAW_BBOX(context) && m_profile.size() > 2) {
		QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
		assert(glFunc != nullptr);

		if (glFunc == nullptr)
			return;

		ccGL::Color3v(glFunc, ccColor::red.rgb);
		glFunc->glPushAttrib(GL_LINE_BIT);
		glFunc->glLineWidth(static_cast<GLfloat>(2));

		glFunc->glBegin(GL_LINE_STRIP);
		for (unsigned i = 0; i < m_profile.size(); ++i)	{
			ccGL::Vertex3v(glFunc, m_profile[i].u);
		}
		ccGL::Vertex3v(glFunc, m_profile[0].u);
		glFunc->glEnd();
		glFunc->glPopAttrib();//GL_LINE_BIT
	}
}

void ccPlane::getEquation(CCVector3& N, PointCoordinateType& constVal) const
{
	N = CCVector3(0, 0, 1);
	m_transformation.applyRotation(N);

	constVal = m_transformation.getTranslationAsVec3D().dot(N);
}

bool ccPlane::isVerticalToDirection(CCVector3 dir, double angle_degree)
{
	double err_angle = angle_degree * CC_DEG_TO_RAD;
	CCVector3 N; PointCoordinateType d;
	getEquation(N, d);
	double product = N.dot(dir);
	if ((product > 0 && product < err_angle) || (product < 0 && product > -err_angle)) {
		return true;
	}
	return false;
}

void ccPlane::setProfile(std::vector<CCVector3> profile, bool update)
{
	m_profile = profile;
	
	if (update) {
		ccPlane* new_plane = ccPlane::Fit(profile);
		if (new_plane) {
			m_transformation = new_plane->getTransformation();
			setXWidth(new_plane->getXWidth(), false);
			setYWidth(new_plane->getYWidth(), true);
			delete new_plane;
			new_plane = nullptr;
		}
	}
}

CCVector3 ccPlane::getProfileCenter()
{
	CCVector3 center(0, 0, 0);
	for (auto & pt : m_profile)	{
		center += pt;
	}
	center /= m_profile.size();
	return center;
}

ccPlane* ccPlane::Fit(CCLib::GenericIndexedCloudPersist *cloud, double* rms/*=0*/, std::vector<CCVector3> * profile /*= 0*/, const PointCoordinateType* planeEquation /*= 0*/)
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
	PointCoordinateType* theLSPlane = const_cast<PointCoordinateType*>(Yk.getLSPlane());
	if (!theLSPlane)
	{
		ccLog::Warning("[ccPlane::Fit] Not enough points to fit a plane!");
		return 0;
	}
	if (planeEquation) {
		if (CCVector3(theLSPlane).dot(CCVector3(planeEquation)) < 0) {
			theLSPlane[0] = -theLSPlane[0];
			theLSPlane[1] = -theLSPlane[1];
			theLSPlane[2] = -theLSPlane[2];
			theLSPlane[3] = -theLSPlane[3];
			const CCVector3* X = Yk.getLSPlaneX();
			const CCVector3* Y = Yk.getLSPlaneY();
			const CCVector3* N = Yk.getLSPlaneNormal();
			Yk.setLSPlane(theLSPlane, -(*X), -(*Y), -(*N));
		}
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
	CCVector2 minXY(0, 0), maxXY(0, 0);
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
	if (profile) {
		//we project the input points on a plane
		std::vector<CCLib::PointProjectionTools::IndexedCCVector2> points2D;
		if (Yk.projectIndexedPointsOn2DPlane(points2D, theLSPlane))	{
			std::list<CCLib::PointProjectionTools::IndexedCCVector2*> hullPoints;
			CCLib::PointProjectionTools::extractConvexHull2D(points2D, hullPoints);
			for (auto pt : hullPoints) {
				(*profile).push_back(*(cloud->getPoint((*pt).index)));
			}
			plane->setProfile(*profile);
		}
	}

	return plane;
}

ccPlane * ccPlane::Fit(const std::vector<CCVector3> profiles, const PointCoordinateType* planeEquation /*= 0*/)
{
	if (profiles.size() < 3) {
		return nullptr;
	}
	ccPointCloud* cloud = new ccPointCloud();
	for (auto & pt : profiles) {
		cloud->addPoint(pt);
	}
	ccPlane* plane = ccPlane::Fit(cloud, nullptr, nullptr, planeEquation);
	plane->setProfile(profiles);

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

	//m_profile size
	outStream << (qint32)m_profile.size();
	//m_profile points (3D)
	for (unsigned i = 0; i < m_profile.size(); ++i)
	{
		outStream << m_profile[i].x;
		outStream << m_profile[i].y;
		outStream << m_profile[i].z;
	}

	return true;
}

bool ccPlane::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//parameters (dataVersion>=21)
	QDataStream inStream(&in);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_xWidth, 1);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_yWidth, 1);

	//m_bottom size
	qint32 vertCount;
	inStream >> vertCount;
	if (vertCount > 0) {
		m_profile.resize(vertCount);
		//m_bottom points (2D)
		for (unsigned i = 0; i < m_profile.size(); ++i) {
			ccSerializationHelper::CoordsFromDataStream(inStream, flags, m_profile[i].u, 3);
		}
	}

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
			quadMesh->setTexCoordinatesTable(0);
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
			quadMesh->setTexCoordinatesTable(0);
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

void ccPlane::notifyPlanarEntityChanged(ccGLMatrix mat)
{
	applyGLTransformation_recursive(&mat);
}