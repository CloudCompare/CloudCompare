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

//Always first
#include "ccIncludeGL.h"

#include "ccOctree.h"

//Local
#include "ccCameraSensor.h"
#include "ccNormalVectors.h"
#include "ccScalarField.h"
#include "ccPointCloud.h"
#include "ccBox.h"
#include "ccProgressDialog.h"

//CCCoreLib
#include <Neighbourhood.h>
#include <RayAndBox.h>
#include <ScalarFieldTools.h>

#ifdef QT_DEBUG
//#define DEBUG_PICKING_MECHANISM
#endif

//System
#include <random>

ccOctree::ccOctree(ccGenericPointCloud* aCloud)
	: CCCoreLib::DgmOctree(aCloud)
	, m_theAssociatedCloudAsGPC(aCloud)
	, m_displayedLevel(1)
	, m_displayMode(WIRE)
	, m_glListID(0)
	, m_glListIsDeprecated(true)
	, m_frustumIntersector(nullptr)
{
}

ccOctree::~ccOctree()
{
	if (m_frustumIntersector)
	{
		delete m_frustumIntersector;
		m_frustumIntersector = nullptr;
	}
}

void ccOctree::setDisplayedLevel(int level)
{
	if (level != m_displayedLevel)
	{
		m_displayedLevel = level;
		m_glListIsDeprecated = true;
	}
}

void ccOctree::setDisplayMode(DisplayMode mode)
{
	if (m_displayMode != mode)
	{
		m_displayMode = mode;
		m_glListIsDeprecated = true;
	}
}

void ccOctree::clear()
{
	//warn the others that the octree organization is going to change
	Q_EMIT updated();

	QOpenGLContext* context = QOpenGLContext::currentContext();
	if (context)
	{
		//get the set of OpenGL functions (version 2.1)
		QOpenGLFunctions_2_1* glFunc = context->versionFunctions<QOpenGLFunctions_2_1>();
		assert(glFunc != nullptr);

		if (glFunc && glFunc->glIsList(m_glListID))
		{
			glFunc->glDeleteLists(m_glListID, 1);
		}
	}

	m_glListID = 0;
	m_glListIsDeprecated = true;

	DgmOctree::clear();
}

ccBBox ccOctree::getSquareBB() const
{
	return ccBBox(m_dimMin, m_dimMax, true);
}

ccBBox ccOctree::getPointsBB() const
{
	return ccBBox(m_pointsMin, m_pointsMax, m_numberOfProjectedPoints != 0);
}

void ccOctree::multiplyBoundingBox(const PointCoordinateType multFactor)
{
	m_dimMin *= multFactor;
	m_dimMax *= multFactor;
	m_pointsMin *= multFactor;
	m_pointsMax *= multFactor;

	for (int i = 0; i <= MAX_OCTREE_LEVEL; ++i)
		m_cellSize[i] *= multFactor;
}

void ccOctree::translateBoundingBox(const CCVector3& T)
{
	m_dimMin += T;
	m_dimMax += T;
	m_pointsMin += T;
	m_pointsMax += T;
}

/*** RENDERING METHODS ***/

void ccOctree::draw(CC_DRAW_CONTEXT& context, ccColor::Rgb* pickingColor/*=nullptr*/)
{
	if (	!m_theAssociatedCloudAsGPC
		||	m_thePointsAndTheirCellCodes.empty() )
	{
		return;
	}
	
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert( glFunc != nullptr );
	
	if ( glFunc == nullptr )
		return;

	glFunc->glPushAttrib(GL_LIGHTING_BIT);

	if (m_displayMode == WIRE)
	{
		//this display mode is too heavy to be stored as a GL list
		//(therefore we always render it dynamically)
		
		glFunc->glDisable(GL_LIGHTING);
		if (pickingColor)
			ccGL::Color(glFunc, *pickingColor);
		else
			ccGL::Color(glFunc, ccColor::green);

		void* additionalParameters[] = {	reinterpret_cast<void*>(m_frustumIntersector),
											reinterpret_cast<void*>(glFunc)
		};
		executeFunctionForAllCellsAtLevel(	m_displayedLevel,
											&DrawCellAsABox,
											additionalParameters);
	}
	else
	{
		glDrawParams glParams;
		m_theAssociatedCloudAsGPC->getDrawingParameters(glParams);

		if (pickingColor)
		{
			ccGL::Color(glFunc, *pickingColor);
			glParams.showSF = false;
			glParams.showColors = false;
			glParams.showNorms = false;
		}
		else if (!glParams.showColors)
		{
			ccGL::Color(glFunc, ccColor::white);
		}

		if (glParams.showNorms)
		{
			glFunc->glDisable(GL_RESCALE_NORMAL);
			glFunc->glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, CC_DEFAULT_CLOUD_AMBIENT_COLOR.rgba);
			glFunc->glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, CC_DEFAULT_CLOUD_SPECULAR_COLOR.rgba);
			glFunc->glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, CC_DEFAULT_CLOUD_DIFFUSE_COLOR.rgba);
			glFunc->glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, CC_DEFAULT_CLOUD_EMISSION_COLOR.rgba);
			glFunc->glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, CC_DEFAULT_CLOUD_SHININESS);
			glFunc->glEnable(GL_LIGHTING);

			glFunc->glEnable(GL_COLOR_MATERIAL);
			glFunc->glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
		}

		//shall we recompile the GL list?
		if (m_glListIsDeprecated)
		{
			if (m_glListID == 0)
			{
				m_glListID = glFunc->glGenLists(1);
			}
			glFunc->glNewList(m_glListID, GL_COMPILE);

			if (m_displayMode == MEAN_POINTS)
			{
				void* additionalParameters[] = {	reinterpret_cast<void*>(&glParams),
													reinterpret_cast<void*>(m_theAssociatedCloudAsGPC),
													reinterpret_cast<void*>(glFunc)
				};

				if (glParams.showNorms)
				{
					glFunc->glEnable(GL_RESCALE_NORMAL);
					glFunc->glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, CC_DEFAULT_CLOUD_AMBIENT_COLOR.rgba);
					glFunc->glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, CC_DEFAULT_CLOUD_SPECULAR_COLOR.rgba);
					glFunc->glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, CC_DEFAULT_CLOUD_DIFFUSE_COLOR.rgba);
					glFunc->glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, CC_DEFAULT_CLOUD_EMISSION_COLOR.rgba);
					glFunc->glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, CC_DEFAULT_CLOUD_SHININESS);
					glFunc->glPushAttrib(GL_LIGHTING_BIT);
					glFunc->glEnable(GL_LIGHTING);
				}

				glFunc->glBegin(GL_POINTS);
				executeFunctionForAllCellsAtLevel(	m_displayedLevel,
													&DrawCellAsAPoint,
													additionalParameters);
				glFunc->glEnd();

				if (glParams.showNorms)
				{
					glFunc->glPopAttrib(); //GL_LIGHTING_BIT
				}
			}
			else if (m_displayMode == MEAN_CUBES)
			{
				//by default we use a box as primitive
				PointCoordinateType cs = getCellSize(m_displayedLevel);
				CCVector3 dims(cs, cs, cs);
				ccBox box(dims);
				box.showColors(glParams.showColors || glParams.showSF);
				box.showNormals(glParams.showNorms);

				//trick: replace all normal indexes so that they point on the first one
				{
					if (box.arePerTriangleNormalsEnabled())
						for (unsigned i = 0; i < box.size(); ++i)
							box.setTriangleNormalIndexes(i, 0, 0, 0);
				}

				//fake context
				CC_DRAW_CONTEXT fakeContext = context;
				fakeContext.drawingFlags = CC_DRAW_3D | CC_DRAW_FOREGROUND | CC_LIGHT_ENABLED;
				fakeContext.display = nullptr;

				void* additionalParameters[] = {	reinterpret_cast<void*>(&glParams),
													reinterpret_cast<void*>(m_theAssociatedCloudAsGPC),
													reinterpret_cast<void*>(&box),
													reinterpret_cast<void*>(&fakeContext)
				};

				executeFunctionForAllCellsAtLevel(	m_displayedLevel,
													&DrawCellAsAPrimitive,
													additionalParameters);
			}
			else
			{
				assert(false);
			}

			glFunc->glEndList();
			m_glListIsDeprecated = false;
		}

		glFunc->glCallList(m_glListID);

		if (glParams.showNorms)
		{
			glFunc->glDisable(GL_COLOR_MATERIAL);
			//DGM FIXME: is it still true with Qt5.4+?
			glFunc->glDisable(GL_RESCALE_NORMAL);
			glFunc->glDisable(GL_LIGHTING);
		}
	}

	glFunc->glPopAttrib();
}

bool ccOctree::DrawCellAsABox(	const CCCoreLib::DgmOctree::octreeCell& cell,
								void** additionalParameters,
								CCCoreLib::NormalizedProgress* nProgress/*=nullptr*/)
{
	ccOctreeFrustumIntersector* ofi = static_cast<ccOctreeFrustumIntersector*>(additionalParameters[0]);
	QOpenGLFunctions_2_1* glFunc     = static_cast<QOpenGLFunctions_2_1*>(additionalParameters[1]);
	assert(glFunc != nullptr);

	CCVector3 bbMin;
	CCVector3 bbMax;
	cell.parentOctree->computeCellLimits(cell.truncatedCode, cell.level, bbMin, bbMax, true);

	ccOctreeFrustumIntersector::OctreeCellVisibility vis = ccOctreeFrustumIntersector::CELL_OUTSIDE_FRUSTUM;
	if (ofi)
		vis = ofi->positionFromFrustum(cell.truncatedCode, cell.level);

	// outside
	if (vis == ccOctreeFrustumIntersector::CELL_OUTSIDE_FRUSTUM)
	{
		ccGL::Color(glFunc, ccColor::green);
	}
	else
	{
		glFunc->glPushAttrib(GL_LINE_BIT);
		glFunc->glLineWidth(2.0f);
		// inside
		if (vis == ccOctreeFrustumIntersector::CELL_INSIDE_FRUSTUM)
			ccGL::Color(glFunc, ccColor::magenta);
		// intersecting
		else
			ccGL::Color(glFunc, ccColor::blue);
	}

	glFunc->glBegin(GL_LINE_LOOP);
	ccGL::Vertex3v(glFunc, bbMin.u);
	ccGL::Vertex3(glFunc, bbMax.x,bbMin.y,bbMin.z);
	ccGL::Vertex3(glFunc, bbMax.x,bbMax.y,bbMin.z);
	ccGL::Vertex3(glFunc, bbMin.x,bbMax.y,bbMin.z);
	glFunc->glEnd();

	glFunc->glBegin(GL_LINE_LOOP);
	ccGL::Vertex3(glFunc, bbMin.x,bbMin.y,bbMax.z);
	ccGL::Vertex3(glFunc, bbMax.x,bbMin.y,bbMax.z);
	ccGL::Vertex3v(glFunc, bbMax.u);
	ccGL::Vertex3(glFunc, bbMin.x, bbMax.y, bbMax.z);
	glFunc->glEnd();

	glFunc->glBegin(GL_LINES);
	ccGL::Vertex3v(glFunc, bbMin.u);
	ccGL::Vertex3(glFunc, bbMin.x,bbMin.y,bbMax.z);
	ccGL::Vertex3(glFunc, bbMax.x,bbMin.y,bbMin.z);
	ccGL::Vertex3(glFunc, bbMax.x,bbMin.y,bbMax.z);
	ccGL::Vertex3(glFunc, bbMax.x,bbMax.y,bbMin.z);
	ccGL::Vertex3v(glFunc, bbMax.u);
	ccGL::Vertex3(glFunc, bbMin.x,bbMax.y,bbMin.z);
	ccGL::Vertex3(glFunc, bbMin.x,bbMax.y,bbMax.z);
	glFunc->glEnd();

	// not outside
	if (vis != ccOctreeFrustumIntersector::CELL_OUTSIDE_FRUSTUM)
	{
		glFunc->glPopAttrib();
	}

	return true;
}

bool ccOctree::DrawCellAsAPoint(const CCCoreLib::DgmOctree::octreeCell& cell,
								void** additionalParameters,
								CCCoreLib::NormalizedProgress* nProgress/*=nullptr*/)
{
	//variables additionnelles
	glDrawParams* glParams			= reinterpret_cast<glDrawParams*>(additionalParameters[0]);
	ccGenericPointCloud* cloud		= reinterpret_cast<ccGenericPointCloud*>(additionalParameters[1]);
	QOpenGLFunctions_2_1* glFunc	= static_cast<QOpenGLFunctions_2_1*>(additionalParameters[2]);
	assert(glFunc != nullptr);

	if (glParams->showSF)
	{
		ScalarType dist = CCCoreLib::ScalarFieldTools::computeMeanScalarValue(cell.points);
		const ccColor::Rgb* col = cloud->geScalarValueColor(dist);
		ccGL::Color(glFunc, col ? *col : ccColor::lightGreyRGB);
	}
	else if (glParams->showColors)
	{
		ccColor::Rgb col = ComputeAverageColor(cell.points, cloud);
		ccGL::Color(glFunc, col);
	}

	if (glParams->showNorms)
	{
		CCVector3 N = ComputeAverageNorm(cell.points, cloud);
		ccGL::Normal3v(glFunc, N.u);
	}

	const CCVector3* gravityCenter = CCCoreLib::Neighbourhood(cell.points).getGravityCenter();
	ccGL::Vertex3v(glFunc, gravityCenter->u);

	return true;
}

bool ccOctree::DrawCellAsAPrimitive(const CCCoreLib::DgmOctree::octreeCell& cell,
									void** additionalParameters,
									CCCoreLib::NormalizedProgress* nProgress/*=nullptr*/)
{
	//variables additionnelles
	glDrawParams* glParams			= reinterpret_cast<glDrawParams*>(additionalParameters[0]);
	ccGenericPointCloud* cloud		= reinterpret_cast<ccGenericPointCloud*>(additionalParameters[1]);
	ccGenericPrimitive*	primitive	= reinterpret_cast<ccGenericPrimitive*>(additionalParameters[2]);
	CC_DRAW_CONTEXT* context		= reinterpret_cast<CC_DRAW_CONTEXT*>(additionalParameters[3]);

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context->glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return false;

	CCVector3 cellCenter;
	cell.parentOctree->computeCellCenter(cell.truncatedCode, cell.level, cellCenter, true);

	if (glParams->showSF)
	{
		ScalarType dist = CCCoreLib::ScalarFieldTools::computeMeanScalarValue(cell.points);
		const ccColor::Rgb* rgb = cloud->geScalarValueColor(dist);
		if (rgb)
			primitive->setColor(*rgb);
	}
	else if (glParams->showColors)
	{
		ccColor::Rgb col = ComputeAverageColor(cell.points, cloud);
		primitive->setColor(col);
	}

	if (glParams->showNorms)
	{
		CCVector3 N = ComputeAverageNorm(cell.points, cloud);
		if (primitive->getTriNormsTable())
		{
			//only one normal!
			primitive->getTriNormsTable()->setValue(0, ccNormalVectors::GetNormIndex(N.u));
		}
	}

	glFunc->glPushMatrix();
	ccGL::Translate(glFunc, cellCenter.x, cellCenter.y, cellCenter.z);
	primitive->draw(*context);
	glFunc->glPopMatrix();

	return true;
}

ccColor::Rgb ccOctree::ComputeAverageColor(CCCoreLib::ReferenceCloud* subset, ccGenericPointCloud* sourceCloud)
{
	if (!subset || subset->size() == 0 || !sourceCloud)
		return {};

	assert(sourceCloud->hasColors());
	assert(subset->getAssociatedCloud() == static_cast<CCCoreLib::GenericIndexedCloud*>(sourceCloud));

	Tuple3Tpl<double> sum(0, 0, 0);

	unsigned n = subset->size();
	for (unsigned i = 0; i < n; ++i)
	{
		const ccColor::Rgb& _theColor = sourceCloud->getPointColor(subset->getPointGlobalIndex(i));
		sum.x += _theColor.r;
		sum.y += _theColor.g;
		sum.z += _theColor.b;
	}

	return ccColor::Rgb(static_cast<ColorCompType>(sum.x / n),
						static_cast<ColorCompType>(sum.y / n),
						static_cast<ColorCompType>(sum.z / n));
}

CCVector3 ccOctree::ComputeAverageNorm(CCCoreLib::ReferenceCloud* subset, ccGenericPointCloud* sourceCloud)
{
	CCVector3 N(0, 0, 0);

	if (!subset || subset->size() == 0 || !sourceCloud)
		return N;

	assert(sourceCloud->hasNormals());
	assert(subset->getAssociatedCloud() == static_cast<CCCoreLib::GenericIndexedCloud*>(sourceCloud));

	unsigned n = subset->size();
	for (unsigned i = 0; i < n; ++i)
	{
		const CCVector3& Ni = sourceCloud->getPointNormal(subset->getPointGlobalIndex(i));
		N += Ni;
	}

	N.normalize();
	return N;
}

bool ccOctree::intersectWithFrustum(ccCameraSensor* sensor, std::vector<unsigned>& inCameraFrustum)
{
	if (!sensor)
		return false;

	// initialization
	float globalPlaneCoefficients[6][4];
	CCVector3 globalCorners[8];
	CCVector3 globalEdges[6];
	CCVector3 globalCenter; 
	sensor->computeGlobalPlaneCoefficients(globalPlaneCoefficients, globalCorners, globalEdges, globalCenter);

	if (!m_frustumIntersector)
	{
		m_frustumIntersector = new ccOctreeFrustumIntersector();
		if (!m_frustumIntersector->build(this))
		{
			ccLog::Warning("[ccOctree::intersectWithFrustum] Not enough memory!");
			return false;
		}
	}

	// get points of cells in frustum
	std::vector< std::pair<unsigned, CCVector3> > pointsToTest;
	m_frustumIntersector->computeFrustumIntersectionWithOctree(pointsToTest, inCameraFrustum, globalPlaneCoefficients, globalCorners, globalEdges, globalCenter);
	
	// project points
	for (size_t i = 0; i < pointsToTest.size(); i++)
	{
		if (sensor->isGlobalCoordInFrustum(pointsToTest[i].second/*, false*/))
			inCameraFrustum.push_back(pointsToTest[i].first);
	}

	return true;
}

bool ccOctree::pointPicking(const CCVector2d& clickPos,
							const ccGLCameraParameters& camera,
							PointDescriptor& output,
							double pickWidth_pix/*=3.0*/) const
{
	output.point = nullptr;
	output.squareDistd = -1.0;

	if (!m_theAssociatedCloudAsGPC)
	{
		assert(false);
		return false;
	}

	if (m_thePointsAndTheirCellCodes.empty())
	{
		//nothing to do
		return false;
	}
	
	CCVector3d clickPosd(clickPos.x, clickPos.y, 0.0);
	CCVector3d X(0, 0, 0);
	if (!camera.unproject(clickPosd, X))
	{
		return false;
	}

	ccGLMatrix trans;
	bool hasGLTrans = m_theAssociatedCloudAsGPC->getAbsoluteGLTransformation(trans);

	//compute 3D picking 'ray'
	CCVector3 rayAxis;
	CCVector3 rayOrigin;
	{
		CCVector3d clickPosd2(clickPos.x, clickPos.y, 1.0);
		CCVector3d Y(0, 0, 0);
		if (!camera.unproject(clickPosd2, Y))
		{
			return false;
		}

		rayAxis = (Y-X).toPC();
		rayOrigin = X.toPC();

		if (hasGLTrans)
		{
			ccGLMatrix iTrans = trans.inverse();
			iTrans.applyRotation(rayAxis);
			iTrans.apply(rayOrigin);
		}

		rayAxis.normalize(); //normalize afterwards as the local transformation may have a scale != 1
	}

	CCVector3 margin(0, 0, 0);
	double maxFOV_rad = 0;
	if (camera.perspective)
	{
		maxFOV_rad = 0.002 * pickWidth_pix; //empirical conversion from pixels to FOV angle (in radians)
	}
	else
	{
		double maxRadius = pickWidth_pix * camera.fov_deg / 2;
		margin = CCVector3(1, 1, 1) * static_cast<PointCoordinateType>(maxRadius);
	}

	//first test with the total bounding box
	CCCoreLib::Ray<PointCoordinateType> ray(rayAxis, rayOrigin);
	if (!CCCoreLib::AABB<PointCoordinateType>(m_dimMin - margin, m_dimMax + margin).intersects(ray))
	{
		//no intersection
		return true; //DGM: false would mean that an error occurred! (output.point == 0 means that nothing has been found)
	}

	//no need to go too deep
	const unsigned char maxLevel = findBestLevelForAGivenPopulationPerCell(10);

	//starting level of subdivision
	unsigned char level = 1;
	//binary shift for cell code truncation at current level
	unsigned char currentBitDec = GET_BIT_SHIFT(level);
	//current cell code
	CellCode currentCellCode = INVALID_CELL_CODE;
	CellCode currentCellTruncatedCode = INVALID_CELL_CODE;
	//whether the current cell should be skipped or not
	bool skipThisCell = false;

#ifdef DEBUG_PICKING_MECHANISM
	m_theAssociatedCloud->enableScalarField();
#endif

	//ray with origin expressed in the local coordinate system!
	CCCoreLib::Ray<PointCoordinateType> rayLocal(rayAxis, rayOrigin - m_dimMin);

	//visibility table (if any)
	const ccGenericPointCloud::VisibilityTableType* visTable = m_theAssociatedCloudAsGPC->isVisibilityTableInstantiated() ? &m_theAssociatedCloudAsGPC->getTheVisibilityArray() : nullptr;

	//scalar field with hidden values (if any)
	ccScalarField* activeSF = nullptr;
	if (	m_theAssociatedCloudAsGPC->sfShown()
		&&	m_theAssociatedCloudAsGPC->isA(CC_TYPES::POINT_CLOUD)
		&&	!visTable //if the visibility table is instantiated, we always display ALL points
		)
	{
		ccPointCloud* pc = static_cast<ccPointCloud*>(m_theAssociatedCloudAsGPC);
		ccScalarField* sf = pc->getCurrentDisplayedScalarField();
		if (sf && sf->mayHaveHiddenValues() && sf->getColorScale())
		{
			//we must take this SF display parameters into account as some points may be hidden!
			activeSF = sf;
		}
	}

	//let's sweep through the octree
	for (cellsContainer::const_iterator it = m_thePointsAndTheirCellCodes.begin(); it != m_thePointsAndTheirCellCodes.end(); ++it)
	{
		CellCode truncatedCode = (it->theCode >> currentBitDec);
		
		//new cell?
		if (truncatedCode != currentCellTruncatedCode)
		{
			//look for the biggest 'parent' cell that englobes this cell and the previous one (if any)
			while (level > 1)
			{
				unsigned char bitDec = GET_BIT_SHIFT(level-1);
				if ((it->theCode >> bitDec) == (currentCellCode >> bitDec))
				{
					//same parent cell, we can stop here
					break;
				}
				--level;
			}

			currentCellCode = it->theCode;

			//now try to go deeper with the new cell
			while (level < maxLevel)
			{
				Tuple3i cellPos;
				getCellPos(it->theCode, level, cellPos, false);

				//first test with the total bounding box
				PointCoordinateType halfCellSize = getCellSize(level) / 2;
				CCVector3 cellCenter(	(2* cellPos.x + 1) * halfCellSize,
										(2* cellPos.y + 1) * halfCellSize,
										(2* cellPos.z + 1) * halfCellSize);

				CCVector3 halfCell = CCVector3(halfCellSize, halfCellSize, halfCellSize);

				if (camera.perspective)
				{
					double radialSqDist = 0.0;
					double sqDistToOrigin = 0.0;
					rayLocal.squareDistances(cellCenter, radialSqDist, sqDistToOrigin);

					double dx = sqrt(sqDistToOrigin);
					double dy = std::max<double>(0, sqrt(radialSqDist) - CCCoreLib::SQRT_3 * halfCellSize);
					double fov_rad = atan2(dy, dx);

					skipThisCell = (fov_rad > maxFOV_rad);
				}
				else
				{
					skipThisCell = !CCCoreLib::AABB<PointCoordinateType>(	cellCenter - halfCell - margin,
																cellCenter + halfCell + margin).intersects(rayLocal);
				}

				if (skipThisCell)
					break;
				else
					++level;
			}
			
			currentBitDec = GET_BIT_SHIFT(level);
			currentCellTruncatedCode = (currentCellCode >> currentBitDec);
		}

#ifdef DEBUG_PICKING_MECHANISM
		m_theAssociatedCloud->setPointScalarValue(it->theIndex, level);
#endif

		if (!skipThisCell)
		{
			//we shouldn't test points that are actually hidden!
			if (	(!visTable || visTable->at(it->theIndex) == CCCoreLib::POINT_VISIBLE)
				&&	(!activeSF || activeSF->getColor(activeSF->getValue(it->theIndex)))
				)
			{
				//test the point
				const CCVector3* P = m_theAssociatedCloud->getPoint(it->theIndex);
				CCVector3 Q = *P;
				if (hasGLTrans)
				{
					trans.apply(Q);
				}

				CCVector3d Q2D;
				bool insideFrustum = false;
				camera.project(Q, Q2D, &insideFrustum);
				if (insideFrustum)
				{
					if (	std::abs(Q2D.x - clickPos.x) <= pickWidth_pix
						&&	std::abs(Q2D.y - clickPos.y) <= pickWidth_pix )
					{
						double squareDist = CCVector3d(X.x - Q.x, X.y - Q.y, X.z - Q.z).norm2d();
						if (!output.point || squareDist < output.squareDistd)
						{
							output.point = P;
							output.pointIndex = it->theIndex;
							output.squareDistd = squareDist;
						}
					}
				}
			}
		}
	}

	return true;
}

PointCoordinateType ccOctree::GuessNaiveRadius(ccGenericPointCloud* cloud)
{
	if (!cloud)
	{
		assert(false);
		return 0;
	}

	PointCoordinateType largestDim = cloud->getOwnBB().getMaxBoxDim();

	return largestDim / std::min<unsigned>(100, std::max<unsigned>(1, cloud->size() / 100));
}

PointCoordinateType ccOctree::GuessBestRadiusAutoComputeOctree(	ccGenericPointCloud* cloud,
																const BestRadiusParams& params,
																QWidget* parentWidget/*=nullptr*/)
{
	if (!cloud)
	{
		assert(false);
		return 0;
	}

	if (!cloud->getOctree())
	{
		ccProgressDialog pDlg(true, parentWidget);
		if (!cloud->computeOctree(&pDlg))
		{
			ccLog::Error(tr("Could not compute octree for cloud '%1'").arg(cloud->getName()));
			return 0;
		}
	}

	return ccOctree::GuessBestRadius(cloud, params, cloud->getOctree().data());
}


PointCoordinateType ccOctree::GuessBestRadius(	ccGenericPointCloud* cloud,
												const BestRadiusParams& params,
												CCCoreLib::DgmOctree* inputOctree/*=nullptr*/,
												CCCoreLib::GenericProgressCallback* progressCb/*=nullptr*/)
{
	if (!cloud)
	{
		assert(false);
		return 0;
	}

	CCCoreLib::DgmOctree* octree = inputOctree;
	if (!octree)
	{
		octree = new CCCoreLib::DgmOctree(cloud);
		if (octree->build(progressCb) <= 0)
		{
			delete octree;
			ccLog::Warning("[GuessBestRadius] Failed to compute the cloud octree");
			return 0;
		}
	}

	PointCoordinateType bestRadius = GuessNaiveRadius(cloud);
	if (bestRadius == 0)
	{
		ccLog::Warning("[GuessBestRadius] The cloud has invalid dimensions");
		return 0;
	}

	if (cloud->size() < 100)
	{
		//no need to do anything else for very small clouds!
		return bestRadius;
	}

	//we are now going to sample the cloud so as to compute statistics on the density
	{
		const unsigned sampleCount = std::min<unsigned>(200, cloud->size() / 10);

		double aimedPop = params.aimedPopulationPerCell;
		PointCoordinateType radius = bestRadius;
		PointCoordinateType lastRadius = radius;
		double lastMeanPop = 0;

		std::random_device rd;   // non-deterministic generator
		std::mt19937 gen(rd());  // to seed mersenne twister.
		std::uniform_int_distribution<unsigned> dist(0, cloud->size() - 1);

		//we may have to do this several times
		for (size_t attempt = 0; attempt < 10; ++attempt)
		{
			int totalCount = 0;
			int totalSquareCount = 0;
			int minPop = 0;
			int maxPop = 0;
			int aboveMinPopCount = 0;

			unsigned char octreeLevel = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(radius);

			for (size_t i = 0; i < sampleCount; ++i)
			{
				unsigned randomIndex = dist(gen);
				assert(randomIndex < cloud->size());

				const CCVector3* P = cloud->getPoint(randomIndex);
				CCCoreLib::DgmOctree::NeighboursSet Yk;
				int n = octree->getPointsInSphericalNeighbourhood(*P, radius, Yk, octreeLevel);
				assert(n >= 1);

				totalCount += n;
				totalSquareCount += n * n;
				if (i == 0)
				{
					minPop = maxPop = n;
				}
				else
				{
					if (n < minPop)
						minPop = n;
					else if (n > maxPop)
						maxPop = n;
				}

				if (n >= params.minCellPopulation)
				{
					++aboveMinPopCount;
				}
			}

			double meanPop = static_cast<double>(totalCount) / sampleCount;
			double stdDevPop = sqrt(std::abs(static_cast<double>(totalSquareCount) / sampleCount - meanPop * meanPop));
			double aboveMinPopRatio = static_cast<double>(aboveMinPopCount) / sampleCount;

			ccLog::Print(QString("[GuessBestRadius] Radius = %1 -> samples population in [%2 ; %3] (mean %4 / std. dev. %5 / %6% above minimum)")
				.arg(radius)
				.arg(minPop)
				.arg(maxPop)
				.arg(meanPop)
				.arg(stdDevPop)
				.arg(aboveMinPopRatio * 100)
			);

			if (std::abs(meanPop - aimedPop) < params.aimedPopulationRange)
			{
				//we have found a correct radius
				bestRadius = radius;

				if (aboveMinPopRatio < params.minAboveMinRatio)
				{
					//ccLog::Warning("[GuessBestRadius] The cloud density is very inhomogeneous! You may have to increase the radius to get valid normals everywhere... but the result will be smoother");
					aimedPop = params.aimedPopulationPerCell + (2.0*stdDevPop)/* * (1.0-aboveMinPopRatio)*/;
					assert(aimedPop >= params.aimedPopulationPerCell);
				}
				else
				{
					break;
				}
			}

			//otherwise we have to find a better estimate for the radius
			PointCoordinateType newRadius = radius;
			//(warning: we consider below that the number of points is proportional to the SURFACE of the neighborhood)
			assert(meanPop >= 1.0);
			if (attempt == 0)
			{
				//this is our best (only) guess for the moment
				bestRadius = radius;

				newRadius = radius * sqrt(aimedPop / meanPop);
			}
			else
			{
				//keep track of our best guess nevertheless
				if (std::abs(meanPop - aimedPop) < std::abs(bestRadius - aimedPop))
				{
					bestRadius = radius;
				}

				double slope = (radius*radius - lastRadius * lastRadius) / (meanPop - lastMeanPop);
				PointCoordinateType newSquareRadius = lastRadius * lastRadius + (aimedPop - lastMeanPop) * slope;
				if (newSquareRadius > 0)
				{
					newRadius = sqrt(newSquareRadius);
				}
				else
				{
					//can't do any better!
					break;
				}
			}

			lastRadius = radius;
			lastMeanPop = meanPop;

			radius = newRadius;
		}
	}

	if (octree && !inputOctree)
	{
		delete octree;
		octree = nullptr;
	}

	return bestRadius;
}
