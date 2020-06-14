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

#ifdef USE_TBB
#include <tbb/parallel_for.h>
#endif

#include "ccGenericPointCloud.h"

//CCLib
#include <DistanceComputationTools.h>
#include <GenericProgressCallback.h>
#include <Neighbourhood.h>
#include <ReferenceCloud.h>

//Local
#include "ccGenericGLDisplay.h"
#include "ccOctreeProxy.h"
#include "ccPointCloud.h"
#include "ccProgressDialog.h"
#include "ccScalarField.h"
#include "ccSensor.h"


ccGenericPointCloud::ccGenericPointCloud(QString name, unsigned uniqueID)
	: ccShiftedObject(name, uniqueID)
	, m_pointSize(0)
{
	setVisible(true);
	lockVisibility(false);
}

ccGenericPointCloud::ccGenericPointCloud(const ccGenericPointCloud& cloud)
	: ccShiftedObject(cloud)
	, m_pointsVisibility(cloud.m_pointsVisibility)
	, m_pointSize(cloud.m_pointSize)
{
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

bool ccGenericPointCloud::resetVisibilityArray()
{
	try
	{
		m_pointsVisibility.resize(size());
	}
	catch (const std::bad_alloc&)
	{
		unallocateVisibilityArray();
		return false;
	}

	std::fill(m_pointsVisibility.begin(), m_pointsVisibility.end(), POINT_VISIBLE); //by default, all points are visible

	return true;
}

void ccGenericPointCloud::invertVisibilityArray()
{
	if (m_pointsVisibility.empty())
	{
		assert(false);
		return;
	}

	for (unsigned char& vis : m_pointsVisibility)
	{
		vis = (vis == POINT_HIDDEN ? POINT_VISIBLE : POINT_HIDDEN);
	}
}

void ccGenericPointCloud::unallocateVisibilityArray()
{
	m_pointsVisibility.resize(0);
}

bool ccGenericPointCloud::isVisibilityTableInstantiated() const
{
	return !m_pointsVisibility.empty();
}

void ccGenericPointCloud::deleteOctree()
{
	ccOctreeProxy* oct = getOctreeProxy();
	if (oct != nullptr)
	{
		removeChild(oct);
	}
}

ccOctreeProxy* ccGenericPointCloud::getOctreeProxy() const
{
	for (auto child : m_children)
	{
		if (child->isA(CC_TYPES::POINT_OCTREE))
		{
			return static_cast<ccOctreeProxy*>(child);
		}
	}

	return nullptr;
}

ccOctree::Shared ccGenericPointCloud::getOctree() const
{
	ccOctreeProxy* proxy = getOctreeProxy();
	if (proxy != nullptr)
	{
		return proxy->getOctree();
	}
	else
	{
		return ccOctree::Shared(nullptr);
	}
}

void ccGenericPointCloud::setOctree(ccOctree::Shared octree, bool autoAddChild/*=true*/)
{
	if (!octree || octree->getNumberOfProjectedPoints() == 0)
	{
		assert(false);
		return;
	}

	deleteOctree();

	ccOctreeProxy* proxy = new ccOctreeProxy(octree);
	proxy->setDisplay(getDisplay());
	proxy->setVisible(true);
	proxy->setEnabled(false);
	if (autoAddChild)
	{
		addChild(proxy);
	}
}

ccOctree::Shared ccGenericPointCloud::computeOctree(CCLib::GenericProgressCallback* progressCb, bool autoAddChild/*=true*/)
{
	deleteOctree();
	
	ccOctree::Shared octree = ccOctree::Shared(new ccOctree(this));
	if (octree->build(progressCb) > 0)
	{
		setOctree(octree, autoAddChild);
	}
	else
	{
		octree.clear();
	}

	return octree;
}

ccBBox ccGenericPointCloud::getOwnBB(bool withGLFeatures/*=false*/)
{
	ccBBox box;

	if (size())
	{
		getBoundingBox(box.minCorner(), box.maxCorner());
		box.setValidity(true);
	}
	
	return box;
}

bool ccGenericPointCloud::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//'global shift & scale' (dataVersion>=39)
	saveShiftInfoToFile(out);
	
	//'visibility' array (dataVersion>=20)
	bool hasVisibilityArray = isVisibilityTableInstantiated();
	if (out.write((const char*)&hasVisibilityArray,sizeof(bool)) < 0)
		return WriteError();
	if (hasVisibilityArray)
	{
		if (!ccSerializationHelper::GenericArrayToFile<unsigned char, 1, unsigned char>(m_pointsVisibility, out))
			return false;
	}

	//'point size' (dataVersion>=24)
	if (out.write((const char*)&m_pointSize,1) < 0)
		return WriteError();

	return true;
}

bool ccGenericPointCloud::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	if (dataVersion < 20)
		return CorruptError();

	if (dataVersion < 33)
	{
		//'coordinates shift' (dataVersion>=20)
		if (in.read((char*)m_globalShift.u,sizeof(double)*3) < 0)
			return ReadError();

		m_globalScale = 1.0;
	}
	else
	{
		//'global shift & scale' (dataVersion>=33)
		if (!loadShiftInfoFromFile(in))
			return ReadError();
	}

	//'visibility' array (dataVersion>=20)
	bool hasVisibilityArray = false;
	if (in.read((char*)&hasVisibilityArray,sizeof(bool)) < 0)
		return ReadError();
	if (hasVisibilityArray)
	{
		if (!ccSerializationHelper::GenericArrayFromFile<unsigned char, 1, unsigned char>(m_pointsVisibility, in, dataVersion))
		{
			unallocateVisibilityArray();
			return false;
		}
	}

	//'point size' (dataVersion>=24)
	if (dataVersion >= 24)
	{
		if (in.read((char*)&m_pointSize,1) < 0)
			return WriteError();
	}
	else
	{
		m_pointSize = 0; // follows default setting
	}

	return true;
}

void ccGenericPointCloud::importParametersFrom(const ccGenericPointCloud* cloud)
{
	if (!cloud)
	{
		assert(false);
		return;
	}

	//original center
	setGlobalShift(cloud->getGlobalShift());
	setGlobalScale(cloud->getGlobalScale());
	//keep the transformation history!
	setGLTransformationHistory(cloud->getGLTransformationHistory());
	//custom point size
	setPointSize(cloud->getPointSize());
	//meta-data
	setMetaData(cloud->metaData());
}

#ifdef QT_DEBUG
//for tests
#include "ccPointCloud.h"
#include <ScalarField.h>
#endif

bool ccGenericPointCloud::pointPicking(	const CCVector2d& clickPos,
										const ccGLCameraParameters& camera,
										int& nearestPointIndex,
										double& nearestSquareDist,
										double pickWidth/*=2.0*/,
										double pickHeight/*=2.0*/,
										bool autoComputeOctree/*=false*/)
{
	//can we use the octree to accelerate the point picking process?
	if (pickWidth == pickHeight)
	{
		ccOctree::Shared octree = getOctree();
		if (!octree && autoComputeOctree)
		{
			ccProgressDialog pDlg(false, getDisplay() ? getDisplay()->asWidget() : nullptr);
			octree = computeOctree(&pDlg);
		}

		if (octree)
		{
			//we can now use the octree to do faster point picking
#ifdef QT_DEBUG
			CCLib::ScalarField* sf = nullptr;
			if (getClassID() == CC_TYPES::POINT_CLOUD)
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(this);
				int sfIdx = pc->getScalarFieldIndexByName("octree_picking");
				if (sfIdx < 0)
				{
					sfIdx = pc->addScalarField("octree_picking");
				}
				if (sfIdx >= 0)
				{
					pc->setCurrentScalarField(sfIdx);
					pc->setCurrentDisplayedScalarField(sfIdx);
					pc->showSF(true);
					sf = pc->getScalarField(sfIdx);
				}
			}
#endif
			ccOctree::PointDescriptor point;
			if (octree->pointPicking(clickPos, camera, point, pickWidth))
			{
#ifdef QT_DEBUG
				if (sf)
				{
					sf->computeMinAndMax();
					if (getDisplay())
						getDisplay()->redraw();
				}
#endif
				if (point.point)
				{
					nearestPointIndex = point.pointIndex;
					nearestSquareDist = point.squareDistd;
					return true;
				}
				else
				{
					//nothing found
					return false;
				}
			}
			else
			{
				ccLog::Warning("[Point picking] Failed to use the octree. We'll fall back to the slow process...");
			}
		}
	}

	//otherwise we go 'brute force' (works quite well in fact?!)
	nearestPointIndex = -1;
	nearestSquareDist = -1.0;
	{
		//back project the clicked point in 3D
		CCVector3d clickPosd(clickPos.x, clickPos.y, 0);
		CCVector3d X(0, 0, 0);
		if (!camera.unproject(clickPosd, X))
		{
			return false;
		}

		//warning: we have to handle the relative GL transformation!
		ccGLMatrix trans;
		bool noGLTrans = !getAbsoluteGLTransformation(trans);

		//visibility table (if any)
		const ccGenericPointCloud::VisibilityTableType* visTable = isVisibilityTableInstantiated() ? &getTheVisibilityArray() : nullptr;

		//scalar field with hidden values (if any)
		ccScalarField* activeSF = nullptr;
		if (	sfShown()
			&&	isA(CC_TYPES::POINT_CLOUD)
			&&	!visTable //if the visibility table is instantiated, we always display ALL points
			)
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(this);
			ccScalarField* sf = pc->getCurrentDisplayedScalarField();
			if (sf && sf->mayHaveHiddenValues() && sf->getColorScale())
			{
				//we must take this SF display parameters into account as some points may be hidden!
				activeSF = sf;
			}
		}

		int pointCount = static_cast<int>(size());
#ifdef USE_TBB
		tbb::parallel_for( 0, pointCount, [&](int i)
#else
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < pointCount; ++i)
#endif
		{
			//we shouldn't test points that are actually hidden!
			if (	(!visTable || visTable->at(i) == POINT_VISIBLE)
				&&	(!activeSF || activeSF->getColor(activeSF->getValue(i)))
				)
			{
				const CCVector3* P = getPoint(i);

				CCVector3d Q2D;
				bool insideFrustum = false;
				if (noGLTrans)
				{
					camera.project(*P, Q2D, &insideFrustum);
				}
				else
				{
					CCVector3 P3D = *P;
					trans.apply(P3D);
					camera.project(P3D, Q2D, &insideFrustum);
				}

				if (!insideFrustum)
				{
					// Point is not inside the frustum
#ifdef USE_TBB
					return;
#else
					continue;
#endif
				}

				if (	fabs(Q2D.x - clickPos.x) <= pickWidth
					&&	fabs(Q2D.y - clickPos.y) <= pickHeight)
				{
					const double squareDist = CCVector3d(X.x - P->x, X.y - P->y, X.z - P->z).norm2d();
					if (nearestPointIndex < 0 || squareDist < nearestSquareDist)
					{
						nearestSquareDist = squareDist;
						nearestPointIndex = i;
					}
				}
			}
		}
#ifdef USE_TBB
		);
#endif
	}
	
	return (nearestPointIndex >= 0);
}

CCLib::ReferenceCloud* ccGenericPointCloud::getTheVisiblePoints(const VisibilityTableType* visTable/*=nullptr*/, bool silent/*=false*/) const
{
	if (!visTable)
	{
		visTable = &m_pointsVisibility;
	}

	unsigned count = size();
	if (!visTable || visTable->size() != count)
	{
		assert(false);
		ccLog::Warning("[ccGenericPointCloud::getTheVisiblePoints] No visibility table instantiated!");
		return nullptr;
	}

	//count the number of points to copy
	unsigned pointCount = 0;
	{
		for (unsigned i = 0; i < count; ++i)
		{
			if (visTable->at(i) == POINT_VISIBLE)
			{
				++pointCount;
			}
		}
	}

	//we create an entity with the 'visible' vertices only
	CCLib::ReferenceCloud* rc = new CCLib::ReferenceCloud(const_cast<ccGenericPointCloud*>(this));

	if (pointCount)
	{
		if (rc->reserve(pointCount))
		{
			for (unsigned i = 0; i < count; ++i)
			{
				if (visTable->at(i) == POINT_VISIBLE)
				{
					rc->addPointIndex(i); //can't fail (see above)
				}
			}
		}
		else
		{
			ccLog::Warning("[ccGenericPointCloud::getTheVisiblePoints] Not enough memory!");
			delete rc;
			rc = nullptr;
		}
	}
	else if (!silent)
	{
		ccLog::Warning("[ccGenericPointCloud::getTheVisiblePoints] No point in selection");
	}

	return rc;
}
