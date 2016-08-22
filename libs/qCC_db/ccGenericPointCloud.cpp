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

#include "ccGenericPointCloud.h"

//CCLib
#include <Neighbourhood.h>
#include <DistanceComputationTools.h>

//Local
#include "ccOctreeProxy.h"
#include "ccSensor.h"
#include "ccGenericGLDisplay.h"
#include "ccProgressDialog.h"
#include "ccPointCloud.h"
#include "ccScalarField.h"

ccGenericPointCloud::ccGenericPointCloud(QString name)
	: ccShiftedObject(name)
	, m_pointsVisibility(0)
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

void ccGenericPointCloud::invertVisibilityArray()
{
	if (!m_pointsVisibility || m_pointsVisibility->currentSize() == 0)
	{
		assert(false);
		return;
	}

	unsigned count = m_pointsVisibility->currentSize();
	for (unsigned i = 0; i < count; ++i)
	{
		m_pointsVisibility->setValue(i, m_pointsVisibility->getValue(i) == POINT_HIDDEN ? POINT_VISIBLE : POINT_HIDDEN);
	}
}

void ccGenericPointCloud::unallocateVisibilityArray()
{
	if (m_pointsVisibility)
	{
		m_pointsVisibility->release();
		m_pointsVisibility = 0;
	}
}

bool ccGenericPointCloud::isVisibilityTableInstantiated() const
{
	return m_pointsVisibility && m_pointsVisibility->isAllocated();
}

unsigned char ccGenericPointCloud::testVisibility(const CCVector3& P) const
{
	unsigned char bestVisibility = 255; //impossible value

	for (ccHObject::Container::const_iterator it = m_children.begin(); it != m_children.end(); ++it)
	{
		if ((*it)->isKindOf(CC_TYPES::SENSOR))
		{
			unsigned char visibility = static_cast<ccSensor*>(*it)->checkVisibility(P);

			if (visibility == POINT_VISIBLE)
			{
				return POINT_VISIBLE; //shortcut
			}

			bestVisibility = std::min<unsigned char>(visibility,bestVisibility);
		}
	}

	return (bestVisibility == 255 ? POINT_VISIBLE : bestVisibility);
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
	for (size_t i=0; i<m_children.size(); ++i)
	{
		if (m_children[i]->isA(CC_TYPES::POINT_OCTREE))
			return static_cast<ccOctreeProxy*>(m_children[i]);
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
		return ccOctree::Shared(0);
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

CCLib::ReferenceCloud* ccGenericPointCloud::getTheVisiblePoints(VisibilityTableType* visTable/*=0*/) const
{
	if (!visTable)
	{
		visTable = m_pointsVisibility;
	}

	unsigned count = size();
	if (!visTable || visTable->currentSize() != count)
	{
		assert(false);
		ccLog::Warning("[ccGenericPointCloud::getTheVisiblePoints] No visibility table instantiated!");
		return 0;
	}

	//count the number of points to copy
	unsigned pointCount = 0;
	{
		for (unsigned i=0; i<count; ++i)
			if (visTable->getValue(i) == POINT_VISIBLE)
				++pointCount;
	}

	if (pointCount == 0)
	{
		ccLog::Warning("[ccGenericPointCloud::getTheVisiblePoints] No point in selection");
		return 0;
	}

	//we create an entity with the 'visible' vertices only
	CCLib::ReferenceCloud* rc = new CCLib::ReferenceCloud(const_cast<ccGenericPointCloud*>(this));
	if (rc->reserve(pointCount))
	{
		for (unsigned i=0; i<count; ++i)
			if (visTable->getValue(i) == POINT_VISIBLE)
				rc->addPointIndex(i); //can't fail (see above)
	}
	else
	{
		delete rc;
		rc = 0;
		ccLog::Error("[ccGenericPointCloud::getTheVisiblePoints] Not enough memory!");
	}

	return rc;
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
		assert(m_pointsVisibility);
		if (!ccSerializationHelper::GenericArrayToFile(*m_pointsVisibility,out))
			return false;
	}

	//'point size' (dataVersion>=24)
	if (out.write((const char*)&m_pointSize,1) < 0)
		return WriteError();

	return true;
}

bool ccGenericPointCloud::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags))
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
		if (in.read((char*)&m_pointSize,1) < 0)
			return WriteError();
	}
	else
	{
		m_pointSize = 0; //= follows default setting
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
			ccProgressDialog pDlg(false, getDisplay() ? getDisplay()->asWidget() : 0);
			octree = computeOctree(&pDlg);
		}

		if (octree)
		{
			//we can now use the octree to do faster point picking
#ifdef QT_DEBUG
			CCLib::ScalarField* sf = 0;
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
		CCVector3d X(0,0,0);
		if (!camera.unproject(clickPosd, X))
		{
			return false;
		}

		//warning: we have to handle the relative GL transformation!
		ccGLMatrix trans;
		bool noGLTrans = !getAbsoluteGLTransformation(trans);

		//visibility table (if any)
		const ccGenericPointCloud::VisibilityTableType* visTable = isVisibilityTableInstantiated() ? getTheVisibilityArray() : 0;

		//scalar field with hidden values (if any)
		ccScalarField* activeSF = 0;
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

#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i=0; i<static_cast<int>(size()); ++i)
		{
			//we shouldn't test points that are actually hidden!
			if (	(!visTable || visTable->getValue(i) == POINT_VISIBLE)
				&&	(!activeSF || activeSF->getColor(activeSF->getValue(i)))
				)
			{
				const CCVector3* P = getPoint(i);

				CCVector3d Q2D;
				if (noGLTrans)
				{
					camera.project(*P, Q2D);
				}
				else
				{
					CCVector3 P3D = *P;
					trans.apply(P3D);
					camera.project(P3D, Q2D);
				}

				if (	fabs(Q2D.x-clickPos.x) <= pickWidth
					&&	fabs(Q2D.y-clickPos.y) <= pickHeight)
				{
					double squareDist = CCVector3d(X.x-P->x, X.y-P->y, X.z-P->z).norm2d();
					if (nearestPointIndex < 0 || squareDist < nearestSquareDist)
					{
						nearestSquareDist = squareDist;
						nearestPointIndex = static_cast<int>(i);
					}
				}
			}
		}
	}

	return (nearestPointIndex >= 0);
}
