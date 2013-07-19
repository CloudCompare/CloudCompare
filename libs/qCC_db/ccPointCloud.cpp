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

//Always first
#include "ccIncludeGL.h"

#include "ccPointCloud.h"

//CCLib
#include <ManualSegmentationTools.h>
#include <GeometricalAnalysisTools.h>
#include <ReferenceCloud.h>

#include "ccNormalVectors.h"
#include "ccColorScalesManager.h"
#include "ccOctree.h"
#include "ccKdTree.h"
#include "ccGenericMesh.h"
#include "ccMesh.h"
#include "ccMeshGroup.h"
#include "ccImage.h"
#include "cc2DLabel.h"
#include "ccGLUtils.h"
#include "ccColorRampShader.h"

//system
#include <assert.h>

ccPointCloud::ccPointCloud(QString name) throw()
	: ChunkedPointCloud()
	, ccGenericPointCloud(name)
	, m_rgbColors(0)
	, m_normals(0)
	, m_currentDisplayedScalarField(0)
	, m_currentDisplayedScalarFieldIndex(-1)
{
	init();
}

ccPointCloud* ccPointCloud::From(CCLib::GenericCloud* cloud)
{
	ccPointCloud* pc = new ccPointCloud("Cloud");

	unsigned n = cloud->size();
	if (n == 0)
	{
		ccLog::Warning("[ccPointCloud::From] Input cloud is empty!");
	}
	else
	{
		if (!pc->reserveThePointsTable(n))
		{
			ccLog::Error("[ccPointCloud::From] Not enough memory to duplicate cloud!");
			delete pc;
			pc = 0;
		}
		else
		{
			//import points
			cloud->placeIteratorAtBegining();
			for (unsigned i=0; i<n; i++)
				pc->addPoint(*cloud->getNextPoint());
		}
	}

	return pc;
}

ccPointCloud* ccPointCloud::From(const CCLib::GenericIndexedCloud* cloud)
{
	ccPointCloud* pc = new ccPointCloud("Cloud");

	unsigned n = cloud->size();
	if (n == 0)
	{
		ccLog::Warning("[ccPointCloud::From] Input cloud is empty!");
	}
	else
	{
		if (!pc->reserveThePointsTable(n))
		{
			ccLog::Error("[ccPointCloud] Not enough memory to duplicate cloud!");
			delete pc;
			pc = 0;
		}
		else
		{
			//import points
			for (unsigned i=0; i<n; i++)
			{
				CCVector3 P;
				cloud->getPoint(i,P);
				pc->addPoint(P);
			}
		}
	}

	return pc;
}

ccPointCloud* ccPointCloud::partialClone(const CCLib::ReferenceCloud* selection, int* warnings/*=0*/) const
{
	if (warnings)
		*warnings = 0;

	if (!selection || selection->getAssociatedCloud() != static_cast<const GenericIndexedCloud*>(this))
	{
		ccLog::Error("[ccPointCloud::partialClone] Invalid parameters");
		return 0;
	}

	unsigned n = selection->size();
	if (n == 0)
	{
		ccLog::Warning("[ccPointCloud::partialClone] Selection is empty");
		return 0;
	}

	ccPointCloud* result = new ccPointCloud(getName()+QString(".extract"));

	if (!result->reserveThePointsTable(n))
	{
		ccLog::Error("[ccPointCloud::partialClone] Not enough memory to duplicate cloud!");
		delete result;
		return 0;
	}

	//import points
	{
		for (unsigned i=0; i<n; i++)
			result->addPoint(*getPointPersistentPtr(selection->getPointGlobalIndex(i)));
	}

	//visibility
	result->setVisible(isVisible());
	result->setDisplay(getDisplay());

	//RGB colors
	if (hasColors())
	{
		if (result->reserveTheRGBTable())
		{
			for (unsigned i=0; i<n; i++)
				result->addRGBColor(getPointColor(selection->getPointGlobalIndex(i)));
			result->showColors(colorsShown());
		}
		else
		{
			ccLog::Warning("[ccPointCloud::partialClone] Not enough memory to copy RGB colors!");
			if (warnings)
				*warnings |= WRN_OUT_OF_MEM_FOR_COLORS;
		}
	}

	//normals
	if (hasNormals())
	{
		if (result->reserveTheNormsTable())
		{
			for (unsigned i=0; i<n; i++)
				result->addNormIndex(getPointNormalIndex(selection->getPointGlobalIndex(i)));
			result->showNormals(normalsShown());
		}
		else
		{
			ccLog::Warning("[ccPointCloud::partialClone] Not enough memory to copy normals!");
			if (warnings)
				*warnings |= WRN_OUT_OF_MEM_FOR_NORMALS;
		}
	}

	//scalar fields
	unsigned sfCount = getNumberOfScalarFields();
	if (sfCount != 0)
	{
		for (unsigned k=0; k<sfCount; ++k)
		{
			const ccScalarField* sf = static_cast<ccScalarField*>(getScalarField(k));
			assert(sf);
			if (sf)
			{
				//we create a new scalar field with same name
				int sfIdx = result->addScalarField(sf->getName());
				if (sfIdx>=0) //success
				{
					ccScalarField* currentScalarField = static_cast<ccScalarField*>(result->getScalarField(sfIdx));
					assert(currentScalarField);
					if (currentScalarField->resize(n))
					{
						//we copy data to new SF
						for (unsigned i=0; i<n; i++)
							currentScalarField->setValue(i,sf->getValue(selection->getPointGlobalIndex(i)));

						currentScalarField->computeMinAndMax();
						//copy color ramp parameters
						currentScalarField->setColorRampSteps(sf->getColorRampSteps());
						currentScalarField->setColorScale(sf->getColorScale());
						currentScalarField->showNaNValuesInGrey(sf->areNaNValuesShownInGrey());
						currentScalarField->setLogScale(sf->logScale());
						currentScalarField->setSymmetricalScale(sf->symmetricalScale());
						currentScalarField->alwaysShowZero(sf->isZeroAlwaysShown());
						currentScalarField->setMinDisplayed(sf->displayRange().start());
						currentScalarField->setMaxDisplayed(sf->displayRange().stop());
						currentScalarField->setSaturationStart(sf->saturationRange().start());
						currentScalarField->setSaturationStop(sf->saturationRange().stop());
					}
					else
					{
						//if we don't have enough memory, we cancel SF creation
						result->deleteScalarField(sfIdx);
						ccLog::Warning(QString("[ccPointCloud::partialClone] Not enough memory to copy scalar field '%1'!").arg(sf->getName()));
						if (warnings)
							*warnings |= WRN_OUT_OF_MEM_FOR_SFS;
					}
				}
			}
		}

		unsigned copiedSFCount = getNumberOfScalarFields();
		if (copiedSFCount)
		{
			//we display the same scalar field as the source (if we managed to copy it!)
			if (getCurrentDisplayedScalarField())
			{
				int sfIdx = result->getScalarFieldIndexByName(getCurrentDisplayedScalarField()->getName());
				if (sfIdx)
					result->setCurrentDisplayedScalarField(sfIdx);
				else
					result->setCurrentDisplayedScalarField((int)copiedSFCount-1);
			}
			//copy visibility
			result->showSF(sfShown());
		}
	}

	//Meshes //TODO
	/*Lib::GenericIndexedMesh* theMesh = source->_getMesh();
	if (theMesh)
	{
	//REVOIR --> on pourrait le faire pour chaque sous-mesh non ?
	CCLib::GenericIndexedMesh* newTri = CCLib::ManualSegmentationTools::segmentMesh(theMesh,selection,true,NULL,this);
	setMesh(newTri);
	if (source->areMeshesDisplayed()) showTri();
	}

	//PoV & Scanners
	bool importScanners = true;
	if (source->isMultipleScansModeActivated())
	if (activateMultipleScansMode())
	{
	scanIndexesTableType* _theScans = source->getTheScansIndexesArray();
	for (i=0;i<n;++i) cubeVertexesIndexes.setValue(i,_theScans->getValue(i));
	}
	else importScanners=false;

	if (importScanners)
	{
	//on insere les objets "capteur" (pas de copie ici, la même instance peut-être partagee par plusieurs listes)
	for (i=1;i<=source->getNumberOfSensors();++i)
	setSensor(source->_getSensor(i),i);
	}
	*/

	//original center
	const double* shift = getOriginalShift();
	if (shift)
		result->setOriginalShift(shift[0],shift[1],shift[2]);

	//custom point size
	result->setPointSize(getPointSize());

	return result;
}

ccPointCloud::~ccPointCloud()
{
	clear();
}

void ccPointCloud::init() throw()
{
	showSFColorsScale(false);
	setCurrentDisplayedScalarField(-1);
	showSF(false);
}

void ccPointCloud::clear()
{
	ChunkedPointCloud::clear();
	ccGenericPointCloud::clear();

	showSFColorsScale(false);
	unallocateColors();
	unallocateNorms();
	enableTempColor(false);

	updateModificationTime();
}

ccGenericPointCloud* ccPointCloud::clone(ccGenericPointCloud* destCloud/*=0*/)
{
	if (destCloud)
	{
		if (destCloud->isA(CC_POINT_CLOUD))
		{
			return cloneThis(static_cast<ccPointCloud*>(destCloud));
		}
		else
		{
			ccLog::Error("[ccPointCloud::clone] Invalid destination cloud provided! Not a ccPointCloud...");
			return 0;
		}
	}

	return cloneThis();
}

ccPointCloud* ccPointCloud::cloneThis(ccPointCloud* destCloud/*=0*/)
{
	ccPointCloud* result = destCloud ? destCloud : new ccPointCloud();

	result->setVisible(isVisible());

	result->append(this,0); //there was (virtually) no point before

	result->showColors(colorsShown());
	result->showSF(sfShown());
	result->showNormals(normalsShown());
	result->setEnabled(isEnabled());

	result->setCurrentDisplayedScalarField(getCurrentDisplayedScalarFieldIndex());

	//original shift
	const double* shift = getOriginalShift();
	result->setOriginalShift(shift[0],shift[1],shift[2]);

	result->setName(getName()+QString(".clone"));

	return result;
}

const ccPointCloud& ccPointCloud::operator +=(ccPointCloud* addedCloud)
{
	if (isLocked())
	{
		ccLog::Error("[ccPointCloud::fusion] Cloud is locked");
		return *this;
	}

	return append(addedCloud,size());
}

const ccPointCloud& ccPointCloud::append(ccPointCloud* addedCloud, unsigned pointCountBefore)
{
	assert(addedCloud);

	unsigned addedPoints = addedCloud->size();

	if (!reserve(pointCountBefore+addedPoints))
	{
		ccLog::Error("[ccPointCloud::append] Not enough memory!");
		return *this;
	}

	//fuse display parameters
	setVisible(isVisible() || addedCloud->isVisible());

	//3D points (already reserved)
	if (size() == pointCountBefore) //in some cases points have already been copied! (ok it's tricky)
	{
		//we remove structures that are not compatible with fusion process
		deleteOctree();
		unallocateVisibilityArray();

		for (unsigned i=0;i<addedPoints;i++)
			addPoint(*addedCloud->getPoint(i));
	}

	//deprecate internal structures
	updateModificationTime();

	//Colors (already reserved)
	if (hasColors() || addedCloud->hasColors())
	{
		//fuse display parameters
		showColors(colorsShown() || addedCloud->colorsShown());

		//if the added cloud has no color
		if (!addedCloud->hasColors())
		{
			//we set a white color to new points
			for (unsigned i=0;i<addedPoints;i++)
				addRGBColor(ccColor::white);
		}
		else //otherwise
		{
			//if this cloud hadn't any color before
			if (!hasColors())
			{
				//we try to resrve a new array
				if (reserveTheRGBTable())
				{
					for (unsigned i=0;i<pointCountBefore;i++)
						addRGBColor(ccColor::white);
				}
				else
				{
					ccLog::Warning("[ccPointCloud::fusion] Not enough memory: failed to allocate colors!");
					showColors(false);
				}
			}

			//we import colors (if necessary)
			if (hasColors() && m_rgbColors->currentSize() == pointCountBefore)
				for (unsigned i=0;i<addedPoints;i++)
					addRGBColor(addedCloud->m_rgbColors->getValue(i));
		}
	}

	//normales (reserved)
	if (hasNormals() || addedCloud->hasNormals())
	{
		//fuse display parameters
		showNormals(normalsShown() || addedCloud->normalsShown());

		//if the added cloud hasn't any normal
		if (!addedCloud->hasNormals())
		{
			//we associate imported points with '0' normals
			for (unsigned i=0;i<addedPoints;i++)
				addNormIndex(0);
		}
		else //otherwise
		{
			//if this cloud hasn't any normal
			if (!hasNormals())
			{
				//we try to resrve a new array
				if (reserveTheNormsTable())
				{
					for (unsigned i=0;i<pointCountBefore;i++)
						addNorm(0);
				}
				else
				{
					ccLog::Warning("[ccPointCloud::fusion] Not enough memory: failed to allocate normals!");
					showNormals(false);
				}
			}

			//we import normals (if necessary)
			if (hasNormals() && m_normals->currentSize() == pointCountBefore)
				for (unsigned i=0;i<addedPoints;i++)
					addNormIndex(addedCloud->m_normals->getValue(i));
		}
	}

	//scalar fields (resized)
	unsigned sfCount = getNumberOfScalarFields();
	unsigned newSFCount = addedCloud->getNumberOfScalarFields();
	if (sfCount!=0 || newSFCount!=0)
	{
		std::vector<bool> sfUpdated(sfCount, false);

		//first we fuse the new SF with the existing one
		for (unsigned k=0;k<newSFCount;++k)
		{
			const CCLib::ScalarField* sf = addedCloud->getScalarField((int)k);
			if (sf)
			{
				//does this field already exist (same name)?
				int sfIdx = getScalarFieldIndexByName(sf->getName());
				if (sfIdx>=0) //yes
				{
					CCLib::ScalarField* sameSF = getScalarField(sfIdx);
					assert(sameSF && sameSF->capacity()>=pointCountBefore+addedPoints);
					//we fill it with new values (it should have been already 'reserved' (if necessary)
					if (sameSF->currentSize() == pointCountBefore)
						for (unsigned i=0; i<addedPoints; i++)
							sameSF->addElement(sf->getValue(i));
					sameSF->computeMinAndMax();

					//flag this SF as 'updated'
					assert(sfIdx<(int)sfCount);
					sfUpdated[sfIdx]=true;
				}
				else //otherwise we create a new SF
				{
					ccScalarField* newSF = new ccScalarField(sf->getName());
					//we fill the begining with NaN (as there is no equivalent in the current cloud)
					if (newSF->resize(pointCountBefore+addedPoints,true,NAN_VALUE))
					{
						//we copy the new values
						for (unsigned i=0; i<addedPoints; i++)
							newSF->setValue(pointCountBefore+i,sf->getValue(i));
						newSF->computeMinAndMax();

						//add scalar field to this cloud
						sfIdx = addScalarField(newSF);
						assert(sfIdx>=0);
					}
					else
					{
						newSF->release();
						newSF=0;
						ccLog::Warning("[ccPointCloud::fusion] Not enough memory: failed to allocate a copy of scalar field '%s'",sf->getName());
					}
				}
			}
		}

		//let's check if there are non-updated fields
		for (unsigned j=0;j<sfCount;++j)
		{
			if (!sfUpdated[j])
			{
				CCLib::ScalarField* sf = getScalarField(j);
				assert(sf);

				if (sf->currentSize() == pointCountBefore)
				{
					//we fill the end with NaN (as there is no equivalent in the added cloud)
					ScalarType NaN = sf->NaN();
					for (unsigned i=0; i<addedPoints; i++)
						sf->addElement(NaN);
				}
			}
		}

		//in case something bad happened
		if (getNumberOfScalarFields()==0)
		{
			setCurrentDisplayedScalarField(-1);
			showSF(false);
		}
		else
		{
			//if there was no scalar field before
			if (sfCount==0)
			{
				//and if the added cloud has one displayed
				const ccScalarField* dispSF = addedCloud->getCurrentDisplayedScalarField();
				if (dispSF)
				{
					//we set it as displayed on the current cloud also
					int sfIdx = getScalarFieldIndexByName(dispSF->getName()); //same name!
					setCurrentDisplayedScalarField(sfIdx);
				}
			}

			//fuse display parameters
			showSF(sfShown() || addedCloud->sfShown());
		}
	}

	//Has the cloud been recentered?
	const double* shift = addedCloud->getOriginalShift();
	if (fabs(shift[0])+fabs(shift[1])+fabs(shift[1])>0.0)
		ccLog::Warning(QString("[ccPointCloud::fusion] Global shift information for cloud '%1' will be lost!").arg(addedCloud->getName()));

	//children (not yet reserved)
	unsigned c,childrenCount = addedCloud->getChildrenNumber();
	for (c=0;c<childrenCount;++c)
	{
		ccHObject* child = addedCloud->getChild(c);
		if (child->isKindOf(CC_MESH)) //mesh
		{
			ccGenericMesh* mesh = static_cast<ccGenericMesh*>(child);

			//detach from father?
			//mesh->setFlagState(CC_FATHER_DEPENDANT,false);
			//addedCloud->removeChild(mesh);
			//ccGenericMesh* addedTri = mesh;

			//or clone?
			ccGenericMesh* cloneMesh = mesh->clone(mesh->getAssociatedCloud()==addedCloud ? this : 0);
			if (cloneMesh)
			{
				//change mesh vertices
				if (cloneMesh->getAssociatedCloud() == this)
					cloneMesh->shiftTriangleIndexes(pointCountBefore);
				addChild(cloneMesh,true);
			}
			else
			{
				ccLog::Warning(QString("[ccPointCloud::fusion] Not enough memory: failed to clone sub mesh %1!").arg(mesh->getName()));
			}
		}
		else if (child->isKindOf(CC_IMAGE))
		{
			//ccImage* image = static_cast<ccImage*>(child);

			//DGM FIXME: take image ownership! (dirty)
			child->setFlagState(CC_FATHER_DEPENDANT,false);
			addedCloud->removeChild(child);
			addChild(child,true);
		}
		else if (child->isA(CC_2D_LABEL))
		{
			//clone label and update points if necessary
			cc2DLabel* label = static_cast<cc2DLabel*>(child);
			cc2DLabel* newLabel = new cc2DLabel(label->getName());
			for (unsigned j=0;j<label->size();++j)
			{
				const cc2DLabel::PickedPoint& P = label->getPoint(j);
				if (P.cloud == addedCloud)
					newLabel->addPoint(this,pointCountBefore+P.index);
				else
					newLabel->addPoint(P.cloud,P.index);
			}
			newLabel->setDisplayedIn3D(label->isDisplayedIn3D());
			newLabel->setDisplayedIn2D(label->isDisplayedIn2D());
			newLabel->setCollapsed(label->isCollapsed());
			newLabel->setPosition(label->getPosition()[0],label->getPosition()[1]);
			newLabel->setVisible(label->isVisible());
			newLabel->setDisplay(getDisplay());
			addChild(newLabel,true);
		}
	}

	return *this;
}

void ccPointCloud::unallocateNorms()
{
	if (m_normals)
		m_normals->release();
	m_normals=0;

	showNormals(false);
}

void ccPointCloud::unallocateColors()
{
	if (m_rgbColors)
		m_rgbColors->release();
	m_rgbColors=0;

	showColors(false);
	enableTempColor(false);
}

bool ccPointCloud::reserveThePointsTable(unsigned newNumberOfPoints)
{
	return m_points->reserve(newNumberOfPoints);
}

bool ccPointCloud::reserveTheRGBTable()
{
	assert(m_points);
	if (!m_points->isAllocated())
	{
		ccLog::Error("[ccPointCloud::reserveTheRGBTable] Internal error: properties (re)allocation before points allocation is forbidden!");
		return false;
	}

	if (!m_rgbColors)
	{
		m_rgbColors = new ColorsTableType();
		m_rgbColors->link();
	}

	if (!m_rgbColors->reserve(m_points->capacity()))
	{
		m_rgbColors->release();
		m_rgbColors = 0;

		ccLog::Error("[ccPointCloud::reserveTheRGBTable] Not enough memory!");
		return false;
	}

	return true;
}

bool ccPointCloud::resizeTheRGBTable(bool fillWithWhite/*=false*/)
{
	assert(m_points);
	if (!m_points->isAllocated())
	{
		ccLog::Error("[ccPointCloud::resizeTheRGBTable] Internal error: properties (re)allocation before points allocation is forbidden!");
		return false;
	}

	if (!m_rgbColors)
	{
		m_rgbColors = new ColorsTableType();
		m_rgbColors->link();
	}

	if (!m_rgbColors->resize(m_points->currentSize(), fillWithWhite, fillWithWhite ? ccColor::white : 0))
	{
		m_rgbColors->release();
		m_rgbColors = 0;

		ccLog::Error("[ccPointCloud::resizeTheRGBTable] Not enough memory!");
		return false;
	}

	return true;
}

bool ccPointCloud::reserveTheNormsTable()
{
	assert(m_points);
	if (!m_points->isAllocated())
	{
		ccLog::Error("[ccPointCloud::reserveTheNormsTable] Internal error: properties (re)allocation before points allocation is forbidden!");
		return false;
	}

	if (!m_normals)
	{
		m_normals = new NormsIndexesTableType();
		m_normals->link();
	}

	if (!m_normals->reserve(m_points->capacity()))
	{
		m_normals->release();
		m_normals = 0;

		ccLog::Error("[ccPointCloud::reserveTheNormsTable] Not enough memory!");
		return false;
	}

	return true;
}

bool ccPointCloud::resizeTheNormsTable()
{
	if (!m_points->isAllocated())
	{
		ccLog::Error("[ccPointCloud::resizeTheNormsTable] Internal error: properties (re)allocation before points allocation is forbidden!");
		return false;
	}

	if (!m_normals)
	{
		m_normals = new NormsIndexesTableType();
		m_normals->link();
	}

	if (!m_normals->resize(m_points->currentSize(),true,0))
	{
		m_normals->release();
		m_normals = 0;

		ccLog::Error("[ccPointCloud::resizeTheNormsTable] Not enough memory!");
		return false;
	}

	return true;
}

bool ccPointCloud::reserve(unsigned newNumberOfPoints)
{
	//reserve works only to enlarge the cloud
	if (newNumberOfPoints < size())
		return false;

	//call parent method first (for points + scalar fields)
	if (!ChunkedPointCloud::reserve(newNumberOfPoints))
	{
		ccLog::Error("[ccPointCloud::reserve] Not enough memory!");
		return false;
	}

	if (hasColors() && !reserveTheRGBTable()) //colors
	{
		ccLog::Error("[ccPointCloud::reserve] Not enough memory!");
		return false;
	}

	if (hasNormals() && !reserveTheNormsTable()) //normals
	{
		ccLog::Error("[ccPointCloud::reserve] Not enough memory!");
		return false;
	}

	return true;
}

bool ccPointCloud::resize(unsigned newNumberOfPoints)
{
	//can't reduce the size if the cloud is locked!
	if (newNumberOfPoints < size() && isLocked())
		return false;

	//call parent method first (for points + scalar fields)
	if (!ChunkedPointCloud::resize(newNumberOfPoints))
	{
		ccLog::Error("[ccPointCloud::resize] Not enough memory!");
		return false;
	}

	updateModificationTime();

	if (hasColors() && !resizeTheRGBTable(false)) //colors
	{
		ccLog::Error("[ccPointCloud::resize] Not enough memory!");
		return false;
	}

	if (hasNormals() && !resizeTheNormsTable()) //normals
	{
		ccLog::Error("[ccPointCloud::resize] Not enough memory!");
		return false;
	}

	return true;
}

void ccPointCloud::showSFColorsScale(bool state)
{
	m_sfColorScaleDisplayed = state;
}

bool ccPointCloud::sfColorScaleShown() const
{
	return m_sfColorScaleDisplayed;
}

const colorType* ccPointCloud::getPointScalarValueColor(unsigned pointIndex) const
{
	assert(m_currentDisplayedScalarField && m_currentDisplayedScalarField->getColorScale());

	return m_currentDisplayedScalarField->getValueColor(pointIndex);
}

const colorType* ccPointCloud::geScalarValueColor(ScalarType d) const
{
	assert(m_currentDisplayedScalarField && m_currentDisplayedScalarField->getColorScale());

	return m_currentDisplayedScalarField->getColor(d);
}

ScalarType ccPointCloud::getPointDisplayedDistance(unsigned pointIndex) const
{
	assert(m_currentDisplayedScalarField);
	assert(pointIndex<m_currentDisplayedScalarField->currentSize());

	return m_currentDisplayedScalarField->getValue(pointIndex);
}

const colorType* ccPointCloud::getPointColor(unsigned pointIndex) const
{
	assert(hasColors());
	assert(pointIndex<m_rgbColors->currentSize());

	return m_rgbColors->getValue(pointIndex);
}

const normsType& ccPointCloud::getPointNormalIndex(unsigned pointIndex) const
{
	assert(m_normals && pointIndex<m_normals->currentSize());

	return m_normals->getValue(pointIndex);
}

const PointCoordinateType* ccPointCloud::getPointNormal(unsigned pointIndex) const
{
	assert(m_normals && pointIndex<m_normals->currentSize());

	return ccNormalVectors::GetNormal(m_normals->getValue(pointIndex));
}

void ccPointCloud::setPointColor(unsigned pointIndex, const colorType* col)
{
	assert(m_rgbColors && pointIndex<m_rgbColors->currentSize());

	m_rgbColors->setValue(pointIndex, col);
}

void ccPointCloud::setPointNormalIndex(unsigned pointIndex, normsType norm)
{
	assert(m_normals && pointIndex<m_normals->currentSize());

	m_normals->setValue(pointIndex, norm);
}

void ccPointCloud::setPointNormal(unsigned pointIndex, const PointCoordinateType* N)
{
	setPointNormalIndex(pointIndex, ccNormalVectors::GetNormIndex(N));
}

bool ccPointCloud::hasColors() const
{
	return m_rgbColors && m_rgbColors->isAllocated();
}

bool ccPointCloud::hasNormals() const
{
	return m_normals && m_normals->isAllocated();
}

bool ccPointCloud::hasScalarFields() const
{
	return (getNumberOfScalarFields()>0);
}

bool ccPointCloud::hasDisplayedScalarField() const
{
	return m_currentDisplayedScalarField && m_currentDisplayedScalarField->getColorScale();
}

/*bool ccPointCloud::isScalarFieldEnabled() const
{
//shortcut for CCLib
return hasDisplayedScalarField();
}
//*/

void ccPointCloud::refreshBB()
{
	invalidateBoundingBox();
	updateModificationTime();
}

void ccPointCloud::addGreyColor(colorType g)
{
	assert(m_rgbColors && m_rgbColors->isAllocated());
	const colorType G[3]={g,g,g};
	m_rgbColors->addElement(G);
}

void ccPointCloud::addRGBColor(const colorType* C)
{
	assert(m_rgbColors && m_rgbColors->isAllocated());
	m_rgbColors->addElement(C);
}

void ccPointCloud::addRGBColor(colorType r, colorType g, colorType b)
{
	assert(m_rgbColors && m_rgbColors->isAllocated());
	const colorType C[3]={r,g,b};
	m_rgbColors->addElement(C);
}

void ccPointCloud::addNorm(PointCoordinateType Nx, PointCoordinateType Ny, PointCoordinateType Nz)
{
	PointCoordinateType N[3]={Nx,Ny,Nz};
	addNorm(N);
}

void ccPointCloud::addNorm(const PointCoordinateType* N)
{
	addNormIndex(ccNormalVectors::GetNormIndex(N));
}

void ccPointCloud::addNormIndex(normsType index)
{
	assert(m_normals && m_normals->isAllocated());
	m_normals->addElement(index);
}

void ccPointCloud::addNormAtIndex(const PointCoordinateType* N, unsigned index)
{
	assert(m_normals && m_normals->isAllocated());
	//we get the real normal vector corresponding to current index
	CCVector3 P(ccNormalVectors::GetNormal(m_normals->getValue(index)));
	//we add the provided vector (N)
	CCVector3::vadd(P.u,N,P.u);
	P.normalize();
	//we recode the resulting vector
	normsType nIndex = ccNormalVectors::GetNormIndex(P.u);
	m_normals->setValue(index,nIndex);
}

bool ccPointCloud::convertNormalToRGB()
{
	if (!hasNormals())
		return false;

	if (!ccNormalVectors::GetUniqueInstance()->enableNormalHSVColorsArray())
		return false;
	const colorType* normalHSV = ccNormalVectors::GetUniqueInstance()->getNormalHSVColorArray();

	if (!resizeTheRGBTable(false))
		return false;
	assert(m_normals && m_rgbColors);

	unsigned i,count=size();
	for (i=0;i<count;++i)
	{
		const colorType* rgb = normalHSV+3*m_normals->getValue(i);
		m_rgbColors->setValue(i,rgb);
	}

	//showColors(true);
	return true;
}

void ccPointCloud::setNormsTable(NormsIndexesTableType* norms)
{
	if (m_normals == norms)
		return;

	if (m_normals)
		m_normals->release();

	m_normals = norms;
	if (m_normals)
		m_normals->link();
}

bool ccPointCloud::colorize(float r, float g, float b)
{
	assert(r >= 0.0f && r <= 1.0f);
	assert(g >= 0.0f && g <= 1.0f);
	assert(b >= 0.0f && b <= 1.0f);

	if (hasColors())
	{
		m_rgbColors->placeIteratorAtBegining();
		for (unsigned i=0;i<m_rgbColors->currentSize();i++)
		{
			colorType* p = m_rgbColors->getCurrentValue();
			{
				p[0] = static_cast<colorType>(static_cast<float>(p[0]) * r);
				p[1] = static_cast<colorType>(static_cast<float>(p[1]) * g);
				p[2] = static_cast<colorType>(static_cast<float>(p[2]) * b);
			}
			m_rgbColors->forwardIterator();
		}
	}
	else
	{
		if (!resizeTheRGBTable(false))
			return false;

		colorType RGB[3] = {	static_cast<colorType>(static_cast<float>(MAX_COLOR_COMP) * r) ,
			static_cast<colorType>(static_cast<float>(MAX_COLOR_COMP) * g) ,
			static_cast<colorType>(static_cast<float>(MAX_COLOR_COMP) * b) };
		m_rgbColors->fill(RGB);
	}

	return true;
}

bool ccPointCloud::setRGBColorByHeight(unsigned char heightDim, ccColorScale::Shared colorScale)
{
	if (!colorScale || heightDim > 2) //X=0, Y=1, Z=2
	{
		ccLog::Error("[ccPointCloud::colorizeWithDefaultRamp] Invalid paramter!");
		return false;
	}

	//allocate colors if necessary
	if (!hasColors())
		if (!resizeTheRGBTable(false))
			return false;

	enableTempColor(false);

	PointCoordinateType bbMin[3],bbMax[3];
	getBoundingBox(bbMin,bbMax);
	const PointCoordinateType& dMin = bbMin[heightDim];
	PointCoordinateType height = bbMax[heightDim]-dMin;

	if (fabs(height) < ZERO_TOLERANCE) //flat cloud!
	{
		return setRGBColor(colorScale->getColorByIndex(0));
	}

	unsigned count = size();
	for (unsigned i=0; i<count; i++)
	{
		const CCVector3* Q = getPoint(i);
		double realtivePos = (double)(Q->u[heightDim]-dMin) / (double)height;

		m_rgbColors->setValue(i,colorScale->getColorByRelativePos(realtivePos));
	}

	return true;
}

bool ccPointCloud::setRGBColor(colorType r, colorType g, colorType b)
{
	colorType c[3] = {r,g,b};
	return setRGBColor(c);
}

bool ccPointCloud::setRGBColor(const colorType* col)
{
	enableTempColor(false);

	//allocate colors if necessary
	if (!hasColors())
		if (!reserveTheRGBTable())
			return false;

	m_rgbColors->fill(col);

	return true;
}

CCVector3 ccPointCloud::computeGravityCenter()
{
	return CCLib::GeometricalAnalysisTools::computeGravityCenter(this);
}

void ccPointCloud::applyGLTransformation(const ccGLMatrix& trans)
{
	return applyRigidTransformation(trans);
}

void ccPointCloud::applyRigidTransformation(const ccGLMatrix& trans)
{
	unsigned i,count=size();
	for (i=0;i<count;i++)
		trans.apply(*point(i));

	//we must also take care of the normals!
	if (hasNormals())
	{
		bool recoded = false;

		//if there is more points than the size of the compressed normals array,
		//we recompress the array instead of recompressing each normal
		if (count>ccNormalVectors::GetNumberOfVectors())
		{
			NormsIndexesTableType* newNorms = new NormsIndexesTableType;
			if (newNorms->reserve(ccNormalVectors::GetNumberOfVectors()))
			{
				for (i=0;i<ccNormalVectors::GetNumberOfVectors();i++)
				{
					CCVector3 new_n(ccNormalVectors::GetNormal(i));
					trans.applyRotation(new_n);
					normsType newNormIndex = ccNormalVectors::GetNormIndex(new_n.u);
					newNorms->addElement(newNormIndex);
				}

				m_normals->placeIteratorAtBegining();
				for (i=0;i<count;i++)
				{
					m_normals->setValue(i,newNorms->getValue(m_normals->getCurrentValue()));
					m_normals->forwardIterator();
				}
				recoded=true;
			}
			newNorms->clear();
			newNorms->release();
			newNorms=0;
		}

		//if there is less points than the compressed normals array size
		//(or if there is not enough memory to instantiate the temporary
		//array), we recompress each normal ...
		if (!recoded)
		{
			//on recode direct chaque normale
			m_normals->placeIteratorAtBegining();
			for (i=0;i<count;i++)
			{
				normsType* _theNormIndex = m_normals->getCurrentValuePtr();
				CCVector3 new_n(ccNormalVectors::GetNormal(*_theNormIndex));
				trans.applyRotation(new_n.u);
				*_theNormIndex = ccNormalVectors::GetNormIndex(new_n.u);
				m_normals->forwardIterator();
			}
		}
	}

	//the octree is invalidated by rotation...
	deleteOctree();

	// ... as the bounding box
	refreshBB();
}

void ccPointCloud::translate(const CCVector3& T)
{
	if (fabs(T.x)+fabs(T.y)+fabs(T.z) < ZERO_TOLERANCE)
		return;

	unsigned i,count=size();
	for (i=0;i<count;i++)
		*point(i) += T;

	updateModificationTime();

	//--> instead, we update BBox directly!
	PointCoordinateType* bbMin = m_points->getMin();
	PointCoordinateType* bbMax = m_points->getMax();
	CCVector3::vadd(bbMin,T.u,bbMin);
	CCVector3::vadd(bbMax,T.u,bbMax);

	//same thing for the octree
	ccOctree* oct = getOctree();
	if (oct)
		oct->translateBoundingBox(T);

	//and same thing for the Kd-tree(s)!
	ccHObject::Container kdtrees;
	filterChildren(kdtrees, false, CC_POINT_KDTREE);
	{
		for (size_t i=0; i<kdtrees.size(); ++i)
			static_cast<ccKdTree*>(kdtrees[i])->translateBoundingBox(T);
	}
}

void ccPointCloud::multiply(PointCoordinateType fx, PointCoordinateType fy, PointCoordinateType fz)
{
	unsigned i,count=size();
	for (i=0;i<count;i++)
	{
		CCVector3* P = point(i);
		P->x *= fx;
		P->y *= fy;
		P->z *= fz;
	}

	updateModificationTime();

	//refreshBB();
	//--> instead, we update BBox directly!
	PointCoordinateType* bbMin = m_points->getMin();
	PointCoordinateType* bbMax = m_points->getMax();
	bbMin[0] *= fx;
	bbMax[0] *= fx;
	if (fx<0.0)
		std::swap(bbMin[0],bbMax[0]);
	bbMin[1] *= fy;
	bbMax[1] *= fy;
	if (fy<0.0)
		std::swap(bbMin[1],bbMax[1]);
	bbMin[2] *= fz;
	bbMax[2] *= fz;
	if (fz<0.0)
		std::swap(bbMin[2],bbMax[2]);

	//same thing for the octree
	ccOctree* oct = getOctree();
	if (oct)
	{
		if (fx==fy && fx==fz && fx>0 && fy>0 && fz>0)
			oct->multiplyBoundingBox(fx);
		else
			deleteOctree();
	}

	//and same thing for the Kd-tree(s)!
	ccHObject::Container kdtrees;
	filterChildren(kdtrees, false, CC_POINT_KDTREE);
	{
		for (size_t i=0; i<kdtrees.size(); ++i)
			static_cast<ccKdTree*>(kdtrees[i])->multiplyBoundingBox(fx);
	}
}

void ccPointCloud::invertNormals()
{
	if (!hasNormals())
		return;

	m_normals->placeIteratorAtBegining();
	for (unsigned i=0;i<m_normals->currentSize();++i)
	{
		ccNormalVectors::InvertNormal(*m_normals->getCurrentValuePtr());
		m_normals->forwardIterator();
	}
}

void ccPointCloud::swapPoints(unsigned firstIndex, unsigned secondIndex)
{
	assert(!isLocked());

	if ((firstIndex==secondIndex)||(firstIndex>=size())||(secondIndex>=size()))
		return;

	//points + associated SF values
	ChunkedPointCloud::swapPoints(firstIndex,secondIndex);

	//colors
	if (hasColors())
		m_rgbColors->swap(firstIndex,secondIndex);

	//normals
	if (hasNormals())
		m_normals->swap(firstIndex,secondIndex);
}

void ccPointCloud::getDrawingParameters(glDrawParams& params) const
{
	//color override
	if (isColorOverriden())
	{
		params.showColors=true;
		params.showNorms=false;
		params.showSF=false;
	}
	else
	{
		//a scalar field must have been selected for display!
		params.showSF = hasDisplayedScalarField() && sfShown();
		params.showNorms = hasNormals() &&  normalsShown();
		//colors are not displayed if scalar field is displayed
		params.showColors = !params.showSF && hasColors() && colorsShown();
	}
}

//Vertex indexes for OpenGL "arrays" drawing
static PointCoordinateType s_normBuffer[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3];
static colorType s_rgbBuffer3ub[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3];
static float s_rgbBuffer3f[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3];

//helpers (for ColorRamp shader)

inline float GetNormalizedValue(const ScalarType& sfVal, const ccScalarField::Range& displayRange)
{
	return static_cast<float>((sfVal-displayRange.start())/displayRange.range());
}

inline float GetSymmetricalNormalizedValue(const ScalarType& sfVal, const ccScalarField::Range& saturationRange)
{
	//normalized sf value
	ScalarType relativeValue = 0;
	if (fabs(sfVal) > saturationRange.start()) //we avoid the 'flat' SF case by the way
	{
		if (sfVal < 0)
			relativeValue = (sfVal+saturationRange.start())/saturationRange.max();
		else
			relativeValue = (sfVal-saturationRange.start())/saturationRange.max();
	}
	return (1.0f+static_cast<float>(relativeValue))/2.0f;	//normalized sf value
}

void ccPointCloud::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!m_points->isAllocated())
		return;

	if (MACRO_Draw3D(context))
	{
		//we get display parameters
		glDrawParams glParams;
		getDrawingParameters(glParams);
		glParams.showNorms &= bool(MACRO_LightIsEnabled(context));

		//can't display a SF without... a SF... and an active color scale!
		assert(!glParams.showSF || (m_currentDisplayedScalarField && m_currentDisplayedScalarField->getColorScale()));

		//standard case: list names pushing
		bool pushName = MACRO_DrawEntityNames(context);
		//special case: point names pushing (for picking)
		bool pushPointNames = MACRO_DrawPointNames(context);
		pushName |= pushPointNames;

		if (pushName)
		{
			//not fast at all!
			if (MACRO_DrawFastNamesOnly(context))
				return;

			glPushName(getUniqueID());
			//minimal display for picking mode!
			glParams.showNorms = false;
			glParams.showColors = false;
			if (glParams.showSF && m_currentDisplayedScalarField->areNaNValuesShownInGrey())
				glParams.showSF = false; //--> we keep it only if SF 'NaN' values are potentially hidden
		}

		bool colorMaterialEnabled = false;

		if (glParams.showSF || glParams.showColors)
		{
			glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
			glEnable(GL_COLOR_MATERIAL);
			colorMaterialEnabled = true;
		}

		if (glParams.showColors && isColorOverriden())
		{
			glColor3ubv(m_tempColor);
			glParams.showColors=false;
		}
		else
		{
			glColor3ubv(context.pointsDefaultCol);
		}

		//in the case we need normals (i.e. lighting)
		ccNormalVectors* compressedNormals = 0;
		if (glParams.showNorms)
		{
			//DGM: Strangely, when Qt::renderPixmap is called, the OpenGL version is sometimes 1.0!
			glEnable((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_1_2 ? GL_RESCALE_NORMAL : GL_NORMALIZE));
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,	  CC_DEFAULT_CLOUD_AMBIENT_COLOR  );
			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  CC_DEFAULT_CLOUD_SPECULAR_COLOR );
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   CC_DEFAULT_CLOUD_DIFFUSE_COLOR  );
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  CC_DEFAULT_CLOUD_EMISSION_COLOR );
			glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, CC_DEFAULT_CLOUD_SHININESS);
			glEnable(GL_LIGHTING);

			if (glParams.showSF)
			{
				//we must get rid of lights 'color' if a scalar field is displayed!
				glPushAttrib(GL_LIGHTING_BIT);
				ccGLUtils::MakeLightsNeutral();
			}
			compressedNormals = ccNormalVectors::GetUniqueInstance();
		}

		// L.O.D.
		unsigned numberOfPoints = size();
		unsigned decimStep = 1;
		if (numberOfPoints>MAX_LOD_POINTS_NUMBER && context.decimateCloudOnMove &&  MACRO_LODActivated(context))
		{
			decimStep = int(ceil(float(numberOfPoints) / float(MAX_LOD_POINTS_NUMBER)));
		}

		/*** DISPLAY ***/

		//custom point size?
		glPushAttrib(GL_POINT_BIT);
		if (m_pointSize != 0)
			glPointSize((GLfloat)m_pointSize);

		if (!pushPointNames) //standard "full" display
		{
			//if some points are hidden (= visibility table instantiated), we can't use display arrays :(
			if (isVisibilityTableInstantiated())
			{
				glBegin(GL_POINTS);

				for (unsigned j=0;j<numberOfPoints;j+=decimStep)
				{
					//we must test each point visibility
					if (m_pointsVisibility->getValue(j) == POINT_VISIBLE)
					{
						if (glParams.showSF)
						{
							assert(j<m_currentDisplayedScalarField->currentSize());
							const colorType* col = m_currentDisplayedScalarField->getValueColor(j);
							//we force display of points hidden because of their scalar field value
							//to be sure that the user don't miss them (during manual segmentation for instance)
							glColor3ubv(col ? col : ccColor::lightGrey);
						}
						else if (glParams.showColors)
						{
							glColor3ubv(m_rgbColors->getValue(j));
						}
						if (glParams.showNorms)
						{
							glNormal3fv(compressedNormals->getNormal(m_normals->getValue(j)));
						}
						glVertex3fv(m_points->getValue(j));
					}
				}

				glEnd();
			}
			else if (glParams.showSF) //no visibility table enabled + scalar field
			{
				assert(m_currentDisplayedScalarField);
				const ccScalarField::Range& sfDisplayRange = m_currentDisplayedScalarField->displayRange();
				const ccScalarField::Range& sfSaturationRange = m_currentDisplayedScalarField->saturationRange();

				//the fact that NaN values SHOULD be hidden, doesn't mean that we ACTUALLY hide points...
				bool hiddenPoints = (	!m_currentDisplayedScalarField->areNaNValuesShownInGrey()
					&& ( sfDisplayRange.stop() <= sfDisplayRange.max() || sfDisplayRange.start() >= sfDisplayRange.min()) );

				//color ramp shader initialization
				ccColorRampShader* colorRampShader = context.colorRampShader;

				if (m_currentDisplayedScalarField->logScale())
				{
					//FIXME: shader doesn't support log scale yet!
					colorRampShader = 0;
				}

				if (colorRampShader)
				{
					//max available space for frament's shader uniforms
					GLint maxBytes=0;
					glGetIntegerv(GL_MAX_FRAGMENT_UNIFORM_COMPONENTS,&maxBytes);
					GLint maxComponents = (maxBytes>>2)-4; //leave space for the other uniforms!
					unsigned steps = m_currentDisplayedScalarField->getColorRampSteps();
					assert(steps!=0);

					if (steps > CC_MAX_SHADER_COLOR_RAMP_SIZE || maxComponents < (GLint)steps)
					{
						ccLog::WarningDebug("Color ramp steps exceed shader limits!");
						colorRampShader = 0;
					}
					else
					{
						float sfMinSatRel = 0.0f;
						float sfMaxSatRel = 1.0f;
						if (!m_currentDisplayedScalarField->symmetricalScale())
						{
							sfMinSatRel = GetNormalizedValue(sfSaturationRange.start(),sfDisplayRange);	//doesn't need to be between 0 and 1!
							sfMaxSatRel = GetNormalizedValue(sfSaturationRange.stop(),sfDisplayRange);	//doesn't need to be between 0 and 1!
						}
						else
						{
							//we can only handle 'maximum' saturation
							sfMinSatRel = GetSymmetricalNormalizedValue(-sfSaturationRange.stop(),sfSaturationRange);
							sfMaxSatRel = GetSymmetricalNormalizedValue(sfSaturationRange.stop(),sfSaturationRange);
							//we'll have to handle the 'minimum' saturation manually!
						}

						const ccColorScale::Shared& colorScale = m_currentDisplayedScalarField->getColorScale();
						assert(colorScale);

						colorRampShader->start();
						if (!colorRampShader->setup(sfMinSatRel, sfMaxSatRel, steps, colorScale))
						{
							//An error occured during shader initialization?
							ccLog::WarningDebug("Failed to init ColorRamp shader!");
							colorRampShader->stop();
							colorRampShader = 0;
						}
						else if (glParams.showNorms)
						{
							//we must get rid of lights material (other than ambiant) for the red and green fields
							glPushAttrib(GL_LIGHTING_BIT);

							//we use the ambiant light to pass the scalar value (and 'grayed' marker) without any
							//modification from the GPU pipeline, even if normals are enabled!
							glDisable(GL_COLOR_MATERIAL);
							glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
							glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
							glEnable(GL_COLOR_MATERIAL);

							GLint maxLightCount;
							glGetIntegerv(GL_MAX_LIGHTS,&maxLightCount);
							for (int i=0;i<maxLightCount;++i)
							{
								if (glIsEnabled(GL_LIGHT0+i))
								{
									float diffuse[4],ambiant[4],specular[4];

									glGetLightfv(GL_LIGHT0+i,GL_AMBIENT,ambiant);
									glGetLightfv(GL_LIGHT0+i,GL_DIFFUSE,diffuse);
									glGetLightfv(GL_LIGHT0+i,GL_SPECULAR,specular);

									ambiant[0]  = ambiant[1]  = 1.0f;
									diffuse[0]  = diffuse[1]  = 0.0f;
									specular[0] = specular[1] = 0.0f;

									glLightfv(GL_LIGHT0+i,GL_DIFFUSE,diffuse);
									glLightfv(GL_LIGHT0+i,GL_AMBIENT,ambiant);
									glLightfv(GL_LIGHT0+i,GL_SPECULAR,specular);
								}
							}
						}
					}
				}

				//if all points should be displayed (fastest case)
				if (!hiddenPoints)
				{
					glEnableClientState(GL_VERTEX_ARRAY);
					glEnableClientState(GL_COLOR_ARRAY);

					if (colorRampShader)
						glColorPointer(3,GL_FLOAT,0,s_rgbBuffer3f);
					else
						glColorPointer(3,GL_UNSIGNED_BYTE,0,s_rgbBuffer3ub);

					if (glParams.showNorms)
					{
						glNormalPointer(GL_FLOAT,0,s_normBuffer);
						glEnableClientState(GL_NORMAL_ARRAY);
					}

					unsigned k,chunks = m_points->chunksCount();
					for (k=0;k<chunks;++k)
					{
						unsigned chunkSize = m_points->chunkSize(k);

						//Scalar field colors
						ScalarType* _sf = m_currentDisplayedScalarField->chunkStartPtr(k);
						if (colorRampShader)
						{
							float* _sfColors = s_rgbBuffer3f;
							if (!m_currentDisplayedScalarField->symmetricalScale())
							{
								for (unsigned j=0;j<chunkSize;j+=decimStep,_sf+=decimStep,_sfColors+=3)
								{
									bool valid = sfDisplayRange.isInRange(*_sf);							//NaN values are also rejected!
									_sfColors[0] = GetNormalizedValue(*_sf,sfDisplayRange);					//normalized sf value
									_sfColors[1] = valid ? 1.0f : 0.0f;										//flag: whether point is grayed out or not
									_sfColors[2] = 1.0f;													//reference value (to get the true lighting value)
								}
							}
							else //symmetrical scale
							{
								//we must handle the values between -minSat et +minSat 'manually'
								for (unsigned j=0;j<chunkSize;j+=decimStep,_sf+=decimStep,_sfColors+=3)
								{
									bool valid = sfDisplayRange.isInRange(*_sf);							//NaN values are also rejected!
									_sfColors[0] = GetSymmetricalNormalizedValue(*_sf,sfSaturationRange);	//normalized sf value
									_sfColors[1] = valid ? 1.0f : 0.0f;										//flag: whether point is grayed out or not
									_sfColors[2] = 1.0f;													//reference value (to get the true lighting value)
								}
							}
						}
						else
						{
							colorType* _sfColors = s_rgbBuffer3ub;
							for (unsigned j=0;j<chunkSize;j+=decimStep,_sf+=decimStep)
							{
								//we need to convert scalar value to color into a temporary structure
								const colorType* col = m_currentDisplayedScalarField->getColor(*_sf);
								assert(col);
								*_sfColors++ = *col++;
								*_sfColors++ = *col++;
								*_sfColors++ = *col++;
							}
						}

						//normals
						if (glParams.showNorms)
						{
							PointCoordinateType* _normals = s_normBuffer;
							const normsType* _normalsIndexes = m_normals->chunkStartPtr(k);
							for (unsigned j=0;j<chunkSize;j+=decimStep,_normalsIndexes+=decimStep)
							{
								const PointCoordinateType* N = compressedNormals->getNormal(*_normalsIndexes);
								*(_normals)++ = *(N)++;
								*(_normals)++ = *(N)++;
								*(_normals)++ = *(N)++;
							}
						}

						if (decimStep > 1)
							chunkSize = (unsigned)floor((float)chunkSize/(float)decimStep);

						glVertexPointer(3,GL_FLOAT,decimStep*3*sizeof(PointCoordinateType),m_points->chunkStartPtr(k));
						glDrawArrays(GL_POINTS,0,chunkSize);
					}

					if (glParams.showNorms)
						glDisableClientState(GL_NORMAL_ARRAY);

					glDisableClientState(GL_VERTEX_ARRAY);
					glDisableClientState(GL_COLOR_ARRAY);
				}
				else //potentially hidden points
				{
					glBegin(GL_POINTS);

					if (glParams.showNorms) //with normals (slowest case!)
					{
						if (colorRampShader)
						{
							if (!m_currentDisplayedScalarField->symmetricalScale())
							{
								for (unsigned j=0;j<numberOfPoints;j+=decimStep)
								{
									assert(j<m_currentDisplayedScalarField->currentSize());
									const ScalarType sf = m_currentDisplayedScalarField->getValue(j);
									if (sfDisplayRange.isInRange(sf)) //NaN values are rejected
									{
										glColor3f(GetNormalizedValue(sf,sfDisplayRange),1.0f,1.0f);
										glNormal3fv(compressedNormals->getNormal(m_normals->getValue(j)));
										glVertex3fv(m_points->getValue(j));
									}
								}
							}
							else
							{
								for (unsigned j=0;j<numberOfPoints;j+=decimStep)
								{
									assert(j<m_currentDisplayedScalarField->currentSize());
									const ScalarType sf = m_currentDisplayedScalarField->getValue(j);
									if (sfDisplayRange.isInRange(sf)) //NaN values are rejected
									{
										glColor3f(GetSymmetricalNormalizedValue(sf,sfSaturationRange),1.0f,1.0f);
										glNormal3fv(compressedNormals->getNormal(m_normals->getValue(j)));
										glVertex3fv(m_points->getValue(j));
									}
								}
							}
						}
						else
						{
							for (unsigned j=0;j<numberOfPoints;j+=decimStep)
							{
								assert(j<m_currentDisplayedScalarField->currentSize());
								const colorType* col = m_currentDisplayedScalarField->getValueColor(j);
								if (col)
								{
									glColor3ubv(col);
									glNormal3fv(compressedNormals->getNormal(m_normals->getValue(j)));
									glVertex3fv(m_points->getValue(j));
								}
							}
						}
					}
					else //potentially hidden points without normals (a bit faster)
					{
						if (colorRampShader)
						{
							if (!m_currentDisplayedScalarField->symmetricalScale())
							{
								for (unsigned j=0;j<numberOfPoints;j+=decimStep)
								{
									assert(j<m_currentDisplayedScalarField->currentSize());
									const ScalarType sf = m_currentDisplayedScalarField->getValue(j);
									if (sfDisplayRange.isInRange(sf)) //NaN values are rejected
									{
										glColor3f(GetNormalizedValue(sf,sfDisplayRange),1.0f,1.0f);
										glVertex3fv(m_points->getValue(j));
									}
								}
							}
							else
							{
								for (unsigned j=0;j<numberOfPoints;j+=decimStep)
								{
									assert(j<m_currentDisplayedScalarField->currentSize());
									const ScalarType sf = m_currentDisplayedScalarField->getValue(j);
									if (sfDisplayRange.isInRange(sf)) //NaN values are rejected
									{
										glColor3f(GetSymmetricalNormalizedValue(sf,sfSaturationRange),1.0f,1.0f);
										glVertex3fv(m_points->getValue(j));
									}
								}
							}
						}
						else
						{
							for (unsigned j=0;j<numberOfPoints;j+=decimStep)
							{
								assert(j<m_currentDisplayedScalarField->currentSize());
								const colorType* col = m_currentDisplayedScalarField->getValueColor(j);
								if (col)
								{
									glColor3ubv(col);
									glVertex3fv(m_points->getValue(j));
								}
							}
						}
					}
					glEnd();
				}

				if (colorRampShader)
				{
					colorRampShader->stop();

					if (glParams.showNorms)
						glPopAttrib(); //GL_LIGHTING_BIT
				}
			}
			else if (glParams.showNorms) //no visibility table enabled, no scalar field + normals
			{
				glNormalPointer(GL_FLOAT,0,s_normBuffer);
				glEnableClientState(GL_VERTEX_ARRAY);
				glEnableClientState(GL_NORMAL_ARRAY);
				if (glParams.showColors)
					glEnableClientState(GL_COLOR_ARRAY);

				unsigned k,chunks = m_points->chunksCount();
				for (k=0;k<chunks;++k)
				{
					unsigned chunkSize = m_points->chunkSize(k);

					//normals
					PointCoordinateType* _normals = s_normBuffer;
					const normsType* _normalsIndexes = m_normals->chunkStartPtr(k);
					for (unsigned j=0;j<chunkSize;j+=decimStep,_normalsIndexes+=decimStep)
					{
						const PointCoordinateType* N = compressedNormals->getNormal(*_normalsIndexes);
						*(_normals)++ = *(N)++;
						*(_normals)++ = *(N)++;
						*(_normals)++ = *(N)++;
					}

					//colors
					if (glParams.showColors)
						glColorPointer(3,GL_UNSIGNED_BYTE,decimStep*3*sizeof(colorType),m_rgbColors->chunkStartPtr(k));

					if (decimStep > 1)
						chunkSize = (unsigned)floor((float)chunkSize/(float)decimStep);

					glVertexPointer(3,GL_FLOAT,decimStep*3*sizeof(PointCoordinateType),m_points->chunkStartPtr(k));
					glDrawArrays(GL_POINTS,0,chunkSize);
				}

				glDisableClientState(GL_VERTEX_ARRAY);
				glDisableClientState(GL_NORMAL_ARRAY);
				if (glParams.showColors)
					glDisableClientState(GL_COLOR_ARRAY);
			}
			else //no visibility table enabled, no scalar field, no normals
			{
				/*** Fast way to display simple clouds ***/
				//My old buggy ATI card wasn't supporting "glDrawArrays" with too many points...
				//DGM: well, it's very old now, why bother?! And we use chunked arrays now
				//if (ATI)
				{
					glEnableClientState(GL_VERTEX_ARRAY);
					if (glParams.showColors)
						glEnableClientState(GL_COLOR_ARRAY);

					unsigned k,chunks = m_points->chunksCount();
					assert(!glParams.showColors || m_rgbColors->chunksCount() == chunks);
					for (k=0;k<chunks;++k)
					{
						unsigned chunkSize = m_points->chunkSize(k);
						assert(!glParams.showColors || m_rgbColors->chunkSize(k) == chunkSize);
						if (decimStep > 1)
							chunkSize = (unsigned)floor((float)chunkSize/(float)decimStep);
						glVertexPointer(3,GL_FLOAT,decimStep*3*sizeof(PointCoordinateType),m_points->chunkStartPtr(k));
						if (glParams.showColors)
							glColorPointer(3,GL_UNSIGNED_BYTE,decimStep*3*sizeof(colorType),m_rgbColors->chunkStartPtr(k));
						glDrawArrays(GL_POINTS,0,chunkSize);
					}

					glDisableClientState(GL_VERTEX_ARRAY);
					if (glParams.showColors)
						glDisableClientState(GL_COLOR_ARRAY);
				}
				/*** Slow way to display simple clouds ***/
				/*else
				{
				glBegin(GL_POINTS);
				for (unsigned j=0;j<numberOfPoints;j+=decimStep)
				{
				if (glParams.showColors)
				glColor3ubv(m_rgbColors->getValue(j));

				glVertex3fv(m_points->getValue(j));
				}
				glEnd();
				}
				//*/
			}
		}
		else //special case: point names pushing (for picking) --> no need for colors, normals, etc.
		{
			glPushName(0);
			//however we must look for hidden points!
			if (isVisibilityTableInstantiated())
			{
				for (unsigned j=0;j<numberOfPoints;j+=decimStep)
				{
					if (m_pointsVisibility->getValue(j) == POINT_VISIBLE)
					{
						glLoadName(j);
						glBegin(GL_POINTS);
						glVertex3fv(m_points->getValue(j));
						glEnd();
					}
				}
			}
			else
			{
				//the fact that NaN values SHOULD be hidden, doesn't mean that we ACTUALLY hide points...
				bool hiddenPoints = false;
				if (glParams.showSF)
				{
					assert(m_currentDisplayedScalarField);
					if (!m_currentDisplayedScalarField->areNaNValuesShownInGrey() && m_currentDisplayedScalarField->getColorScale())
					{
						const ccScalarField::Range& sfDisplayRange = m_currentDisplayedScalarField->displayRange();
						hiddenPoints = (sfDisplayRange.stop() <= sfDisplayRange.max() || sfDisplayRange.start() >= sfDisplayRange.min());
					}
				}
				if (hiddenPoints)
				{
					for (unsigned j=0;j<numberOfPoints;j+=decimStep)
					{
						const colorType* col = getPointScalarValueColor(j);
						if (col)
						{
							glLoadName(j);
							glBegin(GL_POINTS);
							glVertex3fv(m_points->getValue(j));
							glEnd();
						}
					}
				}
				else
				{
					for (unsigned j=0;j<numberOfPoints;j+=decimStep)
					{
						glLoadName(j);
						glBegin(GL_POINTS);
						glVertex3fv(m_points->getValue(j));
						glEnd();
					}
				}
			}

			//glEnd();
			glPopName();
		}

		/*** END DISPLAY ***/

		glPopAttrib(); //GL_POINT_BIT

		if (colorMaterialEnabled)
			glDisable(GL_COLOR_MATERIAL);

		//we can now switch the light off
		if (glParams.showNorms)
		{
			if (glParams.showSF)
				glPopAttrib(); //GL_LIGHTING_BIT

			glDisable((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_1_2 ? GL_RESCALE_NORMAL : GL_NORMALIZE));
			glDisable(GL_LIGHTING);
		}

		if (pushName)
			glPopName();
	}
	else if (MACRO_Draw2D(context))
	{
		if (MACRO_Foreground(context) && !context.sfColorScaleToDisplay)
		{
			if (sfColorScaleShown() && sfShown())
			{
				//drawScale(context);
				addColorRampInfo(context);
			}
		}
	}
}

void ccPointCloud::addColorRampInfo(CC_DRAW_CONTEXT& context)
{
	int sfIdx = getCurrentDisplayedScalarFieldIndex();
	if (sfIdx<0)
		return;

	context.sfColorScaleToDisplay = static_cast<ccScalarField*>(getScalarField(sfIdx));
}

ccPointCloud* ccPointCloud::filterPointsByScalarValue(ScalarType minVal, ScalarType maxVal)
{
	QSharedPointer<CCLib::ReferenceCloud> c(CCLib::ManualSegmentationTools::segment(this,minVal,maxVal));

	return (c ? partialClone(c.data()) : 0);
}

void ccPointCloud::hidePointsByScalarValue(ScalarType minVal, ScalarType maxVal)
{
	if (!razVisibilityArray())
	{
		ccLog::Error(QString("[Cloud %1] Visibility table could not be instantiated!").arg(getName()));
		return;
	}

	CCLib::ScalarField* sf = getCurrentOutScalarField();
	if (!sf)
	{
		ccLog::Error(QString("[Cloud %1] Internal error: no activated output scalar field!").arg(getName()));
		return;
	}

	//we use the visibility table to tag the points to filter out
	unsigned count=size();
	for (unsigned i=0;i<count;++i)
	{
		const ScalarType& val = sf->getValue(i);
		if (val<minVal || val>maxVal || val != val) //handle NaN values!
			m_pointsVisibility->setValue(i,POINT_HIDDEN);
	}
}

ccGenericPointCloud* ccPointCloud::createNewCloudFromVisibilitySelection(bool removeSelectedPoints)
{
	if (!isVisibilityTableInstantiated())
	{
		ccLog::Error(QString("[Cloud %1] Visibility table not instantiated!").arg(getName()));
		return 0;
	}

	//we create a new cloud with the "visible" points
	ccPointCloud* result = 0;
	{
		//we create a temporary entity with the visible points only
		CCLib::ReferenceCloud* rc = getTheVisiblePoints();
		if (!rc)
		{
			ccLog::Warning("[ccPointCloud::createNewCloudFromVisibilitySelection] An error occured during points selection!");
			return 0;
		}
		assert(rc->size() != 0);

		//convert selection to cloud
		result = partialClone(rc);

		//don't need this one anymore
		delete rc;
		rc=0;
	}

	if (!result)
	{
		ccLog::Warning("[ccPointCloud::createNewCloudFromVisibilitySelection] An error occured during points duplication!");
		return 0;
	}

	result->setName(getName()+QString(".segmented"));

	//shall the visible points be erased from this cloud?
	if (removeSelectedPoints && !isLocked())
	{
		//we remove all visible points
		unsigned lastPoint = 0;
		unsigned count = size();
		for (unsigned i=0; i<count; ++i)
		{
			if (m_pointsVisibility->getValue(i) != POINT_VISIBLE)
			{
				if (i != lastPoint)
					swapPoints(lastPoint,i);
				++lastPoint;
			}
		}

		//we disable octree
		deleteOctree();

		//TODO
		//ccMesh* mesh = getMesh();
		/*if (theMeshes)
		{
		CCLib::ReferenceCloud* invrc = new CCLib::ReferenceCloud(this);
		unsigned i,count=size();
		invrc->reserve(count-result->size());

		for (i=0;i<count;++i)
		if (m_pointsVisibility->getValue(i) != POINT_VISIBLE)
		invrc->addPointIndex(i); //can't fail see above

		//REVOIR --> on pourrait le faire pour chaque sous-mesh non ?
		CCLib::GenericIndexedMesh* newTri = CCLib::ManualSegmentationTools::segmentMesh(theMeshes,invrc,true,NULL,this);
		setMesh(newTri);

		delete invrc;
		}
		//*/

		resize(lastPoint);
		refreshBB();
	}

	return result;
}

ccScalarField* ccPointCloud::getCurrentDisplayedScalarField() const
{
	return static_cast<ccScalarField*>(getScalarField(m_currentDisplayedScalarFieldIndex));
}

int ccPointCloud::getCurrentDisplayedScalarFieldIndex() const
{
	return m_currentDisplayedScalarFieldIndex;
}

void ccPointCloud::setCurrentDisplayedScalarField(int index)
{
	m_currentDisplayedScalarFieldIndex=index;
	m_currentDisplayedScalarField=static_cast<ccScalarField*>(getScalarField(index));

	if (m_currentDisplayedScalarFieldIndex>=0 && m_currentDisplayedScalarField)
		setCurrentOutScalarField(m_currentDisplayedScalarFieldIndex);
}

void ccPointCloud::deleteScalarField(int index)
{
	//we 'store' the currently displayed SF, as the SF order maybe mixed up
	setCurrentInScalarField(m_currentDisplayedScalarFieldIndex);

	//the father does all the work
	ChunkedPointCloud::deleteScalarField(index);

	//current SF should still be up-to-date!
	if (m_currentInScalarFieldIndex < 0 && getNumberOfScalarFields() > 0)
		setCurrentInScalarField((int)getNumberOfScalarFields()-1);

	setCurrentDisplayedScalarField(m_currentInScalarFieldIndex);
	showSF(m_currentInScalarFieldIndex>=0);
}

void ccPointCloud::deleteAllScalarFields()
{
	//the father does all the work
	ChunkedPointCloud::deleteAllScalarFields();

	//update the currently displayed SF
	setCurrentDisplayedScalarField(-1);
	showSF(false);
}

bool ccPointCloud::setRGBColorWithCurrentScalarField(bool mixWithExistingColor/*=false*/)
{
	if (!m_currentDisplayedScalarField || !m_currentDisplayedScalarField->getColorScale())
	{
		ccLog::Warning("[ccPointCloud::setColorWithCurrentScalarField] No active scalar field or color scale!");
		return false;
	}

	unsigned count = size();

	if (!mixWithExistingColor || !hasColors())
	{
		if (!hasColors())
			if (!resizeTheRGBTable(false))
				return false;

		for (unsigned i=0; i<count; i++)
		{
			const colorType* col = getPointScalarValueColor(i);
			m_rgbColors->setValue(i,col ? col : ccColor::black);
		}
	}
	else
	{
		m_rgbColors->placeIteratorAtBegining();
		for (unsigned i=0; i<count; i++)
		{
			const colorType* col = getPointScalarValueColor(i);
			if (col)
			{
				colorType* _color = m_rgbColors->getCurrentValue();
				_color[0] = static_cast<colorType>(static_cast<float>(_color[0])*static_cast<float>(col[0])/static_cast<float>(MAX_COLOR_COMP));
				_color[1] = static_cast<colorType>(static_cast<float>(_color[1])*static_cast<float>(col[1])/static_cast<float>(MAX_COLOR_COMP));
				_color[2] = static_cast<colorType>(static_cast<float>(_color[2])*static_cast<float>(col[2])/static_cast<float>(MAX_COLOR_COMP));
			}
			m_rgbColors->forwardIterator();
		}
	}

	return true;
}

void ccPointCloud::unrollOnCylinder(double radius, CCVector3* center, unsigned char dim/*=2*/, CCLib::GenericProgressCallback* progressCb/*=NULL*/)
{
	assert(dim <= 2);
	uchar dim1 = (dim<2 ? dim+1 : 0);
	uchar dim2 = (dim1<2 ? dim1+1 : 0);

	unsigned numberOfPoints = size();

	CCLib::NormalizedProgress* nprogress=0;
	if (progressCb)
	{
		progressCb->reset();
		nprogress = new CCLib::NormalizedProgress(progressCb,numberOfPoints);
		progressCb->setMethodTitle("Unroll (cylinder)");
		progressCb->setInfo(qPrintable(QString("Number of points = %1").arg(numberOfPoints)));
		progressCb->start();
	}

	//compute cylinder center (if none was provided)
	CCVector3 C;
	if (!center)
	{
		C = getBB().getCenter();
		center = &C;
	}

	for (unsigned i=0;i<numberOfPoints;i++)
	{
		CCVector3 *Q = point(i);

		PointCoordinateType P0 = Q->u[dim1] - center->u[dim1];
		PointCoordinateType P1 = Q->u[dim2] - center->u[dim2];
		PointCoordinateType P2 = Q->u[dim]  - center->u[dim];

		PointCoordinateType u = sqrt(P0 * P0 + P1 * P1);
		PointCoordinateType lon = atan2(P0,P1);

		//we project the point
		Q->u[dim1] = lon*radius;
		Q->u[dim2] = u-radius;
		Q->u[dim]  = P2;

		// and its normal if necessary
		if (hasNormals())
		{
			const PointCoordinateType *n = ccNormalVectors::GetNormal(m_normals->getValue(i));

			PointCoordinateType px = P0+n[dim1];
			PointCoordinateType py = P1+n[dim2];
			PointCoordinateType nlon = atan2(px,py);
			PointCoordinateType nu = sqrt(px*px+py*py);

			CCVector3 n2;
			n2.u[dim1] = (nlon-lon)*radius;
			n2.u[dim2] = nu - u;
			n2.u[dim]  = n[dim];

			n2.normalize();
			setPointNormal(i,n2.u);
		}

		//process canceled by user?
		if (nprogress && !nprogress->oneStep())
			break;
	}

	refreshBB();

	if (progressCb)
		progressCb->stop();
}

void ccPointCloud::unrollOnCone(double baseRadius, double alpha_deg, const CCVector3& apex, unsigned char dim/*=2*/, CCLib::GenericProgressCallback* progressCb/*=NULL*/)
{
	assert(dim < 3);
	uchar dim1 = (dim<2 ? dim+1 : 0);
	uchar dim2 = (dim1<2 ? dim1+1 : 0);

	unsigned numberOfPoints = size();

	CCLib::NormalizedProgress* nprogress=0;
	if (progressCb)
	{
		progressCb->reset();
		nprogress = new CCLib::NormalizedProgress(progressCb,numberOfPoints);
		progressCb->setMethodTitle("Unroll (cone)");
		progressCb->setInfo(qPrintable(QString("Number of points = %1").arg(numberOfPoints)));
		progressCb->start();
	}

	float tan_alpha = tan(alpha_deg*CC_DEG_TO_RAD);
	float cos_alpha = cos(alpha_deg*CC_DEG_TO_RAD);
	float sin_alpha = sin(alpha_deg*CC_DEG_TO_RAD);

	for (unsigned i=0; i<numberOfPoints; i++)
	{
		CCVector3 *P = point(i);
		PointCoordinateType P0 = P->u[dim1] - apex.u[dim1];
		PointCoordinateType P1 = P->u[dim2] - apex.u[dim2];
		PointCoordinateType P2 = P->u[dim]  - apex.u[dim];

		PointCoordinateType u = sqrt(P0 * P0 + P1 * P1);
		PointCoordinateType lon = atan2(P0,P1);

		//projection on the cone
		PointCoordinateType radialDist = (u+P2*tan_alpha);
		PointCoordinateType orthoDist = radialDist * cos_alpha;
		PointCoordinateType z2 = P2 - orthoDist*sin_alpha;//(P2+u*tan_alpha)*q;

		//we project point
		P->u[dim1] = lon*baseRadius;
		P->u[dim2] = orthoDist;
		P->u[dim] = z2/cos_alpha + apex.u[dim];

		//and its normal if necessary
		if (hasNormals())
		{
			const PointCoordinateType *n = ccNormalVectors::GetNormal(m_normals->getValue(i));

			PointCoordinateType dX = cos(lon)*n[dim1]-sin(lon)*n[dim2];
			PointCoordinateType dZ = sin(lon)*n[dim1]+cos(lon)*n[dim2];

			CCVector3 n2;
			n2.u[dim1] = dX;
			n2.u[dim2] = cos_alpha*dZ-sin_alpha*n[dim];
			n2.u[dim]  = sin_alpha*dZ+cos_alpha*n[dim];
			n2.normalize();

			setPointNormal(i,n2.u);
		}

		//process canceled by user?
		if (nprogress && !nprogress->oneStep())
			break;
	}

	refreshBB();

	if (nprogress)
	{
		delete nprogress;
		nprogress=0;
	}
}

int ccPointCloud::addScalarField(const char* uniqueName)
{
	//we don't accept two SF with the same name!
	if (getScalarFieldIndexByName(uniqueName)>=0)
	{
		ccLog::Warning("[ccPointCloud::addScalarField] Names already exists!");
		return -1;
	}

	//Nouveau champ scalaire
	ccScalarField* sf = new ccScalarField(uniqueName);
	if (size() && !sf->resize(size()))
	{
		sf->release();
		ccLog::Warning("[ccPointCloud::addScalarField] Not enough memory!");
		return -1;
	}

	m_scalarFields.push_back(sf);
	sf->link();

	return (int)m_scalarFields.size()-1;
}

int ccPointCloud::addScalarField(ccScalarField* sf)
{
	assert(sf);

	//we don't accept two SF with the same name!
	if (getScalarFieldIndexByName(sf->getName())>=0)
		return -1;

	if (sf->currentSize() < m_points->capacity())
		if (!sf->resize(m_points->capacity()))
			return -1;

	sf->link();
	m_scalarFields.push_back(sf);

	return (int)m_scalarFields.size()-1;
}

bool ccPointCloud::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPointCloud::toFile_MeOnly(out))
		return false;

	//points array (dataVersion>=20)
	if (!m_points)
		return ccLog::Error("Internal error - point cloud has no valid points array? (not enough memory?)");
	if (!ccSerializationHelper::GenericArrayToFile(*m_points,out))
		return false;

	//colors array (dataVersion>=20)
	{
		bool hasColorsArray = hasColors();
		if (out.write((const char*)&hasColorsArray,sizeof(bool))<0)
			return WriteError();
		if (hasColorsArray)
		{
			assert(m_rgbColors);
			if (!m_rgbColors->toFile(out))
				return false;
		}
	}

	//normals array (dataVersion>=20)
	{
		bool hasNormalsArray = hasNormals();
		if (out.write((const char*)&hasNormalsArray,sizeof(bool))<0)
			return WriteError();
		if (hasNormalsArray)
		{
			assert(m_normals);
			if (!m_normals->toFile(out))
				return false;
		}
	}

	//scalar field(s)
	{
		//number of scalar fields (dataVersion>=20)
		uint32_t sfCount = (uint32_t)getNumberOfScalarFields();
		if (out.write((const char*)&sfCount,4)<0)
			return WriteError();

		//scalar fields (dataVersion>=20)
		for (uint32_t i=0;i<sfCount;++i)
		{
			ccScalarField* sf = static_cast<ccScalarField*>(getScalarField(i));
			assert(sf);
			if (!sf->toFile(out))
				return false;
		}

		//'show NaN values in grey' state (27>dataVersion>=20)
		//if (out.write((const char*)&m_greyForNanScalarValues,sizeof(bool))<0)
		//	return WriteError();

		//'show current sf color scale' state (dataVersion>=20)
		if (out.write((const char*)&m_sfColorScaleDisplayed,sizeof(bool))<0)
			return WriteError();

		//Displayed scalar field index (dataVersion>=20)
		int32_t displayedScalarFieldIndex = (int32_t)m_currentDisplayedScalarFieldIndex;
		if (out.write((const char*)&displayedScalarFieldIndex,4)<0)
			return WriteError();
	}

	return true;
}

bool ccPointCloud::fromFile_MeOnly(QFile& in, short dataVersion)
{
	if (!ccGenericPointCloud::fromFile_MeOnly(in, dataVersion))
		return false;

	//points array (dataVersion>=20)
	if (!m_points)
		return ccLog::Error("Internal error: point cloud has no valid point array! (not enough memory?)");
	if (!ccSerializationHelper::GenericArrayFromFile(*m_points,in,dataVersion))
		return false;

	//colors array (dataVersion>=20)
	{
		bool hasColorsArray = false;
		if (in.read((char*)&hasColorsArray,sizeof(bool))<0)
			return ReadError();
		if (hasColorsArray)
		{
			if (!m_rgbColors)
			{
				m_rgbColors = new ColorsTableType;
				m_rgbColors->link();
			}
			unsigned classID=0;
			if (!ReadClassIDFromFile(classID, in, dataVersion))
				return false;
			if (classID != CC_RGB_COLOR_ARRAY)
				return CorruptError();
			if (!m_rgbColors->fromFile(in,dataVersion))
			{
				unallocateColors();
				return false;
			}
		}
	}

	//normals array (dataVersion>=20)
	{
		bool hasNormalsArray = false;
		if (in.read((char*)&hasNormalsArray,sizeof(bool))<0)
			return ReadError();
		if (hasNormalsArray)
		{
			if (!m_normals)
			{
				m_normals = new NormsIndexesTableType();
				m_normals->link();
			}
			unsigned classID=0;
			if (!ReadClassIDFromFile(classID, in, dataVersion))
				return false;
			if (classID != CC_NORMAL_INDEXES_ARRAY)
				return CorruptError();
			if (!m_normals->fromFile(in,dataVersion))
			{
				unallocateNorms();
				return false;
			}
		}
	}

	//scalar field(s)
	{
		//number of scalar fields (dataVersion>=20)
		uint32_t sfCount = 0;
		if (in.read((char*)&sfCount,4)<0)
			return ReadError();

		//scalar fields (dataVersion>=20)
		for (uint32_t i=0;i<sfCount;++i)
		{
			ccScalarField* sf = new ccScalarField();
			if (!sf->fromFile(in,dataVersion))
			{
				sf->release();
				return false;
			}
			addScalarField(sf);
		}

		if (dataVersion < 27)
		{
			//'show NaN values in grey' state (27>dataVersion>=20)
			bool greyForNanScalarValues = true;
			if (in.read((char*)&greyForNanScalarValues,sizeof(bool))<0)
				return ReadError();

			//update all scalar fields accordingly (old way)
			for (unsigned i=0; i<getNumberOfScalarFields(); ++i)
			{
				static_cast<ccScalarField*>(getScalarField(i))->showNaNValuesInGrey(greyForNanScalarValues);
			}
		}

		//'show current sf color scale' state (dataVersion>=20)
		if (in.read((char*)&m_sfColorScaleDisplayed,sizeof(bool))<0)
			return ReadError();

		//Displayed scalar field index (dataVersion>=20)
		int32_t displayedScalarFieldIndex = 0;
		if (in.read((char*)&displayedScalarFieldIndex,4)<0)
			return ReadError();
		if (displayedScalarFieldIndex<(int32_t)sfCount)
			setCurrentDisplayedScalarField(displayedScalarFieldIndex);
	}

	updateModificationTime();

	return true;
}
