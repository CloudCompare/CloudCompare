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
#include <ManualSegmentationTools.h>

//local
#include "ccNormalVectors.h"
#include "ccColorScalesManager.h"
#include "ccOctree.h"
#include "ccKdTree.h"
#include "ccGenericMesh.h"
#include "ccMesh.h"
#include "ccImage.h"
#include "cc2DLabel.h"
#include "ccMaterial.h"
#include "ccColorRampShader.h"
#include "ccPolyline.h"
#include "ccScalarField.h"
#include "ccGenericGLDisplay.h"
#include "ccGBLSensor.h"

//system
#include <assert.h>

ccPointCloud::ccPointCloud(QString name) throw()
	: ChunkedPointCloud()
	, ccGenericPointCloud(name)
	, m_rgbColors(0)
	, m_normals(0)
	, m_sfColorScaleDisplayed(false)
	, m_currentDisplayedScalarField(0)
	, m_currentDisplayedScalarFieldIndex(-1)
{
	showSF(false);
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
	result->setEnabled(isEnabled());

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
				if (sfIdx >= 0) //success
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
				if (sfIdx >= 0)
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
	for (i=0; i<n; ++i) cubeVertexesIndexes.setValue(i,_theScans->getValue(i));
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
	result->setGlobalShift(getGlobalShift());
	result->setGlobalScale(getGlobalScale());

	//custom point size
	result->setPointSize(getPointSize());

	return result;
}

ccPointCloud::~ccPointCloud()
{
	clear();
}

void ccPointCloud::clear()
{
	unalloactePoints();
	unallocateColors();
	unallocateNorms();
	//enableTempColor(false); //DGM: why?
}

void ccPointCloud::unalloactePoints()
{
	showSFColorsScale(false); //SFs will be destroyed
	ChunkedPointCloud::clear();
	ccGenericPointCloud::clear();

	notifyGeometryUpdate(); //calls releaseVBOs()
}

void ccPointCloud::notifyGeometryUpdate()
{
	ccHObject::notifyGeometryUpdate();

	releaseVBOs();
}

ccGenericPointCloud* ccPointCloud::clone(ccGenericPointCloud* destCloud/*=0*/, bool ignoreChildren/*=false*/)
{
	if (destCloud && !destCloud->isA(CC_TYPES::POINT_CLOUD))
	{
		ccLog::Error("[ccPointCloud::clone] Invalid destination cloud provided! Not a ccPointCloud...");
		return 0;
	}

	return cloneThis(static_cast<ccPointCloud*>(destCloud), ignoreChildren);
}

ccPointCloud* ccPointCloud::cloneThis(ccPointCloud* destCloud/*=0*/, bool ignoreChildren/*=false*/)
{
	ccPointCloud* result = destCloud ? destCloud : new ccPointCloud();

	result->setVisible(isVisible());
	if (!destCloud)
		result->setDisplay(getDisplay());

	result->append(this,0,ignoreChildren); //there was (virtually) no point before

	result->showColors(colorsShown());
	result->showSF(sfShown());
	result->showNormals(normalsShown());
	result->setEnabled(isEnabled());

	result->setCurrentDisplayedScalarField(getCurrentDisplayedScalarFieldIndex());
	result->setPointSize(getPointSize());

	//original shift
	result->setGlobalShift(getGlobalShift());
	result->setGlobalScale(getGlobalScale());

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

const ccPointCloud& ccPointCloud::append(ccPointCloud* addedCloud, unsigned pointCountBefore, bool ignoreChildren/*=false*/)
{
	assert(addedCloud);

	unsigned addedPoints = addedCloud->size();

	if (!reserve(pointCountBefore+addedPoints))
	{
		ccLog::Error("[ccPointCloud::append] Not enough memory!");
		return *this;
	}

	//merge display parameters
	setVisible(isVisible() || addedCloud->isVisible());

	//3D points (already reserved)
	if (size() == pointCountBefore) //in some cases points have already been copied! (ok it's tricky)
	{
		//we remove structures that are not compatible with fusion process
		deleteOctree();
		unallocateVisibilityArray();

		for (unsigned i=0; i<addedPoints; i++)
			addPoint(*addedCloud->getPoint(i));
	}

	//deprecate internal structures
	notifyGeometryUpdate(); //calls releaseVBOs()

	//Colors (already reserved)
	if (hasColors() || addedCloud->hasColors())
	{
		//merge display parameters
		showColors(colorsShown() || addedCloud->colorsShown());

		//if the added cloud has no color
		if (!addedCloud->hasColors())
		{
			//we set a white color to new points
			for (unsigned i=0; i<addedPoints; i++)
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
					for (unsigned i=0; i<pointCountBefore; i++)
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
				for (unsigned i=0; i<addedPoints; i++)
					addRGBColor(addedCloud->m_rgbColors->getValue(i));
		}
	}

	//normales (reserved)
	if (hasNormals() || addedCloud->hasNormals())
	{
		//merge display parameters
		showNormals(normalsShown() || addedCloud->normalsShown());

		//if the added cloud hasn't any normal
		if (!addedCloud->hasNormals())
		{
			//we associate imported points with '0' normals
			for (unsigned i=0; i<addedPoints; i++)
				addNormIndex(0);
		}
		else //otherwise
		{
			//if this cloud hasn't any normal
			if (!hasNormals())
			{
				//we try to reserve a new array
				if (reserveTheNormsTable())
				{
					for (unsigned i=0; i<pointCountBefore; i++)
						addNormIndex(0);
				}
				else
				{
					ccLog::Warning("[ccPointCloud::fusion] Not enough memory: failed to allocate normals!");
					showNormals(false);
				}
			}

			//we import normals (if necessary)
			if (hasNormals() && m_normals->currentSize() == pointCountBefore)
				for (unsigned i=0; i<addedPoints; i++)
					addNormIndex(addedCloud->m_normals->getValue(i));
		}
	}

	//scalar fields (resized)
	unsigned sfCount = getNumberOfScalarFields();
	unsigned newSFCount = addedCloud->getNumberOfScalarFields();
	if (sfCount != 0 || newSFCount != 0)
	{
		std::vector<bool> sfUpdated(sfCount, false);

		//first we merge the new SF with the existing one
		for (unsigned k=0; k<newSFCount; ++k)
		{
			const ccScalarField* sf = static_cast<ccScalarField*>(addedCloud->getScalarField(static_cast<int>(k)));
			if (sf)
			{
				//does this field already exist (same name)?
				int sfIdx = getScalarFieldIndexByName(sf->getName());
				if (sfIdx >= 0) //yes
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
					sfUpdated[sfIdx] = true;
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
						//copy color ramp parameters
						newSF->setColorRampSteps(sf->getColorRampSteps());
						newSF->setColorScale(sf->getColorScale());
						newSF->showNaNValuesInGrey(sf->areNaNValuesShownInGrey());
						newSF->setLogScale(sf->logScale());
						newSF->setSymmetricalScale(sf->symmetricalScale());
						newSF->alwaysShowZero(sf->isZeroAlwaysShown());
						newSF->setMinDisplayed(sf->displayRange().start());
						newSF->setMaxDisplayed(sf->displayRange().stop());
						newSF->setSaturationStart(sf->saturationRange().start());
						newSF->setSaturationStop(sf->saturationRange().stop());

						//add scalar field to this cloud
						sfIdx = addScalarField(newSF);
						assert(sfIdx >= 0);
					}
					else
					{
						newSF->release();
						newSF = 0;
						ccLog::Warning("[ccPointCloud::fusion] Not enough memory: failed to allocate a copy of scalar field '%s'",sf->getName());
					}
				}
			}
		}

		//let's check if there are non-updated fields
		for (unsigned j=0; j<sfCount; ++j)
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
		if (getNumberOfScalarFields() == 0)
		{
			setCurrentDisplayedScalarField(-1);
			showSF(false);
		}
		else
		{
			//if there was no scalar field before
			if (sfCount == 0)
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

			//merge display parameters
			showSF(sfShown() || addedCloud->sfShown());
		}
	}

	//Has the cloud been recentered/rescaled?
	{
		if (addedCloud->isShifted())
			ccLog::Warning(QString("[ccPointCloud::fusion] Global shift/scale information for cloud '%1' will be lost!").arg(addedCloud->getName()));
	}

	//children (not yet reserved)
	if (!ignoreChildren)
	{
		unsigned childrenCount = addedCloud->getChildrenNumber();
		for (unsigned c=0; c<childrenCount; ++c)
		{
			ccHObject* child = addedCloud->getChild(c);
			if (child->isA(CC_TYPES::MESH)) //mesh --> FIXME: what for the other types of MESH?
			{
				ccMesh* mesh = static_cast<ccMesh*>(child);

				//detach from father?
				//addedCloud->detachChild(mesh);
				//ccGenericMesh* addedTri = mesh;

				//or clone?
				ccMesh* cloneMesh = mesh->cloneMesh(mesh->getAssociatedCloud()==addedCloud ? this : 0);
				if (cloneMesh)
				{
					//change mesh vertices
					if (cloneMesh->getAssociatedCloud() == this)
						cloneMesh->shiftTriangleIndexes(pointCountBefore);
					addChild(cloneMesh);
				}
				else
				{
					ccLog::Warning(QString("[ccPointCloud::fusion] Not enough memory: failed to clone sub mesh %1!").arg(mesh->getName()));
				}
			}
			else if (child->isKindOf(CC_TYPES::IMAGE))
			{
				//ccImage* image = static_cast<ccImage*>(child);

				//DGM FIXME: take image ownership! (dirty)
				addedCloud->transferChild(child,*this);
			}
			else if (child->isA(CC_TYPES::LABEL_2D))
			{
				//clone label and update points if necessary
				cc2DLabel* label = static_cast<cc2DLabel*>(child);
				cc2DLabel* newLabel = new cc2DLabel(label->getName());
				for (unsigned j=0; j<label->size(); ++j)
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
				addChild(newLabel);
			}
			else if (child->isA(CC_TYPES::GBL_SENSOR))
			{
				//copy sensor object
				ccGBLSensor* sensor = new ccGBLSensor(*static_cast<ccGBLSensor*>(child));
				addChild(sensor);
				sensor->setDisplay(getDisplay());
				sensor->setVisible(child->isVisible());
			}
		}
	}

	//We should update the VBOs (just in case)
	releaseVBOs();

	return *this;
}

void ccPointCloud::unallocateNorms()
{
	if (m_normals)
	{
		m_normals->release();
		m_normals = 0;

		//We should update the VBOs to gain some free space in VRAM
		releaseVBOs();
	}

	showNormals(false);
}

void ccPointCloud::unallocateColors()
{
	if (m_rgbColors)
	{
		m_rgbColors->release();
		m_rgbColors = 0;

		//We should update the VBOs to gain some free space in VRAM
		releaseVBOs();
	}

	showColors(false);
	enableTempColor(false);
}

bool ccPointCloud::reserveThePointsTable(unsigned newNumberOfPoints)
{
	return m_points->reserve(newNumberOfPoints);
}

bool ccPointCloud::reserveTheRGBTable()
{
	//ccLog::Warning(QString("[ccPointCloud::reserveTheRGBTable] Cloud is %1 and its capacity is '%2'").arg(m_points->isAllocated() ? "allocated" : "not allocated").arg(m_points->capacity()));
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
	}

	//We must update the VBOs
	releaseVBOs();

	//double check
	return m_rgbColors && m_rgbColors->capacity() >= m_points->capacity();
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
	}

	//We must update the VBOs
	releaseVBOs();

	//double check
	return m_rgbColors && m_rgbColors->currentSize() == m_points->currentSize();
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
	}

	//We must update the VBOs
	releaseVBOs();

	//double check
	return m_normals && m_normals->capacity() >= m_points->capacity();
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
	}

	//We must update the VBOs
	releaseVBOs();

	//double check
	return m_normals && m_normals->currentSize() == m_points->currentSize();
}

bool ccPointCloud::reserve(unsigned newNumberOfPoints)
{
	//reserve works only to enlarge the cloud
	if (newNumberOfPoints < size())
		return false;

	//call parent method first (for points + scalar fields)
	if (	!ChunkedPointCloud::reserve(newNumberOfPoints)
		||	hasColors() && !reserveTheRGBTable()
		||	hasNormals() && !reserveTheNormsTable() )
	{
		ccLog::Error("[ccPointCloud::reserve] Not enough memory!");
		return false;
	}

	//ccLog::Warning(QString("[ccPointCloud::reserve] Cloud is %1 and its capacity is '%2'").arg(m_points->isAllocated() ? "allocated" : "not allocated").arg(m_points->capacity()));

	//double check
	return	                   m_points->capacity()    >= newNumberOfPoints
		&&	( !hasColors()  || m_rgbColors->capacity() >= newNumberOfPoints )
		&&	( !hasNormals() || m_normals->capacity()   >= newNumberOfPoints );
}

bool ccPointCloud::resize(unsigned newNumberOfPoints)
{
	//can't reduce the size if the cloud if it is locked!
	if (newNumberOfPoints < size() && isLocked())
		return false;

	//call parent method first (for points + scalar fields)
	if (!ChunkedPointCloud::resize(newNumberOfPoints))
	{
		ccLog::Error("[ccPointCloud::resize] Not enough memory!");
		return false;
	}

	notifyGeometryUpdate(); //calls releaseVBOs()

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

	//double check
	return	                   m_points->currentSize()    == newNumberOfPoints
		&&	( !hasColors()  || m_rgbColors->currentSize() == newNumberOfPoints )
		&&	( !hasNormals() || m_normals->currentSize()   == newNumberOfPoints );
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
	assert(m_rgbColors && pointIndex < m_rgbColors->currentSize());

	return m_rgbColors->getValue(pointIndex);
}

const normsType& ccPointCloud::getPointNormalIndex(unsigned pointIndex) const
{
	assert(m_normals && pointIndex < m_normals->currentSize());

	return m_normals->getValue(pointIndex);
}

const CCVector3& ccPointCloud::getPointNormal(unsigned pointIndex) const
{
	assert(m_normals && pointIndex < m_normals->currentSize());

	return ccNormalVectors::GetNormal(m_normals->getValue(pointIndex));
}

void ccPointCloud::setPointColor(unsigned pointIndex, const colorType* col)
{
	assert(m_rgbColors && pointIndex < m_rgbColors->currentSize());

	m_rgbColors->setValue(pointIndex, col);

	//We must update the VBOs
	releaseVBOs();
}

void ccPointCloud::setPointNormalIndex(unsigned pointIndex, normsType norm)
{
	assert(m_normals && pointIndex < m_normals->currentSize());

	m_normals->setValue(pointIndex, norm);

	//We must update the VBOs
	releaseVBOs();
}

void ccPointCloud::setPointNormal(unsigned pointIndex, const CCVector3& N)
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
	return (getNumberOfScalarFields() > 0);
}

bool ccPointCloud::hasDisplayedScalarField() const
{
	return m_currentDisplayedScalarField && m_currentDisplayedScalarField->getColorScale();
}

void ccPointCloud::refreshBB()
{
	invalidateBoundingBox();
	notifyGeometryUpdate();	//calls releaseVBOs()
}

void ccPointCloud::addGreyColor(colorType g)
{
	assert(m_rgbColors && m_rgbColors->isAllocated());
	const colorType G[3] = {g,g,g};
	m_rgbColors->addElement(G);

	//We must update the VBOs
	releaseVBOs();
}

void ccPointCloud::addRGBColor(const colorType* C)
{
	assert(m_rgbColors && m_rgbColors->isAllocated());
	m_rgbColors->addElement(C);

	//We must update the VBOs
	releaseVBOs();
}

void ccPointCloud::addRGBColor(colorType r, colorType g, colorType b)
{
	assert(m_rgbColors && m_rgbColors->isAllocated());
	const colorType C[3] = {r,g,b};
	m_rgbColors->addElement(C);

	//We must update the VBOs
	releaseVBOs();
}

void ccPointCloud::addNorm(const CCVector3& N)
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

	//We must update the VBOs
	releaseVBOs();
}

bool ccPointCloud::convertNormalToRGB()
{
	if (!hasNormals())
		return false;

	if (!ccNormalVectors::GetUniqueInstance()->enableNormalHSVColorsArray())
	{
		ccLog::Warning("[ccPointCloud::convertNormalToRGB] Not enough memory!");
		return false;
	}
	const colorType* normalHSV = ccNormalVectors::GetUniqueInstance()->getNormalHSVColorArray();

	if (!resizeTheRGBTable(false))
	{
		ccLog::Warning("[ccPointCloud::convertNormalToRGB] Not enough memory!");
		return false;
	}
	assert(m_normals && m_rgbColors);

	unsigned count = size();
	for (unsigned i=0; i<count; ++i)
	{
		const colorType* rgb = normalHSV + 3*m_normals->getValue(i);
		m_rgbColors->setValue(i,rgb);
	}

	//We must update the VBOs
	releaseVBOs();

	return true;
}

bool ccPointCloud::convertNormalToDipDirSFs(ccScalarField* dipSF, ccScalarField* dipDirSF)
{
	if (!dipSF && !dipDirSF)
	{
		assert(false);
		return false;
	}

	if (	(dipSF && !dipSF->resize(size()))
		||	(dipDirSF &&  !dipDirSF->resize(size())) )
	{
		ccLog::Warning("[ccPointCloud::convertNormalToDipDirSFs] Not enough memory!");
		return false;
	}

	unsigned count = size();
	for (unsigned i=0; i<count; ++i)
	{
		CCVector3 N(this->getPointNormal(i));
		PointCoordinateType dip,dipDir;
		ccNormalVectors::ConvertNormalToDipAndDipDir(N,dip,dipDir);
		if (dipSF)
			dipSF->setValue(i,static_cast<ScalarType>(dip));
		if (dipDirSF)
			dipDirSF->setValue(i,static_cast<ScalarType>(dipDir));
	}

	if (dipSF)
		dipSF->computeMinAndMax();
	if (dipDirSF)
		dipDirSF->computeMinAndMax();

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

	//We must update the VBOs
	releaseVBOs();
}

bool ccPointCloud::colorize(float r, float g, float b)
{
	assert(r >= 0.0f && r <= 1.0f);
	assert(g >= 0.0f && g <= 1.0f);
	assert(b >= 0.0f && b <= 1.0f);

	if (hasColors())
	{
		assert(m_rgbColors);
		m_rgbColors->placeIteratorAtBegining();
		for (unsigned i=0; i<m_rgbColors->currentSize(); i++)
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

		colorType RGB[3] = {static_cast<colorType>(static_cast<float>(MAX_COLOR_COMP) * r) ,
							static_cast<colorType>(static_cast<float>(MAX_COLOR_COMP) * g) ,
							static_cast<colorType>(static_cast<float>(MAX_COLOR_COMP) * b) };
		m_rgbColors->fill(RGB);
	}

	//We must update the VBOs
	releaseVBOs();

	return true;
}

//Contribution from Michael J Smith
bool ccPointCloud::setRGBColorByBanding(unsigned char dim, int freq)
{
	if (freq == 0 || dim > 2) //X=0, Y=1, Z=2
	{
		ccLog::Error("[ccPointCloud::setRGBColorByBanding] Invalid paramter!");
		return false;
	}

	//allocate colors if necessary
	if (!hasColors())
		if (!resizeTheRGBTable(false))
			return false;

	enableTempColor(false);
	assert(m_rgbColors);

 	double minHeight = getBB().minCorner().u[dim];
	double height = getBB().getDiagVec().u[dim];
	
	if (fabs(height) < ZERO_TOLERANCE) //flat cloud!
		height = 1.0;

	/* Repeats per spacing of 1 */
	double bands = freq * (2 * M_PI);

	unsigned count = size();
	for (unsigned i=0; i<count; i++)
	{
		const CCVector3* P = getPoint(i);

		double z = bands * (P->u[dim] - minHeight) / height;
		colorType col[3] = {	static_cast<colorType>( ((sin(z + 0) + 1.0) / 2.0) * MAX_COLOR_COMP ),
								static_cast<colorType>( ((sin(z + 2) + 1.0) / 2.0) * MAX_COLOR_COMP ),
								static_cast<colorType>( ((sin(z + 4) + 1.0) / 2.0) * MAX_COLOR_COMP ) };

		m_rgbColors->setValue(i,col);
	}

	//We must update the VBOs
	releaseVBOs();

	return true;
}

bool ccPointCloud::setRGBColorByHeight(unsigned char heightDim, ccColorScale::Shared colorScale)
{
	if (!colorScale || heightDim > 2) //X=0, Y=1, Z=2
	{
		ccLog::Error("[ccPointCloud::setRGBColorByHeight] Invalid paramter!");
		return false;
	}

	//allocate colors if necessary
	if (!hasColors())
		if (!resizeTheRGBTable(false))
			return false;

	enableTempColor(false);
	assert(m_rgbColors);

	double minHeight = getBB().minCorner().u[heightDim];
	double height = getBB().getDiagVec().u[heightDim];
	if (fabs(height) < ZERO_TOLERANCE) //flat cloud!
	{
		return setRGBColor(colorScale->getColorByIndex(0));
	}

	unsigned count = size();
	for (unsigned i=0; i<count; i++)
	{
		const CCVector3* Q = getPoint(i);
		double realtivePos = (Q->u[heightDim] - minHeight) / height;
		const colorType* col = colorScale->getColorByRelativePos(realtivePos);
		if (!col) //DGM: yes it happens if we encounter a point with NaN coordinates!!!
			col = ccColor::black;
		m_rgbColors->setValue(i,col);
	}

	//We must update the VBOs
	releaseVBOs();

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

	assert(m_rgbColors);
	m_rgbColors->fill(col);

	//We must update the VBOs
	releaseVBOs();

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
	//transparent call
	ccGenericPointCloud::applyGLTransformation(trans);

	unsigned count = size();
	for (unsigned i=0; i<count; i++)
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
				for (unsigned i=0; i<ccNormalVectors::GetNumberOfVectors(); i++)
				{
					CCVector3 new_n(ccNormalVectors::GetNormal(i));
					trans.applyRotation(new_n);
					normsType newNormIndex = ccNormalVectors::GetNormIndex(new_n.u);
					newNorms->addElement(newNormIndex);
				}

				m_normals->placeIteratorAtBegining();
				for (unsigned j=0; j<count; j++)
				{
					m_normals->setValue(j,newNorms->getValue(m_normals->getCurrentValue()));
					m_normals->forwardIterator();
				}
				recoded = true;
			}
			newNorms->clear();
			newNorms->release();
			newNorms = 0;
		}

		//if there is less points than the compressed normals array size
		//(or if there is not enough memory to instantiate the temporary
		//array), we recompress each normal ...
		if (!recoded)
		{
			//on recode direct chaque normale
			m_normals->placeIteratorAtBegining();
			for (unsigned i=0; i<count; i++)
			{
				normsType* _theNormIndex = m_normals->getCurrentValuePtr();
				CCVector3 new_n(ccNormalVectors::GetNormal(*_theNormIndex));
				trans.applyRotation(new_n);
				*_theNormIndex = ccNormalVectors::GetNormIndex(new_n.u);
				m_normals->forwardIterator();
			}
		}
	}

	//the octree is invalidated by rotation...
	deleteOctree();

	// ... as the bounding box
	refreshBB(); //calls notifyGeometryUpdate + releaseVBOs
}

void ccPointCloud::translate(const CCVector3& T)
{
	if (fabs(T.x)+fabs(T.y)+fabs(T.z) < ZERO_TOLERANCE)
		return;

	unsigned count = size();
	{
		for (unsigned i=0; i<count; i++)
			*point(i) += T;
	}

	notifyGeometryUpdate(); //calls releaseVBOs()

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
	filterChildren(kdtrees, false, CC_TYPES::POINT_KDTREE);
	{
		for (size_t i=0; i<kdtrees.size(); ++i)
			static_cast<ccKdTree*>(kdtrees[i])->translateBoundingBox(T);
	}
}

void ccPointCloud::multiply(PointCoordinateType fx, PointCoordinateType fy, PointCoordinateType fz)
{
	unsigned count = size();
	{
		for (unsigned i=0; i<count; i++)
		{
			CCVector3* P = point(i);
			P->x *= fx;
			P->y *= fy;
			P->z *= fz;
		}
	}

	notifyGeometryUpdate(); //calls releaseVBOs()

	//refreshBB();
	//--> instead, we update BBox directly! (faster)
	PointCoordinateType* bbMin = m_points->getMin();
	PointCoordinateType* bbMax = m_points->getMax();
	bbMin[0] *= fx;
	bbMax[0] *= fx;
	if (fx < 0)
		std::swap(bbMin[0],bbMax[0]);
	bbMin[1] *= fy;
	bbMax[1] *= fy;
	if (fy < 0)
		std::swap(bbMin[1],bbMax[1]);
	bbMin[2] *= fz;
	bbMax[2] *= fz;
	if (fz < 0)
		std::swap(bbMin[2],bbMax[2]);

	//same thing for the octree
	ccOctree* oct = getOctree();
	if (oct)
	{
		if (fx==fy && fx==fz && fx > 0)
			oct->multiplyBoundingBox(fx);
		else
			deleteOctree();
	}

	//and same thing for the Kd-tree(s)!
	ccHObject::Container kdtrees;
	filterChildren(kdtrees, false, CC_TYPES::POINT_KDTREE);
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
	for (unsigned i=0; i<m_normals->currentSize(); ++i)
	{
		ccNormalVectors::InvertNormal(*m_normals->getCurrentValuePtr());
		m_normals->forwardIterator();
	}

	//We must update the VBOs
	releaseVBOs();
}

void ccPointCloud::swapPoints(unsigned firstIndex, unsigned secondIndex)
{
	assert(!isLocked());
	assert(firstIndex < size() && secondIndex < size());

	if (firstIndex == secondIndex)
		return;

	//points + associated SF values
	ChunkedPointCloud::swapPoints(firstIndex,secondIndex);

	//colors
	if (hasColors())
	{
		assert(m_rgbColors);
		m_rgbColors->swap(firstIndex,secondIndex);
	}

	//normals
	if (hasNormals())
	{
		assert(m_normals);
		m_normals->swap(firstIndex,secondIndex);
	}

	//We must update the VBOs
	releaseVBOs();
}

void ccPointCloud::getDrawingParameters(glDrawParams& params) const
{
	//color override
	if (isColorOverriden())
	{
		params.showColors	= true;
		params.showNorms	= false;
		params.showSF		= false;
	}
	else
	{
		//a scalar field must have been selected for display!
		params.showSF		= hasDisplayedScalarField() && sfShown() && m_currentDisplayedScalarField->currentSize() >= size();
		params.showNorms	= hasNormals() &&  normalsShown()  && m_normals->currentSize() >= size();
		//colors are not displayed if scalar field is displayed
		params.showColors	= !params.showSF && hasColors() && colorsShown() && m_rgbColors->currentSize() >= size();
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

//the GL type depends on the PointCoordinateType 'size' (float or double)
static GLenum GL_COORD_TYPE = sizeof(PointCoordinateType) == 4 ? GL_FLOAT : GL_DOUBLE;

void ccPointCloud::glChunkVertexPointer(unsigned chunkIndex, unsigned decimStep, bool useVBOs)
{
	if (	useVBOs
		&&	m_vboManager.state == vboSet::INITIALIZED
		&&	m_vboManager.vbos.size() > static_cast<size_t>(chunkIndex)
		&&	m_vboManager.vbos[chunkIndex]
		&&	m_vboManager.vbos[chunkIndex]->isCreated())
	{
		//we can use VBOs directly
		if (m_vboManager.vbos[chunkIndex]->bind())
		{
			glVertexPointer(3,GL_COORD_TYPE,decimStep*3*sizeof(PointCoordinateType),0);
			m_vboManager.vbos[chunkIndex]->release();
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			m_vboManager.state = vboSet::FAILED;
			//recall the method
			glChunkVertexPointer(chunkIndex, decimStep, false);
		}
	}
	else
	{
		assert(m_points && m_points->chunkStartPtr(chunkIndex));
		//standard OpenGL copy
		glVertexPointer(3,GL_COORD_TYPE,decimStep*3*sizeof(PointCoordinateType),m_points->chunkStartPtr(chunkIndex));
	}
}

void ccPointCloud::glChunkNormalPointer(unsigned chunkIndex, unsigned decimStep, bool useVBOs)
{
	assert(m_normals);

	if (	useVBOs
		&&	m_vboManager.state == vboSet::INITIALIZED
		&&	m_vboManager.hasNormals
		&&	m_vboManager.vbos.size() > static_cast<size_t>(chunkIndex)
		&&	m_vboManager.vbos[chunkIndex]
		&&	m_vboManager.vbos[chunkIndex]->isCreated())
	{
		//we can use VBOs directly
		if (m_vboManager.vbos[chunkIndex]->bind())
		{
			const GLbyte* start = 0; //fake pointer used to prevent warnings on Linux
			int normalDataShift = m_vboManager.vbos[chunkIndex]->normalShift;
			glNormalPointer(GL_COORD_TYPE,decimStep*3*sizeof(PointCoordinateType),(const GLvoid*)(start + normalDataShift));
			m_vboManager.vbos[chunkIndex]->release();
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			m_vboManager.state = vboSet::FAILED;
			//recall the method
			glChunkNormalPointer(chunkIndex, decimStep, false);
		}
	}
	else
	{
		assert(m_normals && m_normals->chunkStartPtr(chunkIndex));
		//we must decode normals in a dedicated static array
		PointCoordinateType* _normals = s_normBuffer;
		const normsType* _normalsIndexes = m_normals->chunkStartPtr(chunkIndex);
		unsigned chunkSize = m_normals->chunkSize(chunkIndex);

		//compressed normals set
		const ccNormalVectors* compressedNormals = ccNormalVectors::GetUniqueInstance();
		assert(compressedNormals);

		for (unsigned j=0; j<chunkSize; j+=decimStep,_normalsIndexes+=decimStep)
		{
			const CCVector3& N = compressedNormals->getNormal(*_normalsIndexes);
			*(_normals)++ = N.x;
			*(_normals)++ = N.y;
			*(_normals)++ = N.z;
		}
		glNormalPointer(GL_COORD_TYPE,0,s_normBuffer);
	}
}

void ccPointCloud::glChunkColorPointer(unsigned chunkIndex, unsigned decimStep, bool useVBOs)
{
	assert(m_rgbColors);
	assert(sizeof(colorType) == 1);

	if (	useVBOs
		&&	m_vboManager.state == vboSet::INITIALIZED
		&&	m_vboManager.hasColors
		&&	m_vboManager.vbos.size() > static_cast<size_t>(chunkIndex)
		&&	m_vboManager.vbos[chunkIndex]
		&&	m_vboManager.vbos[chunkIndex]->isCreated())
	{
		//we can use VBOs directly
		if (m_vboManager.vbos[chunkIndex]->bind())
		{
			const GLbyte* start = 0; //fake pointer used to prevent warnings on Linux
			int colorDataShift = m_vboManager.vbos[chunkIndex]->rgbShift;
			glColorPointer(3,GL_UNSIGNED_BYTE,decimStep*3*sizeof(colorType),(const GLvoid*)(start + colorDataShift));
			m_vboManager.vbos[chunkIndex]->release();
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			m_vboManager.state = vboSet::FAILED;
			//recall the method
			glChunkColorPointer(chunkIndex, decimStep, false);
		}
	}
	else
	{
		assert(m_rgbColors && m_rgbColors->chunkStartPtr(chunkIndex));
		//standard OpenGL copy
		glColorPointer(3,GL_UNSIGNED_BYTE,decimStep*3*sizeof(colorType),m_rgbColors->chunkStartPtr(chunkIndex));
	}
}

void ccPointCloud::glChunkSFPointer(unsigned chunkIndex, unsigned decimStep, bool useVBOs)
{
	assert(m_currentDisplayedScalarField);
	assert(sizeof(colorType) == 1);

	if (	useVBOs
		&&	m_vboManager.state == vboSet::INITIALIZED
		&&	m_vboManager.hasColors
		&&	m_vboManager.vbos.size() > static_cast<size_t>(chunkIndex)
		&&	m_vboManager.vbos[chunkIndex]
		&&	m_vboManager.vbos[chunkIndex]->isCreated())
	{
		assert(m_vboManager.colorIsSF && m_vboManager.sourceSF == m_currentDisplayedScalarField);
		//we can use VBOs directly
		if (m_vboManager.vbos[chunkIndex]->bind())
		{
			const GLbyte* start = 0; //fake pointer used to prevent warnings on Linux
			int colorDataShift = m_vboManager.vbos[chunkIndex]->rgbShift;
			glColorPointer(3,GL_UNSIGNED_BYTE,decimStep*3*sizeof(colorType),(const GLvoid*)(start + colorDataShift));
			m_vboManager.vbos[chunkIndex]->release();
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			m_vboManager.state = vboSet::FAILED;
			//recall the method
			glChunkSFPointer(chunkIndex, decimStep, false);
		}
	}
	else
	{
		assert(m_rgbColors && m_rgbColors->chunkStartPtr(chunkIndex));
		assert(m_currentDisplayedScalarField);
		//we must decode normals in a dedicated static array
		ScalarType* _sf = m_currentDisplayedScalarField->chunkStartPtr(chunkIndex);
		colorType* _sfColors = s_rgbBuffer3ub;
		unsigned chunkSize = m_currentDisplayedScalarField->chunkSize(chunkIndex);
		for (unsigned j=0; j<chunkSize; j+=decimStep,_sf+=decimStep)
		{
			//we need to convert scalar value to color into a temporary structure
			const colorType* col = m_currentDisplayedScalarField->getColor(*_sf);
			assert(col);
			*_sfColors++ = *col++;
			*_sfColors++ = *col++;
			*_sfColors++ = *col++;
		}
		glColorPointer(3,GL_UNSIGNED_BYTE,0,s_rgbBuffer3ub);
	}
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
		//no normals shading without light!
		if (!MACRO_LightIsEnabled(context))
			glParams.showNorms = false;

		//can't display a SF without... a SF... and an active color scale!
		assert(!glParams.showSF || hasDisplayedScalarField());

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

			glPushName(getUniqueIDForDisplay());
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
			glParams.showColors = false;
		}
		else
		{
			glColor3ubv(context.pointsDefaultCol);
		}

		//in the case we need normals (i.e. lighting)
		if (glParams.showNorms)
		{
			//DGM: Strangely, when Qt::renderPixmap is called, the OpenGL version is sometimes 1.0!
			glEnable((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_1_2 ? GL_RESCALE_NORMAL : GL_NORMALIZE));
			glMaterialfv(GL_FRONT_AND_BACK,	GL_AMBIENT,		CC_DEFAULT_CLOUD_AMBIENT_COLOR  );
			glMaterialfv(GL_FRONT_AND_BACK,	GL_SPECULAR,	CC_DEFAULT_CLOUD_SPECULAR_COLOR );
			glMaterialfv(GL_FRONT_AND_BACK,	GL_DIFFUSE,		CC_DEFAULT_CLOUD_DIFFUSE_COLOR  );
			glMaterialfv(GL_FRONT_AND_BACK,	GL_EMISSION,	CC_DEFAULT_CLOUD_EMISSION_COLOR );
			glMaterialf (GL_FRONT_AND_BACK,	GL_SHININESS,	CC_DEFAULT_CLOUD_SHININESS);
			glEnable(GL_LIGHTING);

			if (glParams.showSF)
			{
				//we must get rid of lights 'color' if a scalar field is displayed!
				glPushAttrib(GL_LIGHTING_BIT);
				ccMaterial::MakeLightsNeutral();
			}
		}

		// L.O.D.
		unsigned numberOfPoints = size();
		unsigned decimStep = 1;
		if (numberOfPoints > MAX_LOD_POINTS_NUMBER && context.decimateCloudOnMove &&  MACRO_LODActivated(context))
		{
			decimStep = static_cast<int>(ceil(static_cast<float>(numberOfPoints) / MAX_LOD_POINTS_NUMBER));
		}

		/*** DISPLAY ***/

		//custom point size?
		glPushAttrib(GL_POINT_BIT);
		if (m_pointSize != 0)
			glPointSize(static_cast<GLfloat>(m_pointSize));

		if (!pushPointNames) //standard "full" display
		{
			//if some points are hidden (= visibility table instantiated), we can't use display arrays :(
			if (isVisibilityTableInstantiated())
			{
				//compressed normals set
				const ccNormalVectors* compressedNormals = ccNormalVectors::GetUniqueInstance();
				assert(compressedNormals);

				glBegin(GL_POINTS);

				for (unsigned j=0; j<numberOfPoints; j+=decimStep)
				{
					//we must test each point visibility
					if (m_pointsVisibility->getValue(j) == POINT_VISIBLE)
					{
						if (glParams.showSF)
						{
							assert(j < m_currentDisplayedScalarField->currentSize());
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
							ccGL::Normal3v(compressedNormals->getNormal(m_normals->getValue(j)).u);
						}
						ccGL::Vertex3v(m_points->getValue(j));
					}
				}

				glEnd();
			}
			else if (glParams.showSF) //no visibility table enabled + scalar field
			{
				assert(m_currentDisplayedScalarField);

				//if some points may not be displayed, we'll have to be smarter!
				bool hiddenPoints = m_currentDisplayedScalarField->mayHaveHiddenValues();

				//whether VBOs are available (faster)
				bool useVBOs = false;
				if (!hiddenPoints && context.useVBOs)
				{
					//can't use VBOs if some points are hidden
					useVBOs = updateVBOs(glParams);
				}

				//color ramp shader initialization
				ccColorRampShader* colorRampShader = context.colorRampShader;
				{
					//color ramp shader is not compatible with VBOs (and VBOs are faster)
					if (useVBOs)
					{
						colorRampShader = 0;
					}
					//FIXME: color ramp shader doesn't support log scale yet!
					if (m_currentDisplayedScalarField->logScale())
					{
						colorRampShader = 0;
					}
				}

				const ccScalarField::Range& sfDisplayRange = m_currentDisplayedScalarField->displayRange();
				const ccScalarField::Range& sfSaturationRange = m_currentDisplayedScalarField->saturationRange();

				if (colorRampShader)
				{
					//max available space for frament's shader uniforms
					GLint maxBytes = 0;
					glGetIntegerv(GL_MAX_FRAGMENT_UNIFORM_COMPONENTS,&maxBytes);
					GLint maxComponents = (maxBytes>>2)-4; //leave space for the other uniforms!
					unsigned steps = m_currentDisplayedScalarField->getColorRampSteps();
					assert(steps != 0);

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
							//An error occurred during shader initialization?
							ccLog::WarningDebug("Failed to init ColorRamp shader!");
							colorRampShader->stop();
							colorRampShader = 0;
						}
						else if (glParams.showNorms)
						{
							//we must get rid of lights material (other than ambient) for the red and green fields
							glPushAttrib(GL_LIGHTING_BIT);

							//we use the ambient light to pass the scalar value (and 'grayed' marker) without any
							//modification from the GPU pipeline, even if normals are enabled!
							glDisable(GL_COLOR_MATERIAL);
							glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
							glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
							glEnable(GL_COLOR_MATERIAL);

							GLint maxLightCount;
							glGetIntegerv(GL_MAX_LIGHTS,&maxLightCount);
							for (GLint i=0; i<maxLightCount; ++i)
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
					if (glParams.showNorms)
						glEnableClientState(GL_NORMAL_ARRAY);

					unsigned chunks = m_points->chunksCount();
					for (unsigned k=0; k<chunks; ++k)
					{
						unsigned chunkSize = m_points->chunkSize(k);

						//points
						glChunkVertexPointer(k,decimStep,useVBOs);
						//normals
						if (glParams.showNorms)
							glChunkNormalPointer(k,decimStep,useVBOs);
						//SF colors
						if (colorRampShader)
						{
							ScalarType* _sf = m_currentDisplayedScalarField->chunkStartPtr(k);
							float* _sfColors = s_rgbBuffer3f;
							if (!m_currentDisplayedScalarField->symmetricalScale())
							{
								for (unsigned j=0; j<chunkSize; j+=decimStep,_sf+=decimStep,_sfColors+=3)
								{
									_sfColors[0] = GetNormalizedValue(*_sf,sfDisplayRange);					//normalized sf value
									_sfColors[1] = sfDisplayRange.isInRange(*_sf) ? 1.0f : 0.0f;			//flag: whether point is grayed out or not (NaN values are also rejected!)
									_sfColors[2] = 1.0f;													//reference value (to get the true lighting value)
								}
							}
							else //symmetrical scale
							{
								//we must handle the values between -minSat et +minSat 'manually'
								for (unsigned j=0; j<chunkSize; j+=decimStep,_sf+=decimStep,_sfColors+=3)
								{
									_sfColors[0] = GetSymmetricalNormalizedValue(*_sf,sfSaturationRange);	//normalized sf value
									_sfColors[1] = sfDisplayRange.isInRange(*_sf) ? 1.0f : 0.0f;			//flag: whether point is grayed out or not (NaN values are also rejected!)
									_sfColors[2] = 1.0f;													//reference value (to get the true lighting value)
								}
							}
							glColorPointer(3,GL_FLOAT,0,s_rgbBuffer3f);
						}
						else
						{
							glChunkSFPointer(k,decimStep,useVBOs);
						}

						if (decimStep > 1)
							chunkSize = static_cast<unsigned>( floor(static_cast<float>(chunkSize)/decimStep) );
						glDrawArrays(GL_POINTS,0,chunkSize);
					}

					if (glParams.showNorms)
						glDisableClientState(GL_NORMAL_ARRAY);
					glDisableClientState(GL_COLOR_ARRAY);
					glDisableClientState(GL_VERTEX_ARRAY);
				}
				else //potentially hidden points
				{
					//compressed normals set
					const ccNormalVectors* compressedNormals = ccNormalVectors::GetUniqueInstance();
					assert(compressedNormals);

					glBegin(GL_POINTS);

					if (glParams.showNorms) //with normals (slowest case!)
					{
						if (colorRampShader)
						{
							if (!m_currentDisplayedScalarField->symmetricalScale())
							{
								for (unsigned j=0; j<numberOfPoints; j+=decimStep)
								{
									assert(j < m_currentDisplayedScalarField->currentSize());
									const ScalarType sf = m_currentDisplayedScalarField->getValue(j);
									if (sfDisplayRange.isInRange(sf)) //NaN values are rejected
									{
										glColor3f(GetNormalizedValue(sf,sfDisplayRange),1.0f,1.0f);
										ccGL::Normal3v(compressedNormals->getNormal(m_normals->getValue(j)).u);
										ccGL::Vertex3v(m_points->getValue(j));
									}
								}
							}
							else
							{
								for (unsigned j=0; j<numberOfPoints; j+=decimStep)
								{
									assert(j < m_currentDisplayedScalarField->currentSize());
									const ScalarType sf = m_currentDisplayedScalarField->getValue(j);
									if (sfDisplayRange.isInRange(sf)) //NaN values are rejected
									{
										glColor3f(GetSymmetricalNormalizedValue(sf,sfSaturationRange),1.0f,1.0f);
										ccGL::Normal3v(compressedNormals->getNormal(m_normals->getValue(j)).u);
										ccGL::Vertex3v(m_points->getValue(j));
									}
								}
							}
						}
						else
						{
							for (unsigned j=0; j<numberOfPoints; j+=decimStep)
							{
								assert(j < m_currentDisplayedScalarField->currentSize());
								const colorType* col = m_currentDisplayedScalarField->getValueColor(j);
								if (col)
								{
									glColor3ubv(col);
									ccGL::Normal3v(compressedNormals->getNormal(m_normals->getValue(j)).u);
									ccGL::Vertex3v(m_points->getValue(j));
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
								for (unsigned j=0; j<numberOfPoints; j+=decimStep)
								{
									assert(j < m_currentDisplayedScalarField->currentSize());
									const ScalarType sf = m_currentDisplayedScalarField->getValue(j);
									if (sfDisplayRange.isInRange(sf)) //NaN values are rejected
									{
										glColor3f(GetNormalizedValue(sf,sfDisplayRange),1.0f,1.0f);
										ccGL::Vertex3v(m_points->getValue(j));
									}
								}
							}
							else
							{
								for (unsigned j=0; j<numberOfPoints; j+=decimStep)
								{
									assert(j < m_currentDisplayedScalarField->currentSize());
									const ScalarType sf = m_currentDisplayedScalarField->getValue(j);
									if (sfDisplayRange.isInRange(sf)) //NaN values are rejected
									{
										glColor3f(GetSymmetricalNormalizedValue(sf,sfSaturationRange),1.0f,1.0f);
										ccGL::Vertex3v(m_points->getValue(j));
									}
								}
							}
						}
						else
						{
							for (unsigned j=0; j<numberOfPoints; j+=decimStep)
							{
								assert(j < m_currentDisplayedScalarField->currentSize());
								const colorType* col = m_currentDisplayedScalarField->getValueColor(j);
								if (col)
								{
									glColor3ubv(col);
									ccGL::Vertex3v(m_points->getValue(j));
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
			else //no visibility table enabled, no scalar field
			{
				bool useVBOs = context.useVBOs ? updateVBOs(glParams) : false;

				unsigned chunks = m_points->chunksCount();

				glEnableClientState(GL_VERTEX_ARRAY);
				if (glParams.showNorms)
					glEnableClientState(GL_NORMAL_ARRAY);
				if (glParams.showColors)
					glEnableClientState(GL_COLOR_ARRAY);

				for (unsigned k=0; k<chunks; ++k)
				{
					unsigned chunkSize = m_points->chunkSize(k);

					//points
					glChunkVertexPointer(k,decimStep,useVBOs);
					//normals
					if (glParams.showNorms)
						glChunkNormalPointer(k,decimStep,useVBOs);
					//colors
					if (glParams.showColors)
						glChunkColorPointer(k,decimStep,useVBOs);

					if (decimStep > 1)
						chunkSize = static_cast<unsigned>(floor(static_cast<float>(chunkSize)/decimStep));
					glDrawArrays(GL_POINTS,0,chunkSize);
				}

				glDisableClientState(GL_VERTEX_ARRAY);
				if (glParams.showNorms)
					glDisableClientState(GL_NORMAL_ARRAY);
				if (glParams.showColors)
					glDisableClientState(GL_COLOR_ARRAY);
			}
		}
		else //special case: point names pushing (for picking) --> no need for colors, normals, etc.
		{
			glPushName(0);
			//however we must take hidden points into account!
			if (isVisibilityTableInstantiated())
			{
				for (unsigned j=0; j<numberOfPoints; j+=decimStep)
				{
					if (m_pointsVisibility->getValue(j) == POINT_VISIBLE)
					{
						glLoadName(j);
						glBegin(GL_POINTS);
						ccGL::Vertex3v(m_points->getValue(j));
						glEnd();
					}
				}
			}
			else //no visibility table instantiated...
			{
				//... but potentially points with NAN SF values (also hidden!)
				bool hiddenPoints = false;
				if (glParams.showSF)
				{
					assert(m_currentDisplayedScalarField);
					hiddenPoints = m_currentDisplayedScalarField->mayHaveHiddenValues() && m_currentDisplayedScalarField->getColorScale();
				}
				
				if (hiddenPoints) //potentially hidden points
				{
					for (unsigned j=0; j<numberOfPoints; j+=decimStep)
					{
						//we must generate the synthetic "color" of each point
						const colorType* col = getPointScalarValueColor(j);
						if (col)
						{
							glLoadName(j);
							glBegin(GL_POINTS);
							ccGL::Vertex3v(m_points->getValue(j));
							glEnd();
						}
					}
				}
				else
				{
					//no hidden point
					for (unsigned j=0; j<numberOfPoints; j+=decimStep)
					{
						glLoadName(j);
						glBegin(GL_POINTS);
						ccGL::Vertex3v(m_points->getValue(j));
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
	if (sfIdx < 0)
		return;

	context.sfColorScaleToDisplay = static_cast<ccScalarField*>(getScalarField(sfIdx));
}

ccPointCloud* ccPointCloud::filterPointsByScalarValue(ScalarType minVal, ScalarType maxVal)
{
	if (!getCurrentOutScalarField())
		return 0;

	QSharedPointer<CCLib::ReferenceCloud> c(CCLib::ManualSegmentationTools::segment(this,minVal,maxVal));

	return (c ? partialClone(c.data()) : 0);
}

void ccPointCloud::hidePointsByScalarValue(ScalarType minVal, ScalarType maxVal)
{
	if (!resetVisibilityArray())
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
	for (unsigned i=0; i<count; ++i)
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
			//a warning message has already been issued by getTheVisiblePoints!
			//ccLog::Warning("[ccPointCloud::createNewCloudFromVisibilitySelection] An error occurred during points selection!");
			return 0;
		}
		assert(rc->size() != 0);

		//convert selection to cloud
		result = partialClone(rc);

		//don't need this one anymore
		delete rc;
		rc = 0;
	}

	if (!result)
	{
		ccLog::Warning("[ccPointCloud::createNewCloudFromVisibilitySelection] An error occurred during segmentation!");
		return 0;
	}

	result->setName(getName()+QString(".segmented"));

	//shall the visible points be erased from this cloud?
	if (removeSelectedPoints && !isLocked())
	{
		//we drop the octree before modifying this cloud's contents
		deleteOctree();

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

		//TODO: handle associated meshes

		resize(lastPoint);
		
		refreshBB(); //calls notifyGeometryUpdate + releaseVBOs
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
	m_currentDisplayedScalarFieldIndex = index;
	m_currentDisplayedScalarField = static_cast<ccScalarField*>(getScalarField(index));

	if (m_currentDisplayedScalarFieldIndex >= 0 && m_currentDisplayedScalarField)
		setCurrentOutScalarField(m_currentDisplayedScalarFieldIndex);
}

void ccPointCloud::deleteScalarField(int index)
{
	//we 'store' the currently displayed SF, as the SF order may be mixed up
	setCurrentInScalarField(m_currentDisplayedScalarFieldIndex);

	//the father does all the work
	ChunkedPointCloud::deleteScalarField(index);

	//current SF should still be up-to-date!
	if (m_currentInScalarFieldIndex < 0 && getNumberOfScalarFields() > 0)
		setCurrentInScalarField((int)getNumberOfScalarFields()-1);

	setCurrentDisplayedScalarField(m_currentInScalarFieldIndex);
	showSF(m_currentInScalarFieldIndex >= 0);
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
	if (!hasDisplayedScalarField())
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

	//We must update the VBOs
	releaseVBOs();

	return true;
}

bool ccPointCloud::interpolateColorsFrom(	ccGenericPointCloud* cloud,
											CCLib::GenericProgressCallback* progressCb/*=NULL*/,
											unsigned char octreeLevel/*=7*/)
{
	if (!cloud || cloud->size() == 0)
	{
		ccLog::Warning("[ccPointCloud::interpolateColorsFrom] Invalid/empty input cloud!");
		return false;
	}

	//check that both bounding boxes intersect!
	ccBBox box = getBB();
	ccBBox otherBox = cloud->getBB();

	CCVector3 dimSum = box.getDiagVec() + otherBox.getDiagVec();
	CCVector3 dist = box.getCenter() - otherBox.getCenter();
	if (	fabs(dist.x) > dimSum.x / 2
		||	fabs(dist.y) > dimSum.y / 2
		||	fabs(dist.z) > dimSum.z / 2)
	{
		ccLog::Warning("[ccPointCloud::interpolateColorsFrom] Clouds are too far from each other! Can't proceed.");
		return false;
	}

	if (!resizeTheRGBTable(false))
	{
		ccLog::Warning("[ccPointCloud::interpolateColorsFrom] Not enough memory!");
		return false;
	}

	//compute the closest-point set of 'this cloud' relatively to 'input cloud'
	//(to get a mapping between the resulting vertices and the input points)
	int result = 0;
	CCLib::ReferenceCloud CPSet(cloud);
	{
		CCLib::DistanceComputationTools::Cloud2CloudDistanceComputationParams params;
		params.CPSet = &CPSet;
		params.octreeLevel = octreeLevel; //TODO: find a better way to set the right octree level!

		result = CCLib::DistanceComputationTools::computeHausdorffDistance(this, cloud, params, progressCb);
	}

	if (result < 0)
	{
		ccLog::Warning("[ccPointCloud::interpolateColorsFrom] Closest-point set computation failed!");
		unallocateColors();
		return false;
	}
		
	//import colors
	unsigned CPsize = CPSet.size();
	assert(CPsize == size());
	for (unsigned i=0; i<CPsize; ++i)
	{
		unsigned index = CPSet.getPointGlobalIndex(i);
		setPointColor(i,cloud->getPointColor(index));
	}

	//We must update the VBOs
	releaseVBOs();

	return true;
}

void ccPointCloud::unrollOnCylinder(PointCoordinateType radius,
									CCVector3* center,
									unsigned char dim/*=2*/,
									CCLib::GenericProgressCallback* progressCb/*=NULL*/)
{
	assert(dim <= 2);
	uchar dim1 = (dim<2 ? dim+1 : 0);
	uchar dim2 = (dim1<2 ? dim1+1 : 0);

	unsigned numberOfPoints = size();

	CCLib::NormalizedProgress* nprogress = 0;
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

	for (unsigned i=0; i<numberOfPoints; i++)
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
			const CCVector3& N = ccNormalVectors::GetNormal(m_normals->getValue(i));

			PointCoordinateType px = P0+N.u[dim1];
			PointCoordinateType py = P1+N.u[dim2];
			PointCoordinateType nlon = atan2(px,py);
			PointCoordinateType nu = sqrt(px*px+py*py);

			CCVector3 n2;
			n2.u[dim1] = (nlon-lon)*radius;
			n2.u[dim2] = nu - u;
			n2.u[dim]  = N.u[dim];

			n2.normalize();
			setPointNormal(i,n2);
		}

		//process canceled by user?
		if (nprogress && !nprogress->oneStep())
			break;
	}

	refreshBB(); //calls notifyGeometryUpdate + releaseVBOs

	if (progressCb)
		progressCb->stop();
}

void ccPointCloud::unrollOnCone(PointCoordinateType baseRadius,
								double alpha_deg,
								const CCVector3& apex,
								unsigned char dim/*=2*/,
								CCLib::GenericProgressCallback* progressCb/*=NULL*/)
{
	assert(dim < 3);
	uchar dim1 = (dim<2 ? dim+1 : 0);
	uchar dim2 = (dim1<2 ? dim1+1 : 0);

	unsigned numberOfPoints = size();

	CCLib::NormalizedProgress* nprogress = 0;
	if (progressCb)
	{
		progressCb->reset();
		nprogress = new CCLib::NormalizedProgress(progressCb,numberOfPoints);
		progressCb->setMethodTitle("Unroll (cone)");
		progressCb->setInfo(qPrintable(QString("Number of points = %1").arg(numberOfPoints)));
		progressCb->start();
	}

	PointCoordinateType tan_alpha = static_cast<PointCoordinateType>( tan(alpha_deg*CC_DEG_TO_RAD) );
	PointCoordinateType cos_alpha = static_cast<PointCoordinateType>( cos(alpha_deg*CC_DEG_TO_RAD) );
	PointCoordinateType sin_alpha = static_cast<PointCoordinateType>( sin(alpha_deg*CC_DEG_TO_RAD) );

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
			const CCVector3& N = ccNormalVectors::GetNormal(m_normals->getValue(i));

			PointCoordinateType dX = cos(lon)*N.u[dim1]-sin(lon)*N.u[dim2];
			PointCoordinateType dZ = sin(lon)*N.u[dim1]+cos(lon)*N.u[dim2];

			CCVector3 n2;
			n2.u[dim1] = dX;
			n2.u[dim2] = cos_alpha*dZ-sin_alpha*N.u[dim];
			n2.u[dim]  = sin_alpha*dZ+cos_alpha*N.u[dim];
			n2.normalize();

			setPointNormal(i,n2);
		}

		//process canceled by user?
		if (nprogress && !nprogress->oneStep())
			break;
	}

	refreshBB(); //calls notifyGeometryUpdate + releaseVBOs

	if (nprogress)
	{
		delete nprogress;
		nprogress = 0;
	}
}

int ccPointCloud::addScalarField(const char* uniqueName)
{
	//create new scalar field
	ccScalarField* sf = new ccScalarField(uniqueName);

	int sfIdx = addScalarField(sf);

	//failure?
	if (sfIdx < 0)
	{
		sf->release();
		return -1;
	}

	return sfIdx;
}

int ccPointCloud::addScalarField(ccScalarField* sf)
{
	assert(sf);

	//we don't accept two SFs with the same name!
	if (getScalarFieldIndexByName(sf->getName()) >= 0)
	{
		ccLog::Warning(QString("[ccPointCloud::addScalarField] Name '%1' already exists!").arg(sf->getName()));
		return -1;
	}

	//auto-resize
	if (sf->currentSize() < m_points->capacity())
	{
		if (!sf->resize(m_points->capacity()))
		{
			ccLog::Warning("[ccPointCloud::addScalarField] Not enough memory!");
			return -1;
		}
	}

	try
	{
		m_scalarFields.push_back(sf);
	}
	catch(std::bad_alloc)
	{
		ccLog::Warning("[ccPointCloud::addScalarField] Not enough memory!");
		sf->release();
		return -1;
	}

	sf->link();

	return static_cast<int>(m_scalarFields.size())-1;
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
		if (out.write((const char*)&hasColorsArray,sizeof(bool)) < 0)
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
		if (out.write((const char*)&hasNormalsArray,sizeof(bool)) < 0)
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
		if (out.write((const char*)&sfCount,4) < 0)
			return WriteError();

		//scalar fields (dataVersion>=20)
		for (uint32_t i=0; i<sfCount; ++i)
		{
			ccScalarField* sf = static_cast<ccScalarField*>(getScalarField(i));
			assert(sf);
			if (!sf || !sf->toFile(out))
				return false;
		}

		//'show NaN values in grey' state (27>dataVersion>=20)
		//if (out.write((const char*)&m_greyForNanScalarValues,sizeof(bool)) < 0)
		//	return WriteError();

		//'show current sf color scale' state (dataVersion>=20)
		if (out.write((const char*)&m_sfColorScaleDisplayed,sizeof(bool)) < 0)
			return WriteError();

		//Displayed scalar field index (dataVersion>=20)
		int32_t displayedScalarFieldIndex = (int32_t)m_currentDisplayedScalarFieldIndex;
		if (out.write((const char*)&displayedScalarFieldIndex,4) < 0)
			return WriteError();
	}

	return true;
}

bool ccPointCloud::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccGenericPointCloud::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//points array (dataVersion>=20)
	if (!m_points)
	{
		return ccLog::Error("Internal error: point cloud has no valid point array! (not enough memory?)");
	}
	else
	{
		bool result = false;
		bool fileCoordIsDouble = (flags & ccSerializableObject::DF_POINT_COORDS_64_BITS);
		if (!fileCoordIsDouble && sizeof(PointCoordinateType) == 8) //file is 'float' and current type is 'double'
		{
			result = ccSerializationHelper::GenericArrayFromTypedFile<3,PointCoordinateType,float>(*m_points,in,dataVersion);
		}
		else if (fileCoordIsDouble && sizeof(PointCoordinateType) == 4) //file is 'double' and current type is 'float'
		{
			result = ccSerializationHelper::GenericArrayFromTypedFile<3,PointCoordinateType,double>(*m_points,in,dataVersion);
		}
		else
		{
			result = ccSerializationHelper::GenericArrayFromFile(*m_points,in,dataVersion);
		}
		if (!result)
			return false;

#ifdef _DEBUG
		//test: look for NaN values
		{
			unsigned nanPointsCount = 0;
			for (unsigned i=0; i<size(); ++i)
			{
				if (	point(i)->x != point(i)->x
					||	point(i)->y != point(i)->y
					||	point(i)->z != point(i)->z )
				{
					*point(i) = CCVector3(0,0,0);
					++nanPointsCount;
				}
			}

			if (nanPointsCount)
			{
				ccLog::Warning(QString("[BIN] Cloud '%1' contains %2 NaN point(s)!").arg(getName()).arg(nanPointsCount));
			}
		}
#endif
	}

	//colors array (dataVersion>=20)
	{
		bool hasColorsArray = false;
		if (in.read((char*)&hasColorsArray,sizeof(bool)) < 0)
			return ReadError();
		if (hasColorsArray)
		{
			if (!m_rgbColors)
			{
				m_rgbColors = new ColorsTableType;
				m_rgbColors->link();
			}
			CC_CLASS_ENUM classID = ReadClassIDFromFile(in, dataVersion);
			if (classID != CC_TYPES::RGB_COLOR_ARRAY)
				return CorruptError();
			if (!m_rgbColors->fromFile(in, dataVersion, flags))
			{
				unallocateColors();
				return false;
			}
		}
	}

	//normals array (dataVersion>=20)
	{
		bool hasNormalsArray = false;
		if (in.read((char*)&hasNormalsArray,sizeof(bool)) < 0)
			return ReadError();
		if (hasNormalsArray)
		{
			if (!m_normals)
			{
				m_normals = new NormsIndexesTableType();
				m_normals->link();
			}
			CC_CLASS_ENUM classID = ReadClassIDFromFile(in, dataVersion);
			if (classID != CC_TYPES::NORMAL_INDEXES_ARRAY)
				return CorruptError();
			if (!m_normals->fromFile(in, dataVersion, flags))
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
		if (in.read((char*)&sfCount,4) < 0)
			return ReadError();

		//scalar fields (dataVersion>=20)
		for (uint32_t i=0; i<sfCount; ++i)
		{
			ccScalarField* sf = new ccScalarField();
			if (!sf->fromFile(in, dataVersion, flags))
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
			if (in.read((char*)&greyForNanScalarValues,sizeof(bool)) < 0)
				return ReadError();

			//update all scalar fields accordingly (old way)
			for (unsigned i=0; i<getNumberOfScalarFields(); ++i)
			{
				static_cast<ccScalarField*>(getScalarField(i))->showNaNValuesInGrey(greyForNanScalarValues);
			}
		}

		//'show current sf color scale' state (dataVersion>=20)
		if (in.read((char*)&m_sfColorScaleDisplayed,sizeof(bool)) < 0)
			return ReadError();

		//Displayed scalar field index (dataVersion>=20)
		int32_t displayedScalarFieldIndex = 0;
		if (in.read((char*)&displayedScalarFieldIndex,4) < 0)
			return ReadError();
		if (displayedScalarFieldIndex<(int32_t)sfCount)
			setCurrentDisplayedScalarField(displayedScalarFieldIndex);
	}

	//notifyGeometryUpdate(); //FIXME: we can't call it now as the dependent 'pointers' are not valid yet!

	//We should update the VBOs (just in case)
	releaseVBOs();

	return true;
}

unsigned ccPointCloud::getUniqueIDForDisplay() const
{
	if (m_parent && m_parent->isA(CC_TYPES::FACET))
		return m_parent->getUniqueID();
	else
		return getUniqueID();
}

CCLib::ReferenceCloud* ccPointCloud::crop(const ccBBox& box, bool inside/*=true*/)
{
	if (!box.isValid())
	{
		ccLog::Warning("[ccPointCloud::crop] Invalid bounding-box");
		return 0;
	}

	unsigned count = size();
	if (count == 0)
	{
		ccLog::Warning("[ccPointCloud::crop] Cloud is empty!");
		return 0;
	}

	CCLib::ReferenceCloud* ref = new CCLib::ReferenceCloud(this);
	if (!ref->reserve(count))
	{
		ccLog::Warning("[ccPointCloud::crop] Not enough memory!");
		delete ref;
		return 0;
	}

	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = point(i);
		bool pointIsInside = box.contains(*P);
		if (inside == pointIsInside)
		{
			ref->addPointIndex(i);
		}
	}

	if (ref->size() == 0)
	{
		//no points inside selection!
		ref->clear(true);
	}
	else
	{
		ref->resize(ref->size());
	}

	return ref;
}

CCLib::ReferenceCloud* ccPointCloud::crop2D(const ccPolyline* poly, unsigned char orthoDim, bool inside/*=true*/)
{
	if (!poly)
	{
		ccLog::Warning("[ccPointCloud::crop2D] Invalid input polyline");
		return 0;
	}
	if (orthoDim > 2)
	{
		ccLog::Warning("[ccPointCloud::crop2D] Invalid input polyline");
		return 0;
	}

	unsigned count = size();
	if (count == 0)
	{
		ccLog::Warning("[ccPointCloud::crop] Cloud is empty!");
		return 0;
	}

	CCLib::ReferenceCloud* ref = new CCLib::ReferenceCloud(this);
	if (!ref->reserve(count))
	{
		ccLog::Warning("[ccPointCloud::crop] Not enough memory!");
		delete ref;
		return 0;
	}

	unsigned char X = ((orthoDim+1) % 3);
	unsigned char Y = ((X+1) % 3);

	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = point(i);

		CCVector2 P2D( P->u[X], P->u[Y] );
		bool pointIsInside = CCLib::ManualSegmentationTools::isPointInsidePoly(P2D,poly);
		if (inside == pointIsInside)
		{
			ref->addPointIndex(i);
		}
	}

	if (ref->size() == 0)
	{
		//no points inside selection!
		ref->clear(true);
	}
	else
	{
		ref->resize(ref->size());
	}

	return ref;
}

static bool CatchGLErrors(const char* context)
{
	//catch GL errors
	{
		GLenum err = glGetError();

		//see http://www.opengl.org/sdk/docs/man/xhtml/glGetError.xml
		switch(err)
		{
		case GL_NO_ERROR:
			return false;
		case GL_INVALID_ENUM:
			ccLog::Warning("[%s] OpenGL error: invalid enumerator",context);
			break;
		case GL_INVALID_VALUE:
			ccLog::Warning("[%s] OpenGL error: invalid value",context);
			break;
		case GL_INVALID_OPERATION:
			ccLog::Warning("[%s] OpenGL error: invalid operation",context);
			break;
		case GL_STACK_OVERFLOW:
			ccLog::Warning("[%s] OpenGL error: stack overflow",context);
			break;
		case GL_STACK_UNDERFLOW:
			ccLog::Warning("[%s] OpenGL error: stack underflow",context);
			break;
		case GL_OUT_OF_MEMORY:
			ccLog::Warning("[%s] OpenGL error: out of memory",context);
			break;
		case GL_INVALID_FRAMEBUFFER_OPERATION:
			ccLog::Warning("[%s] OpenGL error: invalid framebuffer operation",context);
			break;
		}
	}

	return true;
}

//DGM: normals are so slow that it's a waste of memory and time to load them in VBOs!
#define DONT_LOAD_NORMALS_IN_VBOS

bool ccPointCloud::updateVBOs(const glDrawParams& glParams)
{
	if (isColorOverriden())
	{
		//nothing to do (we don't display true colors, SF or normals!)
		return false;
	}

	if (m_vboManager.state == vboSet::FAILED)
	{
		//ccLog::Warning(QString("[ccPointCloud::updateVBOs] VBOs are in a 'failed' state... we won't try to update them! (cloud '%1')").arg(getName()));
		return false;
	}

	if (!m_currentDisplay)
	{
		ccLog::Warning(QString("[ccPointCloud::updateVBOs] Need an associated GL context! (cloud '%1')").arg(getName()));
		assert(false);
		return false;
	}

	//fiels to init/update
	enum UPDATE_FIELDS {	UPDATE_POINTS	= 1,
							UPDATE_COLORS	= 2,
							UPDATE_NORMALS	= 4,
	};
	int updateFlags = 0;

	if (m_vboManager.state == vboSet::INITIALIZED)
	{
		//let's check if something has changed
		if ( glParams.showColors && ( !m_vboManager.hasColors || m_vboManager.colorIsSF ) )
		{
			updateFlags |= UPDATE_COLORS;
		}
		
		if (	glParams.showSF
		&& (		!m_vboManager.hasColors
				||	!m_vboManager.colorIsSF
				||	 m_vboManager.sourceSF != m_currentDisplayedScalarField
				||	 m_currentDisplayedScalarField->getModificationFlag() == true ) )
		{
			updateFlags |= UPDATE_COLORS;
		}

#ifndef DONT_LOAD_NORMALS_IN_VBOS
		if ( glParams.showNorms && !m_vboManager.hasNormals )
		{
			updateFlags |= UPDATE_NORMALS;
		}
#endif
		//nothing to do?
		if (updateFlags == 0)
			return true;
	}
	else
	{
		updateFlags = UPDATE_POINTS | UPDATE_COLORS | UPDATE_NORMALS;
	}

	size_t chunksCount = m_points->chunksCount();
	//allocate per-chunk descriptors if necessary
	if (m_vboManager.vbos.size() != chunksCount)
	{
		//properly remove the elements that are not needed anymore!
		for (size_t i=chunksCount; i<m_vboManager.vbos.size(); ++i)
		{
			if (m_vboManager.vbos[i])
			{
				m_vboManager.vbos[i]->destroy();
				delete m_vboManager.vbos[i];
				m_vboManager.vbos[i] = 0;
			}
		}

		//resize the container
		try
		{
			m_vboManager.vbos.resize(chunksCount,0);
		}
		catch(std::bad_alloc)
		{
			ccLog::Warning(QString("[ccPointCloud::updateVBOs] Not enough memory! (cloud '%1')").arg(getName()));
			m_vboManager.state = vboSet::FAILED;
			return false;
		}
	}

	//init VBOs
	unsigned pointsInVBOs = 0;
	int totalSizeBytesBefore = m_vboManager.totalMemSizeBytes;
	m_vboManager.totalMemSizeBytes = 0;
	{
		m_currentDisplay->makeContextCurrent();

		assert(!glParams.showSF		|| (m_currentDisplayedScalarField && m_currentDisplayedScalarField->chunksCount() >= chunksCount));
		assert(!glParams.showColors	|| (m_rgbColors && m_rgbColors->chunksCount() >= chunksCount));
#ifndef DONT_LOAD_NORMALS_IN_VBOS
		assert(!glParams.showNorms	|| (m_normals && m_normals->chunksCount() >= chunksCount));
#endif

		m_vboManager.hasColors  = glParams.showSF || glParams.showColors;
		m_vboManager.colorIsSF  = glParams.showSF;
		m_vboManager.sourceSF   = glParams.showSF ? m_currentDisplayedScalarField : 0;
#ifndef DONT_LOAD_NORMALS_IN_VBOS
		m_vboManager.hasNormals = glParams.showNorms;
#else
		m_vboManager.hasNormals  = false;
#endif

		//process each chunk
		for (unsigned i=0; i<chunksCount; ++i)
		{
			int chunkSize = static_cast<int>(m_points->chunkSize(i));

			int chunkUpdateFlags = updateFlags;
			bool reallocated = false;
			if (!m_vboManager.vbos[i])
				m_vboManager.vbos[i] = new VBO();

			//allocate memory for current VBO
			int vboSizeBytes = m_vboManager.vbos[i]->init(chunkSize,m_vboManager.hasColors,m_vboManager.hasNormals,&reallocated);
			if (vboSizeBytes > 0)
			{
				//ccLog::Print(QString("[VBO] VBO #%1 initialized (ID=%2)").arg(i).arg(m_vboManager.vbos[i]->bufferId()));

				if (reallocated)
				{
					//if the vbo is reallocated, then all its content has been cleared!
					chunkUpdateFlags = UPDATE_POINTS | UPDATE_COLORS | UPDATE_NORMALS;
				}

				m_vboManager.vbos[i]->bind();

				//load points
				if (chunkUpdateFlags & UPDATE_POINTS)
				{
					m_vboManager.vbos[i]->write(0,m_points->chunkStartPtr(i),sizeof(PointCoordinateType)*chunkSize*3);
				}
				//load colors
				if (chunkUpdateFlags & UPDATE_COLORS)
				{
					if (glParams.showSF)
					{
						//copy SF colors in static array
						{
							assert(m_vboManager.sourceSF);
							colorType* _sfColors = s_rgbBuffer3ub;
							ScalarType* _sf = m_vboManager.sourceSF->chunkStartPtr(i);
							assert(m_vboManager.sourceSF->chunkSize(i) == chunkSize);
							for (int j=0; j<chunkSize; j++,_sf++)
							{
								//we need to convert scalar value to color into a temporary structure
								const colorType* col = m_vboManager.sourceSF->getColor(*_sf);
								if (!col)
									col = ccColor::lightGrey;
								*_sfColors++ = *col++;
								*_sfColors++ = *col++;
								*_sfColors++ = *col++;
							}
						}
						//then send them in VRAM
						m_vboManager.vbos[i]->write(m_vboManager.vbos[i]->rgbShift,s_rgbBuffer3ub,sizeof(colorType)*chunkSize*3);
						//upadte 'modification' flag for current displayed SF
						m_vboManager.sourceSF->setModificationFlag(false);
					}
					else if (glParams.showColors)
					{
						m_vboManager.vbos[i]->write(m_vboManager.vbos[i]->rgbShift,m_rgbColors->chunkStartPtr(i),sizeof(colorType)*chunkSize*3);
					}
				}
#ifndef DONT_LOAD_NORMALS_IN_VBOS
				//load normals
				if (glParams.showNorms && (chunkUpdateFlags & UPDATE_NORMALS))
				{
					//we must decode the normals first!
					normsType* inNorms = m_normals->chunkStartPtr(i);
					PointCoordinateType* outNorms = s_normBuffer;
					for (int j=0; j<chunkSize; ++j)
					{
						const CCVector3& N = ccNormalVectors::GetNormal(*inNorms++);
						*(outNorms)++ = N.x;
						*(outNorms)++ = N.y;
						*(outNorms)++ = N.z;
					}
					m_vboManager.vbos[i]->write(m_vboManager.vbos[i]->normalShift,s_normBuffer,sizeof(PointCoordinateType)*chunkSize*3);
				}
#endif
				m_vboManager.vbos[i]->release();

				//if an error is detected
				if (CatchGLErrors("ccPointCloud::updateVBOs"))
				{
					vboSizeBytes = -1;
				}
				else
				{
					m_vboManager.totalMemSizeBytes += vboSizeBytes;
					pointsInVBOs += chunkSize;
				}
			}

			if (vboSizeBytes < 0) //VBO initialization failed
			{
				m_vboManager.vbos[i]->destroy();
				delete m_vboManager.vbos[i];
				m_vboManager.vbos[i] = 0;

				//we can stop here
				if (i == 0)
				{
					ccLog::Warning(QString("[ccPointCloud::updateVBOs] Failed to initialize VBOs (not enough memory?) (cloud '%1')").arg(getName()));
					m_vboManager.state = vboSet::FAILED;
					m_vboManager.vbos.clear();
					return false;
				}
				else
				{
					//shouldn't be better for the next VBOs!
					break;
				}
			}
		}
	}

	//Display vbo(s) status
	//{
	//	for (unsigned i=0; i<chunksCount; ++i)
	//		ccLog::Print(QString("[VBO] VBO #%1 status: %2 (ID=%3)")
	//			.arg(i)
	//			.arg(m_vboManager.vbos[i] && m_vboManager.vbos[i]->isCreated() ? "created" : "not created")
	//			.arg(m_vboManager.vbos[i] ? m_vboManager.vbos[i]->bufferId() : -1));
	//}

	if (m_vboManager.totalMemSizeBytes != totalSizeBytesBefore)
		ccLog::Print(QString("[VBO] VBO(s) (re)initialized for cloud '%1' (%2 Mb = %3% of points could be loaded)")
			.arg(getName())
			.arg(static_cast<double>(m_vboManager.totalMemSizeBytes)/(1<<20),0,'f',2)
			.arg(static_cast<double>(pointsInVBOs)/size() * 100.0,0,'f',2));

	m_vboManager.state = vboSet::INITIALIZED;

	return true;
}

int ccPointCloud::VBO::init(int count, bool withColors, bool withNormals, bool* reallocated/*=0*/)
{
	//required memory
	int totalSizeBytes = sizeof(PointCoordinateType) * count * 3;
	if (withColors)
	{
		rgbShift = totalSizeBytes;
		totalSizeBytes += sizeof(colorType) * count * 3;
	}
	if (withNormals)
	{
		normalShift = totalSizeBytes;
		totalSizeBytes += sizeof(PointCoordinateType) * count * 3;
	}

	if (!isCreated())
	{
		if (!create())
		{
			//no message as it will probably happen on a lof of (old) graphic cards
			return false;
		}
		
		setUsagePattern(QGLBuffer::DynamicDraw);	//"StaticDraw: The data will be set once and used many times for drawing operations."
													//"DynamicDraw: The data will be modified repeatedly and used many times for drawing operations.
	}

	if (!bind())
	{
		ccLog::Warning("[ccPointCloud::VBO::init] Failed to bind VBO to active context!");
		destroy();
		return -1;
	}

	if (totalSizeBytes != size())
	{
		allocate(totalSizeBytes);
		if (reallocated)
			*reallocated = true;

		if (size() != totalSizeBytes)
		{
			ccLog::Warning("[ccPointCloud::VBO::init] Not enough (GPU) memory!");
			release();
			destroy();
			return -1;
		}
	}
	else
	{
		//nothing to do
	}

	release();
	
	CatchGLErrors("ccPointCloud::vbo.init");

	return totalSizeBytes;
}

void ccPointCloud::releaseVBOs()
{
	if (m_vboManager.state == vboSet::NEW)
		return;

	if (m_currentDisplay)
	{
		m_currentDisplay->makeContextCurrent();
		//'destroy' all vbos
		for (size_t i=0; i<m_vboManager.vbos.size(); ++i)
		{
			if (m_vboManager.vbos[i])
			{
				m_vboManager.vbos[i]->destroy();
				delete m_vboManager.vbos[i];
				m_vboManager.vbos[i] = 0;
			}
		}
	}
	else
	{
		assert(m_vboManager.vbos.empty());
	}

	m_vboManager.vbos.clear();
	m_vboManager.hasColors = false;
	m_vboManager.hasNormals = false;
	m_vboManager.colorIsSF = false;
	m_vboManager.sourceSF = 0;
	m_vboManager.totalMemSizeBytes = 0;
	m_vboManager.state = vboSet::NEW;
}

void ccPointCloud::removeFromDisplay(const ccGenericGLDisplay* win)
{
	if (win == m_currentDisplay)
		releaseVBOs();

	//call parent's method
	ccGenericPointCloud::removeFromDisplay(win);
}
