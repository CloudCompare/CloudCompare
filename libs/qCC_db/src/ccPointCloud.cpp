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

#include "ccPointCloud.h"

//CCCoreLib
#include <GeometricalAnalysisTools.h>
#include <ManualSegmentationTools.h>
#include <ReferenceCloud.h>

//local
#include "cc2DLabel.h"
#include "ccChunk.h"
#include "ccColorRampShader.h"
#include "ccColorScalesManager.h"
#include "ccFastMarchingForNormsDirection.h"
#include "ccFrustum.h"
#include "ccGBLSensor.h"
#include "ccGenericGLDisplay.h"
#include "ccGenericMesh.h"
#include "ccImage.h"
#include "ccKdTree.h"
#include "ccMaterial.h"
#include "ccMesh.h"
#include "ccMinimumSpanningTreeForNormsDirection.h"
#include "ccNormalVectors.h"
#include "ccOctree.h"
#include "ccPointCloudLOD.h"
#include "ccPolyline.h"
#include "ccProgressDialog.h"
#include "ccScalarField.h"

//Qt
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QSettings>

//system
#include <cassert>
#include <queue>

static const char s_deviationSFName[] = "Deviation";

ccPointCloud::ccPointCloud(QString name/*=QString()*/, unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/) throw()
	: BaseClass(name, uniqueID)
	, m_rgbaColors(nullptr)
	, m_normals(nullptr)
	, m_sfColorScaleDisplayed(false)
	, m_currentDisplayedScalarField(nullptr)
	, m_currentDisplayedScalarFieldIndex(-1)
	, m_visibilityCheckEnabled(false)
	, m_lod(nullptr)
	, m_fwfData(nullptr)
{
	setName(name); //sadly we cannot use the ccGenericPointCloud constructor argument
	showSF(false);
}

ccPointCloud* ccPointCloud::From(CCCoreLib::GenericCloud* cloud, const ccGenericPointCloud* sourceCloud/*=nullptr*/)
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
			pc = nullptr;
		}
		else
		{
			//import points
			cloud->placeIteratorAtBeginning();
			for (unsigned i = 0; i < n; i++)
			{
				pc->addPoint(*cloud->getNextPoint());
			}
		}
	}

	if (pc && sourceCloud)
	{
		pc->importParametersFrom(sourceCloud);
	}

	return pc;
}

ccPointCloud* ccPointCloud::From(const CCCoreLib::GenericIndexedCloud* cloud, const ccGenericPointCloud* sourceCloud/*=nullptr*/)
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
			pc = nullptr;
		}
		else
		{
			//import points
			for (unsigned i = 0; i < n; i++)
			{
				CCVector3 P;
				cloud->getPoint(i, P);
				pc->addPoint(P);
			}
		}
	}

	if (pc && sourceCloud)
	{
		pc->importParametersFrom(sourceCloud);
	}

	return pc;
}

void UpdateGridIndexes(const std::vector<int>& newIndexMap, std::vector<ccPointCloud::Grid::Shared>& grids)
{
	for (ccPointCloud::Grid::Shared& scanGrid : grids)
	{
		unsigned cellCount = scanGrid->w * scanGrid->h;
		scanGrid->validCount = 0;
		scanGrid->minValidIndex = -1;
		scanGrid->maxValidIndex = -1;
		int* _gridIndex = scanGrid->indexes.data();
		for (size_t j = 0; j < cellCount; ++j, ++_gridIndex)
		{
			if (*_gridIndex >= 0)
			{
				assert(static_cast<size_t>(*_gridIndex) < newIndexMap.size());
				*_gridIndex = newIndexMap[*_gridIndex];
				if (*_gridIndex >= 0)
				{
					if (scanGrid->validCount)
					{
						scanGrid->minValidIndex = std::min(scanGrid->minValidIndex, static_cast<unsigned>(*_gridIndex));
						scanGrid->maxValidIndex = std::max(scanGrid->maxValidIndex, static_cast<unsigned>(*_gridIndex));
					}
					else
					{
						scanGrid->minValidIndex = scanGrid->maxValidIndex = static_cast<unsigned>(*_gridIndex);
					}
					++scanGrid->validCount;
				}
			}
		}
	}
}

ccPointCloud* ccPointCloud::partialClone(const CCCoreLib::ReferenceCloud* selection, int* warnings/*=nullptr*/) const
{
	if (warnings)
	{
		*warnings = 0;
	}

	if (!selection || selection->getAssociatedCloud() != static_cast<const GenericIndexedCloud*>(this))
	{
		ccLog::Error("[ccPointCloud::partialClone] Invalid parameters");
		return nullptr;
	}

	ccPointCloud* result = new ccPointCloud(getName() + QString(".extract"));

	//visibility
	result->setVisible(isVisible());
	result->setDisplay(getDisplay());
	result->setEnabled(isEnabled());

	//other parameters
	result->importParametersFrom(this);

	//from now on we will need some points to proceed ;)
	unsigned n = selection->size();
	if (n)
	{
		if (!result->reserveThePointsTable(n))
		{
			ccLog::Error("[ccPointCloud::partialClone] Not enough memory to duplicate cloud!");
			delete result;
			return nullptr;
		}

		//import points
		{
			for (unsigned i = 0; i < n; i++)
			{
				result->addPoint(*getPointPersistentPtr(selection->getPointGlobalIndex(i)));
			}
		}

		//RGB colors
		if (hasColors())
		{
			if (result->reserveTheRGBTable())
			{
				for (unsigned i = 0; i < n; i++)
				{
					result->addColor(getPointColor(selection->getPointGlobalIndex(i)));
				}
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
				for (unsigned i = 0; i < n; i++)
				{
					result->addNormIndex(getPointNormalIndex(selection->getPointGlobalIndex(i)));
				}
				result->showNormals(normalsShown());
			}
			else
			{
				ccLog::Warning("[ccPointCloud::partialClone] Not enough memory to copy normals!");
				if (warnings)
					*warnings |= WRN_OUT_OF_MEM_FOR_NORMALS;
			}
		}

		//waveform
		if (hasFWF())
		{
			if (result->reserveTheFWFTable())
			{
				try
				{
					for (unsigned i = 0; i < n; i++)
					{
						const ccWaveform& w = m_fwfWaveforms[selection->getPointGlobalIndex(i)];
						if (!result->fwfDescriptors().contains(w.descriptorID()))
						{
							//copy only the necessary descriptors
							result->fwfDescriptors().insert(w.descriptorID(), m_fwfDescriptors[w.descriptorID()]);
						}
						result->waveforms().push_back(w);
					}
					//we will use the same FWF data container
					result->fwfData() = fwfData();
				}
				catch (const std::bad_alloc&)
				{
					ccLog::Warning("[ccPointCloud::partialClone] Not enough memory to copy waveform signals!");
					result->clearFWFData();
					if (warnings)
						*warnings |= WRN_OUT_OF_MEM_FOR_FWF;
				}
			}
			else
			{
				ccLog::Warning("[ccPointCloud::partialClone] Not enough memory to copy waveform signals!");
				if (warnings)
					*warnings |= WRN_OUT_OF_MEM_FOR_FWF;
			}
		}

		//scalar fields
		unsigned sfCount = getNumberOfScalarFields();
		if (sfCount != 0)
		{
			for (unsigned k = 0; k < sfCount; ++k)
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
						if (currentScalarField->resizeSafe(n))
						{
							currentScalarField->setGlobalShift(sf->getGlobalShift());

							//we copy data to new SF
							for (unsigned i = 0; i < n; i++)
								currentScalarField->setValue(i, sf->getValue(selection->getPointGlobalIndex(i)));

							currentScalarField->computeMinAndMax();
							//copy display parameters
							currentScalarField->importParametersFrom(sf);
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
						result->setCurrentDisplayedScalarField(static_cast<int>(copiedSFCount) - 1);
				}
				//copy visibility
				result->showSF(sfShown());
			}
		}

		//scan grids
		if (gridCount() != 0)
		{
			try
			{
				//we need a map between old and new indexes
				std::vector<int> newIndexMap(size(), -1);
				{
					for (unsigned i = 0; i < n; i++)
					{
						newIndexMap[selection->getPointGlobalIndex(i)] = i;
					}
				}

				//duplicate the grid structure(s)
				std::vector<Grid::Shared> newGrids;
				{
					for (size_t i = 0; i < gridCount(); ++i)
					{
						const Grid::Shared& scanGrid = grid(i);
						if (scanGrid->validCount != 0) //no need to copy empty grids!
						{
							//duplicate the grid
							newGrids.push_back(Grid::Shared(new Grid(*scanGrid)));
						}
					}
				}

				//then update the indexes
				UpdateGridIndexes(newIndexMap, newGrids);

				//and keep the valid (non empty) ones
				for (Grid::Shared& scanGrid : newGrids)
				{
					if (scanGrid->validCount)
					{
						result->addGrid(scanGrid);
					}
				}
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				ccLog::Warning(QString("[ccPointCloud::partialClone] Not enough memory to copy the grid structure(s)"));
			}
		}

		//Meshes //TODO
		/*Lib::GenericIndexedMesh* theMesh = source->_getMesh();
		if (theMesh)
		{
		//REVOIR --> on pourrait le faire pour chaque sous-mesh non ?
		CCCoreLib::GenericIndexedMesh* newTri = CCCoreLib::ManualSegmentationTools::segmentMesh(theMesh,selection,true,nullptr,this);
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
		//on insere les objets "capteur" (pas de copie ici, la meme instance peut-etre partagee par plusieurs listes)
		for (i=1;i<=source->getNumberOfSensors();++i)
		setSensor(source->_getSensor(i),i);
		}
		*/
	}
	return result;
}

ccPointCloud::~ccPointCloud()
{
	clear();

	if (m_lod)
	{
		delete m_lod;
		m_lod = nullptr;
	}
}

void ccPointCloud::clear()
{
	unallocatePoints();
	unallocateColors();
	unallocateNorms();
	//enableTempColor(false); //DGM: why?
}

void ccPointCloud::unallocatePoints()
{
	clearLOD();	// we have to clear the LOD structure before clearing the colors / SFs, so we can't leave it to notifyGeometryUpdate()
	showSFColorsScale(false); //SFs will be destroyed
	BaseClass::reset();
	ccGenericPointCloud::clear();

	notifyGeometryUpdate(); //calls releaseVBOs()
}

void ccPointCloud::notifyGeometryUpdate()
{
	ccHObject::notifyGeometryUpdate();

	releaseVBOs();
	clearLOD();
}

void ccPointCloud::setDisplay(ccGenericGLDisplay* win)
{
	if (m_currentDisplay && win != m_currentDisplay)
	{
		//be sure to release the VBOs before switching to another (or no) display!
		releaseVBOs();
	}

	BaseClass::setDisplay(win);
}


ccGenericPointCloud* ccPointCloud::clone(ccGenericPointCloud* destCloud/*=nullptr*/, bool ignoreChildren/*=false*/)
{
	if (destCloud && !destCloud->isA(CC_TYPES::POINT_CLOUD))
	{
		ccLog::Error("[ccPointCloud::clone] Invalid destination cloud provided! Not a ccPointCloud...");
		return nullptr;
	}

	return cloneThis(static_cast<ccPointCloud*>(destCloud), ignoreChildren);
}

ccPointCloud* ccPointCloud::cloneThis(ccPointCloud* destCloud/*=nullptr*/, bool ignoreChildren/*=false*/)
{
	ccPointCloud* result = destCloud ? destCloud : new ccPointCloud();

	result->setVisible(isVisible());
	if (!destCloud)
		result->setDisplay(getDisplay());

	result->append(this, 0, ignoreChildren); //there was (virtually) no point before

	result->showColors(colorsShown());
	result->showSF(sfShown());
	result->showNormals(normalsShown());
	result->setEnabled(isEnabled());
	result->setCurrentDisplayedScalarField(getCurrentDisplayedScalarFieldIndex());

	//import other parameters
	result->importParametersFrom(this);

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

	return append(addedCloud, size());
}

const ccPointCloud& ccPointCloud::append(ccPointCloud* addedCloud, unsigned pointCountBefore, bool ignoreChildren/*=false*/)
{
	assert(addedCloud);

	unsigned addedPoints = addedCloud->size();

	if (!reserve(pointCountBefore + addedPoints))
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

		for (unsigned i = 0; i < addedPoints; i++)
		{
			addPoint(*addedCloud->getPoint(i));
		}
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
			for (unsigned i = 0; i < addedPoints; i++)
			{
				addColor(ccColor::white);
			}
		}
		else //otherwise
		{
			//if this cloud hadn't any color before
			if (!hasColors())
			{
				//we try to reserve a new array
				if (reserveTheRGBTable())
				{
					for (unsigned i = 0; i < pointCountBefore; i++)
					{
						addColor(ccColor::white);
					}
				}
				else
				{
					ccLog::Warning("[ccPointCloud::fusion] Not enough memory: failed to allocate colors!");
					showColors(false);
				}
			}

			//we import colors (if necessary)
			if (hasColors() && m_rgbaColors->currentSize() == pointCountBefore)
			{
				for (unsigned i = 0; i < addedPoints; i++)
				{
					addColor(addedCloud->m_rgbaColors->getValue(i));
				}
			}
		}
	}

	//normals (reserved)
	if (hasNormals() || addedCloud->hasNormals())
	{
		//merge display parameters
		showNormals(normalsShown() || addedCloud->normalsShown());

		//if the added cloud hasn't any normal
		if (!addedCloud->hasNormals())
		{
			//we associate imported points with '0' normals
			for (unsigned i = 0; i < addedPoints; i++)
			{
				addNormIndex(0);
			}
		}
		else //otherwise
		{
			//if this cloud hasn't any normal
			if (!hasNormals())
			{
				//we try to reserve a new array
				if (reserveTheNormsTable())
				{
					for (unsigned i = 0; i < pointCountBefore; i++)
					{
						addNormIndex(0);
					}
				}
				else
				{
					ccLog::Warning("[ccPointCloud::fusion] Not enough memory: failed to allocate normals!");
					showNormals(false);
				}
			}

			//we import normals (if necessary)
			if (hasNormals() && m_normals->currentSize() == pointCountBefore)
			{
				for (unsigned i = 0; i < addedPoints; i++)
				{
					addNormIndex(addedCloud->m_normals->getValue(i));
				}
			}
		}
	}

	//waveform
	if (hasFWF() || addedCloud->hasFWF())
	{
		//if the added cloud hasn't any waveform
		if (!addedCloud->hasFWF())
		{
			//we associate imported points with empty waveform
			for (unsigned i = 0; i < addedPoints; i++)
			{
				m_fwfWaveforms.emplace_back(0);
			}
		}
		else //otherwise
		{
			//if this cloud hasn't any FWF
			bool success = true;
			uint64_t fwfDataOffset = 0;
			if (!hasFWF())
			{
				//we try to reserve a new array
				if (reserveTheFWFTable())
				{
					for (unsigned i = 0; i < pointCountBefore; i++)
					{
						m_fwfWaveforms.emplace_back(0);
					}
					//we will simply use the other cloud FWF data container
					fwfData() = addedCloud->fwfData();
				}
				else
				{
					success = false;
					ccLog::Warning("[ccPointCloud::fusion] Not enough memory: failed to allocate waveforms!");
				}
			}
			else if (fwfData() != addedCloud->fwfData())
			{
				//we need to merge the two FWF data containers!
				assert(!fwfData()->empty() && !addedCloud->fwfData()->empty());
				FWFDataContainer* mergedContainer = new FWFDataContainer;
				try
				{
					fwfDataOffset = fwfData()->size();
					mergedContainer->reserve(fwfData()->size() + addedCloud->fwfData()->size());
					mergedContainer->insert(mergedContainer->end(), fwfData()->begin(), fwfData()->end());
					mergedContainer->insert(mergedContainer->end(), addedCloud->fwfData()->begin(), addedCloud->fwfData()->end());
					fwfData() = SharedFWFDataContainer(mergedContainer);
				}
				catch (const std::bad_alloc&)
				{
					success = false;
					delete mergedContainer;
					mergedContainer = nullptr;
					ccLog::Warning("[ccPointCloud::fusion] Not enough memory: failed to merge waveform containers!");
				}
			}

			if (success)
			{
				//assert(hasFWF()); //DGM: new waveforms are not pushed yet, so there might not be any in the cloud at the moment!
				size_t lostWaveformCount = 0;

				//map from old descriptor IDs to new ones
				QMap<uint8_t, uint8_t> descriptorIDMap;

				//first: copy the wave descriptors
				try
				{
					size_t newKeyCount = addedCloud->m_fwfDescriptors.size();
					assert(newKeyCount < 256); //IDs should range from 1 to 255
					
					if (!m_fwfDescriptors.empty())
					{
						assert(m_fwfDescriptors.size() < 256); //IDs should range from 1 to 255
						
						//we'll have to find free descriptor IDs (not used in the destination cloud) before merging
						std::queue<uint8_t> freeDescriptorIDs;
						for (uint8_t k = 0; k < 255; ++k)
						{
							if (!m_fwfDescriptors.contains(k + 1))
							{
								freeDescriptorIDs.push(k + 1);
								//if we have found enough free descriptor IDs
								if (freeDescriptorIDs.size() == newKeyCount)
								{
									//we can stop here
									break;
								}
							}
						}

						for (auto it = addedCloud->m_fwfDescriptors.begin(); it != addedCloud->m_fwfDescriptors.end(); ++it)
						{
							if (freeDescriptorIDs.empty())
							{
								ccLog::Warning("[ccPointCloud::fusion] Not enough free FWF descriptor IDs on destination cloud: some waveforms won't be imported!");
								break;
							}
							uint8_t newKey = freeDescriptorIDs.front();
							freeDescriptorIDs.pop();
							descriptorIDMap.insert(it.key(), newKey); //remember the ID transposition
							m_fwfDescriptors.insert(newKey, it.value()); //insert the descriptor at its new position (ID)
						}
					}
					else
					{
						for (auto it = addedCloud->m_fwfDescriptors.begin(); it != addedCloud->m_fwfDescriptors.end(); ++it)
						{
							descriptorIDMap.insert(it.key(), it.key());    //same descriptor ID, no conversion
							m_fwfDescriptors.insert(it.key(), it.value()); //insert the descriptor at the same position (ID)
						}
					}
				}
				catch (const std::bad_alloc&)
				{
					success = false;
					clearFWFData();
					ccLog::Warning("[ccPointCloud::fusion] Not enough memory: failed to copy waveform descriptors!");
				}

				//and now import waveforms
				if (success && m_fwfWaveforms.size() == pointCountBefore)
				{
					for (unsigned i = 0; i < addedPoints; i++)
					{
						ccWaveform w = addedCloud->waveforms()[i];
						if (descriptorIDMap.contains(w.descriptorID())) //the waveform can be imported :)
						{
							//update the byte offset
							w.setDataDescription(w.dataOffset() + fwfDataOffset, w.byteCount());
							//and the (potentially new) descriptor ID
							w.setDescriptorID(descriptorIDMap[w.descriptorID()]);

							m_fwfWaveforms.push_back(w);
						}
						else //the waveform is associated to a descriptor that couldn't be imported :(
						{
							m_fwfWaveforms.emplace_back(0);
							++lostWaveformCount;
						}
					}
				}

				if (lostWaveformCount)
				{
					ccLog::Warning(QString("[ccPointCloud::fusion] %1 waveform(s) were lost in the fusion process").arg(lostWaveformCount));
				}
			}
		}
	}

	//scalar fields (resized)
	unsigned sfCount = getNumberOfScalarFields();
	unsigned newSFCount = addedCloud->getNumberOfScalarFields();
	if (sfCount != 0 || newSFCount != 0)
	{
		std::vector<bool> sfUpdated(sfCount, false);

		//first we merge the new SF with the existing one
		for (unsigned k = 0; k < newSFCount; ++k)
		{
			const ccScalarField* sf = static_cast<ccScalarField*>(addedCloud->getScalarField(static_cast<int>(k)));
			if (sf)
			{
				//does this field already exist (same name)?
				int sfIdx = getScalarFieldIndexByName(sf->getName());
				if (sfIdx >= 0) //yes
				{
					ccScalarField* sameSF = static_cast<ccScalarField*>(getScalarField(sfIdx));
					assert(sameSF && sameSF->capacity() >= pointCountBefore + addedPoints);
					//we fill it with new values (it should have been already 'reserved' (if necessary)
					if (sameSF->currentSize() == pointCountBefore)
					{
						double shift = sf->getGlobalShift() - sameSF->getGlobalShift();
						for (unsigned i = 0; i < addedPoints; i++)
						{
							sameSF->addElement(static_cast<ScalarType>(shift + sf->getValue(i))); //FIXME: we could have accuracy issues here
						}
					}
					sameSF->computeMinAndMax();

					//flag this SF as 'updated'
					assert(sfIdx < static_cast<int>(sfCount));
					sfUpdated[sfIdx] = true;
				}
				else //otherwise we create a new SF
				{
					ccScalarField* newSF = new ccScalarField(sf->getName());
					newSF->setGlobalShift(sf->getGlobalShift());
					//we fill the beginning with NaN (as there is no equivalent in the current cloud)
					if (newSF->resizeSafe(pointCountBefore + addedPoints, true, CCCoreLib::NAN_VALUE))
					{
						//we copy the new values
						for (unsigned i = 0; i < addedPoints; i++)
						{
							newSF->setValue(pointCountBefore + i, sf->getValue(i));
						}
						newSF->computeMinAndMax();
						//copy display parameters
						newSF->importParametersFrom(sf);

						//add scalar field to this cloud
						sfIdx = addScalarField(newSF);
						assert(sfIdx >= 0);
					}
					else
					{
						newSF->release();
						newSF = nullptr;
						ccLog::Warning("[ccPointCloud::fusion] Not enough memory: failed to allocate a copy of scalar field '%s'",sf->getName());
					}
				}
			}
		}

		//let's check if there are non-updated fields
		for (unsigned j = 0; j < sfCount; ++j)
		{
			if (!sfUpdated[j])
			{
				CCCoreLib::ScalarField* sf = getScalarField(j);
				assert(sf);

				if (sf->currentSize() == pointCountBefore)
				{
					//we fill the end with NaN (as there is no equivalent in the added cloud)
					ScalarType NaN = sf->NaN();
					for (unsigned i = 0; i < addedPoints; i++)
					{
						sf->addElement(NaN);
					}
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

	//if the merged cloud has grid structures AND this one is blank or also has grid structures
	if (addedCloud->gridCount() != 0 && (gridCount() != 0 || pointCountBefore == 0))
	{
		//copy the grid structures
		for (size_t i = 0; i < addedCloud->gridCount(); ++i)
		{
			const Grid::Shared& otherGrid = addedCloud->grid(i);
			if (otherGrid && otherGrid->validCount != 0) //no need to copy empty grids!
			{
				try
				{
					//copy the grid data
					Grid::Shared grid(new Grid(*otherGrid));
					{
						//then update the indexes
						unsigned cellCount = grid->w*grid->h;
						int* _gridIndex = grid->indexes.data();
						for (size_t j = 0; j < cellCount; ++j, ++_gridIndex)
						{
							if (*_gridIndex >= 0)
							{
								//shift the index
								*_gridIndex += static_cast<int>(pointCountBefore);
							}
						}

						//don't forget to shift the boundaries as well
						grid->minValidIndex += pointCountBefore;
						grid->maxValidIndex += pointCountBefore;
					}

					addGrid(grid);
				}
				catch (const std::bad_alloc&)
				{
					//not enough memory
					m_grids.resize(0);
					ccLog::Warning(QString("[ccPointCloud::fusion] Not enough memory: failed to copy the grid structure(s) from '%1'").arg(addedCloud->getName()));
					break;
				}
			}
			else
			{
				assert(otherGrid);
			}
		}
	}
	else if (gridCount() != 0) //otherwise we'll have to drop the former grid structures!
	{
		ccLog::Warning(QString("[ccPointCloud::fusion] Grid structure(s) will be dropped as the merged cloud is unstructured"));
		m_grids.resize(0);
	}

	//has the cloud been re-centered/rescaled?
	{
		if (addedCloud->isShifted())
		{
			if (!isShifted())
			{
				//we can keep the global shift information of the merged cloud
				copyGlobalShiftAndScale(*addedCloud);
			}
			else if (	getGlobalScale() != addedCloud->getGlobalScale()
					||	(getGlobalShift() - addedCloud->getGlobalShift()).norm2d() > 1.0e-6)
			{
				//the clouds have different shift & scale information!
				ccLog::Warning(QString("[ccPointCloud::fusion] Global shift/scale information conflict: shift/scale of cloud '%1' will be ignored!").arg(addedCloud->getName()));
			}
		}
	}

	//children (not yet reserved)
	if (!ignoreChildren)
	{
		unsigned childrenCount = addedCloud->getChildrenNumber();
		for (unsigned c = 0; c < childrenCount; ++c)
		{
			ccHObject* child = addedCloud->getChild(c);
			if (!child)
			{
				assert(false);
				continue;
			}
			if (child->isA(CC_TYPES::MESH)) //mesh --> FIXME: what for the other types of MESH?
			{
				ccMesh* mesh = static_cast<ccMesh*>(child);

				//detach from father?
				//addedCloud->detachChild(mesh);
				//ccGenericMesh* addedTri = mesh;

				//or clone?
				ccMesh* cloneMesh = mesh->cloneMesh(mesh->getAssociatedCloud() == addedCloud ? this : nullptr);
				if (cloneMesh)
				{
					//change mesh vertices
					if (cloneMesh->getAssociatedCloud() == this)
					{
						cloneMesh->shiftTriangleIndexes(pointCountBefore);
					}
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
				addedCloud->transferChild(child, *this);
			}
			else if (child->isA(CC_TYPES::LABEL_2D))
			{
				//clone label and update points if necessary
				cc2DLabel* label = static_cast<cc2DLabel*>(child);
				cc2DLabel* newLabel = new cc2DLabel(label->getName());
				for (unsigned j = 0; j < label->size(); ++j)
				{
					cc2DLabel::PickedPoint P = label->getPickedPoint(j);
					if (P._cloud == addedCloud)
					{
						P._cloud = this;
						P.index += pointCountBefore;
					}
					newLabel->addPickedPoint(P);
				}
				newLabel->displayPointLegend(label->isPointLegendDisplayed());
				newLabel->setDisplayedIn2D(label->isDisplayedIn2D());
				newLabel->setCollapsed(label->isCollapsed());
				newLabel->setPosition(label->getPosition()[0], label->getPosition()[1]);
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
	//As well as the LOD structure
	clearLOD();

	return *this;
}

void ccPointCloud::unallocateNorms()
{
	if (m_normals)
	{
		m_normals->release();
		m_normals = nullptr;

		//We should update the VBOs to gain some free space in VRAM
		releaseVBOs();
	}

	showNormals(false);
}

void ccPointCloud::unallocateColors()
{
	if (m_rgbaColors)
	{
		m_rgbaColors->release();
		m_rgbaColors = nullptr;

		//We should update the VBOs to gain some free space in VRAM
		releaseVBOs();
	}

	//remove the grid colors as well!
	for (size_t i = 0; i < m_grids.size(); ++i)
	{
		if (m_grids[i])
		{
			m_grids[i]->colors.resize(0);
		}
	}

	showColors(false);
	enableTempColor(false);
}

bool ccPointCloud::reserveThePointsTable(unsigned newNumberOfPoints)
{
	try
	{
		m_points.reserve(newNumberOfPoints);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}
	return true;
}

bool ccPointCloud::reserveTheRGBTable()
{
	if (m_points.capacity() == 0)
	{
		ccLog::Warning("[ccPointCloud] Calling reserveTheRGBTable with an zero capacity cloud");
	}

	if (!m_rgbaColors)
	{
		m_rgbaColors = new RGBAColorsTableType();
		m_rgbaColors->link();
	}

	if (!m_rgbaColors->reserveSafe(m_points.capacity()))
	{
		m_rgbaColors->release();
		m_rgbaColors = nullptr;
		ccLog::Error("[ccPointCloud::reserveTheRGBTable] Not enough memory!");
	}

	//We must update the VBOs
	colorsHaveChanged();

	//double check
	return m_rgbaColors && m_rgbaColors->capacity() >= m_points.capacity();
}

bool ccPointCloud::resizeTheRGBTable(bool fillWithWhite/*=false*/)
{
	if (m_points.empty())
	{
		ccLog::Warning("[ccPointCloud] Calling resizeTheRGBTable with an empty cloud");
	}

	if (!m_rgbaColors)
	{
		m_rgbaColors = new RGBAColorsTableType();
		m_rgbaColors->link();
	}

	static const ccColor::Rgba s_white(ccColor::MAX, ccColor::MAX, ccColor::MAX, ccColor::MAX);
	if (!m_rgbaColors->resizeSafe(m_points.size(), fillWithWhite, &s_white))
	{
		m_rgbaColors->release();
		m_rgbaColors = nullptr;
		ccLog::Error("[ccPointCloud::resizeTheRGBTable] Not enough memory!");
	}

	//We must update the VBOs
	colorsHaveChanged();

	//double check
	return m_rgbaColors && m_rgbaColors->size() == m_points.size();
}

bool ccPointCloud::reserveTheNormsTable()
{
	if (m_points.capacity() == 0)
	{
		ccLog::Warning("[ccPointCloud] Calling reserveTheNormsTable with an zero capacity cloud");
	}

	if (!m_normals)
	{
		m_normals = new NormsIndexesTableType();
		m_normals->link();
	}

	if (!m_normals->reserveSafe(m_points.capacity()))
	{
		m_normals->release();
		m_normals = nullptr;

		ccLog::Error("[ccPointCloud::reserveTheNormsTable] Not enough memory!");
	}

	//We must update the VBOs
	normalsHaveChanged();

	//double check
	return m_normals && m_normals->capacity() >= m_points.capacity();
}

bool ccPointCloud::resizeTheNormsTable()
{
	if (m_points.empty())
	{
		ccLog::Warning("[ccPointCloud] Calling resizeTheNormsTable with an empty cloud");
	}

	if (!m_normals)
	{
		m_normals = new NormsIndexesTableType();
		m_normals->link();
	}

	static const CompressedNormType s_normZero = 0;
	if (!m_normals->resizeSafe(m_points.size(), true, &s_normZero))
	{
		m_normals->release();
		m_normals = nullptr;

		ccLog::Error("[ccPointCloud::resizeTheNormsTable] Not enough memory!");
	}

	//We must update the VBOs
	normalsHaveChanged();

	//double check
	return m_normals && m_normals->size() == m_points.size();
}

bool ccPointCloud::compressFWFData()
{
	if (!m_fwfData || m_fwfData->empty())
	{
		return false;
	}

	try
	{
		size_t initialCount = m_fwfData->size();
		std::vector<size_t> usedIndexes;
		usedIndexes.resize(initialCount, 0);

		for (const ccWaveform& w : m_fwfWaveforms)
		{
			if (w.byteCount() == 0)
			{
				assert(false);
				continue;
			}

			size_t start = w.dataOffset();
			size_t end = w.dataOffset() + w.byteCount();
			for (size_t i = start; i < end; ++i)
			{
				usedIndexes[i] = 1;
			}
		}

		size_t newIndex = 0;
		for (size_t& index : usedIndexes)
		{
			if (index != 0)
			{
				index = ++newIndex; //we need to start at 1 (as 0 means 'not used')
			}
		}

		if (newIndex >= initialCount)
		{
			//nothing to do
			ccLog::Print(QString("[ccPointCloud::compressFWFData] Cloud '%1': no need to compress FWF data").arg(getName()));
			return true;
		}

		//now create the new container
		FWFDataContainer* newContainer = new FWFDataContainer;
		newContainer->reserve(newIndex);

		for (size_t i = 0; i < initialCount; ++i)
		{
			if (usedIndexes[i])
			{
				newContainer->push_back(m_fwfData->at(i));
			}
		}

		//and don't forget to update the waveform descriptors!
		for (ccWaveform& w : m_fwfWaveforms)
		{
			uint64_t offset = w.dataOffset();
			assert(usedIndexes[offset] != 0);
			w.setDataOffset(usedIndexes[offset] - 1);
		}
		m_fwfData = SharedFWFDataContainer(newContainer);

		ccLog::Print(QString("[ccPointCloud::compressFWFData] Cloud '%1': FWF data compressed --> %2 / %3 (%4%)").arg(getName()).arg(newIndex).arg(initialCount).arg(100.0 - (newIndex * 100.0) / initialCount, 0, 'f', 1));
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccPointCloud::compressFWFData] Not enough memory!");
		return false;
	}

	return true;
}

bool ccPointCloud::reserveTheFWFTable()
{
	if (m_points.capacity() == 0)
	{
		ccLog::Warning("[ccPointCloud] Calling reserveTheFWFTable with a zero capacity cloud");
	}

	try
	{
		m_fwfWaveforms.reserve(m_points.capacity());
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("[ccPointCloud::reserveTheFWFTable] Not enough memory!");
		m_fwfWaveforms.resize(0);
	}

	//double check
	return m_fwfWaveforms.capacity() >= m_points.capacity();
}

bool ccPointCloud::hasFWF() const
{
	return		m_fwfData
			&&	!m_fwfData->empty()
			&&	!m_fwfWaveforms.empty();
}

ccWaveformProxy ccPointCloud::waveformProxy(unsigned index) const
{
	static const ccWaveform invalidW;
	static const WaveformDescriptor invalidD;

	if (index < m_fwfWaveforms.size())
	{
		const ccWaveform& w = m_fwfWaveforms[index];
		//check data consistency
		if (m_fwfData && w.dataOffset() + w.byteCount() <= m_fwfData->size())
		{
			if (m_fwfDescriptors.contains(w.descriptorID()))
			{
				WaveformDescriptor& d = const_cast<ccPointCloud*>(this)->m_fwfDescriptors[w.descriptorID()]; //DGM: we really want the reference to the element, not a copy as QMap returns in the const case :(
				return ccWaveformProxy(w, d, m_fwfData->data());
			}
			else
			{
				return ccWaveformProxy(w, invalidD, nullptr);
			}
		}
	}

	//if we are here, then something is wrong
	assert(false);
	return ccWaveformProxy(invalidW, invalidD, nullptr);
}

bool ccPointCloud::resizeTheFWFTable()
{
	if (m_points.capacity() == 0)
	{
		ccLog::Warning("[ccPointCloud] Calling resizeTheFWFTable with an empty cloud");
	}

	try
	{
		m_fwfWaveforms.resize(m_points.capacity());
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("[ccPointCloud::resizeTheFWFTable] Not enough memory!");
		m_fwfWaveforms.resize(0);
	}

	//double check
	return m_fwfWaveforms.capacity() >= m_points.capacity();
}

bool ccPointCloud::reserve(unsigned newNumberOfPoints)
{
	if (newNumberOfPoints == size())
	{
		//nothing to do
		return true;
	}
	else if (newNumberOfPoints < size())
	{
		//reserve works only to enlarge the cloud
		return false;
	}

	//if we are changing the cloud contents, let's stop the LOD construction process
	clearLOD();

	//call parent method first (for points + scalar fields)
	if (	!BaseClass::reserve(newNumberOfPoints)
		||	(hasColors() && !reserveTheRGBTable())
		||	(hasNormals() && !reserveTheNormsTable())
		||	(hasFWF() && !reserveTheFWFTable()))
	{
		ccLog::Error("[ccPointCloud::reserve] Not enough memory!");
		return false;
	}

	//ccLog::Warning(QString("[ccPointCloud::reserve] Cloud is %1 and its capacity is '%2'").arg(m_points.capacity() != 0 ? "allocated" : "not allocated").arg(m_points.capacity()));

	//double check
	return	                   m_points.capacity()       >= newNumberOfPoints
		&&	( !hasColors()  || m_rgbaColors->capacity()  >= newNumberOfPoints )
		&&	( !hasNormals() || m_normals->capacity()     >= newNumberOfPoints )
		&&	( !hasFWF()     || m_fwfWaveforms.capacity() >= newNumberOfPoints );
}

bool ccPointCloud::resize(unsigned newNumberOfPoints)
{
	if (newNumberOfPoints < size() && isLocked())
	{
		//can't reduce the size if the cloud if it is locked!
		return false;
	}

	//if we are changing the cloud contents, let's stop the LOD construction process
	clearLOD();

	//call parent method first (for points + scalar fields)
	if (!BaseClass::resize(newNumberOfPoints))
	{
		ccLog::Error("[ccPointCloud::resize] Not enough memory!");
		return false;
	}

	notifyGeometryUpdate(); //calls releaseVBOs()

	if ((hasColors()  && !resizeTheRGBTable(false))
	||	(hasNormals() && !resizeTheNormsTable())
	||	(hasFWF()     && !resizeTheFWFTable()))
	{
		ccLog::Error("[ccPointCloud::resize] Not enough memory!");
		return false;
	}

	//double check
	return	                   m_points.size()             == newNumberOfPoints
		&&	( !hasColors()  || m_rgbaColors->currentSize() == newNumberOfPoints )
		&&	( !hasNormals() || m_normals->currentSize()    == newNumberOfPoints )
		&&	( !hasFWF()     || m_fwfWaveforms.size()       == newNumberOfPoints );
}

void ccPointCloud::showSFColorsScale(bool state)
{
	m_sfColorScaleDisplayed = state;
}

bool ccPointCloud::sfColorScaleShown() const
{
	return m_sfColorScaleDisplayed;
}

const ccColor::Rgb* ccPointCloud::getPointScalarValueColor(unsigned pointIndex) const
{
	assert(m_currentDisplayedScalarField && m_currentDisplayedScalarField->getColorScale());

	return m_currentDisplayedScalarField->getValueColor(pointIndex);
}

const ccColor::Rgb* ccPointCloud::geScalarValueColor(ScalarType d) const
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

const ccColor::Rgba& ccPointCloud::getPointColor(unsigned pointIndex) const
{
	assert(hasColors());
	assert(m_rgbaColors && pointIndex < m_rgbaColors->currentSize());

	return m_rgbaColors->at(pointIndex);
}

const CompressedNormType& ccPointCloud::getPointNormalIndex(unsigned pointIndex) const
{
	assert(m_normals && pointIndex < m_normals->currentSize());

	return m_normals->getValue(pointIndex);
}

const CCVector3& ccPointCloud::getPointNormal(unsigned pointIndex) const
{
	assert(m_normals && pointIndex < m_normals->currentSize());

	return ccNormalVectors::GetNormal(m_normals->getValue(pointIndex));
}

const CCVector3* ccPointCloud::getNormal(unsigned pointIndex) const
{
	assert(m_normals && pointIndex < m_normals->currentSize());

	return &ccNormalVectors::GetNormal(m_normals->getValue(pointIndex));
}

void ccPointCloud::setPointColor(unsigned pointIndex, const ccColor::Rgba& col)
{
	assert(m_rgbaColors && pointIndex < m_rgbaColors->currentSize());

	m_rgbaColors->setValue(pointIndex, col);

	//We must update the VBOs
	colorsHaveChanged();
}

void ccPointCloud::setPointNormalIndex(unsigned pointIndex, CompressedNormType norm)
{
	assert(m_normals && pointIndex < m_normals->currentSize());

	m_normals->setValue(pointIndex, norm);

	//We must update the VBOs
	normalsHaveChanged();
}

void ccPointCloud::setPointNormal(unsigned pointIndex, const CCVector3& N)
{
	setPointNormalIndex(pointIndex, ccNormalVectors::GetNormIndex(N));
}

bool ccPointCloud::hasColors() const
{
	return m_rgbaColors && m_rgbaColors->isAllocated();
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

void ccPointCloud::invalidateBoundingBox()
{
	BaseClass::invalidateBoundingBox();

	notifyGeometryUpdate();	//calls releaseVBOs()
}

void ccPointCloud::addColor(const ccColor::Rgba& C)
{
	assert(m_rgbaColors && m_rgbaColors->isAllocated());
	m_rgbaColors->emplace_back(C);

	//We must update the VBOs
	colorsHaveChanged();
}

void ccPointCloud::addNorm(const CCVector3& N)
{
	addNormIndex(ccNormalVectors::GetNormIndex(N));
}

void ccPointCloud::addNormIndex(CompressedNormType index)
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
	CCVector3::vadd(P.u, N, P.u);
	P.normalize();
	//we recode the resulting vector
	CompressedNormType nIndex = ccNormalVectors::GetNormIndex(P.u);
	m_normals->setValue(index,nIndex);

	//We must update the VBOs
	normalsHaveChanged();
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
	const std::vector<ccColor::Rgb>& normalHSV = ccNormalVectors::GetUniqueInstance()->getNormalHSVColorArray();

	if (!resizeTheRGBTable(false))
	{
		ccLog::Warning("[ccPointCloud::convertNormalToRGB] Not enough memory!");
		return false;
	}
	assert(m_normals && m_rgbaColors);

	unsigned count = size();
	for (unsigned i = 0; i < count; ++i)
	{
		const ccColor::Rgb& rgb = normalHSV[m_normals->getValue(i)];
		m_rgbaColors->setValue(i, ccColor::Rgba(rgb, ccColor::MAX));
	}

	//We must update the VBOs
	colorsHaveChanged();

	return true;
}

bool ccPointCloud::convertRGBToGreyScale()
{
	if (!hasColors())
	{
		return false;
	}
	assert(m_rgbaColors);

	unsigned count = size();
	for (unsigned i = 0; i < count; ++i)
	{
		ccColor::Rgba& rgba = m_rgbaColors->at(i);
		//conversion from RGB to grey scale (see https://en.wikipedia.org/wiki/Luma_%28video%29)
		double luminance = 0.2126 * rgba.r + 0.7152 * rgba.g + 0.0722 * rgba.b;
		rgba.r = rgba.g = rgba.b = static_cast<unsigned char>(std::max(std::min(luminance, 255.0), 0.0));
	}

	//We must update the VBOs
	colorsHaveChanged();

	return true;
}

bool ccPointCloud::convertNormalToDipDirSFs(ccScalarField* dipSF, ccScalarField* dipDirSF)
{
	if (!dipSF && !dipDirSF)
	{
		assert(false);
		return false;
	}

	if (	(dipSF    && !dipSF->resizeSafe(size()))
		||	(dipDirSF && !dipDirSF->resizeSafe(size())) )
	{
		ccLog::Warning("[ccPointCloud::convertNormalToDipDirSFs] Not enough memory!");
		return false;
	}

	unsigned count = size();
	for (unsigned i = 0; i < count; ++i)
	{
		CCVector3 N(this->getPointNormal(i));
		PointCoordinateType dip;
		PointCoordinateType dipDir;
		ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);
		if (dipSF)
			dipSF->setValue(i, static_cast<ScalarType>(dip));
		if (dipDirSF)
			dipDirSF->setValue(i, static_cast<ScalarType>(dipDir));
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
	normalsHaveChanged();
}

bool ccPointCloud::colorize(float r, float g, float b, float a/*=1.0f*/)
{
	assert(r >= 0.0f && r <= 1.0f);
	assert(g >= 0.0f && g <= 1.0f);
	assert(b >= 0.0f && b <= 1.0f);
	assert(a >= 0.0f && a <= 1.0f);

	if (hasColors())
	{
		assert(m_rgbaColors);
		for (unsigned i = 0; i < m_rgbaColors->currentSize(); i++)
		{
			ccColor::Rgba& p = m_rgbaColors->at(i);
			{
				p.r = static_cast<ColorCompType>(p.r * r);
				p.g = static_cast<ColorCompType>(p.g * g);
				p.b = static_cast<ColorCompType>(p.b * b);
				p.a = static_cast<ColorCompType>(p.a * a);
			}
		}
	}
	else
	{
		if (!resizeTheRGBTable(false))
			return false;

		ccColor::Rgba C(	static_cast<ColorCompType>(ccColor::MAX * r) ,
							static_cast<ColorCompType>(ccColor::MAX * g) ,
							static_cast<ColorCompType>(ccColor::MAX * b) ,
							static_cast<ColorCompType>(ccColor::MAX * a) );
		m_rgbaColors->fill(C);
	}

	//We must update the VBOs
	colorsHaveChanged();

	return true;
}

//Contribution from Michael J Smith
bool ccPointCloud::setRGBColorByBanding(unsigned char dim, double freq)
{
	if (freq == 0 || dim > 2) //X=0, Y=1, Z=2
	{
		ccLog::Warning("[ccPointCloud::setRGBColorByBanding] Invalid parameter!");
		return false;
	}

	//allocate colors if necessary
	if (!hasColors())
		if (!resizeTheRGBTable(false))
			return false;

	enableTempColor(false);
	assert(m_rgbaColors);

	float bands = (2.0 * M_PI) / freq;

	unsigned count = size();
	for (unsigned i = 0; i < count; i++)
	{
		const CCVector3* P = getPoint(i);

		float z = bands * P->u[dim];
		ccColor::Rgba C(	static_cast<ColorCompType>( ((sin(z + 0.0f   ) + 1.0f) / 2.0f) * ccColor::MAX ),
							static_cast<ColorCompType>( ((sin(z + 2.0944f) + 1.0f) / 2.0f) * ccColor::MAX ),
							static_cast<ColorCompType>( ((sin(z + 4.1888f) + 1.0f) / 2.0f) * ccColor::MAX ),
							ccColor::MAX );

		m_rgbaColors->setValue(i, C);
	}

	//We must update the VBOs
	colorsHaveChanged();

	return true;
}

bool ccPointCloud::setRGBColorByHeight(unsigned char heightDim, ccColorScale::Shared colorScale)
{
	if (!colorScale || heightDim > 2) //X=0, Y=1, Z=2
	{
		ccLog::Error("[ccPointCloud::setRGBColorByHeight] Invalid parameter!");
		return false;
	}

	//allocate colors if necessary
	if (!hasColors())
		if (!resizeTheRGBTable(false))
			return false;

	enableTempColor(false);
	assert(m_rgbaColors);

	double minHeight = getOwnBB().minCorner().u[heightDim];
	double height = getOwnBB().getDiagVec().u[heightDim];
	if (CCCoreLib::LessThanEpsilon(std::abs(height))) //flat cloud!
	{
		const ccColor::Rgb& col = colorScale->getColorByIndex(0);
		return setColor(col);
	}

	unsigned count = size();
	for (unsigned i = 0; i < count; i++)
	{
		const CCVector3* Q = getPoint(i);
		double relativePos = (Q->u[heightDim] - minHeight) / height;
		const ccColor::Rgb* col = colorScale->getColorByRelativePos(relativePos);
		if (!col) //DGM: yes it happens if we encounter a point with NaN coordinates!!!
		{
			col = &ccColor::blackRGB;
		}
		m_rgbaColors->setValue(i, ccColor::Rgba(*col, ccColor::MAX));
	}

	//We must update the VBOs
	colorsHaveChanged();

	return true;
}

bool ccPointCloud::setColor(const ccColor::Rgba& col)
{
	enableTempColor(false);

	//allocate colors if necessary
	if (!hasColors())
		if (!reserveTheRGBTable())
			return false;

	assert(m_rgbaColors);
	m_rgbaColors->fill(col);

	//update the grid colors as well!
	for (size_t i = 0; i < m_grids.size(); ++i)
	{
		if (m_grids[i] && !m_grids[i]->colors.empty())
		{
			std::fill(m_grids[i]->colors.begin(), m_grids[i]->colors.end(), col);
		}
	}


	//We must update the VBOs
	colorsHaveChanged();

	return true;
}

CCVector3 ccPointCloud::computeGravityCenter()
{
	return CCCoreLib::GeometricalAnalysisTools::ComputeGravityCenter(this);
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
	for (unsigned i = 0; i < count; i++)
	{
		trans.apply(*point(i));
	}

	//we must also take care of the normals!
	if (hasNormals())
	{
		bool recoded = false;

		//if there is more points than the size of the compressed normals array,
		//we recompress the array instead of recompressing each normal
		if (count > ccNormalVectors::GetNumberOfVectors())
		{
			NormsIndexesTableType newNorms;
			if (newNorms.reserveSafe(ccNormalVectors::GetNumberOfVectors()))
			{
				for (unsigned i = 0; i < ccNormalVectors::GetNumberOfVectors(); i++)
				{
					CCVector3 new_n(ccNormalVectors::GetNormal(i));
					trans.applyRotation(new_n);
					CompressedNormType newNormIndex = ccNormalVectors::GetNormIndex(new_n.u);
					newNorms.emplace_back(newNormIndex);
				}

				for (unsigned j = 0; j < count; j++)
				{
					m_normals->setValue(j, newNorms[m_normals->at(j)]);
				}
				recoded = true;
			}
		}

		//if there is less points than the compressed normals array size
		//(or if there is not enough memory to instantiate the temporary
		//array), we recompress each normal ...
		if (!recoded)
		{
			//on recode direct chaque normale
			for (CompressedNormType& _theNormIndex : *m_normals)
			{
				CCVector3 new_n(ccNormalVectors::GetNormal(_theNormIndex));
				trans.applyRotation(new_n);
				_theNormIndex = ccNormalVectors::GetNormIndex(new_n.u);
			}
		}
	}

	//and the scan grids!
	if (!m_grids.empty())
	{
		ccGLMatrixd transd(trans.data());

		for (Grid::Shared &grid : m_grids)
		{
			if (!grid)
			{
				continue;
			}
			grid->sensorPosition = transd * grid->sensorPosition;
		}
	}

	//and the waveform!
	for (ccWaveform& w : m_fwfWaveforms)
	{
		if (w.descriptorID() != 0)
		{
			w.applyRigidTransformation(trans);
		}
	}

	//the octree is invalidated by rotation...
	deleteOctree();

	// ... as the bounding box
	refreshBB(); //calls notifyGeometryUpdate + releaseVBOs
}

void ccPointCloud::translate(const CCVector3& T)
{
	if (CCCoreLib::LessThanEpsilon(std::abs(T.x) + std::abs(T.y) + std::abs(T.z)))
		return;

	unsigned count = size();
	{
		for (unsigned i = 0; i < count; i++)
			*point(i) += T;
	}

	notifyGeometryUpdate(); //calls releaseVBOs()
	invalidateBoundingBox();

	//same thing for the octree
	ccOctree::Shared octree = getOctree();
	if (octree)
	{
		octree->translateBoundingBox(T);
	}

	//and same thing for the Kd-tree(s)!
	ccHObject::Container kdtrees;
	filterChildren(kdtrees, false, CC_TYPES::POINT_KDTREE);
	{
		for (size_t i = 0; i < kdtrees.size(); ++i)
		{
			static_cast<ccKdTree*>(kdtrees[i])->translateBoundingBox(T);
		}
	}

	//update the transformation history
	{
		ccGLMatrix trans;
		trans.setTranslation(T);
		m_glTransHistory = trans * m_glTransHistory;
	}
}

void ccPointCloud::scale(PointCoordinateType fx, PointCoordinateType fy, PointCoordinateType fz, CCVector3 center)
{
	//transform the points
	{
		unsigned count = size();
		for (unsigned i = 0; i < count; i++)
		{
			CCVector3* P = point(i);
			P->x = (P->x - center.x) * fx + center.x;
			P->y = (P->y - center.y) * fy + center.y;
			P->z = (P->z - center.z) * fz + center.z;
		}
	}

	invalidateBoundingBox();

	//same thing for the octree
	ccOctree::Shared octree = getOctree();
	if (octree)
	{
		if (fx == fy && fx == fz && fx > 0)
		{
			CCVector3 centerInv = -center;
			octree->translateBoundingBox(centerInv);
			octree->multiplyBoundingBox(fx);
			octree->translateBoundingBox(center);
		}
		else
		{
			//we can't keep the octree
			deleteOctree();
		}
	}

	//and same thing for the Kd-tree(s)!
	{
		ccHObject::Container kdtrees;
		filterChildren(kdtrees, false, CC_TYPES::POINT_KDTREE);
		if (fx == fy && fx == fz && fx > 0)
		{
			for (size_t i = 0; i < kdtrees.size(); ++i)
			{
				ccKdTree* kdTree = static_cast<ccKdTree*>(kdtrees[i]);
				CCVector3 centerInv = -center;
				kdTree->translateBoundingBox(centerInv);
				kdTree->multiplyBoundingBox(fx);
				kdTree->translateBoundingBox(center);
			}
		}
		else
		{
			//we can't keep the kd-trees
			for (size_t i = 0; i < kdtrees.size(); ++i)
			{
				removeChild(kdtrees[kdtrees.size() - 1 - i]); //faster to remove the last objects
			}
		}
		kdtrees.resize(0);
	}

	//new we have to compute a proper transformation matrix
	ccGLMatrix scaleTrans;
	{
		ccGLMatrix transToCenter;
		transToCenter.setTranslation(-center);

		ccGLMatrix scaleAndReposition;
		scaleAndReposition.data()[0] = fx;
		scaleAndReposition.data()[5] = fy;
		scaleAndReposition.data()[10] = fz;
		//go back to the original position
		scaleAndReposition.setTranslation(center);

		scaleTrans = scaleAndReposition * transToCenter;
	}

	//update the grids as well
	{
		for (Grid::Shared &grid : m_grids)
		{
			if (grid)
			{
				//update the scan position
				grid->sensorPosition = ccGLMatrixd(scaleTrans.data()) * grid->sensorPosition;
			}
		}
	}

	//updates the sensors
	{
		for (ccHObject* child : m_children)
		{
			if (child && child->isKindOf(CC_TYPES::SENSOR))
			{
				ccSensor* sensor = static_cast<ccSensor*>(child);

				sensor->applyGLTransformation(scaleTrans);

				//update the graphic scale, etc.
				PointCoordinateType meanScale = (fx + fy + fz) / 3;
				//sensor->setGraphicScale(sensor->getGraphicScale() * meanScale);
				if (sensor->isA(CC_TYPES::GBL_SENSOR))
				{
					ccGBLSensor* gblSensor = static_cast<ccGBLSensor*>(sensor);
					gblSensor->setSensorRange(gblSensor->getSensorRange() * meanScale);
				}
			}
		}
	}

	//update the transformation history
	{
		m_glTransHistory = scaleTrans * m_glTransHistory;
	}

	notifyGeometryUpdate(); //calls releaseVBOs()
}

void ccPointCloud::invertNormals()
{
	if (!hasNormals())
		return;

	for (CompressedNormType& n : *m_normals)
	{
		ccNormalCompressor::InvertNormal(n);
	}

	//We must update the VBOs
	normalsHaveChanged();
}

void ccPointCloud::swapPoints(unsigned firstIndex, unsigned secondIndex)
{
	assert(!isLocked());
	assert(firstIndex < size() && secondIndex < size());

	if (firstIndex == secondIndex)
		return;

	//points + associated SF values
	BaseClass::swapPoints(firstIndex, secondIndex);

	//colors
	if (hasColors())
	{
		assert(m_rgbaColors);
		m_rgbaColors->swap(firstIndex, secondIndex);
	}

	//normals
	if (hasNormals())
	{
		assert(m_normals);
		m_normals->swap(firstIndex, secondIndex);
	}

	//We must update the VBOs
	releaseVBOs();
}

void ccPointCloud::getDrawingParameters(glDrawParams& params) const
{
	//color override
	if (isColorOverridden())
	{
		params.showColors	= true;
		params.showNorms	= false;
		params.showSF		= false;
	}
	else
	{
		//a scalar field must have been selected for display!
		params.showSF = hasDisplayedScalarField() && sfShown() && m_currentDisplayedScalarField->currentSize() >= size();
		params.showNorms = hasNormals() && normalsShown() && m_normals->currentSize() >= size();
		//colors are not displayed if scalar field is displayed
		params.showColors = !params.showSF && hasColors() && colorsShown() && m_rgbaColors->currentSize() >= size();
	}
}

//helpers (for ColorRamp shader)

inline float GetNormalizedValue(const ScalarType& sfVal, const ccScalarField::Range& displayRange)
{
	return static_cast<float>((sfVal - displayRange.start()) / displayRange.range());
}

inline float GetSymmetricalNormalizedValue(const ScalarType& sfVal, const ccScalarField::Range& saturationRange)
{
	//normalized sf value
	ScalarType relativeValue = 0;
	if (std::abs(sfVal) > saturationRange.start()) //we avoid the 'flat' SF case by the way
	{
		if (sfVal < 0)
			relativeValue = (sfVal+saturationRange.start())/saturationRange.max();
		else
			relativeValue = (sfVal-saturationRange.start())/saturationRange.max();
	}
	return (1.0f + relativeValue) / 2;	//normalized sf value
}

//the GL type depends on the PointCoordinateType 'size' (float or double)
static GLenum GL_COORD_TYPE = sizeof(PointCoordinateType) == 4 ? GL_FLOAT : GL_DOUBLE;

void ccPointCloud::glChunkVertexPointer(const CC_DRAW_CONTEXT& context, size_t chunkIndex, unsigned decimStep, bool useVBOs)
{
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (useVBOs
		&&	m_vboManager.state == vboSet::INITIALIZED
		&&	m_vboManager.vbos.size() > static_cast<size_t>(chunkIndex)
		&& m_vboManager.vbos[chunkIndex]
		&& m_vboManager.vbos[chunkIndex]->isCreated())
	{
		//we can use VBOs directly
		if (m_vboManager.vbos[chunkIndex]->bind())
		{
			glFunc->glVertexPointer(3, GL_COORD_TYPE, decimStep * 3 * sizeof(PointCoordinateType), nullptr);
			m_vboManager.vbos[chunkIndex]->release();
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			m_vboManager.state = vboSet::FAILED;
			//recall the method
			glChunkVertexPointer(context, chunkIndex, decimStep, false);
		}
	}
	else
	{
		//standard OpenGL copy
		glFunc->glVertexPointer(3, GL_COORD_TYPE, decimStep * 3 * sizeof(PointCoordinateType), ccChunk::Start(m_points, chunkIndex));
	}
}

///Maximum number of points (per cloud) displayed in a single LOD iteration
//warning MUST BE GREATER THAN 'MAX_NUMBER_OF_ELEMENTS_PER_CHUNK'
#ifdef _DEBUG
static const unsigned MAX_POINT_COUNT_PER_LOD_RENDER_PASS = (1 << 16); //~ 64K
#else
static const unsigned MAX_POINT_COUNT_PER_LOD_RENDER_PASS = (1 << 19); //~ 512K
#endif

//Vertex indexes for OpenGL "arrays" drawing
static PointCoordinateType s_pointBuffer [MAX_POINT_COUNT_PER_LOD_RENDER_PASS * 3];
static PointCoordinateType s_normalBuffer[MAX_POINT_COUNT_PER_LOD_RENDER_PASS * 3];
static ColorCompType       s_rgbBuffer4ub[MAX_POINT_COUNT_PER_LOD_RENDER_PASS * 4];
static float               s_rgbBuffer3f [MAX_POINT_COUNT_PER_LOD_RENDER_PASS * 3];

void ccPointCloud::glChunkNormalPointer(const CC_DRAW_CONTEXT& context, size_t chunkIndex, unsigned decimStep, bool useVBOs)
{
	assert(m_normals);

	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

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
			const GLbyte* start = nullptr; //fake pointer used to prevent warnings on Linux
			int normalDataShift = m_vboManager.vbos[chunkIndex]->normalShift;
			glFunc->glNormalPointer(GL_COORD_TYPE, decimStep * 3 * sizeof(PointCoordinateType), static_cast<const GLvoid*>(start + normalDataShift));
			m_vboManager.vbos[chunkIndex]->release();
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			m_vboManager.state = vboSet::FAILED;
			//recall the method
			glChunkNormalPointer(context, chunkIndex, decimStep, false);
		}
	}
	else if (m_normals)
	{
		//we must decode normals in a dedicated static array
		PointCoordinateType* _normals = s_normalBuffer;
		const CompressedNormType* _normalsIndexes = ccChunk::Start(*m_normals, chunkIndex);
		size_t chunkSize = ccChunk::Size(chunkIndex, m_normals->size());

		//compressed normals set
		const ccNormalVectors* compressedNormals = ccNormalVectors::GetUniqueInstance();
		assert(compressedNormals);

		for (size_t j = 0; j < chunkSize; j += decimStep, _normalsIndexes += decimStep)
		{
			const CCVector3& N = compressedNormals->getNormal(*_normalsIndexes);
			*(_normals)++ = N.x;
			*(_normals)++ = N.y;
			*(_normals)++ = N.z;
		}
		glFunc->glNormalPointer(GL_COORD_TYPE, 0, s_normalBuffer);
	}
	else
	{
		assert(false);
	}
}

void ccPointCloud::glChunkColorPointer(const CC_DRAW_CONTEXT& context, size_t chunkIndex, unsigned decimStep, bool useVBOs)
{
	assert(m_rgbaColors);
	assert(sizeof(ColorCompType) == 1);

	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (useVBOs
		&&	m_vboManager.state == vboSet::INITIALIZED
		&&	m_vboManager.hasColors
		&&	m_vboManager.vbos.size() > static_cast<size_t>(chunkIndex)
		&& m_vboManager.vbos[chunkIndex]
		&& m_vboManager.vbos[chunkIndex]->isCreated())
	{
		//we can use VBOs directly
		if (m_vboManager.vbos[chunkIndex]->bind())
		{
			const GLbyte* start = nullptr; //fake pointer used to prevent warnings on Linux
			int colorDataShift = m_vboManager.vbos[chunkIndex]->rgbShift;
			glFunc->glColorPointer(4, GL_UNSIGNED_BYTE, decimStep * 4 * sizeof(ColorCompType), static_cast<const GLvoid*>(start + colorDataShift));
			m_vboManager.vbos[chunkIndex]->release();
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			m_vboManager.state = vboSet::FAILED;
			//recall the method
			glChunkColorPointer(context, chunkIndex, decimStep, false);
		}
	}
	else if (m_rgbaColors)
	{
		assert(m_rgbaColors);
		//standard OpenGL copy
		glFunc->glColorPointer(4, GL_UNSIGNED_BYTE, decimStep * 4 * sizeof(ColorCompType), ccChunk::Start(*m_rgbaColors, chunkIndex));
	}
	else
	{
		assert(false);
	}
}

void ccPointCloud::glChunkSFPointer(const CC_DRAW_CONTEXT& context, size_t chunkIndex, unsigned decimStep, bool useVBOs)
{
	assert(m_currentDisplayedScalarField);
	assert(sizeof(ColorCompType) == 1);

	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (useVBOs
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
			const GLbyte* start = nullptr; //fake pointer used to prevent warnings on Linux
			int colorDataShift = m_vboManager.vbos[chunkIndex]->rgbShift;
			glFunc->glColorPointer(4, GL_UNSIGNED_BYTE, decimStep * 4 * sizeof(ColorCompType), static_cast<const GLvoid*>(start + colorDataShift));
			m_vboManager.vbos[chunkIndex]->release();
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			m_vboManager.state = vboSet::FAILED;
			//call the method again
			glChunkSFPointer(context, chunkIndex, decimStep, false);
		}
	}
	else if (m_currentDisplayedScalarField)
	{
		//we must convert the scalar values to RGB colors in a dedicated static array
		ScalarType* _sf = ccChunk::Start(*m_currentDisplayedScalarField, chunkIndex);
		ColorCompType* _sfColors = s_rgbBuffer4ub;
		size_t chunkSize = ccChunk::Size(chunkIndex, m_currentDisplayedScalarField->size());
		for (size_t j = 0; j < chunkSize; j += decimStep, _sf += decimStep)
		{
			//convert the scalar value to a RGB color
			const ccColor::Rgb* col = m_currentDisplayedScalarField->getColor(*_sf);
			assert(col);
			*_sfColors++ = col->r;
			*_sfColors++ = col->g;
			*_sfColors++ = col->b;
			*_sfColors++ = ccColor::MAX;
		}
		glFunc->glColorPointer(4, GL_UNSIGNED_BYTE, 0, s_rgbBuffer4ub);
	}
}

template <class QOpenGLFunctions> void glLODChunkVertexPointer(	ccPointCloud* cloud,
																QOpenGLFunctions* glFunc,
																const LODIndexSet& indexMap,
																unsigned startIndex,
																unsigned stopIndex)
{
	assert(startIndex < indexMap.size() && stopIndex <= indexMap.size());
	assert(cloud && glFunc);

	PointCoordinateType* _points = s_pointBuffer;
	for (unsigned j = startIndex; j < stopIndex; j++)
	{
		unsigned pointIndex = indexMap[j];
		const CCVector3* P = cloud->getPoint(pointIndex);
		*(_points)++ = P->x;
		*(_points)++ = P->y;
		*(_points)++ = P->z;
	}
	//standard OpenGL copy
	glFunc->glVertexPointer(3, GL_COORD_TYPE, 0, s_pointBuffer);
}

template <class QOpenGLFunctions> void glLODChunkNormalPointer(	NormsIndexesTableType* normals,
																QOpenGLFunctions* glFunc,
																const LODIndexSet& indexMap,
																unsigned startIndex,
																unsigned stopIndex)
{
	assert(startIndex < indexMap.size() && stopIndex <= indexMap.size());
	assert(normals && glFunc);

	//compressed normals set
	const ccNormalVectors* compressedNormals = ccNormalVectors::GetUniqueInstance();
	assert(compressedNormals);

	//we must decode normals in a dedicated static array
	PointCoordinateType* _normals = s_normalBuffer;
	for (unsigned j = startIndex; j < stopIndex; j++)
	{
		unsigned pointIndex = indexMap[j];
		const CCVector3& N = compressedNormals->getNormal(normals->at(pointIndex));
		*(_normals)++ = N.x;
		*(_normals)++ = N.y;
		*(_normals)++ = N.z;
	}
	//standard OpenGL copy
	glFunc->glNormalPointer(GL_COORD_TYPE, 0, s_normalBuffer);
}

template <class QOpenGLFunctions> void glLODChunkColorPointer(	RGBAColorsTableType* colors,
																QOpenGLFunctions* glFunc, 
																const LODIndexSet& indexMap,
																unsigned startIndex,
																unsigned stopIndex)
{
	assert(startIndex < indexMap.size() && stopIndex <= indexMap.size());
	assert(colors && glFunc);
	assert(sizeof(ColorCompType) == 1);

	//we must re-order colors in a dedicated static array
	ColorCompType* _rgba = s_rgbBuffer4ub;
	for (unsigned j = startIndex; j < stopIndex; j++)
	{
		unsigned pointIndex = indexMap[j];
		const ccColor::Rgba& col = colors->at(pointIndex);
		*(_rgba)++ = col.r;
		*(_rgba)++ = col.g;
		*(_rgba)++ = col.b;
		*(_rgba)++ = col.a;
	}
	//standard OpenGL copy
	glFunc->glColorPointer(4, GL_UNSIGNED_BYTE, 0, s_rgbBuffer4ub);
}

template <class QOpenGLFunctions> void glLODChunkSFPointer(	ccScalarField* sf,
															QOpenGLFunctions* glFunc,
															const LODIndexSet& indexMap,
															unsigned startIndex,
															unsigned stopIndex)
{
	assert(startIndex < indexMap.size() && stopIndex <= indexMap.size());
	assert(sf && glFunc);
	assert(sizeof(ColorCompType) == 1);

	//we must re-order and convert SF values to RGB colors in a dedicated static array
	ColorCompType* _sfColors = s_rgbBuffer4ub;
	for (unsigned j = startIndex; j < stopIndex; j++)
	{
		unsigned pointIndex = indexMap[j];
		//convert the scalar value to a RGB color
		const ccColor::Rgb* col = sf->getColor(sf->at(pointIndex));
		assert(col);
		*_sfColors++ = col->r;
		*_sfColors++ = col->g;
		*_sfColors++ = col->b;
		*_sfColors++ = ccColor::MAX;
	}
	//standard OpenGL copy
	glFunc->glColorPointer(4, GL_UNSIGNED_BYTE, 0, s_rgbBuffer4ub);
}

//description of the (sub)set of points to display
struct DisplayDesc : LODLevelDesc
{
	//! Default constructor
	DisplayDesc()
		: LODLevelDesc()
		, endIndex(0)
		, decimStep(1)
		, indexMap(nullptr)
	{}

	//! Constructor from a start index and a count value
	DisplayDesc(unsigned startIndex, unsigned count)
		: LODLevelDesc(startIndex, count)
		, endIndex(startIndex+count)
		, decimStep(1)
		, indexMap(nullptr)
	{}

	//! Set operator
	DisplayDesc& operator = (const LODLevelDesc& desc)
	{
		startIndex = desc.startIndex;
		count = desc.count;
		endIndex = startIndex + count;
		return *this;
	}
	
	//! Last index (excluded)
	unsigned endIndex;
	
	//! Decimation step (for non-octree based LoD)
	unsigned decimStep;

	//! Map of indexes (to invert the natural order)
	LODIndexSet* indexMap;
};

void ccPointCloud::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (m_points.empty())
		return;

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (MACRO_Draw3D(context))
	{
		//we get display parameters
		glDrawParams glParams;
		getDrawingParameters(glParams);
		//no normals shading without light!
		if (!MACRO_LightIsEnabled(context))
		{
			glParams.showNorms = false;
		}

		//can't display a SF without... a SF... and an active color scale!
		assert(!glParams.showSF || hasDisplayedScalarField());

		//color-based entity picking
		bool entityPickingMode = MACRO_EntityPicking(context);
		ccColor::Rgb pickingColor;
		if (entityPickingMode)
		{
			//not fast at all!
			if (MACRO_FastEntityPicking(context))
			{
				return;
			}

			pickingColor = context.entityPicking.registerEntity(this);

			//minimal display for picking mode!
			glParams.showNorms = false;
			glParams.showColors = false;
			if (glParams.showSF && !m_currentDisplayedScalarField->mayHaveHiddenValues())
			{
				glParams.showSF = false; //--> we keep it only if SF 'NaN' values are potentially hidden
			}
		}

		// L.O.D. display
		DisplayDesc toDisplay(0, size());
		if (!entityPickingMode)
		{
			if (	context.decimateCloudOnMove
				&&	toDisplay.count > context.minLODPointCount
				&&	MACRO_LODActivated(context)
				)
			{
				bool skipLoD = false;

				//is there a LoD structure associated yet?
				if (!m_lod || !m_lod->isBroken())
				{
					if (!m_lod || m_lod->isNull())
					{
						//auto-init LoD structure
						//DGM: can't spawn a progress dialog here as the process will be async
						//ccProgressDialog pDlg(false, context.display ? context.display->asWidget() : 0);
						initLOD(/*&pDlg*/);
					}
					else
					{
						assert(m_lod);

						unsigned char maxLevel = m_lod->maxLevel();
						bool underConstruction = m_lod->isUnderConstruction();

						//if the cloud has less LOD levels than the minimum to display
						if (underConstruction || maxLevel == 0)
						{
							//not yet ready
							context.moreLODPointsAvailable = underConstruction;
							context.higherLODLevelsAvailable = false;
						}
						else if (context.stereoPassIndex == 0)
						{
							if (context.currentLODLevel == 0)
							{
								//get the current viewport and OpenGL matrices
								ccGLCameraParameters camera;
								context.display->getGLCameraParameters(camera);
								//replace the viewport and matrices by the real ones
								glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
								glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
								glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());
								//camera frustum
								Frustum frustum(camera.modelViewMat, camera.projectionMat);

								//first time: we flag the cells visibility and count the number of visible points
								m_lod->flagVisibility(frustum, m_clipPlanes.empty() ? nullptr : &m_clipPlanes);
							}

							unsigned remainingPointsAtThisLevel = 0;
							toDisplay.startIndex = 0;
							toDisplay.count = MAX_POINT_COUNT_PER_LOD_RENDER_PASS;
							toDisplay.indexMap = &m_lod->getIndexMap(context.currentLODLevel, toDisplay.count, remainingPointsAtThisLevel);
							if (toDisplay.count == 0)
							{
								//nothing to draw at this level
								toDisplay.indexMap = nullptr;
							}
							else
							{
								assert(toDisplay.count == toDisplay.indexMap->size());
								toDisplay.endIndex = toDisplay.startIndex + toDisplay.count;
							}

							//could we draw more points at the next level?
							context.moreLODPointsAvailable = (remainingPointsAtThisLevel != 0);
							context.higherLODLevelsAvailable = (!m_lod->allDisplayed() && context.currentLODLevel + 1 <= maxLevel);
						}
					}
				}

				if (!toDisplay.indexMap && !skipLoD)
				{
					//if we don't have a LoD map, we can only display points at level 0!
					if (context.currentLODLevel != 0)
					{
						return;
					}

					//we wait for the LOD to be ready
					//meanwhile we will display less points
					if (context.minLODPointCount && toDisplay.count > context.minLODPointCount)
					{
						GLint maxStride = 2048;
#ifdef GL_MAX_VERTEX_ATTRIB_STRIDE
						glFunc->glGetIntegerv(GL_MAX_VERTEX_ATTRIB_STRIDE, &maxStride);
#endif
						//maxStride == decimStep * 3 * sizeof(PointCoordinateType)
						toDisplay.decimStep = static_cast<int>(ceil(static_cast<float>(toDisplay.count) / context.minLODPointCount));
						toDisplay.decimStep = std::min<unsigned>(toDisplay.decimStep, maxStride / (3 * sizeof(PointCoordinateType)));
					}
				}
			}
		}

		//ccLog::Print(QString("Rendering %1 points starting from index %2 (LoD = %3 / PN = %4)").arg(toDisplay.count).arg(toDisplay.startIndex).arg(toDisplay.indexMap ? "yes" : "no").arg(pushName ? "yes" : "no"));

		glFunc->glPushAttrib(GL_LIGHTING_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT | GL_POINT_BIT);

		if (glParams.showSF || glParams.showColors)
		{
			glFunc->glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
			glFunc->glEnable(GL_COLOR_MATERIAL);
			glFunc->glEnable(GL_BLEND);
		}

		if (entityPickingMode)
		{
			ccGL::Color(glFunc, pickingColor);
		}
		else if (glParams.showColors && isColorOverridden())
		{
			ccGL::Color(glFunc, m_tempColor);
			glParams.showColors = false;
		}
		else
		{
			ccGL::Color(glFunc, context.pointsDefaultCol);
		}

		//in the case we need normals (i.e. lighting)
		if (glParams.showNorms)
		{
			glFunc->glEnable(GL_RESCALE_NORMAL);
			glFunc->glMaterialfv(GL_FRONT_AND_BACK,	GL_AMBIENT,		CC_DEFAULT_CLOUD_AMBIENT_COLOR.rgba  );
			glFunc->glMaterialfv(GL_FRONT_AND_BACK,	GL_SPECULAR,	CC_DEFAULT_CLOUD_SPECULAR_COLOR.rgba );
			glFunc->glMaterialfv(GL_FRONT_AND_BACK,	GL_DIFFUSE,		CC_DEFAULT_CLOUD_DIFFUSE_COLOR.rgba  );
			glFunc->glMaterialfv(GL_FRONT_AND_BACK,	GL_EMISSION,	CC_DEFAULT_CLOUD_EMISSION_COLOR.rgba );
			glFunc->glMaterialf (GL_FRONT_AND_BACK,	GL_SHININESS,	CC_DEFAULT_CLOUD_SHININESS);
			glFunc->glEnable(GL_LIGHTING);

			if (glParams.showSF)
			{
				//we must get rid of lights 'color' if a scalar field is displayed!
				ccMaterial::MakeLightsNeutral(context.qGLContext);
			}
		}

		/*** DISPLAY ***/

		//rounded points
		if (context.drawRoundedPoints)
		{
			glFunc->glPushAttrib(GL_POINT_BIT);
			//DGM: alpha/blending doesn't work well because it creates a halo around points with a potentially wrong color (due to the display order)
			//glFunc->glDisable(GL_BLEND);
			glFunc->glEnable(GL_POINT_SMOOTH);
			//glFunc->glEnable(GL_ALPHA_TEST);
			//glFunc->glAlphaFunc(GL_GREATER, 0.5);
			//glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			//glFunc->glEnable(GL_BLEND);
		}

		//custom point size?
		if (m_pointSize != 0)
		{
			glFunc->glPointSize(static_cast<GLfloat>(m_pointSize));
		}

		//main display procedure
		{
			//if some points are hidden (= visibility table instantiated), we can't use display arrays :(
			if (isVisibilityTableInstantiated())
			{
				assert(m_pointsVisibility.size() == m_points.size());
				//compressed normals set
				const ccNormalVectors* compressedNormals = ccNormalVectors::GetUniqueInstance();
				assert(compressedNormals);

				glFunc->glBegin(GL_POINTS);

				if (!entityPickingMode)
				{
					for (unsigned j = toDisplay.startIndex; j < toDisplay.endIndex; j += toDisplay.decimStep)
					{
						//we must test each point visibility
						unsigned pointIndex = toDisplay.indexMap ? toDisplay.indexMap->at(j) : j;
						if (m_pointsVisibility.empty() || m_pointsVisibility[pointIndex] == CCCoreLib::POINT_VISIBLE)
						{
							if (glParams.showSF)
							{
								assert(pointIndex < m_currentDisplayedScalarField->currentSize());
								const ccColor::Rgb* col = m_currentDisplayedScalarField->getValueColor(pointIndex);
								//we force display of points hidden because of their scalar field value
								//to be sure that the user doesn't miss them (during manual segmentation for instance)
								ccGL::Color(glFunc, col ? *col : ccColor::lightGreyRGB); //Make sure all points are visible. No alpha used on purpose 
							}
							else if (glParams.showColors)
							{
								ccGL::Color(glFunc, m_rgbaColors->getValue(pointIndex));
							}
							if (glParams.showNorms)
							{
								ccGL::Normal3v(glFunc, compressedNormals->getNormal(m_normals->getValue(pointIndex)).u);
							}
							ccGL::Vertex3v(glFunc, m_points[pointIndex].u);
						}
					}
				}
				else
				{
					for (unsigned j = toDisplay.startIndex; j < toDisplay.endIndex; j += toDisplay.decimStep)
					{
						//we must test each point visibility
						unsigned pointIndex = toDisplay.indexMap ? toDisplay.indexMap->at(j) : j;
						if (m_pointsVisibility.empty() || m_pointsVisibility[pointIndex] == CCCoreLib::POINT_VISIBLE)
						{
							//in color-based picking mode, we only display the points
							ccGL::Vertex3v(glFunc, m_points[pointIndex].u);
						}
					}
				}

				glFunc->glEnd();
			}
			else if (glParams.showSF) //no visibility table enabled + scalar field
			{
				assert(m_currentDisplayedScalarField);

				//if some points may not be displayed, we'll have to be smarter!
				bool hiddenPoints = m_currentDisplayedScalarField->mayHaveHiddenValues();

				//whether VBOs are available (for faster display) or not
				bool useVBOs = false;
				if (!hiddenPoints && context.useVBOs && !toDisplay.indexMap) //VBOs are not compatible with LoD
				{
					//can't use VBOs if some points are hidden
					useVBOs = updateVBOs(context, glParams);
				}

				//color ramp shader initialization
				ccColorRampShader* colorRampShader = context.colorRampShader;
				{
					//color ramp shader is not compatible with VBOs (and VBOs are faster)
					if (useVBOs)
					{
						colorRampShader = nullptr;
					}
					//FIXME: color ramp shader doesn't support log scale yet!
					if (m_currentDisplayedScalarField->logScale())
					{
						colorRampShader = nullptr;
					}
					//the shader can't be used during color-based color picking
					if (entityPickingMode)
					{
						colorRampShader = nullptr;
					}
				}

				const ccScalarField::Range& sfDisplayRange = m_currentDisplayedScalarField->displayRange();
				const ccScalarField::Range& sfSaturationRange = m_currentDisplayedScalarField->saturationRange();

				if (colorRampShader)
				{
					//max available space for fragment's shader uniforms
					GLint maxBytes = 0;
					glFunc->glGetIntegerv(GL_MAX_FRAGMENT_UNIFORM_COMPONENTS, &maxBytes);
					GLint maxComponents = (maxBytes >> 2) - 4; //leave space for the other uniforms!
					unsigned steps = m_currentDisplayedScalarField->getColorRampSteps();
					assert(steps != 0);

					if (steps > ccColorRampShader::MaxColorRampSize() || maxComponents < static_cast<GLint>(steps))
					{
						ccLog::WarningDebug("Color ramp steps exceed shader limits!");
						colorRampShader = nullptr;
					}
					else
					{
						float sfMinSatRel = 0.0f;
						float sfMaxSatRel = 1.0f;
						if (!m_currentDisplayedScalarField->symmetricalScale())
						{
							sfMinSatRel = GetNormalizedValue(sfSaturationRange.start(), sfDisplayRange);	//doesn't need to be between 0 and 1!
							sfMaxSatRel = GetNormalizedValue(sfSaturationRange.stop(), sfDisplayRange);		//doesn't need to be between 0 and 1!
						}
						else
						{
							//we can only handle 'maximum' saturation
							sfMinSatRel = GetSymmetricalNormalizedValue(-sfSaturationRange.stop(), sfSaturationRange);
							sfMaxSatRel = GetSymmetricalNormalizedValue(sfSaturationRange.stop(), sfSaturationRange);
							//we'll have to handle the 'minimum' saturation manually!
						}

						const ccColorScale::Shared& colorScale = m_currentDisplayedScalarField->getColorScale();
						assert(colorScale);

						colorRampShader->bind();
						if (!colorRampShader->setup(glFunc, sfMinSatRel, sfMaxSatRel, steps, colorScale))
						{
							//An error occurred during shader initialization?
							ccLog::WarningDebug("Failed to init ColorRamp shader!");
							colorRampShader->release();
							colorRampShader = nullptr;
						}
						else if (glParams.showNorms)
						{
							//we must get rid of lights material (other than ambient) for the red and green fields
							glFunc->glPushAttrib(GL_LIGHTING_BIT);

							//we use the ambient light to pass the scalar value (and 'grayed' marker) without any
							//modification from the GPU pipeline, even if normals are enabled!
							glFunc->glDisable(GL_COLOR_MATERIAL);
							glFunc->glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
							glFunc->glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
							glFunc->glEnable(GL_COLOR_MATERIAL);

							GLint maxLightCount;
							glFunc->glGetIntegerv(GL_MAX_LIGHTS, &maxLightCount);
							for (GLint i = 0; i < maxLightCount; ++i)
							{
								if (glFunc->glIsEnabled(GL_LIGHT0 + i))
								{
									float diffuse[4];
									float ambiant[4];
									float specular[4];

									glFunc->glGetLightfv(GL_LIGHT0 + i, GL_AMBIENT, ambiant);
									glFunc->glGetLightfv(GL_LIGHT0 + i, GL_DIFFUSE, diffuse);
									glFunc->glGetLightfv(GL_LIGHT0 + i, GL_SPECULAR, specular);

									ambiant[0] = ambiant[1] = 1.0f;
									diffuse[0] = diffuse[1] = 0.0f;
									specular[0] = specular[1] = 0.0f;

									glFunc->glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, diffuse);
									glFunc->glLightfv(GL_LIGHT0 + i, GL_AMBIENT, ambiant);
									glFunc->glLightfv(GL_LIGHT0 + i, GL_SPECULAR, specular);
								}
							}
						}
					}
				}

				//if all points should be displayed (fastest case)
				if (!hiddenPoints)
				{
					glFunc->glEnableClientState(GL_VERTEX_ARRAY);
					glFunc->glEnableClientState(GL_COLOR_ARRAY);
					if (glParams.showNorms)
					{
						glFunc->glEnableClientState(GL_NORMAL_ARRAY);
					}

					if (toDisplay.indexMap) //LoD display
					{
						unsigned s = toDisplay.startIndex;
						while (s < toDisplay.endIndex)
						{
							unsigned count = std::min(MAX_POINT_COUNT_PER_LOD_RENDER_PASS, toDisplay.endIndex - s);
							unsigned e = s + count;

							//points
							glLODChunkVertexPointer<QOpenGLFunctions_2_1>(this, glFunc, *toDisplay.indexMap, s, e);
							//normals
							if (glParams.showNorms)
							{
								glLODChunkNormalPointer<QOpenGLFunctions_2_1>(m_normals, glFunc, *toDisplay.indexMap, s, e);
							}
							//SF colors
							if (colorRampShader)
							{
								float* _sfColors = s_rgbBuffer3f;
								bool symScale = m_currentDisplayedScalarField->symmetricalScale();
								for (unsigned j = s; j < e; j++, _sfColors += 3)
								{
									unsigned pointIndex = toDisplay.indexMap->at(j);
									ScalarType sfVal = m_currentDisplayedScalarField->getValue(pointIndex);
									//normalized sf value
									_sfColors[0] = symScale ? GetSymmetricalNormalizedValue(sfVal, sfSaturationRange) : GetNormalizedValue(sfVal, sfDisplayRange);
									//flag: whether point is grayed out or not (NaN values are also rejected!)
									_sfColors[1] = sfDisplayRange.isInRange(sfVal) ? 1.0f : 0.0f;
									//reference value (to get the true lighting value)
									_sfColors[2] = 1.0f;
								}
								glFunc->glColorPointer(3, GL_FLOAT, 0, s_rgbBuffer3f);
							}
							else
							{
								glLODChunkSFPointer<QOpenGLFunctions_2_1>(m_currentDisplayedScalarField, glFunc, *toDisplay.indexMap, s, e);
							}

							glFunc->glDrawArrays(GL_POINTS, 0, count);

							s = e;
						}
					}
					else
					{
						size_t chunkCount = ccChunk::Count(m_points);
						for (size_t k = 0; k < chunkCount; ++k)
						{
							size_t chunkSize = ccChunk::Size(k, m_points);

							//points
							glChunkVertexPointer(context, k, toDisplay.decimStep, useVBOs);
							//normals
							if (glParams.showNorms)
							{
								glChunkNormalPointer(context, k, toDisplay.decimStep, useVBOs);
							}
							//SF colors
							if (colorRampShader)
							{
								ScalarType* _sf = ccChunk::Start(*m_currentDisplayedScalarField, k);
								float* _sfColors = s_rgbBuffer3f;
								bool symScale = m_currentDisplayedScalarField->symmetricalScale();
								for (unsigned j = 0; j < chunkSize; j += toDisplay.decimStep, _sf += toDisplay.decimStep, _sfColors += 3)
								{
									//normalized sf value
									_sfColors[0] = symScale ? GetSymmetricalNormalizedValue(*_sf, sfSaturationRange) : GetNormalizedValue(*_sf, sfDisplayRange);
									//flag: whether point is grayed out or not (NaN values are also rejected!)
									_sfColors[1] = sfDisplayRange.isInRange(*_sf) ? 1.0f : 0.0f;
									//reference value (to get the true lighting value)
									_sfColors[2] = 1.0f;
								}
								glFunc->glColorPointer(3, GL_FLOAT, 0, s_rgbBuffer3f);
							}
							else
							{
								glChunkSFPointer(context, k, toDisplay.decimStep, useVBOs);
							}

							if (toDisplay.decimStep > 1)
							{
								chunkSize = static_cast<unsigned>(floor(static_cast<float>(chunkSize) / toDisplay.decimStep));
							}
							glFunc->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(chunkSize));
						}
					}

					if (glParams.showNorms)
					{
						glFunc->glDisableClientState(GL_NORMAL_ARRAY);
					}
					glFunc->glDisableClientState(GL_COLOR_ARRAY);
					glFunc->glDisableClientState(GL_VERTEX_ARRAY);
				}
				else //potentially hidden points
				{
					//compressed normals set
					const ccNormalVectors* compressedNormals = ccNormalVectors::GetUniqueInstance();
					assert(compressedNormals);

					glFunc->glBegin(GL_POINTS);

					if (glParams.showNorms) //with normals (slowest case!)
					{
						if (colorRampShader)
						{
							if (!m_currentDisplayedScalarField->symmetricalScale())
							{
								for (unsigned j = toDisplay.startIndex; j < toDisplay.endIndex; j += toDisplay.decimStep)
								{
									unsigned pointIndex = (toDisplay.indexMap ? toDisplay.indexMap->at(j) : j);
									assert(pointIndex < m_currentDisplayedScalarField->currentSize());
									const ScalarType sf = m_currentDisplayedScalarField->getValue(pointIndex);
									if (sfDisplayRange.isInRange(sf)) //NaN values are rejected
									{
										glFunc->glColor3f(GetNormalizedValue(sf, sfDisplayRange), 1.0f, 1.0f);
										ccGL::Normal3v(glFunc, compressedNormals->getNormal(m_normals->getValue(pointIndex)).u);
										ccGL::Vertex3v(glFunc, m_points[pointIndex].u);
									}
								}
							}
							else
							{
								for (unsigned j = toDisplay.startIndex; j < toDisplay.endIndex; j += toDisplay.decimStep)
								{
									unsigned pointIndex = (toDisplay.indexMap ? toDisplay.indexMap->at(j) : j);
									assert(pointIndex < m_currentDisplayedScalarField->currentSize());
									const ScalarType sf = m_currentDisplayedScalarField->getValue(pointIndex);
									if (sfDisplayRange.isInRange(sf)) //NaN values are rejected
									{
										glFunc->glColor3f(GetSymmetricalNormalizedValue(sf, sfSaturationRange), 1.0f, 1.0f);
										ccGL::Normal3v(glFunc, compressedNormals->getNormal(m_normals->getValue(pointIndex)).u);
										ccGL::Vertex3v(glFunc, m_points[pointIndex].u);
									}
								}
							}
						}
						else
						{
							for (unsigned j = toDisplay.startIndex; j < toDisplay.endIndex; j += toDisplay.decimStep)
							{
								unsigned pointIndex = (toDisplay.indexMap ? toDisplay.indexMap->at(j) : j);
								assert(pointIndex < m_currentDisplayedScalarField->currentSize());
								const ccColor::Rgb* col = m_currentDisplayedScalarField->getValueColor(pointIndex);
								if (col)
								{
									ccGL::Color(glFunc, *col);
									ccGL::Normal3v(glFunc, compressedNormals->getNormal(m_normals->getValue(pointIndex)).u);
									ccGL::Vertex3v(glFunc, m_points[pointIndex].u);
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
								for (unsigned j = toDisplay.startIndex; j < toDisplay.endIndex; j += toDisplay.decimStep)
								{
									unsigned pointIndex = (toDisplay.indexMap ? toDisplay.indexMap->at(j) : j);
									assert(pointIndex < m_currentDisplayedScalarField->currentSize());
									const ScalarType sf = m_currentDisplayedScalarField->getValue(pointIndex);
									if (sfDisplayRange.isInRange(sf)) //NaN values are rejected
									{
										glFunc->glColor3f(GetNormalizedValue(sf, sfDisplayRange), 1.0f, 1.0f);
										ccGL::Vertex3v(glFunc, m_points[pointIndex].u);
									}
								}
							}
							else
							{
								for (unsigned j = toDisplay.startIndex; j < toDisplay.endIndex; j += toDisplay.decimStep)
								{
									unsigned pointIndex = (toDisplay.indexMap ? toDisplay.indexMap->at(j) : j);
									assert(pointIndex < m_currentDisplayedScalarField->currentSize());
									const ScalarType sf = m_currentDisplayedScalarField->getValue(pointIndex);
									if (sfDisplayRange.isInRange(sf)) //NaN values are rejected
									{
										glFunc->glColor3f(GetSymmetricalNormalizedValue(sf, sfSaturationRange), 1.0f, 1.0f);
										ccGL::Vertex3v(glFunc, m_points[pointIndex].u);
									}
								}
							}
						}
						else if (entityPickingMode)
						{
							for (unsigned j = toDisplay.startIndex; j < toDisplay.endIndex; j += toDisplay.decimStep)
							{
								unsigned pointIndex = (toDisplay.indexMap ? toDisplay.indexMap->at(j) : j);
								assert(pointIndex < m_currentDisplayedScalarField->currentSize());
								const ccColor::Rgb* col = m_currentDisplayedScalarField->getValueColor(pointIndex);
								if (col)
								{
									//for entity picking, don't change the color, we just need to know whether the point is visible
									ccGL::Vertex3v(glFunc, m_points[pointIndex].u);
								}
							}
						}
						else
						{
							for (unsigned j = toDisplay.startIndex; j < toDisplay.endIndex; j += toDisplay.decimStep)
							{
								unsigned pointIndex = (toDisplay.indexMap ? toDisplay.indexMap->at(j) : j);
								assert(pointIndex < m_currentDisplayedScalarField->currentSize());
								const ccColor::Rgb* col = m_currentDisplayedScalarField->getValueColor(pointIndex);
								if (col)
								{
									ccGL::Color(glFunc, *col);
									ccGL::Vertex3v(glFunc, m_points[pointIndex].u);
								}
							}
						}
					}
					glFunc->glEnd();
				}

				if (colorRampShader)
				{
					colorRampShader->release();

					if (glParams.showNorms)
					{
						glFunc->glPopAttrib(); //GL_LIGHTING_BIT
					}
				}
			}
			else //no visibility table enabled, no scalar field
			{
				bool useVBOs = context.useVBOs && !toDisplay.indexMap ? updateVBOs(context, glParams) : false; //VBOs are not compatible with LoD

				size_t chunkCount = ccChunk::Count(m_points);

				glFunc->glEnableClientState(GL_VERTEX_ARRAY);
				if (glParams.showNorms)
					glFunc->glEnableClientState(GL_NORMAL_ARRAY);
				if (glParams.showColors)
					glFunc->glEnableClientState(GL_COLOR_ARRAY);

				if (toDisplay.indexMap) //LoD display
				{
					unsigned s = toDisplay.startIndex;
					while (s < toDisplay.endIndex)
					{
						unsigned count = std::min(MAX_POINT_COUNT_PER_LOD_RENDER_PASS, toDisplay.endIndex - s);
						unsigned e = s + count;

						//points
						glLODChunkVertexPointer<QOpenGLFunctions_2_1>(this, glFunc, *toDisplay.indexMap, s, e);
						//normals
						if (glParams.showNorms)
							glLODChunkNormalPointer<QOpenGLFunctions_2_1>(m_normals, glFunc, *toDisplay.indexMap, s, e);
						//colors
						if (glParams.showColors)
							glLODChunkColorPointer<QOpenGLFunctions_2_1>(m_rgbaColors, glFunc, *toDisplay.indexMap, s, e);

						glFunc->glDrawArrays(GL_POINTS, 0, count);
						s = e;
					}
				}
				else
				{
					for (size_t k = 0; k < chunkCount; ++k)
					{
						size_t chunkSize = ccChunk::Size(k, m_points);

						//points
						glChunkVertexPointer(context, k, toDisplay.decimStep, useVBOs);
						//normals
						if (glParams.showNorms)
							glChunkNormalPointer(context, k, toDisplay.decimStep, useVBOs);
						//colors
						if (glParams.showColors)
							glChunkColorPointer(context, k, toDisplay.decimStep, useVBOs);

						if (toDisplay.decimStep > 1)
						{
							chunkSize = static_cast<unsigned>(floor(static_cast<float>(chunkSize) / toDisplay.decimStep));
						}
						glFunc->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(chunkSize));
					}
				}

				glFunc->glDisableClientState(GL_VERTEX_ARRAY);
				if (glParams.showNorms)
					glFunc->glDisableClientState(GL_NORMAL_ARRAY);
				if (glParams.showColors)
					glFunc->glDisableClientState(GL_COLOR_ARRAY);
			}
		}

		/*** END DISPLAY ***/

		if (context.drawRoundedPoints)
		{
			glFunc->glPopAttrib(); //GL_POINT_BIT
		}

		glFunc->glPopAttrib(); //GL_LIGHTING_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT | GL_POINT_BIT --> will switch the light off
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

ccPointCloud* ccPointCloud::filterPointsByScalarValue(ScalarType minVal, ScalarType maxVal, bool outside/*=false*/)
{
	if (!getCurrentOutScalarField())
	{
		return nullptr;
	}

	QSharedPointer<CCCoreLib::ReferenceCloud> c(CCCoreLib::ManualSegmentationTools::segment(this, minVal, maxVal, outside));

	return (c ? partialClone(c.data()) : nullptr);
}

void ccPointCloud::hidePointsByScalarValue(ScalarType minVal, ScalarType maxVal)
{
	if (!resetVisibilityArray())
	{
		ccLog::Error(QString("[Cloud %1] Visibility table could not be instantiated!").arg(getName()));
		return;
	}

	CCCoreLib::ScalarField* sf = getCurrentOutScalarField();
	if (!sf)
	{
		ccLog::Error(QString("[Cloud %1] Internal error: no activated output scalar field!").arg(getName()));
		return;
	}

	//we use the visibility table to tag the points to filter out
	unsigned count = size();
	for (unsigned i = 0; i < count; ++i)
	{
		const ScalarType& val = sf->getValue(i);
		if (val < minVal || val > maxVal || val != val) //handle NaN values!
		{
			m_pointsVisibility[i] = CCCoreLib::POINT_HIDDEN;
		}
	}
}

ccGenericPointCloud* ccPointCloud::createNewCloudFromVisibilitySelection(bool removeSelectedPoints/*=false*/, VisibilityTableType* visTable/*=nullptr*/, bool silent/*=false*/)
{
	if (!visTable)
	{
		if (!isVisibilityTableInstantiated())
		{
			ccLog::Error(QString("[Cloud %1] Visibility table not instantiated!").arg(getName()));
			return nullptr;
		}
		visTable = &m_pointsVisibility;
	}
	else
	{
		if (visTable->size() != size())
		{
			ccLog::Error(QString("[Cloud %1] Invalid input visibility table").arg(getName()));
			return nullptr;
		}
	}

	//we create a new cloud with the "visible" points
	ccPointCloud* result = nullptr;
	{
		//we create a temporary entity with the visible points only
		CCCoreLib::ReferenceCloud* rc = getTheVisiblePoints(visTable, silent);
		if (!rc)
		{
			//a warning message has already been issued by getTheVisiblePoints!
			//ccLog::Warning("[ccPointCloud] An error occurred during points selection!");
			return nullptr;
		}

		//convert selection to cloud
		result = partialClone(rc);

		//don't need this one anymore
		delete rc;
		rc = nullptr;
	}

	if (!result)
	{
		ccLog::Warning("[ccPointCloud] Failed to generate a subset cloud");
		return nullptr;
	}

	QSettings settings;
	settings.beginGroup("SegmentationToolOptions");
	QString segmentedSuffix = settings.value("Segmented", ".segmented").toString();
	settings.endGroup();

	result->setName(getName() + segmentedSuffix);

	//shall the visible points be erased from this cloud?
	if (removeSelectedPoints && !isLocked())
	{
		//we drop the octree before modifying this cloud's contents
		deleteOctree();
		clearLOD();

		unsigned count = size();

		//we have to take care of scan grids first
		{
			//we need a map between old and new indexes
			std::vector<int> newIndexMap(size(), -1);
			{
				unsigned newIndex = 0;
				for (unsigned i = 0; i < count; ++i)
				{
					if (m_pointsVisibility[i] != CCCoreLib::POINT_VISIBLE)
					{
						newIndexMap[i] = newIndex++;
					}
				}
			}

			//then update the indexes
			UpdateGridIndexes(newIndexMap, m_grids);

			//and reset the invalid (empty) ones
			//(DGM: we don't erase them as they may still be useful?)
			for (Grid::Shared &grid : m_grids)
			{
				if (grid->validCount == 0)
				{
					grid->indexes.resize(0);
				}
			}
		}

		//we remove all visible points
		unsigned lastPoint = 0;
		for (unsigned i = 0; i < count; ++i)
		{
			if (m_pointsVisibility[i] != CCCoreLib::POINT_VISIBLE)
			{
				if (i != lastPoint)
				{
					swapPoints(lastPoint, i);
				}
				++lastPoint;
			}
		}

		unallocateVisibilityArray();

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
	BaseClass::deleteScalarField(index);

	//current SF should still be up-to-date!
	if (m_currentInScalarFieldIndex < 0 && getNumberOfScalarFields() > 0)
		setCurrentInScalarField(static_cast<int>(getNumberOfScalarFields() - 1));

	setCurrentDisplayedScalarField(m_currentInScalarFieldIndex);
	showSF(m_currentInScalarFieldIndex >= 0);
}

void ccPointCloud::deleteAllScalarFields()
{
	//the father does all the work
	BaseClass::deleteAllScalarFields();

	//update the currently displayed SF
	setCurrentDisplayedScalarField(-1);
	showSF(false);
}

bool ccPointCloud::convertCurrentScalarFieldToColors(bool mixWithExistingColor/*=false*/)
{
	if (!hasDisplayedScalarField())
	{
		ccLog::Warning("[ccPointCloud::setColorWithCurrentScalarField] No active scalar field or color scale!");
		return false;
	}

	unsigned count = size();

	if (!mixWithExistingColor || !hasColors())
	{
		if (!hasColors() && !resizeTheRGBTable(false))
		{
			return false;
		}

		for (unsigned i = 0; i < count; i++)
		{
			const ccColor::Rgb* col = getPointScalarValueColor(i);
			setPointColor(i, col ? *col : ccColor::blackRGB);
		}
	}
	else //mix with existing colors
	{
		for (unsigned i = 0; i < count; i++)
		{
			const ccColor::Rgb* col = getPointScalarValueColor(i);
			if (col)
			{
				ccColor::Rgba& _color = m_rgbaColors->at(i);
				_color.r = static_cast<ColorCompType>(_color.r * (static_cast<float>(col->r) / ccColor::MAX));
				_color.g = static_cast<ColorCompType>(_color.g * (static_cast<float>(col->g) / ccColor::MAX));
				_color.b = static_cast<ColorCompType>(_color.b * (static_cast<float>(col->b) / ccColor::MAX));
			}
		}
	}

	//We must update the VBOs
	colorsHaveChanged();

	return true;
}

QSharedPointer<CCCoreLib::ReferenceCloud> ccPointCloud::computeCPSet(	ccGenericPointCloud& otherCloud,
																	CCCoreLib::GenericProgressCallback* progressCb/*=nullptr*/,
																	unsigned char octreeLevel/*=0*/)
{
	int result = 0;
	QSharedPointer<CCCoreLib::ReferenceCloud> CPSet;
	CPSet.reset(new CCCoreLib::ReferenceCloud(&otherCloud));

	CCCoreLib::DistanceComputationTools::Cloud2CloudDistancesComputationParams params;
	{
		params.CPSet = CPSet.data();
		params.octreeLevel = octreeLevel;
	}

	//create temporary SF for the nearest neighbors determination (computeCloud2CloudDistances)
	//so that we can properly remove it afterwards!
	static const char s_defaultTempSFName[] = "CPSetComputationTempSF";
	int sfIdx = getScalarFieldIndexByName(s_defaultTempSFName);
	if (sfIdx < 0)
		sfIdx = addScalarField(s_defaultTempSFName);
	if (sfIdx < 0)
	{
		ccLog::Warning("[ccPointCloud::ComputeCPSet] Not enough memory!");
		return QSharedPointer<CCCoreLib::ReferenceCloud>(nullptr);
	}

	int currentInSFIndex = m_currentInScalarFieldIndex;
	int currentOutSFIndex = m_currentOutScalarFieldIndex;
	setCurrentScalarField(sfIdx);

	result = CCCoreLib::DistanceComputationTools::computeCloud2CloudDistances(this, &otherCloud, params, progressCb);

	//restore previous parameters
	setCurrentInScalarField(currentInSFIndex);
	setCurrentOutScalarField(currentOutSFIndex);
	deleteScalarField(sfIdx);

	if (result < 0)
	{
		ccLog::Warning("[ccPointCloud::ComputeCPSet] Closest-point set computation failed!");
		CPSet.clear();
	}

	return CPSet;
}

bool ccPointCloud::interpolateColorsFrom(	ccGenericPointCloud* otherCloud,
											CCCoreLib::GenericProgressCallback* progressCb/*=nullptr*/,
											unsigned char octreeLevel/*=0*/)
{
	if (!otherCloud || otherCloud->size() == 0)
	{
		ccLog::Warning("[ccPointCloud::interpolateColorsFrom] Invalid/empty input cloud!");
		return false;
	}

	//check that both bounding boxes intersect!
	ccBBox box = getOwnBB();
	ccBBox otherBox = otherCloud->getOwnBB();

	CCVector3 dimSum = box.getDiagVec() + otherBox.getDiagVec();
	CCVector3 dist = box.getCenter() - otherBox.getCenter();
	if (	std::abs(dist.x) > dimSum.x / 2
		||	std::abs(dist.y) > dimSum.y / 2
		||	std::abs(dist.z) > dimSum.z / 2)
	{
		ccLog::Warning("[ccPointCloud::interpolateColorsFrom] Clouds are too far from each other! Can't proceed.");
		return false;
	}

	//compute the closest-point set of 'this cloud' relatively to 'input cloud'
	//(to get a mapping between the resulting vertices and the input points)
	QSharedPointer<CCCoreLib::ReferenceCloud> CPSet = computeCPSet(*otherCloud, progressCb, octreeLevel);
	if (!CPSet)
	{
		return false;
	}

	if (!resizeTheRGBTable(false))
	{
		ccLog::Warning("[ccPointCloud::interpolateColorsFrom] Not enough memory!");
		return false;
	}

	//import colors
	unsigned CPSetSize = CPSet->size();
	assert(CPSetSize == size());
	for (unsigned i = 0; i < CPSetSize; ++i)
	{
		unsigned index = CPSet->getPointGlobalIndex(i);
		setPointColor(i, otherCloud->getPointColor(index));
	}

	//We must update the VBOs
	colorsHaveChanged();

	return true;
}

static void ProjectOnCylinder(	const CCVector3& AP,
								const Tuple3ub& dim,
								PointCoordinateType radius,
								PointCoordinateType& delta,
								PointCoordinateType& phi_rad)
{
	//2D distance to the center (XY plane)
	PointCoordinateType APnorm_XY = sqrt(AP.u[dim.x] * AP.u[dim.x] + AP.u[dim.y] * AP.u[dim.y]);
	//longitude (0 = +X = east)
	phi_rad = atan2(AP.u[dim.y], AP.u[dim.x]);
	//deviation
	delta = APnorm_XY - radius;
}

static void ProjectOnCone(	const CCVector3& AP,
							PointCoordinateType alpha_rad,
							const Tuple3ub& dim,
							PointCoordinateType& s,
							PointCoordinateType& delta,
							PointCoordinateType& phi_rad)
{
	//3D distance to the apex
	PointCoordinateType normAP = AP.norm();
	//2D distance to the apex (XY plane)
	PointCoordinateType normAP_XY = sqrt(AP.u[dim.x] * AP.u[dim.x] + AP.u[dim.y] * AP.u[dim.y]);

	//angle between +Z and AP
	PointCoordinateType beta_rad = atan2(normAP_XY, -AP.u[dim.z]);
	//angular deviation
	PointCoordinateType gamma_rad = beta_rad - alpha_rad; //if gamma_rad > 0, the point is outside the cone

	//projection on the cone
	{
		//longitude (0 = +X = east)
		phi_rad = atan2(AP.u[dim.y], AP.u[dim.x]);
		//curvilinear distance from the Apex
		s = normAP * cos(gamma_rad);
		//(normal) deviation
		delta = normAP * sin(gamma_rad);
	}
}

ccPointCloud* ccPointCloud::unroll(	UnrollMode mode,
									UnrollBaseParams* params,
									bool exportDeviationSF/*=false*/,
									double startAngle_deg/*=0.0*/,
									double stopAngle_deg/*=360.0*/,
									CCCoreLib::GenericProgressCallback* progressCb/*=nullptr*/) const
{
	if (	!params
		||	params->axisDim > 2
		||	startAngle_deg >= stopAngle_deg)
	{
		//invalid input parameters
		assert(false);
		return nullptr;
	}

	QString modeStr;
	UnrollCylinderParams* cylParams = nullptr;
	UnrollConeParams* coneParams = nullptr;

	switch (mode)
	{
	case CYLINDER:
		modeStr = "Cylinder";
		cylParams = static_cast<UnrollCylinderParams*>(params);
		break;
	case CONE:
		modeStr = "Cone";
		coneParams = static_cast<UnrollConeParams*>(params);
		break;
	case STRAIGHTENED_CONE:
	case STRAIGHTENED_CONE2:
		modeStr = "Straightened cone";
		coneParams = static_cast<UnrollConeParams*>(params);
		break;
	default:
		assert(false);
		return nullptr;
	}

	Tuple3ub dim;
	dim.z = params->axisDim;
	dim.x = (dim.z < 2 ? dim.z + 1 : 0);
	dim.y = (dim.x < 2 ? dim.x + 1 : 0);

	unsigned numberOfPoints = size();
	CCCoreLib::NormalizedProgress nprogress(progressCb, numberOfPoints);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle(qPrintable(QString("Unroll (%1)").arg(modeStr)));
			progressCb->setInfo(qPrintable(QString("Number of points = %1").arg(numberOfPoints)));
		}
		progressCb->update(0);
		progressCb->start();
	}

	CCCoreLib::ReferenceCloud duplicatedPoints(const_cast<ccPointCloud*>(this));
	std::vector<CCVector3> unrolledPoints;
	{
		//compute an estimate of the final point count
		unsigned newSize = static_cast<unsigned>(std::ceil((stopAngle_deg - startAngle_deg) / 360.0 * size()));
		if (!duplicatedPoints.reserve(newSize))
		{
			ccLog::Error("Not enough memory");
			return nullptr;
		}

		try
		{
			unrolledPoints.reserve(newSize);
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Error("Not enough memory");
			return nullptr;
		}
	}

	std::vector<ScalarType> deviationValues;
	if (exportDeviationSF)
	try
	{
		deviationValues.resize(size());
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return nullptr;
	}

	std::vector<CCVector3> unrolledNormals;
	bool withNormals = hasNormals();
	if (withNormals)
	{
		//for normals, we can simply store at most one unrolled normal per original one
		//same thing for deviation values
		try
		{
			unrolledNormals.resize(size());
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			return nullptr;
		}
	}
	
	double startAngle_rad = CCCoreLib::DegreesToRadians( startAngle_deg );
	double stopAngle_rad = CCCoreLib::DegreesToRadians( stopAngle_deg );

	PointCoordinateType alpha_rad = 0;
	PointCoordinateType sin_alpha = 0;
	if (mode != CYLINDER)
	{
		alpha_rad = CCCoreLib::DegreesToRadians( coneParams->coneAngle_deg );
		sin_alpha = static_cast<PointCoordinateType>(sin(alpha_rad));
	}

	for (unsigned i = 0; i < numberOfPoints; i++)
	{
		const CCVector3* Pin = getPoint(i);

		//we project the point
		CCVector3 AP;
		CCVector3 Pout;
		PointCoordinateType longitude_rad = 0; //longitude (rad)
		PointCoordinateType delta = 0; //distance to the cone/cylinder surface
		PointCoordinateType coneAbscissa = 0;

		switch (mode)
		{
		case CYLINDER:
		{
			AP = *Pin - cylParams->center;
			ProjectOnCylinder(AP, dim, params->radius, delta, longitude_rad);

			//we project the point
			//Pout.u[dim.x] = longitude_rad * radius;
			Pout.u[dim.y] = -delta;
			Pout.u[dim.z] = Pin->u[dim.z];
		}
		break;

		case STRAIGHTENED_CONE:
		{
			AP = *Pin - coneParams->apex;
			ProjectOnCone(AP, alpha_rad, dim, coneAbscissa, delta, longitude_rad);
			//we simply develop the cone as a cylinder
			//Pout.u[dim.x] = phi_rad * params->radius;
			Pout.u[dim.y] = -delta;
			//Pout.u[dim.z] = Pin->u[dim.z];
			Pout.u[dim.z] = coneParams->apex.u[dim.z] - coneAbscissa;
		}
		break;

		case STRAIGHTENED_CONE2:
		{
			AP = *Pin - coneParams->apex;
			ProjectOnCone(AP, alpha_rad, dim, coneAbscissa, delta, longitude_rad);
			//we simply develop the cone as a cylinder
			//Pout.u[dim.x] = phi_rad * coneAbscissa * sin_alpha;
			Pout.u[dim.y] = -delta;
			//Pout.u[dim.z] = Pin->u[dim.z];
			Pout.u[dim.z] = coneParams->apex.u[dim.z] - coneAbscissa;
		}
		break;

		case CONE:
		{
			AP = *Pin - coneParams->apex;
			ProjectOnCone(AP, alpha_rad, dim, coneAbscissa, delta, longitude_rad);
			//unrolling
			PointCoordinateType theta_rad = longitude_rad * sin_alpha; //sin_alpha is a bit arbitrary here. The aim is mostly to reduce the angular range
			//project the point
			Pout.u[dim.y] = -coneAbscissa * cos(theta_rad);
			Pout.u[dim.x] =  coneAbscissa * sin(theta_rad);
			Pout.u[dim.z] = delta;
		}
		break;

		default:
			assert(false);
		}
		
		// first unroll its normal if necessary
		if (withNormals)
		{
			const CCVector3& N = getPointNormal(i);
			CCVector3 AP2 = AP + N;
			CCVector3 N2;

			switch (mode)
			{
			case CYLINDER:
			{
				PointCoordinateType delta2;
				PointCoordinateType longitude2_rad;
				ProjectOnCylinder(AP2, dim, params->radius, delta2, longitude2_rad);

				N2.u[dim.x] = static_cast<PointCoordinateType>((longitude2_rad - longitude_rad) * params->radius);
				N2.u[dim.y] = -(delta2 - delta);
				N2.u[dim.z] = N.u[dim.z];
			}
			break;

			case STRAIGHTENED_CONE:
			{
				PointCoordinateType coneAbscissa2;
				PointCoordinateType delta2;
				PointCoordinateType longitude2_rad;
				ProjectOnCone(AP2, alpha_rad, dim, coneAbscissa2, delta2, longitude2_rad);
				//we simply develop the cone as a cylinder
				N2.u[dim.x] = static_cast<PointCoordinateType>((longitude2_rad - longitude_rad) * params->radius);
				N2.u[dim.y] = -(delta2 - delta);
				N2.u[dim.z] = coneAbscissa - coneAbscissa2;
			}
			break;

			case STRAIGHTENED_CONE2:
			{
				PointCoordinateType coneAbscissa2;
				PointCoordinateType delta2;
				PointCoordinateType longitude2_rad;
				ProjectOnCone(AP2, alpha_rad, dim, coneAbscissa2, delta2, longitude2_rad);
				//we simply develop the cone as a cylinder
				N2.u[dim.x] = static_cast<PointCoordinateType>((longitude2_rad * coneAbscissa - longitude_rad * coneAbscissa2) * sin_alpha);
				N2.u[dim.y] = -(delta2 - delta);
				N2.u[dim.z] = coneAbscissa - coneAbscissa2;
			}
			break;

			case CONE:
			{
				PointCoordinateType coneAbscissa2;
				PointCoordinateType delta2;
				PointCoordinateType longitude2_rad;
				ProjectOnCone(AP2, alpha_rad, dim, coneAbscissa2, delta2, longitude2_rad);
				//unrolling
				PointCoordinateType theta2_rad = longitude2_rad * sin_alpha; //sin_alpha is a bit arbitrary here. The aim is mostly to reduce the angular range
				//project the point
				CCVector3 P2out;
				P2out.u[dim.x] =  coneAbscissa2 * sin(theta2_rad);
				P2out.u[dim.y] = -coneAbscissa2 * cos(theta2_rad);
				P2out.u[dim.z] = delta2;
				N2 = P2out - Pout;
			}
			break;

			default:
				assert(false);
				break;
			}

			N2.normalize();
			unrolledNormals[i] = N2;
		}

		//then compute the deviation (if necessary)
		if (exportDeviationSF)
		{
			deviationValues[i] = static_cast<ScalarType>(delta);
		}

		//then repeat the unrolling process for the coordinates
		//1) position the 'point' at the beginning of the angular range
		double dLongitude_rad = longitude_rad;
		while (dLongitude_rad >= startAngle_rad)
		{
			dLongitude_rad -= 2 * M_PI;
		}
		dLongitude_rad += 2 * M_PI;

		//2) repeat the unrolling process
		for (; dLongitude_rad < stopAngle_rad; dLongitude_rad += 2 * M_PI)
		{
			//do we need to reserve more memory?
			if (duplicatedPoints.size() == duplicatedPoints.capacity())
			{
				unsigned newSize = duplicatedPoints.size() + (1 << 20);
				if (!duplicatedPoints.reserve(newSize))
				{
					ccLog::Error("Not enough memory");
					return nullptr;
				}

				try
				{
					unrolledPoints.reserve(newSize);
				}
				catch (const std::bad_alloc&)
				{
					ccLog::Error("Not enough memory");
					return nullptr;
				}
			}

			//add the point
			switch (mode)
			{
			case CYLINDER:
			case STRAIGHTENED_CONE:
				Pout.u[dim.x] = dLongitude_rad * params->radius;
				break;
			case STRAIGHTENED_CONE2:
				Pout.u[dim.x] = dLongitude_rad * coneAbscissa * sin_alpha;
				break;

			case CONE:
				Pout.u[dim.x] =  coneAbscissa * sin(dLongitude_rad);
				Pout.u[dim.y] = -coneAbscissa * cos(dLongitude_rad);
				//Pout = coneParams->apex + Pout; //nope, this projection is arbitrary and should be centered on (0, 0, 0)
				break;

			default:
				assert(false);
			}

			unrolledPoints.push_back(Pout);
			duplicatedPoints.addPointIndex(i);
		}

		//process canceled by user?
		if (progressCb && !nprogress.oneStep())
		{
			ccLog::Warning("Process cancelled by user");
			return nullptr;
		}
	}

	if (progressCb)
	{
		progressCb->stop();
	}

	//now create the real cloud
	ccPointCloud* clone = partialClone(&duplicatedPoints);
	if (clone)
	{
		CCCoreLib::ScalarField* deviationSF = nullptr;
		if (exportDeviationSF)
		{
			int sfIdx = clone->getScalarFieldIndexByName(s_deviationSFName);
			if (sfIdx < 0)
			{
				sfIdx = clone->addScalarField(s_deviationSFName);
				if (sfIdx < 0)
				{
					ccLog::Warning("[unrollOnCylinder] Not enough memory to init the deviation scalar field");
				}
			}
			if (sfIdx >= 0)
			{
				deviationSF = clone->getScalarField(sfIdx);
				clone->setCurrentDisplayedScalarField(sfIdx);
				clone->showSF(true);
			}
		}

		//update the coordinates, the normals and the deviation SF
		for (unsigned i = 0; i < duplicatedPoints.size(); ++i)
		{
			CCVector3* P = clone->point(i);
			*P = unrolledPoints[i];

			unsigned globalIndex = duplicatedPoints.getPointGlobalIndex(i);
			if (withNormals)
			{
				clone->setPointNormal(i, unrolledNormals[globalIndex]);
			}
			if (deviationSF)
			{
				deviationSF->setValue(i, deviationValues[globalIndex]);
			}
		}

		if (deviationSF)
		{
			deviationSF->computeMinAndMax();
		}

		clone->setName(getName() + ".unrolled");
		clone->refreshBB(); //calls notifyGeometryUpdate + releaseVBOs
	}

	return clone;
}


//void ccPointCloud::unrollOnCone(PointCoordinateType baseRadius,
//	double alpha_deg,
//	const CCVector3& apex,
//	unsigned char Z/*=2*/,
//	CCCoreLib::GenericProgressCallback* progressCb/*=nullptr*/)
//{
//	assert(Z < 3);
//	unsigned char X = (Z < 2 ? Z + 1 : 0);
//	unsigned char Y = (X < 2 ? X + 1 : 0);
//
//	unsigned numberOfPoints = size();
//
//	CCCoreLib::NormalizedProgress nprogress(progressCb, numberOfPoints);
//	if (progressCb)
//	{
//		if (progressCb->textCanBeEdited())
//		{
//			progressCb->setMethodTitle("Unroll (cone)");
//			progressCb->setInfo(qPrintable(QString("Number of points = %1").arg(numberOfPoints)));
//		}
//		progressCb->update(0);
//		progressCb->start();
//	}
//
//	PointCoordinateType tan_alpha = static_cast<PointCoordinateType>(tan(CCCoreLib::DegreesToRadians(alpha_deg)));
//	PointCoordinateType cos_alpha = static_cast<PointCoordinateType>(cos(CCCoreLib::DegreesToRadians(alpha_deg)));
//	PointCoordinateType sin_alpha = static_cast<PointCoordinateType>(sin(CCCoreLib::DegreesToRadians(alpha_deg)));
//
//	for (unsigned i = 0; i < numberOfPoints; i++)
//	{
//		CCVector3* P = point(i);
//		PointCoordinateType P0 = P->u[X] - apex.u[X];
//		PointCoordinateType P1 = P->u[Y] - apex.u[Y];
//		PointCoordinateType P2 = P->u[Z] - apex.u[Z];
//
//		PointCoordinateType u = sqrt(P0 * P0 + P1 * P1);
//		PointCoordinateType lon = atan2(P0, P1);
//
//		//projection on the cone
//		PointCoordinateType radialDist = (u + P2*tan_alpha);
//		PointCoordinateType orthoDist = radialDist * cos_alpha;
//		PointCoordinateType z2 = P2 - orthoDist*sin_alpha; //(P2 + u * tan_alpha) * q;
//
//		//we project point
//		P->u[X] = lon*baseRadius;
//		P->u[Y] = orthoDist;
//		P->u[Z] = z2 / cos_alpha + apex.u[Z];
//
//		//and its normal if necessary
//		if (hasNormals())
//		{
//			const CCVector3& N = ccNormalVectors::GetNormal(m_normals->getValue(i));
//
//			PointCoordinateType dX = cos(lon)*N.u[X] - sin(lon)*N.u[Y];
//			PointCoordinateType dZ = sin(lon)*N.u[X] + cos(lon)*N.u[Y];
//
//			CCVector3 n2;
//			n2.u[X] = dX;
//			n2.u[Y] = cos_alpha*dZ - sin_alpha*N.u[Z];
//			n2.u[Z] = sin_alpha*dZ + cos_alpha*N.u[Z];
//			n2.normalize();
//
//			setPointNormal(i, n2);
//		}
//
//		//process canceled by user?
//		if (progressCb && !nprogress.oneStep())
//		{
//			break;
//		}
//	}
//
//	refreshBB(); //calls notifyGeometryUpdate + releaseVBOs
//}

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
	if (sf->size() < m_points.size())
	{
		if (!sf->resizeSafe(m_points.size()))
		{
			ccLog::Warning("[ccPointCloud::addScalarField] Not enough memory!");
			return -1;
		}
	}
	if (sf->capacity() < m_points.capacity()) //yes, it happens ;)
	{
		if (!sf->reserveSafe(m_points.capacity()))
		{
			ccLog::Warning("[ccPointCloud::addScalarField] Not enough memory!");
			return -1;
		}
	}

	try
	{
		m_scalarFields.push_back(sf);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccPointCloud::addScalarField] Not enough memory!");
		return -1;
	}

	sf->link();

	return static_cast<int>(m_scalarFields.size()) - 1;
}

bool ccPointCloud::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPointCloud::toFile_MeOnly(out))
		return false;

	//points array (dataVersion>=20)
	if (!ccSerializationHelper::GenericArrayToFile<CCVector3, 3, PointCoordinateType>(m_points, out))
		return false;

	//colors array (dataVersion>=20)
	{
		bool hasColorsArray = hasColors();
		if (out.write((const char*)&hasColorsArray, sizeof(bool)) < 0)
			return WriteError();
		if (hasColorsArray)
		{
			assert(m_rgbaColors);
			if (!m_rgbaColors->toFile(out))
				return false;
		}
	}

	//normals array (dataVersion>=20)
	{
		bool hasNormalsArray = hasNormals();
		if (out.write((const char*)&hasNormalsArray, sizeof(bool)) < 0)
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
		uint32_t sfCount = static_cast<uint32_t>(getNumberOfScalarFields());
		if (out.write((const char*)&sfCount, 4) < 0)
			return WriteError();

		//scalar fields (dataVersion>=20)
		for (uint32_t i = 0; i < sfCount; ++i)
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
		if (out.write((const char*)&m_sfColorScaleDisplayed, sizeof(bool)) < 0)
			return WriteError();

		//Displayed scalar field index (dataVersion>=20)
		int32_t displayedScalarFieldIndex = (int32_t)m_currentDisplayedScalarFieldIndex;
		if (out.write((const char*)&displayedScalarFieldIndex, 4) < 0)
			return WriteError();
	}

	//grid structures (dataVersion>=41)
	{
		//number of grids
		uint32_t count = static_cast<uint32_t>(this->gridCount());
		if (out.write((const char*)&count, 4) < 0)
			return WriteError();

		//save each grid
		for (uint32_t i = 0; i < count; ++i)
		{
			const Grid::Shared& g = grid(static_cast<unsigned>(i));
			if (!g || g->indexes.empty())
				continue;

			//width
			uint32_t w = static_cast<uint32_t>(g->w);
			if (out.write((const char*)&w, 4) < 0)
				return WriteError();
			//height
			uint32_t h = static_cast<uint32_t>(g->h);
			if (out.write((const char*)&h, 4) < 0)
				return WriteError();

			//sensor matrix
			if (!g->sensorPosition.toFile(out))
				return WriteError();

			//indexes
			int* _index = g->indexes.data();
			for (uint32_t j = 0; j < w*h; ++j, ++_index)
			{
				int32_t index = static_cast<int32_t>(*_index);
				if (out.write((const char*)&index, 4) < 0)
					return WriteError();
			}

			//grid colors
			bool hasColors = (g->colors.size() == g->indexes.size());
			if (out.write((const char*)&hasColors, 1) < 0)
				return WriteError();

			if (hasColors)
			{
				for (uint32_t j = 0; j < g->colors.size(); ++j)
				{
					if (out.write((const char*)g->colors[j].rgb, 3) < 0)
						return WriteError();
				}
			}
		}
	}

	//Waveforms (dataVersion >= 44)
	bool withFWF = hasFWF();
	if (out.write((const char*)&withFWF, sizeof(bool)) < 0)
	{
		return WriteError();
	}
	if (withFWF)
	{
		//first save the descriptors
		uint32_t descriptorCount = static_cast<uint32_t>(m_fwfDescriptors.size());
		if (out.write((const char*)&descriptorCount, 4) < 0)
		{
			return WriteError();
		}
		for (auto it = m_fwfDescriptors.begin(); it != m_fwfDescriptors.end(); ++it)
		{
			//write the key (descriptor ID)
			if (out.write((const char*)&it.key(), 1) < 0)
			{
				return WriteError();
			}
			//write the descriptor
			if (!it.value().toFile(out))
			{
				return WriteError();
			}
		}

		//then the waveforms
		uint32_t waveformCount = static_cast<uint32_t>(m_fwfWaveforms.size());
		if (out.write((const char*)&waveformCount, 4) < 0)
		{
			return WriteError();
		}
		for (const ccWaveform& w : m_fwfWaveforms)
		{
			if (!w.toFile(out))
			{
				return WriteError();
			}
		}

		//eventually save the data
		uint64_t dataSize = static_cast<uint64_t>(m_fwfData ? m_fwfData->size(): 0);
		if (out.write((const char*)&dataSize, 8) < 0)
		{
			return WriteError();
		}
		if (m_fwfData && out.write((const char*)m_fwfData->data(), dataSize) < 0)
		{
			return WriteError();
		}
	}

	return true;
}

bool ccPointCloud::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGenericPointCloud::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//points array (dataVersion>=20)
	{
		bool result = false;
		bool fileCoordIsDouble = (flags & ccSerializableObject::DF_POINT_COORDS_64_BITS);
		if (!fileCoordIsDouble && sizeof(PointCoordinateType) == 8) //file is 'float' and current type is 'double'
		{
			result = ccSerializationHelper::GenericArrayFromTypedFile<CCVector3, 3, PointCoordinateType, float>(m_points, in, dataVersion);
		}
		else if (fileCoordIsDouble && sizeof(PointCoordinateType) == 4) //file is 'double' and current type is 'float'
		{
			result = ccSerializationHelper::GenericArrayFromTypedFile<CCVector3, 3, PointCoordinateType, double>(m_points, in, dataVersion);
		}
		else
		{
			result = ccSerializationHelper::GenericArrayFromFile<CCVector3, 3, PointCoordinateType>(m_points, in, dataVersion);
		}
		if (!result)
		{
			return false;
		}

#ifdef QT_DEBUG
		//test: look for NaN values
		{
			unsigned nanPointsCount = 0;
			for (unsigned i = 0; i < size(); ++i)
			{
				if (	point(i)->x != point(i)->x
					||	point(i)->y != point(i)->y
					||	point(i)->z != point(i)->z )
				{
					*point(i) = CCVector3(0, 0, 0);
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
		if (in.read((char*)&hasColorsArray, sizeof(bool)) < 0)
			return ReadError();
		if (hasColorsArray)
		{
			if (!m_rgbaColors)
			{
				m_rgbaColors = new RGBAColorsTableType;
				m_rgbaColors->link();
			}
			CC_CLASS_ENUM classID = ReadClassIDFromFile(in, dataVersion);
			if (classID == CC_TYPES::RGB_COLOR_ARRAY)
			{
				QSharedPointer<ColorsTableType> oldRGBColors(new ColorsTableType);
				if (!oldRGBColors->fromFile(in, dataVersion, flags, oldToNewIDMap))
				{
					return false;
				}

				size_t count = oldRGBColors->size();
				if (!m_rgbaColors->reserveSafe(count))
				{
					unallocateColors();
					return MemoryError();
				}

				for (size_t i = 0; i < count; ++i)
				{
					m_rgbaColors->addElement(ccColor::Rgba(oldRGBColors->getValue(i), ccColor::MAX));
				}
			}
			else if (classID == CC_TYPES::RGBA_COLOR_ARRAY)
			{
				if (!m_rgbaColors->fromFile(in, dataVersion, flags, oldToNewIDMap))
				{
					unallocateColors();
					return false;
				}
			}
			else
			{
				return CorruptError();
			}
		}
	}

	//normals array (dataVersion>=20)
	{
		bool hasNormalsArray = false;
		if (in.read((char*)&hasNormalsArray, sizeof(bool)) < 0)
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
			if (!m_normals->fromFile(in, dataVersion, flags, oldToNewIDMap))
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
		if (in.read((char*)&sfCount, 4) < 0)
			return ReadError();

		//scalar fields (dataVersion>=20)
		for (uint32_t i = 0; i < sfCount; ++i)
		{
			ccScalarField* sf = new ccScalarField();
			if (!sf->fromFile(in, dataVersion, flags, oldToNewIDMap))
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
			if (in.read((char*)&greyForNanScalarValues, sizeof(bool)) < 0)
				return ReadError();

			//update all scalar fields accordingly (old way)
			for (unsigned i = 0; i < getNumberOfScalarFields(); ++i)
			{
				static_cast<ccScalarField*>(getScalarField(i))->showNaNValuesInGrey(greyForNanScalarValues);
			}
		}

		//'show current sf color scale' state (dataVersion>=20)
		if (in.read((char*)&m_sfColorScaleDisplayed, sizeof(bool)) < 0)
			return ReadError();

		//Displayed scalar field index (dataVersion>=20)
		int32_t displayedScalarFieldIndex = 0;
		if (in.read((char*)&displayedScalarFieldIndex, 4) < 0)
			return ReadError();
		if (displayedScalarFieldIndex < (int32_t)sfCount)
			setCurrentDisplayedScalarField(displayedScalarFieldIndex);
	}

	//grid structures (dataVersion>=41)
	if (dataVersion >= 41)
	{
		//number of grids
		uint32_t count = 0;
		if (in.read((char*)&count, 4) < 0)
			return ReadError();

		//load each grid
		for (uint32_t i = 0; i < count; ++i)
		{
			Grid::Shared g(new Grid);

			//width
			uint32_t w = 0;
			if (in.read((char*)&w, 4) < 0)
				return ReadError();
			//height
			uint32_t h = 0;
			if (in.read((char*)&h, 4) < 0)
				return ReadError();

			g->w = static_cast<unsigned>(w);
			g->h = static_cast<unsigned>(h);

			//sensor matrix
			if (!g->sensorPosition.fromFile(in, dataVersion, flags, oldToNewIDMap))
				return WriteError();

			try
			{
				g->indexes.resize(w*h);
			}
			catch (const std::bad_alloc&)
			{
				return MemoryError();
			}

			//indexes
			int* _index = g->indexes.data();
			for (uint32_t j = 0; j < w*h; ++j, ++_index)
			{
				int32_t index = 0;
				if (in.read((char*)&index, 4) < 0)
					return ReadError();

				*_index = index;
				if (index >= 0)
				{
					//update min, max and count for valid indexes
					if (g->validCount)
					{
						g->minValidIndex = std::min(static_cast<unsigned>(index), g->minValidIndex);
						g->maxValidIndex = std::max(static_cast<unsigned>(index), g->maxValidIndex);
					}
					else
					{
						g->minValidIndex = g->maxValidIndex = index;
					}
					++g->validCount;
				}
			}

			//grid colors
			bool hasColors = false;
			if (in.read((char*)&hasColors, 1) < 0)
				return ReadError();

			if (hasColors)
			{
				try
				{
					g->colors.resize(g->indexes.size());
				}
				catch (const std::bad_alloc&)
				{
					return MemoryError();
				}

				for (uint32_t j = 0; j < g->colors.size(); ++j)
				{
					if (in.read((char*)g->colors[j].rgb, 3) < 0)
						return ReadError();
				}
			}

			addGrid(g);
		}
	}

	//Waveforms (dataVersion >= 44)
	if (dataVersion >= 44)
	{
		bool withFWF = false;
		if (in.read((char*)&withFWF, sizeof(bool)) < 0)
		{
			return ReadError();
		}
		if (withFWF)
		{
			//first read the descriptors
			uint32_t descriptorCount = 0;
			if (in.read((char*)&descriptorCount, 4) < 0)
			{
				return ReadError();
			}
			for (uint32_t i = 0; i < descriptorCount; ++i)
			{
				//read the descriptor ID
				uint8_t id = 0;
				if (in.read((char*)&id, 1) < 0)
				{
					return ReadError();
				}
				//read the descriptor
				WaveformDescriptor d;
				if (!d.fromFile(in, dataVersion, flags, oldToNewIDMap))
				{
					return ReadError();
				}
				//add the descriptor to the set
				m_fwfDescriptors.insert(id, d);
			}

			//then the waveforms
			uint32_t waveformCount = 0;
			if (in.read((char*)&waveformCount, 4) < 0)
			{
				return ReadError();
			}
			assert(waveformCount >= size());
			try
			{
				m_fwfWaveforms.resize(waveformCount);
			}
			catch (const std::bad_alloc&)
			{
				return MemoryError();
			}
			for (uint32_t i = 0; i < waveformCount; ++i)
			{
				if (!m_fwfWaveforms[i].fromFile(in, dataVersion, flags, oldToNewIDMap))
				{
					return ReadError();
				}
			}

			//eventually save the data
			uint64_t dataSize = 0;
			if (in.read((char*)&dataSize, 8) < 0)
			{
				return ReadError();
			}
			if (dataSize != 0)
			{
				FWFDataContainer* container = new FWFDataContainer;
				try
				{
					container->resize(dataSize);
				}
				catch (const std::bad_alloc&)
				{
					return MemoryError();
				}
				m_fwfData = SharedFWFDataContainer(container);

				if (in.read((char*)m_fwfData->data(), dataSize) < 0)
				{
					return ReadError();
				}
			}
		}
	}

	//notifyGeometryUpdate(); //FIXME: we can't call it now as the dependent 'pointers' are not valid yet!

	//We should update the VBOs (just in case)
	releaseVBOs();

	return true;
}

CCCoreLib::ReferenceCloud* ccPointCloud::crop(const ccBBox& box, bool inside/*=true*/)
{
	if (!box.isValid())
	{
		ccLog::Warning("[ccPointCloud::crop] Invalid bounding-box");
		return nullptr;
	}

	unsigned count = size();
	if (count == 0)
	{
		ccLog::Warning("[ccPointCloud::crop] Cloud is empty!");
		return nullptr;
	}

	CCCoreLib::ReferenceCloud* ref = new CCCoreLib::ReferenceCloud(this);
	if (!ref->reserve(count))
	{
		ccLog::Warning("[ccPointCloud::crop] Not enough memory!");
		delete ref;
		return nullptr;
	}

	for (unsigned i = 0; i < count; ++i)
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

CCCoreLib::ReferenceCloud* ccPointCloud::crop2D(const ccPolyline* poly, unsigned char orthoDim, bool inside/*=true*/)
{
	if (!poly)
	{
		ccLog::Warning("[ccPointCloud::crop2D] Invalid input polyline");
		return nullptr;
	}
	if (orthoDim > 2)
	{
		ccLog::Warning("[ccPointCloud::crop2D] Invalid input polyline");
		return nullptr;
	}

	unsigned count = size();
	if (count == 0)
	{
		ccLog::Warning("[ccPointCloud::crop] Cloud is empty!");
		return nullptr;
	}

	CCCoreLib::ReferenceCloud* ref = new CCCoreLib::ReferenceCloud(this);
	if (!ref->reserve(count))
	{
		ccLog::Warning("[ccPointCloud::crop] Not enough memory!");
		delete ref;
		return nullptr;
	}

	unsigned char X = ((orthoDim+1) % 3);
	unsigned char Y = ((X+1) % 3);

	for (unsigned i=0; i<count; ++i)
	{
		const CCVector3* P = point(i);

		CCVector2 P2D( P->u[X], P->u[Y] );
		bool pointIsInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, poly);
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

static bool CatchGLErrors(GLenum err, const char* context)
{
	//catch GL errors
	{
		//see http://www.opengl.org/sdk/docs/man/xhtml/glGetError.xml
		switch(err)
		{
		case GL_NO_ERROR:
			return false;
		case GL_INVALID_ENUM:
			ccLog::Warning("[%s] OpenGL error: invalid enumerator", context);
			break;
		case GL_INVALID_VALUE:
			ccLog::Warning("[%s] OpenGL error: invalid value", context);
			break;
		case GL_INVALID_OPERATION:
			ccLog::Warning("[%s] OpenGL error: invalid operation", context);
			break;
		case GL_STACK_OVERFLOW:
			ccLog::Warning("[%s] OpenGL error: stack overflow", context);
			break;
		case GL_STACK_UNDERFLOW:
			ccLog::Warning("[%s] OpenGL error: stack underflow", context);
			break;
		case GL_OUT_OF_MEMORY:
			ccLog::Warning("[%s] OpenGL error: out of memory", context);
			break;
		case GL_INVALID_FRAMEBUFFER_OPERATION:
			ccLog::Warning("[%s] OpenGL error: invalid framebuffer operation", context);
			break;
		}
	}

	return true;
}

//DGM: normals are so slow to display that it's a waste of memory and time to load them in VBOs!
#define DONT_LOAD_NORMALS_IN_VBOS

bool ccPointCloud::updateVBOs(const CC_DRAW_CONTEXT& context, const glDrawParams& glParams)
{
	if (isColorOverridden())
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

	if (m_vboManager.state == vboSet::INITIALIZED)
	{
		//let's check if something has changed
		if ( glParams.showColors && ( !m_vboManager.hasColors || m_vboManager.colorIsSF ) )
		{
			m_vboManager.updateFlags |= vboSet::UPDATE_COLORS;
		}
		
		if (	glParams.showSF
		&& (		!m_vboManager.hasColors
				||	!m_vboManager.colorIsSF
				||	 m_vboManager.sourceSF != m_currentDisplayedScalarField
				||	 m_currentDisplayedScalarField->getModificationFlag() == true ) )
		{
			m_vboManager.updateFlags |= vboSet::UPDATE_COLORS;
		}

#ifndef DONT_LOAD_NORMALS_IN_VBOS
		if ( glParams.showNorms && !m_vboManager.hasNormals )
		{
			updateFlags |= UPDATE_NORMALS;
		}
#endif
		//nothing to do?
		if (m_vboManager.updateFlags == 0)
		{
			return true;
		}
	}
	else
	{
		m_vboManager.updateFlags = vboSet::UPDATE_ALL;
	}

	size_t chunksCount = ccChunk::Count(m_points);
	//allocate per-chunk descriptors if necessary
	if (m_vboManager.vbos.size() != chunksCount)
	{
		//properly remove the elements that are not needed anymore!
		for (size_t i = chunksCount; i < m_vboManager.vbos.size(); ++i)
		{
			if (m_vboManager.vbos[i])
			{
				m_vboManager.vbos[i]->destroy();
				delete m_vboManager.vbos[i];
				m_vboManager.vbos[i] = nullptr;
			}
		}

		//resize the container
		try
		{
			m_vboManager.vbos.resize(chunksCount, nullptr);
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning(QString("[ccPointCloud::updateVBOs] Not enough memory! (cloud '%1')").arg(getName()));
			m_vboManager.state = vboSet::FAILED;
			return false;
		}
	}

	//init VBOs
	unsigned pointsInVBOs = 0;
	size_t totalSizeBytesBefore = m_vboManager.totalMemSizeBytes;
	m_vboManager.totalMemSizeBytes = 0;
	{
		//DGM: the context should be already active as this method should only be called from 'drawMeOnly'
		assert(!glParams.showSF		|| m_currentDisplayedScalarField);
		assert(!glParams.showColors	|| m_rgbaColors);
#ifndef DONT_LOAD_NORMALS_IN_VBOS
		assert(!glParams.showNorms	|| (m_normals && m_normals->chunksCount() >= chunksCount));
#endif

		m_vboManager.hasColors  = glParams.showSF || glParams.showColors;
		m_vboManager.colorIsSF  = glParams.showSF;
		m_vboManager.sourceSF   = glParams.showSF ? m_currentDisplayedScalarField : nullptr;
#ifndef DONT_LOAD_NORMALS_IN_VBOS
		m_vboManager.hasNormals = glParams.showNorms;
#else
		m_vboManager.hasNormals  = false;
#endif

		//process each chunk
		for (size_t chunkIndex = 0; chunkIndex < chunksCount; ++chunkIndex)
		{
			int chunkSize = static_cast<int>(ccChunk::Size(chunkIndex, m_points));

			int chunkUpdateFlags = m_vboManager.updateFlags;
			bool reallocated = false;
			if (!m_vboManager.vbos[chunkIndex])
			{
				m_vboManager.vbos[chunkIndex] = new VBO;
			}

			//allocate memory for current VBO
			int vboSizeBytes = m_vboManager.vbos[chunkIndex]->init(chunkSize, m_vboManager.hasColors, m_vboManager.hasNormals, &reallocated);

			QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>(); 
			if (glFunc)
			{
				CatchGLErrors(glFunc->glGetError(), "ccPointCloud::vbo.init");
			}

			if (vboSizeBytes > 0)
			{
				//ccLog::Print(QString("[VBO] VBO #%1 initialized (ID=%2)").arg(chunkIndex).arg(m_vboManager.vbos[chunkIndex]->bufferId()));

				if (reallocated)
				{
					//if the vbo is reallocated, then all its content has been cleared!
					chunkUpdateFlags = vboSet::UPDATE_ALL;
				}

				m_vboManager.vbos[chunkIndex]->bind();

				//load points
				if (chunkUpdateFlags & vboSet::UPDATE_POINTS)
				{
					m_vboManager.vbos[chunkIndex]->write(0, ccChunk::Start(m_points, chunkIndex), sizeof(PointCoordinateType)*chunkSize * 3);
				}
				//load colors
				if (chunkUpdateFlags & vboSet::UPDATE_COLORS)
				{
					if (glParams.showSF)
					{
						//copy SF colors in static array
						{
							assert(m_vboManager.sourceSF);
							ColorCompType* _sfColors = s_rgbBuffer4ub;
							ScalarType* _sf = ccChunk::Start(*m_vboManager.sourceSF, chunkIndex);
							for (int j = 0; j < chunkSize; j++, _sf++)
							{
								//we need to convert scalar value to color into a temporary structure
								const ccColor::Rgb* col = _sf ? m_vboManager.sourceSF->getColor(*_sf) : nullptr;
								if (!col)
									col = &ccColor::lightGreyRGB;
								*_sfColors++ = col->r;
								*_sfColors++ = col->g;
								*_sfColors++ = col->b;
								*_sfColors++ = ccColor::MAX;
							}
						}
						//then send them in VRAM
						m_vboManager.vbos[chunkIndex]->write(m_vboManager.vbos[chunkIndex]->rgbShift, s_rgbBuffer4ub, sizeof(ColorCompType) * chunkSize * 4);
						//upadte 'modification' flag for current displayed SF
						m_vboManager.sourceSF->setModificationFlag(false);
					}
					else if (glParams.showColors)
					{
						m_vboManager.vbos[chunkIndex]->write(m_vboManager.vbos[chunkIndex]->rgbShift, ccChunk::Start(*m_rgbaColors, chunkIndex), sizeof(ColorCompType) * chunkSize * 4);
					}
				}
#ifndef DONT_LOAD_NORMALS_IN_VBOS
				//load normals
				if (glParams.showNorms && (chunkUpdateFlags & UPDATE_NORMALS))
				{
					//we must decode the normals first!
					CompressedNormType* inNorms = m_normals->chunkStartPtr(chunkIndex);
					PointCoordinateType* outNorms = s_normalBuffer;
					for (int j=0; j<chunkSize; ++j)
					{
						const CCVector3& N = ccNormalVectors::GetNormal(*inNorms++);
						*(outNorms)++ = N.x;
						*(outNorms)++ = N.y;
						*(outNorms)++ = N.z;
					}
					m_vboManager.vbos[chunkIndex]->write(m_vboManager.vbos[chunkIndex]->normalShift, s_normalBuffer, sizeof(PointCoordinateType)*chunkSize * 3);
				}
#endif
				m_vboManager.vbos[chunkIndex]->release();

				//if an error is detected
				QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
				assert(glFunc != nullptr);
				if (CatchGLErrors(glFunc->glGetError(), "ccPointCloud::updateVBOs"))
				{
					vboSizeBytes = -1;
				}
				else
				{
					m_vboManager.totalMemSizeBytes += static_cast<size_t>(vboSizeBytes);
					pointsInVBOs += chunkSize;
				}
			}

			if (vboSizeBytes < 0) //VBO initialization failed
			{
				m_vboManager.vbos[chunkIndex]->destroy();
				delete m_vboManager.vbos[chunkIndex];
				m_vboManager.vbos[chunkIndex] = nullptr;

				//we can stop here
				if (chunkIndex == 0)
				{
					ccLog::Warning(QString("[ccPointCloud::updateVBOs] Failed to initialize VBOs (not enough memory?) (cloud '%1')").arg(getName()));
					m_vboManager.state = vboSet::FAILED;
					m_vboManager.vbos.resize(0);
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
	//	for (unsigned chunkIndex=0; chunkIndex<chunksCount; ++chunkIndex)
	//		ccLog::Print(QString("[VBO] VBO #%1 status: %2 (ID=%3)")
	//			.arg(chunkIndex)
	//			.arg(m_vboManager.vbos[chunkIndex] && m_vboManager.vbos[chunkIndex]->isCreated() ? "created" : "not created")
	//			.arg(m_vboManager.vbos[chunkIndex] ? m_vboManager.vbos[chunkIndex]->bufferId() : -1));
	//}

#ifdef _DEBUG
	if (m_vboManager.totalMemSizeBytes != totalSizeBytesBefore)
		ccLog::Print(QString("[VBO] VBO(s) (re)initialized for cloud '%1' (%2 Mb = %3% of points could be loaded)")
			.arg(getName())
			.arg(static_cast<double>(m_vboManager.totalMemSizeBytes) / (1 << 20), 0, 'f', 2)
			.arg(static_cast<double>(pointsInVBOs) / size() * 100.0, 0, 'f', 2));
#endif

	m_vboManager.state = vboSet::INITIALIZED;
	m_vboManager.updateFlags = 0;

	return true;
}

int ccPointCloud::VBO::init(int count, bool withColors, bool withNormals, bool* reallocated/*=nullptr*/)
{
	//required memory
	int totalSizeBytes = sizeof(PointCoordinateType) * count * 3;
	if (withColors)
	{
		rgbShift = totalSizeBytes;
		totalSizeBytes += sizeof(ColorCompType) * count * 4;
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
			//no message as it will probably happen on a lot on (old) graphic cards
			return -1;
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
	
	return totalSizeBytes;
}

size_t ccPointCloud::vboSize() const
{
	return m_vboManager.totalMemSizeBytes;
}

void ccPointCloud::releaseVBOs()
{
	if (m_vboManager.state == vboSet::NEW)
		return;

	if (m_currentDisplay)
	{
		//'destroy' all vbos
		for (size_t i = 0; i < m_vboManager.vbos.size(); ++i)
		{
			if (m_vboManager.vbos[i])
			{
				m_vboManager.vbos[i]->destroy();
				delete m_vboManager.vbos[i];
				m_vboManager.vbos[i] = nullptr;
			}
		}
	}
	else
	{
		assert(m_vboManager.vbos.empty());
	}

	m_vboManager.vbos.resize(0);
	m_vboManager.hasColors = false;
	m_vboManager.hasNormals = false;
	m_vboManager.colorIsSF = false;
	m_vboManager.sourceSF = nullptr;
	m_vboManager.totalMemSizeBytes = 0;
	m_vboManager.state = vboSet::NEW;
}

void ccPointCloud::removeFromDisplay(const ccGenericGLDisplay* win)
{
	if (win == m_currentDisplay)
	{
		releaseVBOs();
	}

	//call parent's method
	ccGenericPointCloud::removeFromDisplay(win);
}

bool ccPointCloud::computeNormalsWithGrids(	double minTriangleAngle_deg/*=1.0*/,
											ccProgressDialog* pDlg/*=nullptr*/)
{
	unsigned pointCount = size();
	if (pointCount < 3)
	{
		ccLog::Warning(QString("[computeNormalsWithGrids] Cloud '%1' has not enough points").arg(getName()));
		return false;
	}

	//we instantiate a temporary structure to store each vertex normal (uncompressed)
	std::vector<CCVector3> theNorms;
	try
	{
		CCVector3 blankNorm(0, 0, 0);
		theNorms.resize(pointCount, blankNorm);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccMesh::computePerVertexNormals] Not enough memory!");
		return false;
	}

	//and we also reserve the memory for the (compressed) normals
	if (!hasNormals())
	{
		if (!resizeTheNormsTable())
		{
			ccLog::Warning("[computeNormalsWithGrids] Not enough memory");
			return false;
		}
	}

	//we hide normals during process
	showNormals(false);

	//progress dialog
	if (pDlg)
	{
		pDlg->setWindowTitle(QObject::tr("Normals computation"));
		pDlg->setAutoClose(false);
		pDlg->show();
		QCoreApplication::processEvents();
	}

	PointCoordinateType minAngleCos = static_cast<PointCoordinateType>(cos( CCCoreLib::DegreesToRadians( minTriangleAngle_deg ) ));
	//double minTriangleAngle_rad = CCCoreLib::DegreesToRadians(minTriangleAngle_deg);

	//for each grid cell
	for (size_t gi = 0; gi < gridCount(); ++gi)
	{
		const Grid::Shared& scanGrid = grid(gi);
		if (scanGrid && scanGrid->indexes.empty())
		{
			//empty grid, we skip it
			continue;
		}
		if (!scanGrid || scanGrid->h == 0 || scanGrid->w == 0 || scanGrid->indexes.size() != scanGrid->h * scanGrid->w)
		{
			//invalid grid
			ccLog::Warning(QString("[computeNormalsWithGrids] Grid structure #%i is invalid").arg(gi + 1));
			continue;
		}

		//progress dialog
		if (pDlg)
		{
			pDlg->setLabelText(QObject::tr("Grid: %1 x %2").arg(scanGrid->w).arg(scanGrid->h));
			pDlg->setValue(0);
			pDlg->setRange(0, static_cast<int>(scanGrid->indexes.size()));
			QCoreApplication::processEvents();
		}

		//the code below has been kindly provided by Romain Janvier
		CCVector3 sensorOrigin = (scanGrid->sensorPosition.getTranslationAsVec3D()/* + m_globalShift*/).toPC();

		for (int j = 0; j < static_cast<int>(scanGrid->h) - 1; ++j)
		{
			for (int i = 0; i < static_cast<int>(scanGrid->w) - 1; ++i)
			{
				//form the triangles with the nearest neighbors
				//and accumulate the corresponding normals
				const int& v0 = scanGrid->indexes[j * scanGrid->w + i];
				const int& v1 = scanGrid->indexes[j * scanGrid->w + (i + 1)];
				const int& v2 = scanGrid->indexes[(j + 1) * scanGrid->w + i];
				const int& v3 = scanGrid->indexes[(j + 1) * scanGrid->w + (i + 1)];

				bool topo[4] = { v0 >= 0, v1 >= 0, v2 >= 0, v3 >= 0 };

				int mask = 0;
				int pixels = 0;

				for (int j = 0; j < 4; ++j)
				{
					if (topo[j])
					{
						mask |= 1 << j;
						pixels += 1;
					}
				}

				if (pixels < 3)
				{
					continue;
				}

				Tuple3i tris[4] =
				{
					{ v0, v2, v1 },
					{ v0, v3, v1 },
					{ v0, v2, v3 },
					{ v1, v2, v3 }
				};

				int tri[2] = { -1, -1 };

				switch (mask)
				{
				case  7: tri[0] = 0; break;
				case 11: tri[0] = 1; break;
				case 13: tri[0] = 2; break;
				case 14: tri[0] = 3; break;
				case 15:
				{
					/* Choose the triangulation with smaller diagonal. */
					double d0 = (*getPoint(v0) - sensorOrigin).normd();
					double d1 = (*getPoint(v1) - sensorOrigin).normd();
					double d2 = (*getPoint(v2) - sensorOrigin).normd();
					double d3 = (*getPoint(v3) - sensorOrigin).normd();
					float ddiff1 = std::abs(d0 - d3);
					float ddiff2 = std::abs(d1 - d2);
					if (ddiff1 < ddiff2)
					{
						tri[0] = 1; tri[1] = 2;
					}
					else
					{
						tri[0] = 0; tri[1] = 3;
					}
					break;
				}

				default:
					continue;
				}

				for (int trCount = 0; trCount < 2; ++trCount)
				{
					int idx = tri[trCount];
					if (idx < 0)
					{
						continue;
					}
					const Tuple3i& t = tris[idx];

					const CCVector3* A = getPoint(t.u[0]);
					const CCVector3* B = getPoint(t.u[1]);
					const CCVector3* C = getPoint(t.u[2]);

					//now check the triangle angles
					if (minTriangleAngle_deg > 0)
					{
						CCVector3 uAB = (*B - *A); uAB.normalize();
						CCVector3 uCA = (*A - *C); uCA.normalize();

						PointCoordinateType cosA = -uCA.dot(uAB);
						if (cosA > minAngleCos)
						{
							continue;
						}

						CCVector3 uBC = (*C - *B); uBC.normalize();
						PointCoordinateType cosB = -uAB.dot(uBC);
						if (cosB > minAngleCos)
						{
							continue;
						}
						
						PointCoordinateType cosC = -uBC.dot(uCA);
						if (cosC > minAngleCos)
						{
							continue;
						}
					}

					//compute face normal (right hand rule)
					CCVector3 N = (*B - *A).cross(*C - *A);

					//we add this normal to all triangle vertices
					theNorms[t.u[0]] += N;
					theNorms[t.u[1]] += N;
					theNorms[t.u[2]] += N;
				}
			}

			if (pDlg)
			{
				//update progress dialog
				if (pDlg->wasCanceled())
				{
					unallocateNorms();
					ccLog::Warning("[computeNormalsWithGrids] Process cancelled by user");
					return false;
				}
				else
				{
					pDlg->setValue(static_cast<unsigned>(j + 1) * scanGrid->w);
				}
			}
		}
	}

	//for each vertex
	{
		for (unsigned i = 0; i < pointCount; i++)
		{
			CCVector3& N = theNorms[i];
			//normalize the 'mean' normal
			N.normalize();
			setPointNormal(i, N);
		}
	}

	//We must update the VBOs
	normalsHaveChanged();

	//we restore the normals
	showNormals(true);

	return true;
}

bool ccPointCloud::orientNormalsWithGrids(ccProgressDialog* pDlg/*=nullptr*/)
{
	unsigned pointCount = size();
	if (pointCount == 0)
	{
		ccLog::Warning(QString("[orientNormalsWithGrids] Cloud '%1' is empty").arg(getName()));
		return false;
	}

	if (!hasNormals())
	{
		ccLog::Warning(QString("[orientNormalsWithGrids] Cloud '%1' has no normals").arg(getName()));
		return false;
	}
	if (gridCount() == 0)
	{
		ccLog::Warning(QString("[orientNormalsWithGrids] Cloud '%1' has no associated grid scan").arg(getName()));
		return false;
	}

	//progress dialog
	if (pDlg)
	{
		pDlg->setWindowTitle(QObject::tr("Orienting normals"));
		pDlg->setLabelText(QObject::tr("Points: %L1").arg( pointCount ) );
		pDlg->setRange(0, static_cast<int>(pointCount));
		pDlg->show();
		QCoreApplication::processEvents();
	}

	//for each grid cell
	int progressIndex = 0;
	for (size_t gi = 0; gi < gridCount(); ++gi)
	{
		const ccPointCloud::Grid::Shared& scanGrid = grid(gi);
		if (scanGrid && scanGrid->indexes.empty())
		{
			//empty grid, we skip it
			continue;
		}
		if (!scanGrid || scanGrid->h == 0 || scanGrid->w == 0 || scanGrid->indexes.size() != scanGrid->h * scanGrid->w)
		{
			//invalid grid
			ccLog::Warning(QString("[orientNormalsWithGrids] Grid structure #%i is invalid").arg(gi + 1));
			continue;
		}

		//ccGLMatrixd toSensorCS = scanGrid->sensorPosition.inverse();
		CCVector3 sensorOrigin = (scanGrid->sensorPosition.getTranslationAsVec3D()/* + m_globalShift*/).toPC();

		const int* _indexGrid = scanGrid->indexes.data();
		for (int j = 0; j < static_cast<int>(scanGrid->h); ++j)
		{
			for (int i = 0; i < static_cast<int>(scanGrid->w); ++i, ++_indexGrid)
			{
				if (*_indexGrid >= 0)
				{
					unsigned pointIndex = static_cast<unsigned>(*_indexGrid);
					assert(pointIndex <= pointCount);
					const CCVector3* P = getPoint(pointIndex);
					//CCVector3 PinSensorCS = toSensorCS * (*P);

					CCVector3 N = getPointNormal(pointIndex);

					//check normal vector sign
					//CCVector3 NinSensorCS(N);
					//toSensorCS.applyRotation(NinSensorCS);
					CCVector3 OP = *P - sensorOrigin;
					OP.normalize();
					PointCoordinateType dotProd = OP.dot(N);
					if (dotProd > 0)
					{
						N = -N;
						setPointNormalIndex(pointIndex, ccNormalVectors::GetNormIndex(N));
					}

					if (pDlg)
					{
						//update progress dialog
						if (pDlg->wasCanceled())
						{
							unallocateNorms();
							ccLog::Warning("[orientNormalsWithGrids] Process cancelled by user");
							return false;
						}
						else
						{
							pDlg->setValue(++progressIndex);
						}
					}
				}
			}
		}
	}

	return true;
}

bool ccPointCloud::orientNormalsTowardViewPoint( CCVector3 & VP, ccProgressDialog* pDlg)
{
	int progressIndex = 0;
	for (unsigned pointIndex = 0; pointIndex < m_points.size(); ++pointIndex)
	{
		const CCVector3* P = getPoint(pointIndex);
		CCVector3 N = getPointNormal(pointIndex);
		CCVector3 OP = *P - VP;
		OP.normalize();
		PointCoordinateType dotProd = OP.dot(N);
		if (dotProd > 0)
		{
			N = -N;
			setPointNormalIndex(pointIndex, ccNormalVectors::GetNormIndex(N));
		}

		if (pDlg)
		{
			//update progress dialog
			if (pDlg->wasCanceled())
			{
				unallocateNorms();
				ccLog::Warning("[orientNormalsWithSensors] Process cancelled by user");
				return false;
			}
			else
			{
				pDlg->setValue(++progressIndex);
			}
		}
	}
	return true;
}

bool ccPointCloud::computeNormalsWithOctree(CCCoreLib::LOCAL_MODEL_TYPES model,
											ccNormalVectors::Orientation preferredOrientation,
											PointCoordinateType defaultRadius,
											ccProgressDialog* pDlg/*=nullptr*/)
{
	//compute the normals the 'old' way ;)
	if (!getOctree() && !computeOctree(pDlg))
	{
		ccLog::Warning(QString("[computeNormals] Could not compute octree for cloud '%1'").arg(getName()));
		return false;
	}

	//computes cloud normals
	QElapsedTimer eTimer;
	eTimer.start();
	NormsIndexesTableType* normsIndexes = new NormsIndexesTableType;
	if (!ccNormalVectors::ComputeCloudNormals(	this,
												*normsIndexes,
												model,
												defaultRadius,
												preferredOrientation,
												static_cast<CCCoreLib::GenericProgressCallback*>(pDlg),
												getOctree().data()))
	{
		ccLog::Warning(QString("[computeNormals] Failed to compute normals on cloud '%1'").arg(getName()));
		return false;
	}
	
	ccLog::Print("[ComputeCloudNormals] Timing: %3.2f s.", eTimer.elapsed() / 1000.0);

	if (!hasNormals())
	{
		if (!resizeTheNormsTable())
		{
			ccLog::Error(QString("Not enough memory to compute normals on cloud '%1'").arg(getName()));
			normsIndexes->release();
			return false;
		}
	}

	//we hide normals during process
	showNormals(false);

	//compress the normals
	{
		for (unsigned j = 0; j < normsIndexes->currentSize(); j++)
		{
			setPointNormalIndex(j, normsIndexes->getValue(j));
		}
	}

	//we don't need this anymore...
	normsIndexes->release();
	normsIndexes = nullptr;

	//we restore the normals
	showNormals(true);

	return true;
}

bool ccPointCloud::orientNormalsWithMST(unsigned kNN/*=6*/,
										ccProgressDialog* pDlg/*=nullptr*/)
{
	return ccMinimumSpanningTreeForNormsDirection::OrientNormals(this, kNN, pDlg);
}

bool ccPointCloud::orientNormalsWithFM(	unsigned char level,
										ccProgressDialog* pDlg/*=nullptr*/)
{
	return ccFastMarchingForNormsDirection::OrientNormals(this, level, pDlg);
}

bool ccPointCloud::hasSensor() const
{
	for (size_t i = 0; i < m_children.size(); ++i)
	{
		ccHObject* child = m_children[i];
		if (child && child->isKindOf(CC_TYPES::SENSOR))
		{
			return true;
		}
	}

	return false;
}

unsigned char ccPointCloud::testVisibility(const CCVector3& P) const
{
	if (m_visibilityCheckEnabled)
	{
		//if we have associated sensors, we can use them to check the visibility of other points 
		unsigned char bestVisibility = 255;
		for (size_t i = 0; i < m_children.size(); ++i)
		{
			ccHObject* child = m_children[i];
			if (child && child->isA(CC_TYPES::GBL_SENSOR))
			{
				ccGBLSensor* sensor = static_cast<ccGBLSensor*>(child);
				unsigned char visibility = sensor->checkVisibility(P);

				if (visibility == CCCoreLib::POINT_VISIBLE)
					return CCCoreLib::POINT_VISIBLE;
				else if (visibility < bestVisibility)
					bestVisibility = visibility;
			}
		}
		if (bestVisibility != 255)
			return bestVisibility;
	}

	return CCCoreLib::POINT_VISIBLE;
}

bool ccPointCloud::initLOD()
{
	if (!m_lod)
	{
		m_lod = new ccPointCloudLOD;
	}
	return m_lod->init(this);
}

void ccPointCloud::clearLOD()
{
	if (m_lod)
	{
		m_lod->clear();
	}
}

void ccPointCloud::clearFWFData()
{
	m_fwfWaveforms.resize(0);
	m_fwfDescriptors.clear();
}

bool ccPointCloud::computeFWFAmplitude(double& minVal, double& maxVal, ccProgressDialog* pDlg/*=nullptr*/) const
{
	minVal = maxVal = 0;
	
	if (size() != m_fwfWaveforms.size())
	{
		return false;
	}

	//progress dialog
	CCCoreLib::NormalizedProgress nProgress(pDlg, static_cast<unsigned>(m_fwfWaveforms.size()));
	if (pDlg)
	{
		pDlg->setWindowTitle(QObject::tr("FWF amplitude"));
		pDlg->setLabelText(QObject::tr("Determining min and max FWF values\nPoints: ") + QString::number(m_fwfWaveforms.size()));
		pDlg->show();
		QCoreApplication::processEvents();
	}

	//for all waveforms
	bool firstTest = true;
	for (unsigned i = 0; i < size(); ++i)
	{
		if (pDlg && !nProgress.oneStep())
		{
			return false;
		}

		ccWaveformProxy proxy = waveformProxy(i);
		if (!proxy.isValid())
		{
			continue;
		}

		double wMinVal = 0.0;
		double wMaxVal = 0.0;
		proxy.getRange(wMinVal, wMaxVal);

		if (firstTest)
		{
			minVal = wMinVal;
			maxVal = wMaxVal;
			firstTest = false;
		}
		else
		{
			if (wMaxVal > maxVal)
			{
				maxVal = wMaxVal;
			}
			if (wMinVal < minVal)
			{
				minVal = wMinVal;
			}
		}
	}

	return !firstTest;
}

bool ccPointCloud::enhanceRGBWithIntensitySF(int sfIdx, bool useCustomIntensityRange/*=false*/, double minI/*=0.0*/, double maxI/*=1.0*/)
{
	CCCoreLib::ScalarField* sf = getScalarField(sfIdx);
	if (!sf || !hasColors())
	{
		//invalid input
		assert(false);
		return false;
	}

	//apply Broovey transform to each point (color)
	if (!useCustomIntensityRange)
	{
		minI = sf->getMin();
		maxI = sf->getMax();
	}

	double intRange = maxI - minI;
	if (intRange < 1.0e-6)
	{
		ccLog::Warning("[ccPointCloud::enhanceRGBWithIntensitySF] Intensity range is too small");
		return false;
	}

	for (unsigned i = 0; i < size(); ++i)
	{
		ccColor::Rgba& col = m_rgbaColors->at(i);

		//current intensity (x3)
		int I = static_cast<int>(col.r) + static_cast<int>(col.g) + static_cast<int>(col.b);
		if (I == 0)
		{
			continue; //black remains black!
		}
		//new intensity
		double newI = 255 * ((sf->getValue(i) - minI) / intRange); //in [0 ; 1]
		//scale factor
		double scale = (3 * newI) / I;

		col.r = static_cast<ColorCompType>(std::max<ScalarType>(std::min<ScalarType>(scale * col.r, 255), 0));
		col.g = static_cast<ColorCompType>(std::max<ScalarType>(std::min<ScalarType>(scale * col.g, 255), 0));
		col.b = static_cast<ColorCompType>(std::max<ScalarType>(std::min<ScalarType>(scale * col.b, 255), 0));
	}

	//We must update the VBOs
	colorsHaveChanged();

	return true;
}

ccMesh* ccPointCloud::triangulateGrid(const Grid& grid, double minTriangleAngle_deg/*=0.0*/) const
{
	//the code below has been kindly provided by Romain Janvier
	CCVector3 sensorOrigin = (grid.sensorPosition.getTranslationAsVec3D()/* + m_globalShift*/).toPC();

	ccMesh* mesh = new ccMesh(const_cast<ccPointCloud*>(this));
	mesh->setName("Grid mesh");
	if (!mesh->reserve(grid.h * grid.w * 2))
	{
		ccLog::Warning("[ccPointCloud::triangulateGrid] Not enough memory");
		return nullptr;
	}

	PointCoordinateType minAngleCos = static_cast<PointCoordinateType>(cos( CCCoreLib::DegreesToRadians( minTriangleAngle_deg ) ));
	//double minTriangleAngle_rad = CCCoreLib::DegreesToRadians(minTriangleAngle_deg);
	
	for (int j = 0; j < static_cast<int>(grid.h) - 1; ++j)
	{
		for (int i = 0; i < static_cast<int>(grid.w) - 1; ++i)
		{
			const int& v0 = grid.indexes[j * grid.w + i];
			const int& v1 = grid.indexes[j * grid.w + (i + 1)];
			const int& v2 = grid.indexes[(j + 1) * grid.w + i];
			const int& v3 = grid.indexes[(j + 1) * grid.w + (i + 1)];

			bool topo[4] = { v0 >= 0, v1 >= 0, v2 >= 0, v3 >= 0 };

			int mask = 0;
			int pixels = 0;

			for (int j = 0; j < 4; ++j)
			{
				if (topo[j])
				{
					mask |= 1 << j;
					pixels += 1;
				}
			}

			if (pixels < 3)
			{
				continue;
			}

			Tuple3i tris[4] =
			{
				{ v0, v2, v1 },
				{ v0, v3, v1 },
				{ v0, v2, v3 },
				{ v1, v2, v3 }
			};

			int tri[2] = { -1, -1 };

			switch (mask)
			{
			case 7 : tri[0] = 0; break;
			case 11: tri[0] = 1; break;
			case 13: tri[0] = 2; break;
			case 14: tri[0] = 3; break;
			case 15:
			{
				/* Choose the triangulation with smaller diagonal. */
				double d0 = (*getPoint(v0) - sensorOrigin).normd();
				double d1 = (*getPoint(v1) - sensorOrigin).normd();
				double d2 = (*getPoint(v2) - sensorOrigin).normd();
				double d3 = (*getPoint(v3) - sensorOrigin).normd();
				float ddiff1 = std::abs(d0 - d3);
				float ddiff2 = std::abs(d1 - d2);
				if (ddiff1 < ddiff2)
				{
					tri[0] = 1; tri[1] = 2;
				}
				else
				{
					tri[0] = 0; tri[1] = 3;
				}
				break;
			}
			
			default:
				continue;
			}

			for (int trCount = 0; trCount < 2; ++trCount)
			{
				int idx = tri[trCount];
				if (idx < 0)
				{
					continue;
				}
				const Tuple3i& t = tris[idx];

				//now check the triangle angles
				if (minTriangleAngle_deg > 0)
				{
					const CCVector3* A = getPoint(t.u[0]);
					const CCVector3* B = getPoint(t.u[1]);
					const CCVector3* C = getPoint(t.u[2]);

					CCVector3 uAB = (*B - *A); uAB.normalize();
					CCVector3 uCA = (*A - *C); uCA.normalize();

					PointCoordinateType cosA = -uCA.dot(uAB);
					if (cosA > minAngleCos)
					{
						continue;
					}

					CCVector3 uBC = (*C - *B); uBC.normalize();
					PointCoordinateType cosB = -uAB.dot(uBC);
					if (cosB > minAngleCos)
					{
						continue;
					}

					PointCoordinateType cosC = -uBC.dot(uCA);
					if (cosC > minAngleCos)
					{
						continue;
					}
				}

				mesh->addTriangle(t.u[0], t.u[1], t.u[2]);
			}
		}
	}

	if (mesh->size() == 0)
	{
		delete mesh;
		mesh = nullptr;
	}
	else
	{
		mesh->shrinkToFit();
		mesh->showColors(colorsShown());
		mesh->showSF(sfShown());
		mesh->showNormals(normalsShown());
		//mesh->setEnabled(isEnabled());
	}

	return mesh;
};

bool ccPointCloud::exportCoordToSF(bool exportDims[3])
{
	if (!exportDims[0] && !exportDims[1] && !exportDims[2])
	{
		//nothing to do?!
		assert(false);
		return true;
	}

	const QString defaultSFName[3] = { "Coord. X", "Coord. Y", "Coord. Z" };

	unsigned ptsCount = size();

	//test each dimension
	for (unsigned d = 0; d < 3; ++d)
	{
		if (!exportDims[d])
		{
			continue;
		}

		int sfIndex = getScalarFieldIndexByName(qPrintable(defaultSFName[d]));
		if (sfIndex < 0)
		{
			sfIndex = addScalarField(qPrintable(defaultSFName[d]));
		}
		if (sfIndex < 0)
		{
			ccLog::Warning("[ccPointCloud::exportCoordToSF] Not enough memory!");
			return false;
		}

		CCCoreLib::ScalarField* sf = getScalarField(sfIndex);
		if (!sf)
		{
			assert(false);
			return false;
		}

		for (unsigned k = 0; k < ptsCount; ++k)
		{
			ScalarType s = static_cast<ScalarType>(getPoint(k)->u[d]);
			sf->setValue(k, s);
		}
		sf->computeMinAndMax();

		setCurrentDisplayedScalarField(sfIndex);
		showSF(true);
	}

	return true;
}

bool ccPointCloud::exportNormalToSF(bool exportDims[3])
{
	if (!exportDims[0] && !exportDims[1] && !exportDims[2])
	{
		//nothing to do?!
		assert(false);
		return true;
	}

	if (!hasNormals())
	{
		ccLog::Warning("Cloud has no normals");
		return false;
	}

	const QString defaultSFName[3] = { "Nx", "Ny", "Nz" };

	unsigned ptsCount = static_cast<unsigned>(m_normals->size());

	//test each dimension
	for (unsigned d = 0; d < 3; ++d)
	{
		if (!exportDims[d])
		{
			continue;
		}

		int sfIndex = getScalarFieldIndexByName(qPrintable(defaultSFName[d]));
		if (sfIndex < 0)
		{
			sfIndex = addScalarField(qPrintable(defaultSFName[d]));
		}
		if (sfIndex < 0)
		{
			ccLog::Warning("[ccPointCloud::exportNormalToSF] Not enough memory!");
			return false;
		}

		CCCoreLib::ScalarField* sf = getScalarField(sfIndex);
		if (!sf)
		{
			assert(false);
			return false;
		}

		for (unsigned k = 0; k < ptsCount; ++k)
		{
			ScalarType s = static_cast<ScalarType>(getPointNormal(k).u[d]);
			sf->setValue(k, s);
		}
		sf->computeMinAndMax();

		setCurrentDisplayedScalarField(sfIndex);
		showSF(true);
	}

	return true;
}
