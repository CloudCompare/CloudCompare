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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2265                                                              $
//$LastChangedDate:: 2012-10-13 22:22:51 +0200 (sam., 13 oct. 2012)        $
//**************************************************************************
//
#include "ccPointCloud.h"

//CCLib
#include <ManualSegmentationTools.h>
#include <GeometricalAnalysisTools.h>
#include <ReferenceCloud.h>

#include "ccIncludeGL.h"
#include "ccNormalVectors.h"
#include "ccColorTablesManager.h"
#include "ccOctree.h"
#include "ccGenericMesh.h"
#include "ccMesh.h"
#include "ccMeshGroup.h"
#include "ccImage.h"
#include "cc2DLabel.h"

//system
#include <assert.h>

ccPointCloud::ccPointCloud(QString name)
	: ChunkedPointCloud()
	, ccGenericPointCloud(name)
	, m_rgbColors(0)
	, m_normals(0)
	, m_currentDisplayedScalarField(0)
	, m_currentDisplayedScalarFieldIndex(-1)
{
    init();
}

ccPointCloud::ccPointCloud(CCLib::GenericCloud* cloud)
	: ChunkedPointCloud()
	, ccGenericPointCloud("Cloud")
	, m_rgbColors(0)
	, m_normals(0)
	, m_currentDisplayedScalarField(0)
	, m_currentDisplayedScalarFieldIndex(-1)
{
    init();

    unsigned n = cloud->size();
    if (n==0)
        return;

    if (!reserveThePointsTable(n))
	{
		ccLog::Error("[ccPointCloud] Not enough memory to duplicate cloud!");
		return;
	}

	//import points
    cloud->placeIteratorAtBegining();
    for (unsigned i=0; i<n; i++)
        addPoint(*cloud->getNextPoint());
}

ccPointCloud::ccPointCloud(const CCLib::GenericIndexedCloud* cloud)
	: ChunkedPointCloud()
	, ccGenericPointCloud("Cloud")
	, m_rgbColors(0)
	, m_normals(0)
	, m_currentDisplayedScalarField(0)
	, m_currentDisplayedScalarFieldIndex(-1)
{
    init();

    unsigned n = cloud->size();
    if (n==0)
        return;

    if (!reserveThePointsTable(n))
	{
		ccLog::Error("[ccPointCloud] Not enough memory to duplicate cloud!");
		return;
	}

	//import points
	CCVector3 P;
    for (unsigned i=0; i<n; i++)
	{
		cloud->getPoint(i,P);
        addPoint(P);
	}
}

ccPointCloud::ccPointCloud(CCLib::ReferenceCloud* selection, ccPointCloud* source)
	: ChunkedPointCloud()
	, ccGenericPointCloud(source ? source->getName() : QString())
	, m_rgbColors(0)
	, m_normals(0)
	, m_currentDisplayedScalarField(0)
	, m_currentDisplayedScalarFieldIndex(-1)
{
    assert(source);
    assert(selection->getAssociatedCloud()==static_cast<GenericIndexedCloud*>(source));

    init();

    unsigned i,n = selection->size();
    if (n==0)
        return;

    if (!reserveThePointsTable(n))
    {
        //ccConsole::Error("[ccPointCloud::extract] Internal error: failed to create new point cloud! Not enough memory?");
        return;
    }

    //Ajout des points
    selection->placeIteratorAtBegining();
    for (i=0; i<n; i++)
    {
        addPoint(*selection->getCurrentPointCoordinates());
        selection->forwardIterator();
    }

    /*** Source settings heritage ***/

    //Visibility
    setVisible(source->isVisible());

    //Colors
    if (source->hasColors())
        if (reserveTheRGBTable())
        {
            ColorsTableType* _theColors = source->m_rgbColors;
            selection->placeIteratorAtBegining();
            for (i=0; i<n; i++)
            {
                addRGBColor(_theColors->getValue(selection->getCurrentPointGlobalIndex()));
                selection->forwardIterator();
            }

            showColors(source->colorsShown());
        }

    //Normals
    if (source->hasNormals())
        if (reserveTheNormsTable())
        {
            NormsIndexesTableType* _theNorms = source->m_normals;
            selection->placeIteratorAtBegining();
            for (i=0; i<n; i++)
            {
                addNormIndex(_theNorms->getValue(selection->getCurrentPointGlobalIndex()));
                selection->forwardIterator();
            }
            showNormals(source->normalsShown());
        }

    //SF
    int nSF = source->getNumberOfScalarFields();
    if (nSF>0)
    {
        int j=0,k=0;
        while (k<nSF)
        {
			ccScalarField* sf = static_cast<ccScalarField*>(source->getScalarField(j));
            if (sf)
            {
                //we create a new scalar field with same name & type
                int sfIdx = addScalarField(sf->getName(),sf->isPositive());
                //if success...
                if (sfIdx>=0)
                {
                    //we copy data to new SF
                    ccScalarField* currentScalarField = static_cast<ccScalarField*>(getScalarField(sfIdx));
                    assert(currentScalarField);
                    if (currentScalarField->resize(n))
                    {
						selection->placeIteratorAtBegining();
                        for (i=0; i<n; i++)
						{
							currentScalarField->setValue(i,sf->getValue(selection->getCurrentPointGlobalIndex()));
							selection->forwardIterator();
						}
                        currentScalarField->computeMinAndMax();
						//same color ramp parameters
						currentScalarField->setColorRampSteps(sf->getColorRampSteps());
						currentScalarField->setColorRamp(sf->getColorRamp());
                    }
                    else
                    {
                        //in we don't have enough memory, we cancel SF creation
                        deleteScalarField(sfIdx);
                    }
                }
                ++k;
            }
            ++j;
        }
        //we display the same scalar field as the source
        setCurrentDisplayedScalarField(source->getCurrentDisplayedScalarFieldIndex());
        //and same visibility
        showSF(source->sfShown());
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
    	//on insère les objets "capteur" (pas de copie ici, la même instance peut-être partagée par plusieurs listes)
    	for (i=1;i<=source->getNumberOfSensors();++i)
    		setSensor(source->_getSensor(i),i);
    }
    */

    //original center
    const double* shift = source->getOriginalShift();
    setOriginalShift(shift[0],shift[1],shift[2]);

	setName(source->getName()+QString(".extract"));
}

ccPointCloud::ccPointCloud(CCLib::ReferenceCloud* selection, ccGenericPointCloud* source)
	: ChunkedPointCloud()
	, ccGenericPointCloud(source ? source->getName() : QString())
	, m_rgbColors(0)
	, m_normals(0)
	, m_currentDisplayedScalarField(0)
	, m_currentDisplayedScalarFieldIndex(-1)
{
    assert(source);
    assert(selection->getAssociatedCloud()==source);

    init();

    unsigned i,n = selection->size();
    if (n==0)
        return;

    if (!reserveThePointsTable(n))
    {
        ccLog::Error("[ccPointCloud::Clone] Failed to create new point cloud! (not enough memory)");
        return;
    }

    //Ajout des points
    selection->placeIteratorAtBegining();
    for (i=0; i<n; i++)
    {
        addPoint(*selection->getCurrentPointCoordinates());
        selection->forwardIterator();
    }

    /*** Source settings heritage ***/

    //Visibility
    setVisible(source->isVisible());

    //Colors
    if (source->hasColors())
    {
        if (reserveTheRGBTable())
        {
            selection->placeIteratorAtBegining();
            for (i=0; i<n; i++)
            {
                addRGBColor(source->getPointColor(selection->getCurrentPointGlobalIndex()));
                selection->forwardIterator();
            }

            showColors(source->colorsShown());
        }
        else
        {
            ccLog::Warning("[ccPointCloud::clone] Failed to import colors! (not enough memory)");
        }
    }

    //Normals
    if (source->hasNormals())
    {
        if (reserveTheNormsTable())
        {
            selection->placeIteratorAtBegining();
            for (i=0; i<n; i++)
            {
                addNormIndex(source->getPointNormalIndex(selection->getCurrentPointGlobalIndex()));
                selection->forwardIterator();
            }
            showNormals(source->normalsShown());
        }
        else
        {
            ccLog::Warning("[ccPointCloud::clone] Failed to import normals! (not enough memory)");
        }
    }

    //SF
    if (source->hasDisplayedScalarField())
    {
        //we create a new scalar field with same name & type
        int sfIdx = addScalarField("Scalar field",source->isDisplayedSFPositive());
        //if success...
        if (sfIdx>=0)
        {
            //we copy data to new SF
            CCLib::ScalarField* currentScalarField = getScalarField(sfIdx);
            assert(currentScalarField);
            if (currentScalarField->reserve(n))
            {
                selection->placeIteratorAtBegining();
                for (i=0; i<n; i++)
                {
					DistanceType d = source->getPointDisplayedDistance(selection->getCurrentPointGlobalIndex());
                    currentScalarField->addElement(d);
                    selection->forwardIterator();
                }
                currentScalarField->computeMinAndMax();

                //we display the same scalar field as the source
                setCurrentDisplayedScalarField(sfIdx);

                //and same display mode
                showSF(source->sfShown());
            }
            else //in we don't have enough memory, we cancel SF creation
            {
                deleteScalarField(sfIdx);
				ccLog::Warning(QString("[ccPointCloud::clone] Failed to import SF '%1'! (not enough memory)").arg(currentScalarField->getName()));
            }
        }
    }

    //original center
    const double* shift = source->getOriginalShift();
    setOriginalShift(shift[0],shift[1],shift[2]);

	setName(source->getName()+QString(".extract"));
}

ccPointCloud::~ccPointCloud()
{
    clear();
}

void ccPointCloud::init()
{
    showSFColorsScale(false);
    setGreyForNanScalarValues(true);
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
                    bool sfPositive = sf->isPositive();
					ccScalarField* newSF = new ccScalarField(sf->getName(),sfPositive);
					//we fill the begining with NaN (as there is no equivalent in the current cloud)
					DistanceType NaN = (sfPositive ? HIDDEN_VALUE : OUT_VALUE);
					if (newSF->resize(pointCountBefore+addedPoints,true,NaN))
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
					DistanceType NaN = (sf->isPositive() ? HIDDEN_VALUE : OUT_VALUE);
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
    if (!m_points->isAllocated())
    {
        ccLog::Error("[ccPointCloud::reserveTheRGBTable] Internal error: properties (re)allocation before points allocation is forbidden!");
        return false;
    }

	if (!m_rgbColors)
	{
		m_rgbColors = new ColorsTableType;
		m_rgbColors->link();
	}
	return m_rgbColors->reserve(m_points->capacity());
}

bool ccPointCloud::resizeTheRGBTable(bool fillWithWhite)
{
    if (!m_points->isAllocated())
    {
        ccLog::Error("[ccPointCloud::resizeTheRGBTable] Internal error: properties (re)allocation before points allocation is forbidden!");
        return false;
    }

	if (!m_rgbColors)
	{
		m_rgbColors = new ColorsTableType;
		m_rgbColors->link();
	}

	return m_rgbColors->resize(m_points->currentSize(),fillWithWhite,fillWithWhite ? ccColor::white : 0);
}

bool ccPointCloud::reserveTheNormsTable()
{
    if (!m_points->isAllocated())
    {
        ccLog::Error("[ccPointCloud::reserveTheNormsTable] Internal error: properties (re)allocation before points allocation is forbidden!");
        return false;
    }

	if (!m_normals)
	{
		m_normals = new NormsIndexesTableType;
		m_normals->link();
	}

    return m_normals->reserve(m_points->capacity());
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
		m_normals = new NormsIndexesTableType;
		m_normals->link();
	}

    return m_normals->resize(m_points->currentSize(),true,0);
}

bool ccPointCloud::reserve(unsigned newNumberOfPoints)
{
    //le reserve n'est possible que pour agrandir le tableau (utiliser resize sinon)
    if (newNumberOfPoints<size())
        return false;

    //les éléments sur lesquels ont peu faire des "push" (à savoir : points, couleurs, normales et distances) sont "reservés" uniquement
    //alors que les autres éléments sont "resizés"
    if (!ChunkedPointCloud::reserve(newNumberOfPoints)) //points (reserve) + champs scalaires (reserve)
    {
        //ccConsole::Error("[ccPointCloud::reserve] Memory reallocation error! (ChunkedPointCloud)");
        return false;
    }

    if (hasColors()) //colors (reserve)
    {
        if (!reserveTheRGBTable())
        {
            //ccConsole::Error("[ccPointCloud::reserve] Memory reallocation error! (colors)");
            return false;
        }
    }

    if (hasNormals()) //normals (reserve)
    {
        if (!reserveTheNormsTable())
        {
            //ccConsole::Error("[ccPointCloud::reserve] Memory reallocation error! (Normals)");
            return false;
        }
    }

    return true;
}

bool ccPointCloud::resize(unsigned newNumberOfPoints)
{
    if (newNumberOfPoints<size() && isLocked())
        return false;

    if (!ChunkedPointCloud::resize(newNumberOfPoints)) //points + champs scalaires
    {
		ccLog::Error("[ccPointCloud::resize] Not enough memory!");
        return false;
    }

    updateModificationTime();

    if (hasColors()) //colors (resize)
    {
        if (!resizeTheRGBTable(false))
        {
			ccLog::Error("[ccPointCloud::resize] Not enough memory!");
            return false;
        }
    }

    if (hasNormals()) //normals (resize)
    {
        if (!resizeTheNormsTable())
        {
			ccLog::Error("[ccPointCloud::resize] Not enough memory!");
            return false;
        }
    }

	//update scalar fields
	for (unsigned i=0;i<getNumberOfScalarFields();++i)
	{
		CCLib::ScalarField* sf = getScalarField(i);
		if (sf)
			sf->computeMinAndMax();
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

bool ccPointCloud::areNanScalarValuesInGrey() const
{
    return m_greyForNanScalarValues;
}

void ccPointCloud::setGreyForNanScalarValues(bool state)
{
    m_greyForNanScalarValues = state;
}

const colorType* ccPointCloud::getPointDistanceColor(unsigned pointIndex) const
{
    assert(m_currentDisplayedScalarField);
    assert(pointIndex<m_currentDisplayedScalarField->currentSize());

    DistanceType normalizedDist = m_currentDisplayedScalarField->getNormalizedValue(pointIndex);
    if (normalizedDist>=0.0)
		return ccColorTablesManager::GetUniqueInstance()->getColor(normalizedDist,m_currentDisplayedScalarField->getColorRampSteps(),m_currentDisplayedScalarField->getColorRamp());
    else
        return (m_greyForNanScalarValues ? ccColor::lightGrey : 0);
}

const colorType* ccPointCloud::getDistanceColor(DistanceType d) const
{
    assert(m_currentDisplayedScalarField);

    DistanceType normalizedDist = m_currentDisplayedScalarField->normalize(d);
    if (normalizedDist<0.0)
        return (m_greyForNanScalarValues ? ccColor::lightGrey : NULL);
    else
        return ccColorTablesManager::GetUniqueInstance()->getColor(normalizedDist,m_currentDisplayedScalarField->getColorRampSteps(),m_currentDisplayedScalarField->getColorRamp());
}

bool ccPointCloud::isDisplayedSFPositive()
{
    assert(m_currentDisplayedScalarField);
	return (m_currentDisplayedScalarField ? m_currentDisplayedScalarField->isPositive() : false);
}

DistanceType ccPointCloud::getPointDisplayedDistance(unsigned pointIndex) const
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

const normsType ccPointCloud::getPointNormalIndex(unsigned pointIndex) const
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
    return m_currentDisplayedScalarField;
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

void ccPointCloud::colorize(float r, float g, float b)
{
    if (hasColors())
    {
        colorType* p;
        m_rgbColors->placeIteratorAtBegining();
		for (unsigned i=0;i<m_rgbColors->currentSize();i++)
        {
            p = m_rgbColors->getCurrentValue();
            *p = colorType(float(*p) * r);
            p++;
            *p = colorType(float(*p) * g);
            p++;
            *p = colorType(float(*p) * b);
            m_rgbColors->forwardIterator();
        }

    }
    else
    {
        colorType RGB[3];
        RGB[0] = colorType(float(MAX_COLOR_COMP)*r);
        RGB[1] = colorType(float(MAX_COLOR_COMP)*g);
        RGB[2] = colorType(float(MAX_COLOR_COMP)*b);
		resizeTheRGBTable(false);
        m_rgbColors->fill(RGB);
    }
}

void ccPointCloud::colorizeWithDefaultRamp(unsigned char heightDim)
{
    if (heightDim > 2) //X=0, Y=1, Z=2
        return;

    if (!hasColors())
        resizeTheRGBTable(false);

    enableTempColor(false);

    PointCoordinateType Mins[3],Maxs[3];
    getBoundingBox(Mins,Maxs);
    PointCoordinateType dMin = Mins[heightDim];
    PointCoordinateType heightCoef = Maxs[heightDim]-dMin;

    if (heightCoef != 0.0)
        heightCoef = 1.0/heightCoef;

    float colorIndex;

    placeIteratorAtBegining();
    unsigned i,count=size();
    for (i=0;i<count;i++)
    {
        const CCVector3* Q=getNextPoint();
        colorIndex = float((Q->u[heightDim]-dMin)*heightCoef);

        m_rgbColors->setValue(i,ccColorTablesManager::GetUniqueInstance()->getColor(colorIndex,DEFAULT_COLOR_RAMP));
    }
}

void ccPointCloud::colorizeByHeight(unsigned char heightDim, const colorType* minColor, const colorType* maxColor)
{
    if (heightDim > 2)
        return;

    if (!hasColors())
        resizeTheRGBTable(false);

    enableTempColor(false);

    PointCoordinateType Mins[3],Maxs[3];
    getBoundingBox(Mins,Maxs);
    PointCoordinateType dMin = Mins[heightDim];
    PointCoordinateType heightCoef = Maxs[heightDim]-dMin;
    PointCoordinateType colorCoef;

    if (heightCoef != 0.0)
        heightCoef = 1.0/heightCoef;

    colorType p[3];
    float dColor[3];
    dColor[0]=float(maxColor[0]-minColor[0]);
    dColor[1]=float(maxColor[1]-minColor[1]);
    dColor[2]=float(maxColor[2]-minColor[2]);

    placeIteratorAtBegining();
    unsigned i,count=size();
    for (i=0;i<count;i++)
    {
        const CCVector3* Q=getNextPoint();
        colorCoef = (Q->u[heightDim]-dMin)*heightCoef;

        p[0] = minColor[0]+colorType(dColor[0]*colorCoef);
        p[1] = minColor[1]+colorType(dColor[1]*colorCoef);
        p[2] = minColor[2]+colorType(dColor[2]*colorCoef);
        m_rgbColors->setValue(i,p);
    }
}

void ccPointCloud::setRGBColor(colorType r, colorType g, colorType b)
{
    colorType c[3] = {r,g,b};
    setRGBColor(c);
}

void ccPointCloud::setRGBColor(const colorType* col)
{
    enableTempColor(false);
    if (!hasColors())
		reserveTheRGBTable();
    m_rgbColors->fill(col);
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
    PointCoordinateType* Mins = m_points->getMin();
    PointCoordinateType* Maxs = m_points->getMax();
    CCVector3::vadd(Mins,T.u,Mins);
    CCVector3::vadd(Maxs,T.u,Maxs);

    //same thing for the octree
    ccOctree* oct = getOctree();
    if (oct)
        oct->translateBoundingBox(T);
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
    PointCoordinateType* Mins = m_points->getMin();
    PointCoordinateType* Maxs = m_points->getMax();
    Mins[0] *= fx;
    Maxs[0] *= fx;
    if (fx<0.0)
        std::swap(Mins[0],Maxs[0]);
    Mins[1] *= fy;
    Maxs[1] *= fy;
    if (fy<0.0)
        std::swap(Mins[1],Maxs[1]);
    Mins[2] *= fz;
    Maxs[2] *= fz;
    if (fz<0.0)
        std::swap(Mins[2],Maxs[2]);

    ccOctree* oct = getOctree();
    if (oct)
    {
        if (fx==fy && fx==fz && fx>0.0 && fy>0.0 && fz>0.0)
            oct->multiplyBoundingBox(fx);
        else
            deleteOctree();
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
static colorType s_rgbBuffer[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3];

void ccPointCloud::drawMeOnly(CC_DRAW_CONTEXT& context)
{
    if (!m_points->isAllocated())
        return;

    if (MACRO_Draw3D(context))
    {
        //standard case: list names pushing
        bool pushName = MACRO_DrawNames(context);
		//special case: point names pushing (for picking)
        bool pushPointNames = MACRO_DrawPointNames(context);

		//we get display parameters
        glDrawParams glParams;
        getDrawingParameters(glParams);
        glParams.showNorms &= bool(MACRO_LightIsEnabled(context));

        bool colorMaterial = false;
		ccColorTablesManager* colorTable = 0;
        if (glParams.showSF || glParams.showColors)
        {
            glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
            glEnable(GL_COLOR_MATERIAL);
            colorMaterial=true;

			if (glParams.showSF)
			{
				assert(m_currentDisplayedScalarField);
				colorTable = ccColorTablesManager::GetUniqueInstance();
			}
        }

        if (glParams.showColors && isColorOverriden())
        {
            glColor3ubv(tempColor);
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
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,ccColor::darker);
            glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,ccColor::lighter);
            glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,ccColor::bright);
            glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,ccColor::darker);
            glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,50.0);
            glEnable(GL_LIGHTING);
			compressedNormals = ccNormalVectors::GetUniqueInstance();
        }

        const colorType* col = 0;
        const PointCoordinateType* N = 0;
        unsigned decimStep;
        //unsigned decimedNumberOfPoints;

        // L.O.D.
		unsigned numberOfPoints=size();
        if (numberOfPoints>MAX_LOD_POINTS_NUMBER && context.decimateCloudOnMove &&  MACRO_LODActivated(context))
        {
            decimStep = int(ceil(float(numberOfPoints) / float(MAX_LOD_POINTS_NUMBER)));
            //decimedNumberOfPoints = int(floor(float(numberOfPoints) / float(decimStep)));
        }
        else
        {
            //decimedNumberOfPoints = numberOfPoints;
            decimStep = 1;
        }

        /*** DISPLAY ***/

        //standard case: list names pushing
        if (pushName)
            glPushName(getUniqueID());

		if (!pushPointNames) //standard "full" display
		{
			//if some points are hidden, we can't use display arrays :(
			if (isVisibilityTableInstantiated())
			{
				glBegin(GL_POINTS);

				//Scalar field?
				if (glParams.showSF)
				{
					for (unsigned j=0;j<numberOfPoints;j+=decimStep)
					{
						if (m_visibilityArray->getValue(j)>0)
						{
							assert(j<m_currentDisplayedScalarField->currentSize());
							DistanceType normalizedDist = m_currentDisplayedScalarField->getNormalizedValue(j);
							//we force display of points hidden because of of their scalar field value
							//to be sure that the user don't miss them (during manual segmentation for instance)
							col = (normalizedDist>=0.0 ? colorTable->getColor(normalizedDist,m_currentDisplayedScalarField->getColorRampSteps(),m_currentDisplayedScalarField->getColorRamp()) : ccColor::lightGrey);

							glColor3ubv(col);
							if (glParams.showNorms)
								glNormal3fv(compressedNormals->getNormal(m_normals->getValue(j)));
							glVertex3fv(m_points->getValue(j));
						}
					}
				}
				else
				{
					for (unsigned j=0;j<numberOfPoints;j+=decimStep)
					{
						if (m_visibilityArray->getValue(j)>0)
						{
							if (glParams.showColors)
								glColor3ubv(m_rgbColors->getValue(j));
							if (glParams.showNorms)
								glNormal3fv(compressedNormals->getNormal(m_normals->getValue(j)));
							glVertex3fv(m_points->getValue(j));
						}
					}
				}
				glEnd();
			}
			else if (glParams.showSF)
			{
				bool hiddenPoints = !m_greyForNanScalarValues;
				if (hiddenPoints)
				{
					//the fact that NaN values SHOULD be hidden, doesn't mean that we actually HIDE points...
					hiddenPoints &= ( m_currentDisplayedScalarField->getMaxDisplayed()<=m_currentDisplayedScalarField->getMax()
									|| m_currentDisplayedScalarField->getMinDisplayed()>=m_currentDisplayedScalarField->getMin());
				}

				//Continuous scalar field, without hidden points or normals (fastest case)
				if (!hiddenPoints)
				{
					glEnableClientState(GL_VERTEX_ARRAY);
					glEnableClientState(GL_COLOR_ARRAY);
					glColorPointer(3,GL_UNSIGNED_BYTE,0,s_rgbBuffer);
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
						colorType* _sfColors = s_rgbBuffer;
						DistanceType* _sf = m_currentDisplayedScalarField->chunkStartPtr(k);
						for (unsigned j=0;j<chunkSize;j+=decimStep,_sf+=decimStep)
						{
							//we need to convert scalar value to color into a temporary structure
							DistanceType normalizedDist = m_currentDisplayedScalarField->normalize(*_sf);
							col = (normalizedDist>=0.0 ? colorTable->getColor(normalizedDist,m_currentDisplayedScalarField->getColorRampSteps(),m_currentDisplayedScalarField->getColorRamp()) : ccColor::lightGrey);
							*_sfColors++=*col++;
							*_sfColors++=*col++;
							*_sfColors++=*col++;
						}

						//normals
						if (glParams.showNorms)
						{
							PointCoordinateType* _normals = s_normBuffer;
							const normsType* _normalsIndexes = m_normals->chunkStartPtr(k);
							for (unsigned j=0;j<chunkSize;j+=decimStep,_normalsIndexes+=decimStep)
							{
							    N = compressedNormals->getNormal(*_normalsIndexes);
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

					glDisableClientState(GL_VERTEX_ARRAY);
					glDisableClientState(GL_COLOR_ARRAY);
					if (glParams.showNorms)
						glDisableClientState(GL_NORMAL_ARRAY);
				}
				else //potentially hidden points
				{
					glBegin(GL_POINTS);

					if (glParams.showNorms)
					{
						for (unsigned j=0;j<numberOfPoints;j+=decimStep)
						{
							//col = getPointDistanceColor(j);
							assert(j<m_currentDisplayedScalarField->currentSize());
							DistanceType normalizedDist = m_currentDisplayedScalarField->getNormalizedValue(j);
							if (normalizedDist>=0.0)
								col = colorTable->getColor(normalizedDist,m_currentDisplayedScalarField->getColorRampSteps(),m_currentDisplayedScalarField->getColorRamp());
							else
								col = (m_greyForNanScalarValues ? ccColor::lightGrey : 0);
							if (col)
							{
								glColor3ubv(col);
								glNormal3fv(compressedNormals->getNormal(m_normals->getValue(j)));
								glVertex3fv(m_points->getValue(j));
							}
						}
					}
					else //potentially hidden points without normals (a bit faster)
					{
						for (unsigned j=0;j<numberOfPoints;j+=decimStep)
						{
							//col = getPointDistanceColor(j);
							assert(j<m_currentDisplayedScalarField->currentSize());
							DistanceType normalizedDist = m_currentDisplayedScalarField->getNormalizedValue(j);
							if (normalizedDist>=0.0)
								col = colorTable->getColor(normalizedDist,m_currentDisplayedScalarField->getColorRampSteps(),m_currentDisplayedScalarField->getColorRamp());
							else
								col = (m_greyForNanScalarValues ? ccColor::lightGrey : 0);
							if (col)
							{
								glColor3ubv(col);
								glVertex3fv(m_points->getValue(j));
							}
						}
					}
					glEnd();
				}
			}
			else if (glParams.showNorms)
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
					    N = compressedNormals->getNormal(*_normalsIndexes);
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
			else
			{
				/*** Fast way to display simple clouds ***/
				//My old buggy ATI card wasn't supporting "glDrawArrays" with too many points...
				//DGM: well, it's very old now, why bother?! And we use chunked arrays now
				//if (false)
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
            if (isVisibilityTableInstantiated())
            {
                for (unsigned j=0;j<numberOfPoints;j+=decimStep)
                {
                    if (m_visibilityArray->getValue(j)>0)
                    {
                        glPushName(j);
                        glBegin(GL_POINTS);
                        glVertex3fv(m_points->getValue(j));
                        glEnd();
                        glPopName();
                    }
                }
            }
			else if (glParams.showSF)
			{
				for (unsigned j=0;j<numberOfPoints;j+=decimStep)
				{
					col = getPointDistanceColor(j);
					if (col)
					{
                        glPushName(j);
                        glBegin(GL_POINTS);
                        glVertex3fv(m_points->getValue(j));
                        glEnd();
                        glPopName();
					}
				}
			}
			else
			{
				for (unsigned j=0;j<numberOfPoints;j+=decimStep)
				{
					glPushName(j);
					glBegin(GL_POINTS);
					glVertex3fv(m_points->getValue(j));
					glEnd();
					glPopName();
				}
			}

			//glEnd();
		}

        /*** END DISPLAY ***/

        if (colorMaterial)
            glDisable(GL_COLOR_MATERIAL);

        //we can now switch the light off
        if (glParams.showNorms)
        {
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
            if (sfColorScaleShown())
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
    context.greyForNanScalarValues = m_greyForNanScalarValues;

    const char* sfName = getScalarFieldName(sfIdx);
    strcpy(context.colorRampTitle,sfName);
}

ccPointCloud* ccPointCloud::filterPointsByScalarValue(DistanceType minVal, DistanceType maxVal)
{
    CCLib::ReferenceCloud* c = CCLib::ManualSegmentationTools::segment(this,minVal,maxVal);

    ccPointCloud* newList = NULL;
    if (c)
    {
        newList = new ccPointCloud(c,this);
        delete c;
    }

    return newList;
}

void ccPointCloud::hidePointsByScalarValue(DistanceType minVal, DistanceType maxVal)
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
    const uchar hiddenValue = 0;
	unsigned i,count=size();
    for (i=0;i<count;++i)
    {
		DistanceType val = sf->getValue(i);
        if (val<minVal || val>maxVal)
            m_visibilityArray->setValue(i,hiddenValue);
    }
}

ccGenericPointCloud* ccPointCloud::createNewCloudFromVisibilitySelection(bool removeSelectedPoints)
{
    if (!isVisibilityTableInstantiated())
    {
        ccLog::Error(QString("[Cloud %1] Visibility table not instantiated!").arg(getName()));
        return 0;
    }

    //we create a temporary entity with the visible points only
    CCLib::ReferenceCloud* rc = getTheVisiblePoints();

    //nothing to do!
    if (!rc || rc->size()==0)
    {
        ccLog::Error("[ccPointCloud::createNewCloudFromVisibilitySelection] No points in selection!");
        if (rc)
			delete rc;
        return 0;
    }

    //we create a new cloud with the "visible" points
    ccPointCloud* result = new ccPointCloud(rc,this);

	//don't need this one anymore
    delete rc;
	rc=0;

    if (!result)
    {
        //ccConsole::Error("An error occured: not enough memory ?");
        return NULL;
    }

    result->setName(getName()+QString(".part"));

    //shall the visible points be erased from this cloud?
    if (removeSelectedPoints && !isLocked())
    {
        //we remove all visible points
        unsigned lastPoint = 0;
		unsigned i,count=size();
        for (i=0;i<count;++i)
        {
            if (m_visibilityArray->getValue(i)==0)
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
        		if (!m_visibilityArray->getValue(i))
        			invrc->addPointIndex(i);

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
    setCurrentDisplayedScalarField(m_currentInScalarFieldIndex);
	showSF(m_currentInScalarFieldIndex>=0);
}

void ccPointCloud::deleteAllScalarFields()
{
    //on appelle la méthode "héritée"
    ChunkedPointCloud::deleteAllScalarFields();
    //on m.a.j. le champ affiché
    setCurrentDisplayedScalarField(-1);
	showSF(false);
}

void ccPointCloud::setColorWithDistances(bool mixWithExistingColor)
{
    if (!m_currentDisplayedScalarField)
        return;

    const colorType* col = NULL;
    DistanceType normalizedDist;

    if (!mixWithExistingColor || !hasColors())
    {
        if (!hasColors())
			resizeTheRGBTable(false);

		unsigned i,count=size();
        for (i=0;i<count;i++)
        {
            col = getPointDistanceColor(i);
            m_rgbColors->setValue(i,(col ? col : ccColor::black));
        }
    }
    else
    {

        float colorCoef = 1.0/float(MAX_COLOR_COMP);
        colorType* _theColors;

        m_rgbColors->placeIteratorAtBegining();
		unsigned i,count=size();
        for (i=0;i<count;i++)
        {
            normalizedDist = m_currentDisplayedScalarField->normalize(m_currentDisplayedScalarField->getValue(i));
            if (normalizedDist < 0.0)
            {
                if (m_greyForNanScalarValues)
                    col = ccColor::lightGrey;
                else
                    col = ccColor::black;
            }
            else
            {
				col = ccColorTablesManager::GetUniqueInstance()->getColor(normalizedDist,m_currentDisplayedScalarField->getColorRampSteps(),m_currentDisplayedScalarField->getColorRamp());
            }

            _theColors = m_rgbColors->getCurrentValue();
            *_theColors = colorType(float(*_theColors)*float(col[0])*colorCoef);
            ++_theColors;
            *_theColors = colorType(float(*_theColors)*float(col[1])*colorCoef);
            ++_theColors;
            *_theColors = colorType(float(*_theColors)*float(col[2])*colorCoef);
            m_rgbColors->forwardIterator();
        }
    }
}

void ccPointCloud::unrollOnCylinder(double radius, CCVector3* center, int dim, CCLib::GenericProgressCallback* progressCb)
{
    PointCoordinateType P0,P1,P2,u,lon;

    uchar dim1 = (dim>0 ? dim-1 : 2);
    uchar dim2 = (dim<2 ? dim+1 : 0);

	unsigned numberOfPoints=size();
    float percent = 0.0;
    float percentAdd = 1.0;
    int palier = 1;
    int count = 0;
    if (progressCb)
    {
        progressCb->reset();
        progressCb->setMethodTitle("Unroll (cylinder)");
        if (numberOfPoints>100)
            palier = int(float(numberOfPoints) * 0.01);
        else
            percentAdd = 100.0 / float(numberOfPoints);

        char buffer[256];
        sprintf(buffer,"Number of points = %i",numberOfPoints);
        progressCb->setInfo(buffer);
        progressCb->start();
    }

	CCVector3 C;
    if (!center)
    {
        PointCoordinateType Mins[3],Maxs[3];
        getBoundingBox(Mins,Maxs);
		C=(CCVector3(Mins)+CCVector3(Maxs))*0.5;
        center = &C;
    }

    PointCoordinateType px,py,nlon,nu;
    CCVector3 n2;

    for (unsigned i=0;i<numberOfPoints;i++)
    {
        CCVector3 *Q = point(i);

        P0 = Q->u[dim1]-center->u[dim1];
        P1 = Q->u[dim2]-center->u[dim2];
        P2 = Q->u[dim]-center->u[dim];

        u = sqrt(P0 * P0 + P1 * P1);
        lon = atan2(P0,P1);

        //we project the point
        Q->x = lon*radius;
        Q->y = P2;
        Q->z = u-radius;

        // and its normal if necessary
        if (hasNormals())
        {
            const PointCoordinateType *n = ccNormalVectors::GetNormal(m_normals->getValue(i));

            px = P0+n[dim1];
            py = P1+n[dim2];
            nlon = atan2(px,py);
            nu = sqrt(px*px+py*py);

            n2.x = (nlon-lon)*radius;
            n2.y = n[dim];
            n2.z = nu - u;

            n2.normalize();
            setPointNormal(i,n2.u);
        }

        if (progressCb)
        {
            if (++count==palier)
            {
                count=0;
                percent+=percentAdd;
                progressCb->update(percent);

                if (progressCb->isCancelRequested())
                    break;
            }
        }
    }

    refreshBB();

    if (progressCb)
        progressCb->stop();
}

void ccPointCloud::unrollOnCone(double baseRadius, double alpha, const CCVector3& apex, int dim, CCLib::GenericProgressCallback* progressCb)
{
    uchar dim1 = (dim>0 ? dim-1 : 2);
    uchar dim2 = (dim<2 ? dim+1 : 0);

	unsigned numberOfPoints=size();
	CCLib::NormalizedProgress* nprogress=0;
    if (progressCb)
    {
        progressCb->reset();
		nprogress = new CCLib::NormalizedProgress(progressCb,numberOfPoints);
        progressCb->setMethodTitle("Unroll (cone)");
        char buffer[256];
        sprintf(buffer,"Number of points = %i",numberOfPoints);
        progressCb->setInfo(buffer);
        progressCb->start();
    }

    float tan_alpha = tan(alpha*CC_DEG_TO_RAD);
    float cos_alpha = cos(alpha*CC_DEG_TO_RAD);
    float sin_alpha = sin(alpha*CC_DEG_TO_RAD);
    float q = 1.0/(1.0+tan_alpha*tan_alpha);

    PointCoordinateType P0,P1,P2,u,lon,z2,x2,dX,dZ,lat,alt;
    CCVector3 n2;

    for (unsigned i=0;i<numberOfPoints;i++)
    {
        CCVector3 *P = point(i);
        P0 = P->u[dim1]-apex.u[dim1];
        P1 = P->u[dim2]-apex.u[dim2];
        P2 = P->u[dim]-apex.u[dim];

        u = sqrt(P0 * P0 + P1 * P1);
        lon = atan2(P0,P1);

        //projection sur le cone
        z2 = (P2+u*tan_alpha)*q;
        x2 = z2*tan_alpha;
        //#define ORTHO_CONIC_PROJECTION
#ifdef ORTHO_CONIC_PROJECTION
        lat = sqrt(x2*x2+z2*z2)*cos_alpha;
        if (lat*z2<0.0) lat=-lat;
#else
        lat = P2;
#endif
        //altitude
        dX = u-x2;
        dZ = P2-z2;
        alt = sqrt(dX*dX+dZ*dZ);
        //on regarde de quel côté de la surface du cone le resultat tombe par p.v.
        if (x2*P2 - z2*u<0.0) alt=-alt;

        //we project point
        P->x = lon*baseRadius;
        P->y = lat+apex.u[dim];
        P->z = alt;

        // and its normal if necessary
        if (hasNormals())
        {
            const PointCoordinateType *n = ccNormalVectors::GetNormal(m_normals->getValue(i));

            dX = cos(lon)*n[dim1]-sin(lon)*n[dim2];
            dZ = sin(lon)*n[dim1]+cos(lon)*n[dim2];

            n2.x = dX;
            n2.y = sin_alpha*dZ+cos_alpha*n[dim];
            n2.z = cos_alpha*dZ-sin_alpha*n[dim];

            n2.normalize();
            setPointNormal(i,n2.u);
        }

		if (nprogress && !nprogress->oneStep())
			break;
    }


    refreshBB();

    if (progressCb)
	{
        progressCb->stop();
		delete nprogress;
		nprogress=0;
	}
}

int ccPointCloud::addScalarField(const char* uniqueName, bool isStrictlyPositive)
{
    //we don't accept two SF with the same name!
    if (getScalarFieldIndexByName(uniqueName)>=0)
	{
		ccLog::Warning("[ccPointCloud::addScalarField] Names already exists!");
        return -1;
	}

	//Nouveau champ scalaire
    ccScalarField* sf = new ccScalarField(uniqueName,isStrictlyPositive);
	if (size()>0)
		if (!sf->reserve(size()))
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

		//'show NaN values in grey' state (dataVersion>=20)
		if (out.write((const char*)&m_greyForNanScalarValues,sizeof(bool))<0)
			return WriteError();
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

		//'show NaN values in grey' state (dataVersion>=20)
		if (in.read((char*)&m_greyForNanScalarValues,sizeof(bool))<0)
			return ReadError();
		//'show current sf color scale' state (dataVersion>=20)
		if (in.read((char*)&m_sfColorScaleDisplayed,sizeof(bool))<0)
			return ReadError();

		//Displayed scalar field index (dataVersion>=20)
		int32_t displayedScalarFieldIndex = 0;(int32_t)m_currentDisplayedScalarFieldIndex;
		if (in.read((char*)&displayedScalarFieldIndex,4)<0)
			return ReadError();
		if (displayedScalarFieldIndex<(int32_t)sfCount)
			setCurrentDisplayedScalarField(displayedScalarFieldIndex);
	}

	updateModificationTime();

	return true;
}

