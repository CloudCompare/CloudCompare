//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ChunkedPointCloud.h"

//local
#include "ScalarField.h"

//system
#include <string.h>
#include <assert.h>

using namespace CCLib;

ChunkedPointCloud::ChunkedPointCloud()
	: GenericIndexedCloudPersist()
	, m_points(new GenericChunkedArray<3,PointCoordinateType>())
	, m_validBB(false)
	, m_currentPointIndex(0)
	, m_currentInScalarFieldIndex(-1)
	, m_currentOutScalarFieldIndex(-1)
{
	m_points->link();
}

ChunkedPointCloud::~ChunkedPointCloud()
{
	deleteAllScalarFields();

	m_points->release();
}

void ChunkedPointCloud::clear()
{
	m_points->clear();
	deleteAllScalarFields();
	placeIteratorAtBegining();
	invalidateBoundingBox();
}

void ChunkedPointCloud::forEach(genericPointAction& action)
{
	//there's no point of calling forEach if there's no activated scalar field!
	ScalarField* currentOutScalarFieldArray = getCurrentOutScalarField();
	if (!currentOutScalarFieldArray)
	{
		assert(false);
		return;
	}

	unsigned n = size();
	for (unsigned i=0; i<n; ++i)
	{
		action(*getPoint(i),(*currentOutScalarFieldArray)[i]);
	}
}

void ChunkedPointCloud::getBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
	if (!m_validBB)
	{
		m_points->computeMinAndMax();
		m_validBB = true;
	}

	bbMin = CCVector3(m_points->getMin());
	bbMax = CCVector3(m_points->getMax());
}

void ChunkedPointCloud::invalidateBoundingBox()
{
    m_validBB = false;
}

void ChunkedPointCloud::placeIteratorAtBegining()
{
	m_currentPointIndex = 0;
}

const CCVector3* ChunkedPointCloud::getNextPoint()
{
	return (m_currentPointIndex < m_points->currentSize() ? point(m_currentPointIndex++) : 0);
}

bool ChunkedPointCloud::resize(unsigned newNumberOfPoints)
{
	unsigned oldNumberOfPoints = m_points->currentSize();

	//we try to enlarge the 3D points array
	if (!m_points->resize(newNumberOfPoints))
		return false;

	//then the scalar fields
	for (size_t i=0; i<m_scalarFields.size(); ++i)
	{
		if (!m_scalarFields[i]->resize(newNumberOfPoints))
		{
			//if something fails, we restore the previous size for already processed SFs!
			for (size_t j=0; j<i; ++j)
			{
				m_scalarFields[j]->resize(oldNumberOfPoints);
				m_scalarFields[j]->computeMinAndMax();
			}
			//we can assume that newNumberOfPoints > oldNumberOfPoints, so it should always be ok
			m_points->resize(oldNumberOfPoints);
			return false;
		}
		m_scalarFields[i]->computeMinAndMax();
	}

	return true;
}

bool ChunkedPointCloud::reserve(unsigned newNumberOfPoints)
{
	//we try to enlarge the 3D points array
	if (!m_points->reserve(newNumberOfPoints))
		return false;

	//then the scalar fields
	for (size_t i=0; i<m_scalarFields.size(); ++i)
	{
		if (!m_scalarFields[i]->reserve(newNumberOfPoints))
			return false;
	}

	//double check
	return m_points->capacity() >= newNumberOfPoints;
}

void ChunkedPointCloud::addPoint(const CCVector3 &P)
{
	//NaN coordinates check
	if (	P.x != P.x
		||	P.y != P.y
		||	P.z != P.z )
	{
		//replace NaN point by 0000
		CCVector3 fakeP(0,0,0);
		m_points->addElement(fakeP.u);
	}
	else
	{
		m_points->addElement(P.u);
	}
	m_validBB = false;
}

void ChunkedPointCloud::applyTransformation(PointProjectionTools::Transformation& trans)
{
	unsigned count = size();

	//always apply the scale before everything (applying before or after rotation does not changes anything)
	if (fabs(static_cast<double>(trans.s) - 1.0) > ZERO_TOLERANCE)
	{
		for (unsigned i=0; i<count; ++i)
			*point(i) *= trans.s;
		m_validBB = false; //invalidate bb
	}

	if (trans.R.isValid())
	{
		for (unsigned i=0; i<count; ++i)
		{
			CCVector3* P = point(i);
			(*P) = trans.R * (*P);
		}
		m_validBB = false;
	}

	if (trans.T.norm() > ZERO_TOLERANCE) //T applied only if it makes sense
	{
		for (unsigned i=0; i<count; ++i)
			*point(i) += trans.T;
		m_validBB = false;
	}
}

/***********************/
/***                 ***/
/***  SCALAR FIELDS  ***/
/***                 ***/
/***********************/

bool ChunkedPointCloud::isScalarFieldEnabled() const
{
    ScalarField* currentInScalarFieldArray = getCurrentInScalarField();
	if (!currentInScalarFieldArray)
        return false;

	unsigned sfValuesCount = currentInScalarFieldArray->currentSize();
    return (sfValuesCount>0 && sfValuesCount >= m_points->currentSize());
}

bool ChunkedPointCloud::enableScalarField()
{
    ScalarField* currentInScalarField = getCurrentInScalarField();

	if (!currentInScalarField)
	{
		//if we get there, it means that either the caller has forgot to create
		//(and assign) a scalar field to the cloud, or that we are in a compatibility
		//mode with old/basic behaviour: a unique SF for everything (input/output)

        //we look for any already existing "default" scalar field 
		m_currentInScalarFieldIndex = getScalarFieldIndexByName("Default");
		if (m_currentInScalarFieldIndex < 0)
		{
            //if not, we create it
            m_currentInScalarFieldIndex = addScalarField("Default");
            if (m_currentInScalarFieldIndex < 0) //Something went wrong
                return false;
		}

        currentInScalarField = getCurrentInScalarField();
        assert(currentInScalarField);
	}

	//if there's no output scalar field either, we set this new scalar field as output also
	if (!getCurrentOutScalarField())
		m_currentOutScalarFieldIndex = m_currentInScalarFieldIndex;

	return currentInScalarField->resize(m_points->capacity());
}

void ChunkedPointCloud::setPointScalarValue(unsigned pointIndex, ScalarType value)
{
	assert(m_currentInScalarFieldIndex>=0 && m_currentInScalarFieldIndex<(int)m_scalarFields.size());
	//slow version
	//ScalarField* currentInScalarFieldArray = getCurrentInScalarField();
	//if (currentInScalarFieldArray)
	//	currentInScalarFieldArray->setValue(pointIndex,value);

    //fast version
    m_scalarFields[m_currentInScalarFieldIndex]->setValue(pointIndex,value);
}

ScalarType ChunkedPointCloud::getPointScalarValue(unsigned pointIndex) const
{
	assert(m_currentOutScalarFieldIndex>=0 && m_currentOutScalarFieldIndex<(int)m_scalarFields.size());

	return m_scalarFields[m_currentOutScalarFieldIndex]->getValue(pointIndex);
}

ScalarField* ChunkedPointCloud::getScalarField(int index) const
{
	return (index>=0 && index < static_cast<int>(m_scalarFields.size()) ? m_scalarFields[index] : 0);
}

const char* ChunkedPointCloud::getScalarFieldName(int index) const
{
    return (index>=0 && index < static_cast<int>(m_scalarFields.size()) ? m_scalarFields[index]->getName() : 0);
}

int ChunkedPointCloud::addScalarField(const char* uniqueName)
{
    //we don't accept two SF with the same name!
    if (getScalarFieldIndexByName(uniqueName) >= 0)
        return -1;

	//create requested scalar field
    ScalarField* sf = new ScalarField(uniqueName);
	if (!sf || (size() && !sf->resize(size())))
	{
		//Not enough memory!
		if (sf)
			sf->release();
		return -1;
	}

	try
	{
		//we don't want 'm_scalarFields' to grow by 50% each time! (default behavior of std::vector::push_back)
		m_scalarFields.resize(m_scalarFields.size()+1);
	}
	catch (const std::bad_alloc&) //out of memory
	{
		sf->release();
		return -1;
	}
	
	m_scalarFields.back() = sf;
	sf->link();

	return (int)m_scalarFields.size()-1;
}

void ChunkedPointCloud::deleteScalarField(int index)
{
    int sfCount = static_cast<int>(m_scalarFields.size());
    if (index < 0 || index >= sfCount)
        return;

    //we update SF roles if they point to the deleted scalar field
    if (index == m_currentInScalarFieldIndex)
        m_currentInScalarFieldIndex = -1;
    if (index == m_currentOutScalarFieldIndex)
        m_currentOutScalarFieldIndex = -1;

    //if the deleted SF is not the last one, we swap it with the last element
    int lastIndex = sfCount-1; //lastIndex>=0
    if (index < lastIndex) //i.e.lastIndex>0
    {
        std::swap(m_scalarFields[index],m_scalarFields[lastIndex]);
        //don't forget to update SF roles also if they point to the last element
        if (lastIndex == m_currentInScalarFieldIndex)
            m_currentInScalarFieldIndex = index;
        if (lastIndex == m_currentOutScalarFieldIndex)
            m_currentOutScalarFieldIndex = index;
    }

    //we can always delete the last element (and the vector stays consistent)
	m_scalarFields.back()->release();
    m_scalarFields.pop_back();
}

void ChunkedPointCloud::deleteAllScalarFields()
{
    m_currentInScalarFieldIndex = m_currentOutScalarFieldIndex = -1;

	while (!m_scalarFields.empty())
	{
        m_scalarFields.back()->release();
		m_scalarFields.pop_back();
	}
}

int ChunkedPointCloud::getScalarFieldIndexByName(const char* name) const
{
    size_t sfCount = m_scalarFields.size();
    for (size_t i=0; i<sfCount; ++i)
    {
        //we don't accept two SF with the same name!
        if (strcmp(m_scalarFields[i]->getName(),name) == 0)
            return static_cast<int>(i);
    }

	return -1;
}

bool ChunkedPointCloud::renameScalarField(int index, const char* newName)
{
	if (getScalarFieldIndexByName(newName) < 0)
	{
		ScalarField* sf = getScalarField(index);
		if (sf)
		{
			sf->setName(newName);
			return true;
		}
	}
	return false;
}

void ChunkedPointCloud::swapPoints(unsigned firstIndex, unsigned secondIndex)
{
	if (firstIndex==secondIndex || firstIndex>=m_points->currentSize() || secondIndex>=m_points->currentSize())
        return;

	m_points->swap(firstIndex,secondIndex);

	for (size_t i=0; i<m_scalarFields.size(); ++i)
        m_scalarFields[i]->swap(firstIndex,secondIndex);
}
