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

//Local
#include "ccIndexedTransformationBuffer.h"

static bool IndexedSortOperator(const ccIndexedTransformation& a, const ccIndexedTransformation& b)
{
	return a.getIndex() < b.getIndex();
}

static bool IndexCompOperator(const ccIndexedTransformation& a, double index)
{
	return a.getIndex() < index;
}

void ccIndexedTransformationBuffer::sort()
{
	std::sort(begin(), end(), IndexedSortOperator);
}

bool ccIndexedTransformationBuffer::findNearest(double index,
												const ccIndexedTransformation* &trans1,
												const ccIndexedTransformation* &trans2,
												size_t* trans1IndexInBuffer,
												size_t* trans2IndexInBuffer) const
{
	//no transformation in buffer?
	if (empty())
	{
		return false;
	}

	trans1 = trans2 = 0;
	if (trans1IndexInBuffer)
		*trans1IndexInBuffer = 0;
	if (trans2IndexInBuffer)
		*trans2IndexInBuffer = 0;

	ccIndexedTransformationBuffer::const_iterator it = std::lower_bound(begin(),end(),index,IndexCompOperator);

	//special case: all transformations are BEFORE the input index
	if (it == end())
	{
		trans1 = &back();
		if (trans1IndexInBuffer)
			*trans1IndexInBuffer = size()-1;
		return true;
	}

	//special case: found transformation's index is equal to input index
	if (it->getIndex() == index)
	{
		trans1 = &(*it);
		if (trans1IndexInBuffer)
			*trans1IndexInBuffer = it - begin();
		++it;
		if (it != end())
		{
			trans2 = &(*it);
			if (trans2IndexInBuffer)
				*trans2IndexInBuffer = it - begin();
		}
	}
	else
	{
		if (trans2)
			trans2 = &(*it);
		if (trans2IndexInBuffer)
			*trans2IndexInBuffer = it - begin();
		if (it != begin())
		{
			--it;
			if (trans1)
				trans1 = &(*it);
			if (trans1IndexInBuffer)
				*trans1IndexInBuffer = it - begin();
		}
	}

	return true;
}

bool ccIndexedTransformationBuffer::getInterpolatedTransformation(	double index,
																	ccIndexedTransformation& trans,
																	double maxIndexDistForInterpolation/*=DBL_MAX*/) const
{
	const ccIndexedTransformation *t1, *t2;

	if (!findNearest(index, t1, t2))
		return false;

	if (t1)
	{
		double i1 = t1->getIndex();
		if (i1 == index)
		{
			trans = *t1;
		}
		else
		{
			assert(i1 < index);
			if (i1 + maxIndexDistForInterpolation < index) //trans1 is too far
				return false;

			if (t2)
			{
				double i2 = t2->getIndex();
				if (i2 - maxIndexDistForInterpolation > index) //trans2 is too far
					return false;

			}
			else
			{
				//we don't interpolate outside of the buffer 'interval'
				return false;
			}
		}
	}
	else if (t2)
	{
		if (t2->getIndex() != index) //trans2 is too far
			return false;

		trans = *t2;
	}

	return true;
}
