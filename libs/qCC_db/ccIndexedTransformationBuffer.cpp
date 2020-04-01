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

#include "ccIndexedTransformationBuffer.h"

//CCLib
#include <ParallelSort.h>

ccIndexedTransformationBuffer::ccIndexedTransformationBuffer(const QString& name)
	: ccHObject(name)
	, m_bBoxValidSize(0)
	, m_showAsPolyline(false)
	, m_showTrihedrons(true)
	, m_trihedronsScale(1.0f)
{
	lockVisibility(false);
}

ccIndexedTransformationBuffer::ccIndexedTransformationBuffer(const ccIndexedTransformationBuffer& buffer)
	: ccHObject(buffer)
	, m_bBox(buffer.m_bBox)
	, m_bBoxValidSize(buffer.m_bBoxValidSize)
	, m_showAsPolyline(buffer.m_showAsPolyline)
	, m_showTrihedrons(buffer.m_showTrihedrons)
	, m_trihedronsScale(buffer.m_trihedronsScale)
{
	try
	{
		this->std::vector< ccIndexedTransformation >::operator = (buffer);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccIndexedTransformationBuffer] Failed to copy original content (not enough memory)");
	}
}

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
	ParallelSort(begin(), end(), IndexedSortOperator);
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

	trans1 = trans2 = nullptr;
	if (trans1IndexInBuffer)
		*trans1IndexInBuffer = 0;
	if (trans2IndexInBuffer)
		*trans2IndexInBuffer = 0;

#if defined(_MSC_VER) && _MSC_VER > 1000
	ccIndexedTransformation tIndex; tIndex.setIndex(index);
	ccIndexedTransformationBuffer::const_iterator it = std::lower_bound(begin(),end(),tIndex,IndexedSortOperator);
#else
	ccIndexedTransformationBuffer::const_iterator it = std::lower_bound(begin(),end(),index,IndexCompOperator);
#endif

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
		trans2 = &(*it);
		if (trans2IndexInBuffer)
			*trans2IndexInBuffer = it - begin();
		if (it != begin())
		{
			--it;
			trans1 = &(*it);
			if (trans1IndexInBuffer)
				*trans1IndexInBuffer = it - begin();
		}
	}

	return true;
}

void ccIndexedTransformationBuffer::invalidateBoundingBox()
{
	m_bBox.setValidity(false);
}

ccBBox ccIndexedTransformationBuffer::getOwnBB(bool withGLFeatures/*=false*/)
{
	if (!m_bBox.isValid() || m_bBoxValidSize != size())
	{
		for (ccIndexedTransformationBuffer::const_iterator it=begin(); it!=end(); ++it)
			m_bBox.add(it->getTranslationAsVec3D());

		m_bBoxValidSize = size();
	}

	if (   !withGLFeatures
		|| !m_showTrihedrons
		|| !m_bBox.isValid() )
	{
		return m_bBox;
	}

	ccBBox box = m_bBox;
	box.minCorner() -= CCVector3(m_trihedronsScale,m_trihedronsScale,m_trihedronsScale);
	box.maxCorner() += CCVector3(m_trihedronsScale,m_trihedronsScale,m_trihedronsScale);

	return box;
}

bool ccIndexedTransformationBuffer::getInterpolatedTransformation(	double index,
																	ccIndexedTransformation& trans,
																	double maxIndexDistForInterpolation/*=DBL_MAX*/) const
{
	const ccIndexedTransformation *t1 = nullptr;
	const ccIndexedTransformation *t2 = nullptr;

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

				//interpolate
				trans = ccIndexedTransformation::Interpolate(index, *t1, *t2);
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

bool ccIndexedTransformationBuffer::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//vector size (dataVersion>=34)
	uint32_t count = static_cast<uint32_t>(size());
	if (out.write((const char*)&count,4) < 0)
		return WriteError();

	//transformations (dataVersion>=34)
	for (ccIndexedTransformationBuffer::const_iterator it=begin(); it!=end(); ++it)
		if (!it->toFile(out))
			return false;

	//display options
	{
		//Show polyline (dataVersion>=34)
		if (out.write((const char*)&m_showAsPolyline,sizeof(bool)) < 0)
			return WriteError();
		//Show trihedrons (dataVersion>=34)
		if (out.write((const char*)&m_showTrihedrons,sizeof(bool)) < 0)
			return WriteError();
		//Display scale (dataVersion>=34)
		if (out.write((const char*)&m_trihedronsScale,sizeof(float)) < 0)
			return WriteError();
	}

	return true;
}

bool ccIndexedTransformationBuffer::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//vector size (dataVersion>=34)
	uint32_t count = 0;
	if (in.read((char*)&count, 4) < 0)
		return ReadError();

	//try to resize the vector accordingly
	try
	{
		resize(count);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return MemoryError();
	}

	//transformations (dataVersion>=34)
	for (ccIndexedTransformationBuffer::iterator it = begin(); it != end(); ++it)
		if (!it->fromFile(in, dataVersion, flags, oldToNewIDMap))
			return false;

	//display options
	{
		//Show polyline (dataVersion>=34)
		if (in.read((char*)&m_showAsPolyline, sizeof(bool)) < 0)
			return ReadError();
		//Show trihedrons (dataVersion>=34)
		if (in.read((char*)&m_showTrihedrons, sizeof(bool)) < 0)
			return ReadError();
		//Display scale (dataVersion>=34)
		if (in.read((char*)&m_trihedronsScale, sizeof(float)) < 0)
			return ReadError();
	}

	return true;
}

void ccIndexedTransformationBuffer::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	//no picking enabled on trans. buffers
	if (MACRO_DrawEntityNames(context))
		return;
	//only in 3D
	if (!MACRO_Draw3D(context))
		return;
	
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert( glFunc != nullptr );
	
	if ( glFunc == nullptr )
		return;

	size_t count = size();

	//show path
	{
		ccGL::Color4v(glFunc, ccColor::green.rgba);
		glFunc->glBegin(count > 1 && m_showAsPolyline ? GL_LINE_STRIP : GL_POINTS); //show path as a polyline or points?
		for (ccIndexedTransformationBuffer::const_iterator it=begin(); it!=end(); ++it)
			glFunc->glVertex3fv(it->getTranslation());
		glFunc->glEnd();
	}

	//show trihedrons?
	if (m_showTrihedrons)
	{
		for (ccIndexedTransformationBuffer::const_iterator it=begin(); it!=end(); ++it)
		{
			glFunc->glMatrixMode(GL_MODELVIEW);
			glFunc->glPushMatrix();
			glFunc->glMultMatrixf(it->data());

			//force line width
			glFunc->glPushAttrib(GL_LINE_BIT);
			glFunc->glLineWidth(2.0f);

			glFunc->glBegin(GL_LINES);
			glFunc->glColor3f(1.0f,0.0f,0.0f);
			glFunc->glVertex3f(0.0f,0.0f,0.0f);
			glFunc->glVertex3f(m_trihedronsScale,0.0f,0.0f);
			glFunc->glColor3f(0.0f,1.0f,0.0f);
			glFunc->glVertex3f(0.0f,0.0f,0.0f);
			glFunc->glVertex3f(0.0f,m_trihedronsScale,0.0f);
			glFunc->glColor3f(0.0f,0.7f,1.0f);
			glFunc->glVertex3f(0.0f,0.0f,0.0f);
			glFunc->glVertex3f(0.0f,0.0f,m_trihedronsScale);
			glFunc->glEnd();

			glFunc->glPopAttrib(); //GL_LINE_BIT

			glFunc->glPopMatrix();
		}
	}
}
