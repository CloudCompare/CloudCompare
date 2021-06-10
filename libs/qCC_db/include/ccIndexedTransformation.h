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

#ifndef CC_INDEXED_TRANSFORMATION_HEADER
#define CC_INDEXED_TRANSFORMATION_HEADER

//Local
#include "ccGLMatrix.h"

//! A 4x4 'transformation' matrix (column major order) associated to an index (typically a timestamp)
class QCC_DB_LIB_API ccIndexedTransformation : public ccGLMatrix
{
public:

	//! Default constructor
	/** Matrix is set to identity (see ccGLMatrix::toIdentity) by default.
		Index is set to zero by default.
	**/
	ccIndexedTransformation();

	//! Constructor from a transformation matrix
	/** Index is set to zero by default.
		\param matrix transformation matrix
	**/
	ccIndexedTransformation(const ccGLMatrix& matrix);

	//! Constructor from a transformation matrix and an index
	/** \param matrix transformation matrix
		\param index associated index (e.g. timestamp)
	**/
	ccIndexedTransformation(const ccGLMatrix& matrix, double index);

	//! Copy constructor
	ccIndexedTransformation(const ccIndexedTransformation& trans) = default;

	//! Returns associated index (e.g. timestamp)
	inline double getIndex() const { return m_index; }

	//! Sets associated index (e.g. timestamp)
	inline void setIndex(double index) { m_index = index; }

	//! Interpolates two transformations at an absolute position (index)
	/** Warning: interpolation index must lie between the two input matrices indexes!
		\param interpIndex interpolation position (should be between trans1 and trans2 indexes).
		\param trans1 first transformation
		\param trans2 second transformation
	**/
	static ccIndexedTransformation Interpolate(double interpIndex, const ccIndexedTransformation& trans1, const ccIndexedTransformation& trans2);

	//! Multiplication by a ccGLMatrix operator
	ccIndexedTransformation operator * (const ccGLMatrix& mat) const;

	//! (in place) Multiplication by a ccGLMatrix operator
	/** Warning: index is not modified by this operation.
	**/
	ccIndexedTransformation& operator *= (const ccGLMatrix& mat);

	//! Multiplication operator
	//ccIndexedTransformation operator * (const ccIndexedTransformation& mat) const;
	
	//! (in place) Multiplication operator
	/** Warning: index is not modified by this operation.
	**/
	//ccIndexedTransformation& operator *= (const ccIndexedTransformation& trans)

	//! (in place) Translation operator
	/** Warning: index is not modified by this operation.
	**/
	ccIndexedTransformation& operator += (const CCVector3& T);
	//! (in place) Translation operator
	/** Warning: index is not modified by this operation.
	**/
	ccIndexedTransformation& operator -= (const CCVector3& T);

	//! Returns transposed transformation
	/** Warning: index is not modified by this operation.
	**/
	ccIndexedTransformation transposed() const;

	//! Returns inverse transformation
	/** Warning: index is not modified by this operation.
	**/
	ccIndexedTransformation inverse() const;

	//inherited from ccGLMatrix
	bool toAsciiFile(QString filename, int precision = 12) const override;
	bool fromAsciiFile(QString filename) override;

	//inherited from ccSerializableObject
	bool isSerializable() const override { return true; }
	bool toFile(QFile& out) const override;
	bool fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

protected:

	//! Associated index (e.g. timestamp)
	double m_index;
};

#endif //CC_INDEXED_TRANSFORMATION_HEADER
