//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef SQUARE_MATRIX_HEADER
#define SQUARE_MATRIX_HEADER

//local
#include "CCGeom.h"

//system
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

namespace CCLib
{
	//! Square matrix
	/** Row-major ordered matrix (i.e. elements are accessed with 'values[row][column]')
	**/
	template <typename Scalar> class SquareMatrixTpl
	{
	public:

		//! Default constructor
		/** Warning: invalid matrix.
		**/
		SquareMatrixTpl() { init(0); }

		//! Constructor with a given size
		/** \param size the (square) matrix dimension
		**/
		SquareMatrixTpl(unsigned size) { init(size); }

		//! Constructor from another matrix
		/** \param mat matrix
		**/
		SquareMatrixTpl(const SquareMatrixTpl& mat)
		{
			if (init(mat.m_matrixSize))
			{
				*this = mat;
			}
		}

		//! "From OpenGl" constructor (float version)
		/** The matrix dimension is automatically set to 4.
			It can be forced to 3 (size_3 = true). In this
			case, only the rotation part will be 'imported'.
			\param M16f a table of 16 floats (OpenGL float transformation matrix)
			\param rotationOnly consider only the roation part (3x3 matrix)
		**/
		SquareMatrixTpl(const float M16f[], bool rotationOnly = false)
		{
			unsigned size = (rotationOnly ? 3 : 4);

			if (init(size))
			{
				for (unsigned r = 0; r < size; r++)
					for (unsigned c = 0; c < size; c++)
						m_values[r][c] = static_cast<Scalar>(M16f[c * 4 + r]);
			}
		}

		//! "From OpenGl" constructor (double version)
		/** The matrix dimension is automatically set to 4.
			It can be forced to 3 (size_3 = true). In this
			case, only the rotation part will be 'imported'.
			\param M16d a table of 16 floats (OpenGL double transformation matrix)
			\param rotationOnly consider only the roation part (3x3 matrix)
		**/
		SquareMatrixTpl(const double M16d[], bool rotationOnly = false)
		{
			unsigned size = (rotationOnly ? 3 : 4);

			if (init(size))
			{
				for (unsigned r = 0; r < size; r++)
					for (unsigned c = 0; c < size; c++)
						m_values[r][c] = static_cast<Scalar>(M16d[c * 4 + r]);
			}
		}

		//! Default destructor
		virtual ~SquareMatrixTpl()
		{
			invalidate();
		}

		//! Returns matrix size
		inline unsigned size() const { return m_matrixSize; }

		//! Returns matrix validity
		/** Matrix is invalid if its size is 0!
		**/
		inline bool isValid() const { return (m_matrixSize != 0); }

		//! Invalidates matrix
		/** Size is reset to 0.
		**/
		void invalidate()
		{
			delete [] m_underlyingData;
			m_underlyingData = nullptr;
			
			delete [] m_values;
			m_values = nullptr;

			m_matrixSize = 0;
			matrixSquareSize = 0;
		}

		//! The matrix rows
		/** public for easy/fast access
		**/
		Scalar** m_values = nullptr;

		//! Returns pointer to matrix row
		inline Scalar* row(unsigned index) { return m_values[index]; }

		//! Sets a particular matrix value
		void inline setValue(unsigned row, unsigned column, Scalar value)
		{
			m_values[row][column] = value;
		}

		//! Returns a particular matrix value
		Scalar inline getValue(unsigned row, unsigned column) const
		{
			return m_values[row][column];
		}

		//! Matrix copy operator
		SquareMatrixTpl& operator = (const SquareMatrixTpl& B)
		{
			if (m_matrixSize != B.size())
			{
				invalidate();
				init(B.size());
			}

			for (unsigned r = 0; r < m_matrixSize; r++)
				for (unsigned c = 0; c < m_matrixSize; c++)
					m_values[r][c] = B.m_values[r][c];

			return *this;
		}

		//! Addition
		SquareMatrixTpl operator + (const SquareMatrixTpl& B) const
		{
			SquareMatrixTpl C = *this;
			C += B;

			return C;
		}

		//! In-place addition
		const SquareMatrixTpl& operator += (const SquareMatrixTpl& B)
		{
			assert(B.size() == m_matrixSize);

			for (unsigned r = 0; r < m_matrixSize; r++)
				for (unsigned c = 0; c < m_matrixSize; c++)
					m_values[r][c] += B.m_values[r][c];

			return *this;
		}

		//! Subtraction
		SquareMatrixTpl operator - (const SquareMatrixTpl& B) const
		{
			SquareMatrixTpl C = *this;
			C -= B;

			return C;
		}

		//! In-place subtraction
		const SquareMatrixTpl& operator -= (const SquareMatrixTpl& B)
		{
			assert(B.size() == m_matrixSize);

			for (unsigned r = 0; r < m_matrixSize; r++)
				for (unsigned c = 0; c < m_matrixSize; c++)
					m_values[r][c] -= B.m_values[r][c];

			return *this;
		}

		//! Multiplication (M = A*B)
		SquareMatrixTpl operator * (const SquareMatrixTpl& B) const
		{
			assert(B.size() == m_matrixSize);

			SquareMatrixTpl C(m_matrixSize);

			for (unsigned r = 0; r < m_matrixSize; r++)
			{
				for (unsigned c = 0; c < m_matrixSize; c++)
				{
					Scalar sum = 0;
					for (unsigned k = 0; k < m_matrixSize; k++)
						sum += m_values[r][k] * B.m_values[k][c];
					C.m_values[r][c] = sum;
				}
			}

			return C;
		}

		//! Multiplication by a vector
		inline CCVector3 operator * (const CCVector3& V) const
		{
			if (m_matrixSize == 3)
			{

				CCVector3 result;
				apply(V.u, result.u);

				return result;
			}
			else
			{
				return V;
			}
		}

		//! In-place multiplication
		inline const SquareMatrixTpl& operator *= (const SquareMatrixTpl& B)
		{
			*this = (*this) * B;

			return *this;
		}

		//! Multiplication by a vector
		/** Vec must have the same size as matrix.
			Returns result = M.Vec.
		**/
		void apply(const Scalar Vec[], Scalar result[]) const
		{
			for (unsigned r = 0; r < m_matrixSize; r++)
			{
				Scalar sum = 0;
				for (unsigned k = 0; k < m_matrixSize; k++)
					sum += m_values[r][k] * static_cast<Scalar>(Vec[k]);
				result[r] = sum;
			}
		}

		//! In-place transpose
		void transpose()
		{
			for (unsigned r = 0; r < m_matrixSize - 1; r++)
				for (unsigned c = r + 1; c < m_matrixSize; c++)
					std::swap(m_values[r][c], m_values[c][r]);
		}

		//! Returns the transposed version of this matrix
		SquareMatrixTpl transposed() const
		{
			SquareMatrixTpl T(*this);
			T.transpose();

			return T;
		}

		//! Sets all elements to 0
		void clear()
		{
			for (unsigned r = 0; r < m_matrixSize; ++r)
			{
				memset(m_values[r], 0, sizeof(Scalar)*m_matrixSize);
			}
		}

		//! Returns inverse (Gauss)
		SquareMatrixTpl inv() const
		{
			//we create the n by 2n matrix, composed of this matrix and the identity
			Scalar** tempM = nullptr;
			{
				tempM = new Scalar*[m_matrixSize];
				if (!tempM)
				{
					//not enough memory
					return SquareMatrixTpl();
				}
				for (unsigned i = 0; i < m_matrixSize; i++)
				{
					tempM[i] = new Scalar[2 * m_matrixSize];
					if (!tempM[i])
					{
						//not enough memory
						for (unsigned j = 0; j < i; j++)
							delete[] tempM[j];
						delete[] tempM;
						return SquareMatrixTpl();
					}
				}
			}

			//identity
			{
				for (unsigned i = 0; i < m_matrixSize; i++)
				{
					for (unsigned j = 0; j < m_matrixSize; j++)
					{
						tempM[i][j] = m_values[i][j];
						if (i == j)
							tempM[i][j + m_matrixSize] = 1;
						else
							tempM[i][j + m_matrixSize] = 0;
					}
				}
			}

			//Gauss pivot
			{
				for (unsigned i = 0; i < m_matrixSize; i++)
				{
					//we look for the pivot value (first non zero element)
					unsigned j = i;

					while (tempM[j][i] == 0)
					{
						if (++j >= m_matrixSize)
						{
							//non inversible matrix!
							for (unsigned k = 0; k < m_matrixSize; ++k)
								delete[] tempM[k];
							delete[] tempM;
							return SquareMatrixTpl();
						}
					}

					//swap the 2 rows if they are different
					//(we only start by the ith element (as all the others are zero!)
					if (i != j)
						for (unsigned k = i; k < 2 * m_matrixSize; k++)
							std::swap(tempM[i][k], tempM[j][k]);

					//we scale the matrix to make the pivot equal to 1
					if (tempM[i][i] != 1.0)
					{
						const Scalar& tmpVal = tempM[i][i];
						for (unsigned k = i; k < 2 * m_matrixSize; ++k)
							tempM[i][k] /= tmpVal;
					}

					//after the pivot value, all elements are set to zero
					for (unsigned j = i + 1; j < m_matrixSize; j++)
					{
						if (tempM[j][i] != 0)
						{
							const Scalar& tmpVal = tempM[j][i];
							for (unsigned k = i; k < 2 * m_matrixSize; k++)
								tempM[j][k] -= tempM[i][k] * tmpVal;
						}
					}
				}
			}

			//reduction
			{
				for (unsigned i = m_matrixSize - 1; i > 0; i--)
				{
					for (unsigned j = 0; j < i; j++)
					{
						if (tempM[j][i] != 0)
						{
							const Scalar& tmpVal = tempM[j][i];
							for (unsigned k = i; k < 2 * m_matrixSize; k++)
								tempM[j][k] -= tempM[i][k] * tmpVal;
						}
					}
				}
			}

			//result: second part or tempM
			SquareMatrixTpl result(m_matrixSize);
			{
				for (unsigned i = 0; i < m_matrixSize; i++)
					for (unsigned j = 0; j < m_matrixSize; j++)
						result.m_values[i][j] = tempM[i][j + m_matrixSize];
			}

			//we release temp matrix from memory
			{
				for (unsigned i = 0; i < m_matrixSize; i++)
					delete[] tempM[i];
				delete[] tempM;
				tempM = nullptr;
			}

			return result;
		}

		//! Prints out matrix to console or file
		/** \param fp ASCII FILE handle (or 0 to print to console)
		**/
		void print(FILE* fp = nullptr) const
		{
			for (unsigned r = 0; r < m_matrixSize; r++)
			{
				for (unsigned c = 0; c < m_matrixSize; c++)
				{
					if (fp)
						fprintf(fp, "%6.6f ", m_values[r][c]);
					else
						printf("%6.6f ", m_values[r][c]);
				}

				if (fp)
					fprintf(fp, "\n");
				else
					printf("\n");
			}
		}

		//! Sets matrix to identity
		void toIdentity()
		{
			clear();

			for (unsigned r = 0; r < m_matrixSize; r++)
				m_values[r][r] = 1;
		}

		//! Scales matrix (all elements are multiplied by the same coef.)
		void scale(Scalar coef)
		{
			for (unsigned r = 0; r < m_matrixSize; r++)
				for (unsigned c = 0; c < m_matrixSize; c++)
					m_values[r][c] *= coef;
		}

		//! Returns trace
		Scalar trace() const
		{
			Scalar trace = 0;

			for (unsigned r = 0; r < m_matrixSize; r++)
				trace += m_values[r][r];

			return trace;
		}

		//! Returns determinant
		double computeDet() const
		{
			return computeSubDet(m_values, m_matrixSize);
		}

		//! Creates a rotation matrix from a quaternion (float version)
		/** Shortcut to double version of initFromQuaternion)
			\param q normalized quaternion (4 float values)
		**/
		void initFromQuaternion(const float q[])
		{
			double qd[4] = { static_cast<double>(q[0]),
							static_cast<double>(q[1]),
							static_cast<double>(q[2]),
							static_cast<double>(q[3]) };

			initFromQuaternion(qd);
		}

		//! Creates a rotation matrix from a quaternion (double version)
		/** Quaternion is composed of 4 values: an angle (cos(alpha/2))
			and an axis (sin(alpha/2)*unit vector).
			\param q normalized quaternion (w,x,y,z)
		**/
		void initFromQuaternion(const double q[])
		{
			if (m_matrixSize == 0)
				if (!init(3))
					return;
			assert(m_matrixSize == 3);

			double q00 = q[0] * q[0];
			double q11 = q[1] * q[1];
			double q22 = q[2] * q[2];
			double q33 = q[3] * q[3];
			double q03 = q[0] * q[3];
			double q13 = q[1] * q[3];
			double q23 = q[2] * q[3];
			double q02 = q[0] * q[2];
			double q12 = q[1] * q[2];
			double q01 = q[0] * q[1];

			m_values[0][0] = static_cast<Scalar>(q00 + q11 - q22 - q33);
			m_values[1][1] = static_cast<Scalar>(q00 - q11 + q22 - q33);
			m_values[2][2] = static_cast<Scalar>(q00 - q11 - q22 + q33);
			m_values[0][1] = static_cast<Scalar>(2.0*(q12 - q03));
			m_values[1][0] = static_cast<Scalar>(2.0*(q12 + q03));
			m_values[0][2] = static_cast<Scalar>(2.0*(q13 + q02));
			m_values[2][0] = static_cast<Scalar>(2.0*(q13 - q02));
			m_values[1][2] = static_cast<Scalar>(2.0*(q23 - q01));
			m_values[2][1] = static_cast<Scalar>(2.0*(q23 + q01));
		}

		//! Converts rotation matrix to quaternion
		/** Warning: for 3x3 matrix only!
			From libE57 'best practices' (http://www.libe57.org/best.html)
			\param q quaternion (w,x,y,z)
			\return success
		**/
		bool toQuaternion(double q[/*4*/])
		{
			if (m_matrixSize != 3)
				return false;

			double dTrace = static_cast<double>(m_values[0][0])
				+ static_cast<double>(m_values[1][1])
				+ static_cast<double>(m_values[2][2])
				+ 1.0;

			double w, x, y, z;	//quaternion
			if (dTrace > 1.0e-6)
			{
				double S = 2.0 * sqrt(dTrace);
				x = (m_values[2][1] - m_values[1][2]) / S;
				y = (m_values[0][2] - m_values[2][0]) / S;
				z = (m_values[1][0] - m_values[0][1]) / S;
				w = 0.25 * S;
			}
			else if (m_values[0][0] > m_values[1][1] && m_values[0][0] > m_values[2][2])
			{
				double S = sqrt(1.0 + m_values[0][0] - m_values[1][1] - m_values[2][2]) * 2.0;
				x = 0.25 * S;
				y = (m_values[1][0] + m_values[0][1]) / S;
				z = (m_values[0][2] + m_values[2][0]) / S;
				w = (m_values[2][1] - m_values[1][2]) / S;
			}
			else if (m_values[1][1] > m_values[2][2])
			{
				double S = sqrt(1.0 + m_values[1][1] - m_values[0][0] - m_values[2][2]) * 2.0;
				x = (m_values[1][0] + m_values[0][1]) / S;
				y = 0.25 * S;
				z = (m_values[2][1] + m_values[1][2]) / S;
				w = (m_values[0][2] - m_values[2][0]) / S;
			}
			else
			{
				double S = sqrt(1.0 + m_values[2][2] - m_values[0][0] - m_values[1][1]) * 2.0;
				x = (m_values[0][2] + m_values[2][0]) / S;
				y = (m_values[2][1] + m_values[1][2]) / S;
				z = 0.25 * S;
				w = (m_values[1][0] - m_values[0][1]) / S;
			}

			// normalize the quaternion if the matrix is not a clean rigid body matrix or if it has scaler information.
			double len = sqrt(w*w + x*x + y*y + z*z);
			if (len != 0)
			{
				q[0] = w / len;
				q[1] = x / len;
				q[2] = y / len;
				q[3] = z / len;

				return true;
			}

			return false;
		}

		//! Returns Delta-determinant (see Kramer formula)
		Scalar deltaDeterminant(unsigned column, Scalar* Vec) const
		{
			SquareMatrixTpl mat(m_matrixSize);

			for (unsigned i = 0; i < m_matrixSize; i++)
			{
				if (column == i)
				{
					for (unsigned j = 0; j < m_matrixSize; j++)
					{
						mat.m_values[j][i] = static_cast<Scalar>(*Vec);
						Vec++;
					}
				}
				else
				{
					for (unsigned j = 0; j < m_matrixSize; j++)
						mat.m_values[j][i] = m_values[j][i];
				}
			}

			return mat.computeDet();
		}


		//! Converts a 3*3 or 4*4 matrix to an OpenGL-style float matrix (float[16])
		void toGlMatrix(float M16f[]) const
		{
			assert(m_matrixSize == 3 || m_matrixSize == 4);
			memset(M16f, 0, sizeof(float) * 16);

			for (unsigned r = 0; r < 3; r++)
				for (unsigned c = 0; c < 3; c++)
					M16f[r + c * 4] = static_cast<float>(m_values[r][c]);

			if (m_matrixSize == 4)
				for (unsigned r = 0; r < 3; r++)
				{
					M16f[12 + r] = static_cast<float>(m_values[3][r]);
					M16f[3 + r * 4] = static_cast<float>(m_values[r][3]);
				}

			M16f[15] = 1.0f;
		}

		//! Converts a 3*3 or 4*4 matrix to an OpenGL-style double matrix (double[16])
		void toGlMatrix(double M16d[]) const
		{
			assert(m_matrixSize == 3 || m_matrixSize == 4);
			memset(M16d, 0, sizeof(double) * 16);

			for (unsigned r = 0; r < 3; r++)
				for (unsigned c = 0; c < 3; c++)
					M16d[r + c * 4] = static_cast<double>(m_values[r][c]);

			if (m_matrixSize == 4)
			{
				for (unsigned r = 0; r < 3; r++)
				{
					M16d[12 + r] = static_cast<double>(m_values[3][r]);
					M16d[3 + r * 4] = static_cast<double>(m_values[r][3]);
				}
			}

			M16d[15] = 1.0;
		}

	private:
		//! Internal initialization
		/** \return initilization success
		**/
		bool init(unsigned size)
		{
			m_matrixSize = size;
			matrixSquareSize = m_matrixSize*m_matrixSize;

			if ( size == 0 )
			{
				return true;
			}
			
			m_values = new Scalar*[m_matrixSize]{};
			m_underlyingData = new Scalar[matrixSquareSize]{};
			
			if ( (m_values == nullptr) || (m_underlyingData == nullptr) )
			{
				return false;
			}
									
			for (unsigned i = 0; i < m_matrixSize; i++)
			{
				m_values[i] = m_underlyingData + (i * m_matrixSize);
			}			
			
			return true;
		}

		//! Computes sub-matrix determinant
		double computeSubDet(Scalar** mat, unsigned matSize) const
		{
			if (matSize == 2)
			{
				return static_cast<double>(mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]);
			}

			Scalar** subMat = new Scalar*[matSize - 1];
			if (subMat)
			{
				double subDet = 0;
				double sign = 1.0;

				for (unsigned row = 0; row < matSize; row++)
				{
					unsigned k = 0;
					for (unsigned i = 0; i < matSize; i++)
						if (i != row)
							subMat[k++] = mat[i] + 1;

					subDet += static_cast<double>(mat[row][0]) * computeSubDet(subMat, matSize - 1) * sign;
					sign = -sign;
				}

				delete[] subMat;
				return subDet;
			}
			else
			{
				//not enough memory
				return 0.0;
			}
		}

		//! Matrix size
		unsigned m_matrixSize;

		//! Matrix square-size
		unsigned matrixSquareSize;
		
		//! Stores the actual data, indexed by m_values
		Scalar	*m_underlyingData = nullptr;
	};

	//! Default CC square matrix type (PointCoordinateType)
	using SquareMatrix = SquareMatrixTpl<PointCoordinateType>;

	//! Float square matrix type
	using SquareMatrixf = SquareMatrixTpl<float>;

	//! Double square matrix type
	using SquareMatrixd = SquareMatrixTpl<double>;

} //namespace CCLib

#endif //SQUARE_MATRIX_HEADER
