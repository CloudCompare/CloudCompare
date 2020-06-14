//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCANUPO                       #
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
//#      COPYRIGHT: UEB (UNIVERSITE EUROPEENNE DE BRETAGNE) / CNRS         #
//#                                                                        #
//##########################################################################

#include "ccPointDescriptor.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>

//CCLib
#include <Neighbourhood.h>
#include <Jacobi.h>

//Qt
#include <QMap>

//system
#include <fstream>

/**** SCALE PARAMETERS COMPUTERS ****/
/*									*/
/*  PUT THE CODE OF YOUR OWN BELOW  */
/*  THEN DECLARE IT IN THE "VAULT"  */
/*  (SEE ScaleParamsComputerVault)  */
/*									*/
/************************************/

//useful constant
static const double SQRT_3_DIV_2 = sqrt(3.0)/2;

//! Per-scale "dimensionality" parameters computer (i.e. same as the original CANUPO suite)
class DimensionalityScaleParamsComputer : public ScaleParamsComputer
{
public:

	//! Default constructor
	DimensionalityScaleParamsComputer() : m_firstScale(true) {}

	//inherited from ScaleParamsComputer
	virtual unsigned getID() const { return DESC_DIMENSIONALITY; }

	//inherited from ScaleParamsComputer
	virtual QString getName() const { return "Dimensionality"; }

	//inherited from ScaleParamsComputer
	virtual unsigned dimPerScale() const { return 2; }

	//inherited from ScaleParamsComputer
	virtual void reset()
	{
		//a = 1/3, b = 1/3
		//c = 1 - a - b = 1/3
		//x = b + c/2 = 1/2
		//y = c * SQRT(3)/2 = SQRT(3)/2 * 1/3

		m_defaultParams[0] = 0.5;
		m_defaultParams[1] = SQRT_3_DIV_2 / 3;

		m_firstScale = true;
	}

	//inherited from ScaleParamsComputer
	virtual bool computeScaleParams(CCLib::ReferenceCloud& neighbors, double radius, float params[], bool& invalidScale)
	{
		//PCA analysis
		if (neighbors.size() >= 3)
		{
			CCLib::Neighbourhood Z(&neighbors);

			CCLib::SquareMatrixd eigVectors;
			std::vector<double> eigValues;
			if (Jacobi<double>::ComputeEigenValuesAndVectors(Z.computeCovarianceMatrix(), eigVectors, eigValues, true))
			{
				Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues); //decreasing order of their associated eigenvalues

				double totalVariance = 0;
				CCVector3d sValues(0, 0, 0);
				{
					// contrarily to Brodu's version, here we get directly the eigenvalues!
					for (unsigned j = 0; j < 3; ++j)
					{
						sValues.u[j] = eigValues[j];
						totalVariance += sValues.u[j];
					}
				}
				assert(totalVariance != 0);
				sValues /= totalVariance;

				// Use barycentric coordinates : a for 1D, b for 2D and c for 3D
				// Formula on wikipedia page for barycentric coordinates
				// using directly the triangle in %variance space, they simplify a lot
				double a = std::min<double>(1.0, std::max<double>(0.0, sValues.x - sValues.y));
				double b = std::min<double>(1.0, std::max<double>(0.0, 2 * sValues.x + 4 * sValues.y - 2.0));
				double c = 1.0 - a - b;
				// see original Brodu's code for this transformation
				params[0] = static_cast<float>(b + c / 2);
				params[1] = static_cast<float>(c * SQRT_3_DIV_2);

				//save parameters for next scale
				m_defaultParams[0] = params[0];
				m_defaultParams[1] = params[1];
				m_firstScale = false;
			}
			else if (m_firstScale) //PCA failed at first scale?!
			{
				invalidScale = true;
				params[0] = m_defaultParams[0];
				params[1] = m_defaultParams[1];
			}
		}
		else if (m_firstScale) //less than 3 points at the biggest scale?!
		{
			invalidScale = true;
			params[0] = m_defaultParams[0];
			params[1] = m_defaultParams[1];
		}

		return true;
	}

protected:

	//! Default parameters (or last computed scale's ones!)
	float m_defaultParams[2];
	//! First scale flag
	bool m_firstScale;
};

#ifdef COMPILE_PRIVATE_CANUPO

//! Per-scale "dimensionality" + mean scalar field value parameters computer
class DimensionalityAndSFScaleParamsComputer : public ScaleParamsComputer
{
public:

	//! Default constructor
	DimensionalityAndSFScaleParamsComputer() : m_firstScale(true) {}

	//inherited from ScaleParamsComputer
	virtual unsigned getID() const { return DESC_DIMENSIONALITY_SF; }

	//inherited from ScaleParamsComputer
	virtual QString getName() const { return "Dimensionality + SF"; }

	//inherited from ScaleParamsComputer
	virtual unsigned dimPerScale() const { return 3; }

	//inherited from ScaleParamsComputer
	virtual bool needSF() const { return true; }

	//inherited from ScaleParamsComputer
	virtual void reset()
	{
		//a = 1/3, b = 1/3
		//c = 1 - a - b = 1/3
		//x = b + c/2 = 1/2
		//y = c * SQRT(3)/2 = SQRT(3)/2 * 1/3
		m_defaultParams[0] = 0.5;
		m_defaultParams[1] = SQRT_3_DIV_2 / 3;

		//mean SF value
		m_defaultParams[2] = 0.0;

		m_firstScale = true;
	}

	//inherited from ScaleParamsComputer
	virtual bool computeScaleParams(CCLib::ReferenceCloud& neighbors, double radius, float params[], bool& invalidScale)
	{
		//PCA analysis
		if (neighbors.size() >= 3)
		{
			//first compute the mean SF value
			unsigned validCount = 0;
			double meanVal = 0;
			for (unsigned i=0; i<neighbors.size(); ++i)
			{
				ScalarType val = neighbors.getPointScalarValue(i);
				if (CCLib::ScalarField::ValidValue(val))
				{
					meanVal += val;
					++validCount;
				}
			}

			if (validCount)
			{
				m_defaultParams[2] = static_cast<float>(meanVal / validCount);
			}
			else
			{
				//not enough (valid) SF values
				invalidScale = true;
				params[0] = m_defaultParams[0];
				params[1] = m_defaultParams[1];
				params[2] = m_defaultParams[2];
				return true;
			}

			CCLib::Neighbourhood Z(&neighbors);

			CCLib::SquareMatrixd eigVectors;
			std::vector<double> eigValues;
			if (Jacobi<double>::ComputeEigenValuesAndVectors(Z.computeCovarianceMatrix(), eigVectors, eigValues, true))
			{
				Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues); //decreasing order of their associated eigenvalues

				double totalVariance = 0;
				CCVector3d sValues(0, 0, 0);
				{
					// contrarily to Brodu's version, here we get directly the eigenvalues!
					for (unsigned j = 0; j < 3; ++j)
					{
						sValues.u[j] = eigValues(j);
						totalVariance += sValues.u[j];
					}
				}
				assert(totalVariance != 0);
				sValues /= totalVariance;

				// Use barycentric coordinates : a for 1D, b for 2D and c for 3D
				// Formula on wikipedia page for barycentric coordinates
				// using directly the triangle in %variance space, they simplify a lot
				double a = std::min<double>(1.0, std::max<double>(0.0, sValues.x - sValues.y));
				double b = std::min<double>(1.0, std::max<double>(0.0, 2*sValues.x + 4*sValues.y - 2.0));
				double c = 1.0 - a - b;
				// see original Brodu's code for this transformation
				params[0] = static_cast<float>(b + c/2);
				params[1] = static_cast<float>(c * SQRT_3_DIV_2);

				//save parameters for next scale
				m_defaultParams[0] = params[0];
				m_defaultParams[1] = params[1];
				m_defaultParams[2] = params[2];
				m_firstScale = false;
			}
			else if (m_firstScale) //PCA failed at first scale?!
			{
				invalidScale = true;
				params[0] = m_defaultParams[0];
				params[1] = m_defaultParams[1];
				params[2] = m_defaultParams[2];
			}
		}
		else if (m_firstScale) //less than 3 points at the biggest scale?!
		{
			invalidScale = true;
			params[0] = m_defaultParams[0];
			params[1] = m_defaultParams[1];
			params[2] = m_defaultParams[2];
		}

		return true;
	}

protected:

	//! Default parameters (or last computed scale's ones!)
	float m_defaultParams[3];
	//! First scale flag
	bool m_firstScale;
};

//! Per scale curvature computer
class CurvatureScaleParamsComputer : public ScaleParamsComputer
{
public:

	//! Default constructor
	CurvatureScaleParamsComputer() : m_firstScale(true) {}

	//inherited from ScaleParamsComputer
	virtual unsigned getID() const { return DESC_CURVATURE; }

	//inherited from ScaleParamsComputer
	virtual QString getName() const { return "Gaussian curvature"; }

	//inherited from ScaleParamsComputer
	virtual unsigned dimPerScale() const { return 1; }

	//inherited from ScaleParamsComputer
	virtual void reset()
	{
		m_defaultParams[0] = 0;
		m_firstScale = true;
	}

	//inherited from ScaleParamsComputer
	virtual bool computeScaleParams(CCLib::ReferenceCloud& neighbors, double radius, float params[], bool& invalidScale)
	{
		//Curvature
		if (neighbors.size() >= 6)
		{
			CCLib::Neighbourhood Z(&neighbors);
			params[0] = Z.computeCurvature(*neighbors.getPoint(0), CCLib::Neighbourhood::GAUSSIAN_CURV);

			//save parameters for next scale
			m_defaultParams[0] = params[0];
			m_firstScale = false;
		}
		else if (m_firstScale) //less than 6 points at the biggest scale?!
		{
			invalidScale = true;
			params[0] = m_defaultParams[0];
		}

		return true;
	}

protected:

	//! Default parameters (or last computed scale's ones!)
	float m_defaultParams[1];
	//! First scale flag
	bool m_firstScale;
};

//! Custom parameters computer (TO BE REIMPLEMENTED)
/*class CustomScaleParamsComputer : public ScaleParamsComputer
{
public:

	//! Default constructor
	CustomScaleParamsComputer() : m_firstScale(true) {}

	//inherited from ScaleParamsComputer
	virtual unsigned getID() const { return DESC_CUSTOM; }

	//inherited from ScaleParamsComputer
	virtual QString getName() const { return "Custom"; }

	//inherited from ScaleParamsComputer
	virtual unsigned dimPerScale() const { return 1; }

	//inherited from ScaleParamsComputer
	virtual void reset()
	{
		//FIXME
		m_defaultParams[0] = 0;
		m_firstScale = true;
	}

	//inherited from ScaleParamsComputer
	virtual bool computeScaleParams(CCLib::ReferenceCloud& neighbors, double radius, float params[], bool& invalidScale)
	{
		//Curvature
		if (neighbors.size() >= ?)
		{
			//do something and set the right values for 'params'

			//save parameters for next scale
			m_defaultParams[0] = params[0];
			m_defaultParams[1] = params[1];
			...
			m_firstScale = false;
		}
		else if (m_firstScale) //less than 6 points at the biggest scale?!
		{
			invalidScale = true;
			params[0] = m_defaultParams[0];
			params[1] = m_defaultParams[1];
			...
		}

		return true;
	}

protected:

	//! Default parameters (or last computed scale's ones!)
	float m_defaultParams[?];
	//! First scale flag
	bool m_firstScale;
};
//*/

#endif

/**** SCALE PARAMETERS COMPUTERS VAULT ****/
/*										  */
/*   DON'T FORGET TO ADD YOUR OWN HERE!   */
/*										  */
/******************************************/

struct ScaleParamsComputerVault
{
	//! Default constructor
	ScaleParamsComputerVault()
	{
		//DEFAULT DESCRIPTOR
		map.insert(DESC_DIMENSIONALITY, new DimensionalityScaleParamsComputer);

#ifdef COMPILE_PRIVATE_CANUPO
		map.insert(DESC_DIMENSIONALITY_SF, new DimensionalityAndSFScaleParamsComputer);
		map.insert(DESC_CURVATURE, new CurvatureScaleParamsComputer);
		//ADD YOUR OWN DESCRIPTOR COMPUTERS HERE!
		//map.insert(DESC_CUSTOM, new CustomScaleParamsComputer);
		//map.insert(DESC_XXX, new XXXScaleParamsComputer);
#endif
	}

	//! Destructor
	~ScaleParamsComputerVault()
	{
		//auto delete computers
		for (QMap<unsigned, ScaleParamsComputer*>::Iterator it = map.begin(); it != map.end(); ++it)
		{
			delete it.value();
			it.value() = 0;
		}
		map.clear();
	}

	QMap<unsigned, ScaleParamsComputer*> map;
};


/********** OTHER STUFF ***********/
/*								  */
/*   NO NEED TO LOOK AT THEM ;)   */
/*								  */
/**********************************/

// Unique static instance
static ScaleParamsComputerVault s_vault;

unsigned ScaleParamsComputer::AvailableCount()
{
	return static_cast<unsigned>(s_vault.map.size());
}

ScaleParamsComputer* ScaleParamsComputer::GetByID(unsigned descID)
{
	if (s_vault.map.contains(descID))
	{
		return s_vault.map[descID];
	}
	else
	{
		//couldn't find the corresponding descriptor!
		assert(false);
		return nullptr;
	}
}

ScaleParamsComputer* ScaleParamsComputer::GetByIndex(unsigned index)
{
	QMap<unsigned, ScaleParamsComputer*>::Iterator it = s_vault.map.begin();
	for (unsigned i=0; i<index; ++i)
	{
		++it;
		assert(it != s_vault.map.end());
	}
	return it.value();
}

QByteArray CorePointDescSet::toByteArray() const
{
	int scaleCount = static_cast<int>(m_scales.size());
	int descCount = static_cast<int>(size());

	if (scaleCount == 0 || descCount == 0)
		return QByteArray();

	int totalSize = 4 * sizeof(int) /*header*/ + (scaleCount + descCount * scaleCount * m_dimPerScale) * sizeof(float) /*data*/;

	QByteArray data(totalSize, Qt::Uninitialized);
	
	if (data.capacity() < totalSize) //not enough memory?
		return data;

	char* buffer = data.data();

	//header
	*reinterpret_cast<int*>(buffer) = scaleCount;
	buffer += sizeof(int);
	*reinterpret_cast<int*>(buffer) = descCount;
	buffer += sizeof(int);
	*reinterpret_cast<int*>(buffer) = static_cast<int>(m_descriptorID);
	buffer += sizeof(int);
	*reinterpret_cast<int*>(buffer) = static_cast<int>(m_dimPerScale);
	buffer += sizeof(int);

	//scales
	{
		for (int i=0; i<scaleCount; ++i)
		{
			*reinterpret_cast<float*>(buffer) = m_scales[i];
			buffer += sizeof(float);
		}
	}

	//descriptors
	{
		for (int j=0; j<descCount; ++j)
		{
			const CorePointDesc& desc = at(j);
			assert(desc.params.size() == scaleCount * m_dimPerScale);
			for (size_t i=0; i<desc.params.size(); ++i)
			{
				*reinterpret_cast<float*>(buffer) = desc.params[i];
				buffer += sizeof(float);
			}
		}
	}

	//first scales
	return data;
}

bool CorePointDescSet::fromByteArray(const QByteArray& data)
{
	if (data.size() < 2*sizeof(int))
		return false;

	const char* buffer = data.data();

	//header
	int scaleCount = *reinterpret_cast<const int*>(buffer);
	buffer += sizeof(int);
	int descCount = *reinterpret_cast<const int*>(buffer);
	buffer += sizeof(int);
	int descriptorID = *reinterpret_cast<const int*>(buffer);
	buffer += sizeof(int);
	if (descriptorID <= 0)
		return false;
	m_descriptorID = static_cast<unsigned>(descriptorID);
	int dimPerScale = *reinterpret_cast<const int*>(buffer);
	buffer += sizeof(int);
	if (dimPerScale <= 0)
		return false;
	m_dimPerScale = static_cast<unsigned>(dimPerScale);

	if (scaleCount == 0 || descCount == 0)
		return false;

	//check that the array is big enough!
	int totalSize = 4 * sizeof(int) /*header*/ + (scaleCount + descCount * scaleCount * m_dimPerScale) * sizeof(float) /*data*/;
	if (data.size() < totalSize)
		return false;

	//check that we have enough memory
	std::vector<float> scales;
	try
	{
		resize(descCount);
		scales.resize(scaleCount);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	//scales
	{
		for (int i=0; i<scaleCount; ++i)
		{
			scales[i] = *reinterpret_cast<const float*>(buffer);
			buffer += sizeof(float);
		}

		//check that we have even more memory ;)
		if (!setScales(scales))
			return false;
	}

	//descriptors
	{
		for (int j=0; j<descCount; ++j)
		{
			CorePointDesc& desc = at(j);
			assert(desc.params.size() == scaleCount * m_dimPerScale);
			for (size_t i=0; i<scaleCount*m_dimPerScale; ++i)
			{
				desc.params[i] = *reinterpret_cast<const float*>(buffer);
				buffer += sizeof(float);
			}
		}
	}

	return true;
}

bool CorePointDescSet::setScales(const std::vector<float>& scales)
{
	assert(size() != 0);

	//same size as previous scales? Easy...
	if (scales.size() == m_scales.size())
	{
		m_scales = scales;
		return true;
	}

	//otherwise we must resize the scales vector AND the 'params' structures!
	try
	{
		m_scales = scales;
		size_t scaleCount = m_scales.size();
		
		size_t paramPerDesc = scaleCount*m_dimPerScale;
		for (size_t i=0; i<size(); ++i)
			at(i).params.resize(paramPerDesc);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory!
		return false;
	}
	return true;
}


bool CorePointDescSet::loadFromMSC(QString filename, QString& error, ccPointCloud* corePoints/*=0*/)
{
	error.clear();

	std::ifstream mscfile(qPrintable(filename), std::ifstream::binary); //DGM: warning, toStdString doesn't preserve "local" characters

	if (!mscfile.is_open())
	{
		error = "Failed to open input file";
		return false;
	}

	// read the file header
	int ncorepoints;
	mscfile.read((char*)&ncorepoints,sizeof(ncorepoints));
	int nscales_msc;
	mscfile.read((char*)&nscales_msc, sizeof(int));

	//default for CANUPO MSC files!
	m_descriptorID = DESC_DIMENSIONALITY;
	m_dimPerScale = 2;

	// allocate memory
	{
		try
		{
			resize(ncorepoints);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			error = "Not enough memory";
			return false;
		}

		if (corePoints)
		{
			//not enough memory to load core points!
			if (!corePoints->reserve(ncorepoints))
				corePoints = nullptr;
		}
	}

	//read scales
	{
		std::vector<float> scales;
		try
		{
			scales.resize(nscales_msc);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			error = "Not enough memory";
			return false;
		}

		for (int si=0; si<nscales_msc; ++si)
			mscfile.read((char*)&scales[si], sizeof(float));
		
		if (!setScales(scales)) //automatically resize the descriptors 'params' structure
		{
			error = "Not enough memory";
			return false;
		}
	}

	// now load the points and multiscale information from the msc file.
	// Put the points in the cloud, keep the multiscale information in a separate vector matched by point index
	int ptnparams;
	mscfile.read((char*)&ptnparams, sizeof(int));

	std::vector<CCLib::ScalarField*> paramsSf(3,nullptr);
	if (corePoints)
	{
		//above 3, ptnparams contains additional scalars
		for (int i=3; i<ptnparams; ++i)
		{
			int sfIdx = corePoints->addScalarField(qPrintable(QString("scalar #%1").arg(i-2)));
			paramsSf.push_back(sfIdx >= 0 ? corePoints->getScalarField(sfIdx) : nullptr);

			if (sfIdx < 0)
			{
				//just a warning in fact
				error = "Not enough memory to import additional scalars! (they are not used for classification anyway)";
			}
		}
	}

	//vector<float> avg_ndist_max_scale(ncorepoints);
	for (int pt=0; pt<ncorepoints; ++pt)
	{
		float x = 0.0f;
		float y = 0.0f;
		float z = 0.0f;
		mscfile.read((char*)&x, sizeof(float));
		mscfile.read((char*)&y, sizeof(float));
		mscfile.read((char*)&z, sizeof(float));
		if (corePoints)
			corePoints->addPoint(CCVector3(x,y,z));
		if (ptnparams>=4)
		{
			float dummy;
			mscfile.read((char*)&dummy, sizeof(float)); //already ignored in Brodu's code...

			for (int i=4; i<ptnparams; ++i)
			{
				float param;
				mscfile.read((char*)&param, sizeof(float));
			
				if (static_cast<int>(paramsSf.size()) > i)
					paramsSf[i]->addElement(static_cast<ScalarType>(param));
			}
		}

		for (int s=0; s<nscales_msc; ++s)
		{
			float a = 0.0f;
			float b = 0.0f;
			mscfile.read((char*)(&a), sizeof(float));
			mscfile.read((char*)(&b), sizeof(float));
			
			// see original Brodu's code for this transformation
			float c = 1.0f - a - b;
			float x = b + c/2;
			float y = c * sqrt(3.0f)/2;
			at(pt).params[s*2  ] = x;
			at(pt).params[s*2+1] = y;
		}
		// we don't care for number of neighbors at max and min scales
		{
			int dummyInt;
			for (int i=0; i<nscales_msc; ++i)
				mscfile.read((char*)&dummyInt, sizeof(int));
		}
	}

	mscfile.close();

	//take care of scalar fields
	if (corePoints)
	{
		bool first = true;
		for (size_t i=3; i<paramsSf.size(); ++i)
		{
			if (paramsSf[i])
			{
				paramsSf[i]->computeMinAndMax();
				if (first)
				{
					corePoints->setCurrentDisplayedScalarField(corePoints->getScalarFieldIndexByName(paramsSf[i]->getName()));
					corePoints->showSF(true);
					first = true;
				}
			}
		}
	}

	return true;
}
