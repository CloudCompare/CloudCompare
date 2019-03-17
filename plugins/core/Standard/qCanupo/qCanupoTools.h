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

#ifndef Q_CANUPO_TOOLS_HEADER
#define Q_CANUPO_TOOLS_HEADER

//Local
#include "classifier.h"

//CCLib
#include <GenericIndexedCloud.h>
#include <GenericProgressCallback.h>
#include <DgmOctree.h>

//Qt
#include <QString>

//system
#include <vector>

class ccGenericPointCloud;
class ccPointCloud;
class ccMainAppInterface;
class ccHObject;
class QComboBox;
class ccScalarField;

//! Helper: pre-computed cos and sin values between 0 and Pi
template<unsigned Count> struct CosSinTable
{
	float cosines[Count];
	float sines  [Count];

	CosSinTable()
	{
		for (int i=0; i<Count; ++i)
		{
			cosines[i] = static_cast<float>( cos(M_PI * i / Count) );
			sines  [i] = static_cast<float>( sin(M_PI * i / Count) );
		}
	}
};

//! CANUPO generic tools
class qCanupoTools
{
public:

	//! Returns whether two numbers are almost equal or not
	template<typename T> static bool Fpeq(T a, T b, T epsilon = static_cast<T>(1.0e-6))
	{
		if (b == 0)
			return fabs(a) < epsilon;

		return fabs(a/b - 1) <= epsilon;
	}

	//! Compares two vectors
	/** \return true if both are 'equivalent' (i.e. same number of values, almost same values, same order)
	**/
	static bool CompareVectors(const std::vector<float>& first, const std::vector<float>& second);

	//! Tests whether a vector contains the values of another one (at the end!)
	/** \return number of matching elements
	**/
	static size_t TestVectorsOverlap(const std::vector<float>& first, const std::vector<float>& second);

	//! Computes the 'descriptors' for various scales on core points only
	static bool ComputeCorePointsDescriptors(	CCLib::GenericIndexedCloud* corePoints,
												CorePointDescSet& corePointsDescriptors,
												ccGenericPointCloud* sourceCloud,
												const std::vector<float>& sortedScales,
												bool& invalidDescriptors,
												QString& error, //if any
												unsigned descriptorID = DESC_DIMENSIONALITY,
												int maxThreadCount = 0,
												CCLib::GenericProgressCallback* progressCb = 0,
												CCLib::DgmOctree* inputOctree = 0,
												std::vector<ccScalarField*>* roughnessSFs = 0 /*for tests*/); 

	//! Returns a long description of a given entity (name + [ID])
	static QString GetEntityName(ccHObject* obj);

	//! Returns the cloud associated to the currently selected item of a combox box
	/** Relies on the item associated data (itemData) that should be equal to the cloud's ID.
	**/
	static ccPointCloud* GetCloudFromCombo(QComboBox* comboBox, ccHObject* dbRoot);

	//! Trains a classifier
	static bool TrainClassifier(Classifier& classifier,
								const CorePointDescSet& descriptors1,
								const CorePointDescSet& descriptors2,
								const std::vector<float>& scales,
								ccPointCloud* mscCloud,
								const CorePointDescSet* evaluationDescriptors = 0,
								ccMainAppInterface* app = 0);
	
	//! Classifer evaluation parameters
	/** See original CANUPO paper ('5. Results' section).
	**/
	struct EvalParameters
	{
		//! Default constructor
		EvalParameters()
			: false1(0)
			, false2(0)
			, true1(0)
			, true2(0)
			, mu1(0)
			, mu2(0)
			, var1(0)
			, var2(0)
		{}
		//! Falsely positive
		unsigned false1, false2;
		//! Truly positive
		unsigned true1, true2;
		//! Balanced accuracy
		double ba() const
		{
			unsigned n1 = true1+false1;
			unsigned n2 = true2+false2;
			double a1 = (n1 ? static_cast<double>(true1)/static_cast<double>(n1) : 0);
			double a2 = (n2 ? static_cast<double>(true2)/static_cast<double>(n2) : 0);
			return (a1+a2)/2;
		}
		
		//! Mean distance to boundary line
		double mu1, mu2;
		//! Variance to boundary line
		double var1, var2;
		//! Fisher Discriminant Ratio
		double fdr() const
		{
			double v = var1 + var2;
			return v != 0 ? (mu1-mu2)*(mu1-mu2) / v : 0;
		}
	};

	//! Evaluates a classifier
	static bool EvaluateClassifier(	const Classifier& classifier,
									const CorePointDescSet& descriptors1,
									const CorePointDescSet& descriptors2,
									const std::vector<float>& scales,
									EvalParameters& params);
};

#endif //Q_CANUPO_TOOLS_HEADER
