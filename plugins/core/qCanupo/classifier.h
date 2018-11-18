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

/** This file is directly inspired of the equivalently named file in the
original CANUPO project, by N. Brodu and D. Lague.
**/

#ifndef QCANUPO_CLASSIFIER_HEADER
#define QCANUPO_CLASSIFIER_HEADER

//Local
#include "ccPointDescriptor.h"

//CCLib
#include <CCGeom.h>

//Qt
#include <QString>

//system
#include <vector>

//! Classifier
class Classifier
{
public: //methods
	
	//! 2D point
	typedef Vector2Tpl<float> Point2D;

	//! Default constructor
	Classifier();

	//! Checks the ref. points
	/** Exchanges refPointPos and refPointNeg if necessary
		as the user may have moved them.
	**/
	bool checkRefPoints();

	//! Checks numerical condition
	float classify2D_checkcondnum(const Point2D& P, const Point2D& R, float& condnumber) const;

	//! Classification in the 2D space
	float classify2D(const Point2D& P) const;

	//! Projects a parameter vector in (2D) MSC space
	Point2D project(const CorePointDesc& mscdata) const;

	//! Classification in MSC space
	float classify(const CorePointDesc& mscdata) const;

	//! Classifier's file header info
	struct FileHeader
	{
		unsigned classifierCount;
		unsigned dimPerScale;
		unsigned descID;
	};

	//! Loads a CANUPO's classifier file (.prm)
	static bool Load(	QString filename,
						std::vector<Classifier>& classifiers,
						std::vector<float>& scales,
						QString& error,
						FileHeader* header = 0,
						bool headerOnly = false);

	//! Saves classifier as a CANUPO's classifier file (.prm)
	bool save(	QString filename,
				QString& error );

public: //members

	int class1, class2;
	std::vector<float> weightsAxis1, weightsAxis2;
	std::vector<Point2D> path;
	float absMaxXY, axisScaleRatio;
	Point2D refPointPos, refPointNeg;

	//! Associated descriptor ID (see ccPointDescriptor.h)
	unsigned descriptorID;
	
	//! Dimension (per-scale)
	unsigned dimPerScale;

	//! Associated scales
	std::vector<float> scales;
};

#endif //QCANUPO_CLASSIFIER_HEADER
