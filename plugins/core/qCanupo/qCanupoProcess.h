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

#ifndef Q_CANUPO_PROCESS_HEADER
#define Q_CANUPO_PROCESS_HEADER

//Local
#include "ccPointDescriptor.h"

//CCLib
#include <GenericIndexedCloudPersist.h>

//Qt
#include <QString>

class ccMainAppInterface;
class ccPointCloud;
class QWidget;

//! CANUPO process (classify)
class qCanupoProcess
{
public:

	//! Classify parameters
	struct ClassifyParams
	{
		double samplingDist = 0.0;
		int maxThreadCount = 0;
		double confidenceThreshold = 0.0;
		bool useActiveSFForConfidence = true;
		bool generateAdditionalSF = false;
		bool generateRoughnessSF = false;
	};

	//! Classify a point cloud
	static bool Classify(	QString classifierFilename,
							const ClassifyParams& params,
							ccPointCloud* cloud,
							CCLib::GenericIndexedCloudPersist* corePoints,
							CorePointDescSet& corePointsDescriptors,
							ccPointCloud* realCorePoints = nullptr,
							ccMainAppInterface* app = nullptr,
							QWidget* parentWidget = nullptr,
							bool silent = false);
};

#endif //Q_CANUPO_PROCESS_HEADER
