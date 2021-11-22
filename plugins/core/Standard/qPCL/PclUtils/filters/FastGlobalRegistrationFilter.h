#pragma once

//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include "BaseFilter.h"

//! Filter based on "Q.-Y. Zhou, J. Park, and V. Koltun, Fast Global Registration, ECCV, 2016."
/** See https://github.com/isl-org/FastGlobalRegistration
**/
class FastGlobalRegistrationFilter : public BaseFilter
{
public:
	FastGlobalRegistrationFilter();
	~FastGlobalRegistrationFilter() override;

protected:

	//inherited from BaseFilter
	int compute() override;
	int getParametersFromDialog() override;
	bool checkSelected() const override;
	QString getErrorMessage(int errorCode) const override;

protected: // variables

	std::vector<ccPointCloud*> m_alignedClouds;
	ccPointCloud* m_referenceCloud;
	double m_featureRadius;
};
