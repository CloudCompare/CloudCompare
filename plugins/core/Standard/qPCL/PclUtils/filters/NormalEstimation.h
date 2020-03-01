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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#ifndef Q_PCL_PLUGIN_NORMALESTIMATION_HEADER
#define Q_PCL_PLUGIN_NORMALESTIMATION_HEADER

#include "BaseFilter.h"

class NormalEstimationDialog;

class NormalEstimation : public BaseFilter
{
public:
	NormalEstimation();
	virtual ~NormalEstimation();

	//inherited from BaseFilter
	virtual int compute();

protected:

	//inherited from BaseFilter
	virtual int openInputDialog();
	virtual void getParametersFromDialog();

	NormalEstimationDialog* m_dialog;
	bool m_dialogHasParent;
	int m_knn_radius;
	float m_radius;
	bool m_useKnn;
	bool m_overwrite_curvature;
};

#endif // Q_PCL_PLUGIN_NORMALESTIMATION_HEADER
