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
#ifndef STATISTICALOUTLIERSREMOVER_H
#define STATISTICALOUTLIERSREMOVER_H

#include "BaseFilter.h"

class SORDialog;

class StatisticalOutliersRemover : public BaseFilter
{
public:
	StatisticalOutliersRemover();
	virtual ~StatisticalOutliersRemover();

protected:
	int compute();
	int openInputDialog();
	void getParametersFromDialog();

	SORDialog* m_dialog;
	bool m_dialogHasParent;
	int m_k;
	float m_std;
};

#endif // STATISTICALOUTLIERREMOVER_H
