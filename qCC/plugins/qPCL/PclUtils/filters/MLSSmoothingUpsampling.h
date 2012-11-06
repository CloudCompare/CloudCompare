//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#               COPYRIGHT: Luca Penasa                                   #
//#                                                                        #
//##########################################################################
//
#ifndef MLSSMOOTHINGUPSAMPLING_H
#define MLSSMOOTHINGUPSAMPLING_H

#include "BaseFilter.h"


class MLSDialog;
struct MLSParameters;

class MLSSmoothingUpsampling : public BaseFilter
{
	Q_OBJECT

public:
	MLSSmoothingUpsampling();
	virtual ~MLSSmoothingUpsampling();

protected:

	//inherited from BaseFilter
	int openDialog();
	int compute();
	void getParametersFromDialog();

	MLSDialog* m_dialog;
	MLSParameters * m_parameters; //We directly store all the parameters here
};

#endif // MLSSMOOTHINGUPSAMPLING_H
