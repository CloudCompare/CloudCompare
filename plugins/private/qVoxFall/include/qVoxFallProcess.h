//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qVoxFall                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 3 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                 COPYRIGHT: THE UNIVERSITY OF NEWCASTLE                 #
//#                                                                        #
//##########################################################################

#ifndef Q_VOXFALL_PROCESS_HEADER
#define Q_VOXFALL_PROCESS_HEADER

//Local
#include "qVoxFallDialog.h"

//qCC
#include "ccPointCloud.h"


class ccMainAppInterface;

//! VoxFall process
/** See "VoxFall: Non-Parametric Volumetric Change Detection for Rockfalls",
	Farmakis, I., Guccione, D.E., Thoeni, K. and Giacomini, A., 2024,
	Computers and Geosciences
**/
class qVoxFallProcess
{
public:
	
	static bool Compute(const qVoxFallDialog& dlg,
						QString& errorMessage,
						bool allowDialogs,
						QWidget* parentWidget = nullptr,
						ccMainAppInterface* app = nullptr);

};

#endif //Q_VOXFALL_PROCESS_HEADER



