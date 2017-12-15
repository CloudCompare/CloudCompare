//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qM3C2                       #
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
//#            COPYRIGHT: UNIVERSITE EUROPEENNE DE BRETAGNE                #
//#                                                                        #
//##########################################################################

#ifndef Q_M3C2_PROCESS_HEADER
#define Q_M3C2_PROCESS_HEADER

//Local
#include "qM3C2Dialog.h"

class ccMainAppInterface;

//! M3C2 process
/** See "Accurate 3D comparison of complex topography with terrestrial laser scanner:
	application to the Rangitikei canyon (N-Z)", Lague, D., Brodu, N. and Leroux, J.,
	2013, ISPRS journal of Photogrammmetry and Remote Sensing
**/
class qM3C2Process
{
public:
	
	static bool Compute(const qM3C2Dialog& dlg,
						QString& errorMessage,
						ccPointCloud*& outputCloud,
						bool allowDialogs,
						QWidget* parentWidget = nullptr,
						ccMainAppInterface* app = nullptr);

};

#endif //Q_M3C2_PROCESS_HEADER
