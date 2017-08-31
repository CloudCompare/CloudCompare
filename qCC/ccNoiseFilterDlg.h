//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_NOISE_FILTER_DLG_HEADER
#define CC_NOISE_FILTER_DLG_HEADER

#include <ui_noiseFilterDlg.h>

//! Dialog for noise filtering (based on the distance to the implicit local surface)
class ccNoiseFilterDlg : public QDialog, public Ui::NoiseFilterDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccNoiseFilterDlg(QWidget* parent = 0);
};

#endif //CC_NOISE_FILTER_DLG_HEADER
