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

#ifndef CC_MATCH_SCALES_DIALOG_HEADER
#define CC_MATCH_SCALES_DIALOG_HEADER

#include <ui_matchScalesDlg.h>

//Local
#include "ccLibAlgorithms.h"



//! Scales matching tool dialog
class ccMatchScalesDlg : public QDialog, public Ui::MatchScalesDialog
{
	Q_OBJECT

public:
	//! Default constructor
	ccMatchScalesDlg(	const ccHObject::Container& entities,
						int defaultSelectedIndex = 0,
						QWidget* parent = 0);

	//! Returns selected index
	int getSelectedIndex() const;

	//! Sets the selected matching algorithm
	void setSelectedAlgorithm(ccLibAlgorithms::ScaleMatchingAlgorithm algorithm);

	//! Returns the selected matching algorithm
	ccLibAlgorithms::ScaleMatchingAlgorithm getSelectedAlgorithm() const;
};

#endif //CC_ENTITY_PICKER_DIALOG_HEADER
