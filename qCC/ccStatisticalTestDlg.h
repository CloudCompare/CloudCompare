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

#ifndef CC_STATISTICAL_TEST_DLG_HEADER
#define CC_STATISTICAL_TEST_DLG_HEADER

#include <ui_statisticalTestDlg.h>

//! Dialog for the Local Statistical Test tool
class ccStatisticalTestDlg : public QDialog, public Ui::StatisticalTestDialog
{
public:

	//! Default constructor (for distributions with up to 3 parameters)
	ccStatisticalTestDlg(	QString param1Label,
							QString param2Label,
							QString param3Label = QString(),
							QString windowTitle = QString(),
							QWidget* parent = 0);

	//! Returns 1st parameter value
	double getParam1() const;
	//! Returns 2nd parameter value
	double getParam2() const;
	//! Returns 3rd parameter value
	double getParam3() const;

	//! Returns the number of neighbors
	int getNeighborsNumber() const;
	//! Returns the associated probability
	double getProba() const;
};

#endif //CC_STATISTICAL_TEST_DLG_HEADER