#pragma once

// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include <ui_askThreeDoubleValuesDlg.h>

// Qt
#include <QString>

//! Generic dialog to query 3 (double) values
class ccAskThreeDoubleValuesDlg : public QDialog
    , public Ui::AskThreeDoubleValuesDialog
{
	Q_OBJECT

  public:
	//! Default constructor
	ccAskThreeDoubleValuesDlg(const QString& vName1,
	                          const QString& vName2,
	                          const QString& vName3,
	                          double         minVal,
	                          double         maxVal,
	                          double         defaultVal1,
	                          double         defaultVal2,
	                          double         defaultVal3,
	                          int            precision   = 6,
	                          const QString  windowTitle = QString(),
	                          QWidget*       parent      = nullptr);

	//! Enable the checkbox (bottom-left)
	void showCheckbox(const QString& label, bool state, QString tooltip = QString());

	//! Returns the checkbox state
	bool getCheckboxState() const;
};
