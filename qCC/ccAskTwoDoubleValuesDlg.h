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

#ifndef CC_ASK_TWO_DOUBLE_VALUES_DIALOG_HEADER
#define CC_ASK_TWO_DOUBLE_VALUES_DIALOG_HEADER

#include <ui_askTwoDoubleValuesDlg.h>

//! Dialog to input 2 values with custom labels
class ccAskTwoDoubleValuesDlg : public QDialog, public Ui::AskTwoDoubleValuesDialog
{
	Q_OBJECT
	
public:
	//! Default constructor
	ccAskTwoDoubleValuesDlg(const QString& vName1,
							const QString& vName2,
							double minVal,
							double maxVal,
							double defaultVal1,
							double defaultVal2,
							int precision = 6,
							QString windowTitle = QString(),
							QWidget* parent = nullptr);
};

#endif //CC_ASK_TWO_DOUBLE_VALUES_DIALOG_HEADER
