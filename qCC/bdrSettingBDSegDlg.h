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

#ifndef BDR_SETTING_BDSEG_DLG_HEADER
#define BDR_SETTING_BDSEG_DLG_HEADER

#include "ui_bdrSettingBDSegDlg.h"

namespace Ui
{
	class bdrSettingBDSegDlg;
}

//! Section extraction tool
class bdrSettingBDSegDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrSettingBDSegDlg(QWidget* parent);
	//! Destructor
	~bdrSettingBDSegDlg() override;

	QStringList getParameters();

protected slots:

private: //members
	Ui::bdrSettingBDSegDlg	*m_UI;
};

#endif //BDR_TRACE_FOOTPRINT_HEADER
