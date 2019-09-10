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

#ifndef BDR_CLASS_SETTING_DLG_HEADER
#define BDR_CLASS_SETTING_DLG_HEADER

#include "ui_bdrClassSettingDlg.h"

namespace Ui
{
	class bdrClassSettingDlg;
}

//! Section extraction tool
class bdrClassSettingDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrClassSettingDlg(QWidget* parent);
	//! Destructor
	~bdrClassSettingDlg() override;

protected slots:

private: //members
	Ui::bdrClassSettingDlg	*m_UI;
};

#endif //BDR_TRACE_FOOTPRINT_HEADER
