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

#ifndef BDR_SETTING_GRD_FILTER_DLG_HEADER
#define BDR_SETTING_GRD_FILTER_DLG_HEADER

#include "ui_bdrSettingGrdFilterDlg.h"

namespace Ui
{
	class bdrSettingGrdFilterDlg;
}

//! Section extraction tool
class bdrSettingGrdFilterDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrSettingGrdFilterDlg(QWidget* parent);
	//! Destructor
	~bdrSettingGrdFilterDlg() override;

	QStringList getParameters();

protected slots:

private: //members
	Ui::bdrSettingGrdFilterDlg	*m_UI;
};

#endif //BDR_TRACE_FOOTPRINT_HEADER
