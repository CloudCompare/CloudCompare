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

#ifndef CC_EXPORT_COORD_TO_SF_DLG_HEADER
#define CC_EXPORT_COORD_TO_SF_DLG_HEADER

#include <ui_exportCoordToSFDlg.h>

//! Dialog to choose which dimension(s) (X, Y or Z) should be exported as SF(s)
class ccExportCoordToSFDlg : public QDialog, public Ui::ExportCoordToSFDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccExportCoordToSFDlg(QWidget* parent = 0);

	//! Returns whether X dimension should be exported
	bool exportX() const;
	//! Returns whether Y dimension should be exported
	bool exportY() const;
	//! Returns whether Z dimension should be exported
	bool exportZ() const;
};

#endif //CC_EXPORT_COORD_TO_SF_DLG_HEADER
