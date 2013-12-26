//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qSRA                         #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                           COPYRIGHT: EDF                               #
//#                                                                        #
//##########################################################################

#ifndef QSRA_PROFILE_IMPORT_DLG_HEADER
#define QSRA_PROFILE_IMPORT_DLG_HEADER

#include "ui_profileImportDlg.h"

//! Dialog for importing a 2D revolution profile (qSRA plugin)
class ProfileImportDlg : public QDialog, public Ui::ProfileImportDlg
{
	Q_OBJECT

public:

	//! Default constructor
    ProfileImportDlg(QWidget* parent = 0);

	//! Returns revolution axis dimension index
	/** \return 0(X), 1(Y) or 2(Z).
	**/
	int getAxisDimension() const;

	//! Sets default filename
	void setDefaultFilename(QString filename);

	//! Returns input filename (on completion)
	QString getFilename() const;

	//! Returns whether poyline shift should be ignored along the revolution axis
	bool ignoreAxisShift() const;

protected slots:

	//! Called when the 'browse' tool button is pressed
	void browseFile();

};

#endif //QSRA_PROFILE_IMPORT_DLG_HEADER
