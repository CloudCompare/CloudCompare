//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qRDBIO                      #
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
//#          COPYRIGHT: RIEGL Laser Measurement Systems GmbH               #
//#                                                                        #
//##########################################################################

#ifndef RDB_OPEN_DIALOG_HEADER
#define RDB_OPEN_DIALOG_HEADER

//Qt
#include <QDialog>

//GUI
#include "ui_openRDBDlg.h"

enum CC_RDB_OPEN_DLG_TYPES {	RDB_OPEN_DLG_None		= 0,
								RDB_OPEN_DLG_XYZ		= 1,
								RDB_OPEN_DLG_NORM		= 2,
								RDB_OPEN_DLG_RGB		= 3,
								RDB_OPEN_DLG_Grey		= 4,
								RDB_OPEN_DLG_Scalar		= 5,
						   };
const unsigned RDB_OPEN_DLG_TYPES_NUMBER = 6;
const char RDB_OPEN_DLG_TYPES_NAMES[RDB_OPEN_DLG_TYPES_NUMBER][24] = {	"Ignore",
																		"XYZ",
																		"Normals",
																		"RGB",
																		"Grey",
																		"Scalar",
																	 };
//! RDB Open dialog
class RDBOpenDialog : public QDialog, public Ui::RDBOpenDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit RDBOpenDialog(QWidget* parent = nullptr);
};

#endif
