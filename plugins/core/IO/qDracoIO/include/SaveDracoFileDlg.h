#pragma once

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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include <QDialog>

#include <qCC_io.h>

#include "ui_saveDracoFileDlg.h"

//! DRACO file (https://github.com/google/draco) saving dialog
class SaveDracoFileDlg : public QDialog, public Ui::SaveDracoFileDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit SaveDracoFileDlg(QWidget* parent = nullptr);

	//! Destructor
	virtual ~SaveDracoFileDlg() = default;

	//! Resets default values
	void reset();

protected:

	//! Inits dialog state from persistent settings
	void initFromPersistentSettings();

	//! Saves dialog state from persistent settings
	void saveToPersistentSettings();
};
