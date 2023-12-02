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
//#          COPYRIGHT:  Chris Brown                                       #
//#                                                                        #
//##########################################################################

//Qt
#include <QDialog>

#include <ui_primitiveDistanceDlg.h>

//! Dialog for cloud-to-primitive distances setting
class ccPrimitiveDistanceDlg: public QDialog, public Ui::primitiveDistanceDlg
{
	Q_OBJECT

public:

	//! Default constructor
	ccPrimitiveDistanceDlg(QWidget* parent = nullptr);

	inline bool signedDistances() const { return signedDistCheckBox->isChecked(); }
	inline bool flipNormals() const { return flipNormalsCheckBox->isChecked(); }
	inline bool treatPlanesAsBounded() const { return treatPlanesAsBoundedCheckBox->isChecked(); }
};
