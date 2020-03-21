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

#ifndef CC_PRIMITIVE_DISTANCE_DIALOG_HEADER
#define CC_PRIMITIVE_DISTANCE_DIALOG_HEADER

//Qt
#include <QDialog>
#include <QString>

#include <ui_primitiveDistanceDlg.h>

class ccHObject;
class ccPointCloud;
class ccGenericPointCloud;
class ccGenericMesh;

//! Dialog for cloud sphere or cloud plane comparison setting
class ccPrimitiveDistanceDlg: public QDialog, public Ui::primitiveDistanceDlg
{
	Q_OBJECT

public:

	//! Default constructor
	ccPrimitiveDistanceDlg(QWidget* parent = nullptr);

	//! Default destructor
	~ccPrimitiveDistanceDlg() = default;

	bool signedDistances() { return signedDistCheckBox->isChecked(); }
	bool flipNormals() { return flipNormalsCheckBox->isChecked(); }
	bool treatPlanesAsBounded() { return treatPlanesAsBoundedCheckBox->isChecked(); }
public slots:
	void applyAndExit();
	void cancelAndExit();


protected slots:
	void toggleSigned(bool);
};

#endif
