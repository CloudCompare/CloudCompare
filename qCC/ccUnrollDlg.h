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

#ifndef CC_UNROLL_DLG_HEADER
#define CC_UNROLL_DLG_HEADER

#include <ui_unrollDlg.h>

//CCLib
#include <CCGeom.h>

//! Dialog: unroll clould on a cylinder or a cone
class ccUnrollDlg : public QDialog, public Ui::UnrollDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccUnrollDlg(QWidget* parent = 0);

	//! Projection type
	enum Type { CYLINDER, CONE, STRAIGHTENED_CONE };

	Type getType() const;
	int getAxisDimension() const;
	bool isAxisPositionAuto() const;
	CCVector3 getAxisPosition() const;
	double getRadius() const;
	double getAngle() const;
	bool exportDeviationSF() const;

	void toPersistentSettings() const;
	void fromPersistentSettings();

protected slots:
	void shapeTypeChanged(int index);
	void axisDimensionChanged(int index);
	void axisAutoStateChanged(int checkState);

protected:
	bool coneMode;

};

#endif
