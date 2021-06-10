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

#include <QDialog>

//qCC_db
#include <ccPointCloud.h>

namespace Ui {
	class UnrollDialog;
}

//! Dialog: unroll clould on a cylinder or a cone
class ccUnrollDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccUnrollDlg(QWidget* parent = nullptr);
	~ccUnrollDlg();
	
	ccPointCloud::UnrollMode getType() const;
	int getAxisDimension() const;
	bool isAxisPositionAuto() const;
	CCVector3 getAxisPosition() const;
	void getAngleRange(double& start_deg, double& stop_deg) const;
	double getRadius() const;
	double getConeHalfAngle() const;
	bool exportDeviationSF() const;

	void toPersistentSettings() const;
	void fromPersistentSettings();

protected:
	void shapeTypeChanged(int index);
	void axisDimensionChanged(int index);
	void axisAutoStateChanged(int checkState);

protected:
	bool coneMode;

	Ui::UnrollDialog* m_ui;
};

#endif
