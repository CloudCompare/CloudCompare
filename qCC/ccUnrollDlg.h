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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include <QDialog>

//qCC_db
#include <ccPointCloud.h>

namespace Ui
{
	class UnrollDialog;
}

//! Dialog: unroll a cloud on a cylinder or a cone
class ccUnrollDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccUnrollDlg(ccHObject* dbRootEntity, QWidget* parent = nullptr);
	~ccUnrollDlg() override;
	
	ccPointCloud::UnrollMode getType() const;
	CCVector3d getAxis() const;
	bool isAxisPositionAuto() const;
	bool useArbitraryOutputCS() const;
	CCVector3 getAxisPosition() const;
	void getAngleRange(double& start_deg, double& stop_deg) const;
	double getRadius() const;
	double getConeHalfAngle() const;
	bool exportDeviationSF() const;
	double getConicalProjSpanRatio() const;

	void toPersistentSettings() const;
	void fromPersistentSettings();

protected:
	void shapeTypeChanged(int index);
	void projectionTypeChanged(int index);
	void axisDimensionChanged(int index);
	void axisAutoStateChanged(int checkState);
	void loadParametersFromEntity();
	void axisFromClipboard();
	void centerFromClipboard();

protected:
	Ui::UnrollDialog* m_ui;
	ccHObject* m_dbRootEntity;
};
