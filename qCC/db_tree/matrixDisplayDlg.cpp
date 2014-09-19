//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "matrixDisplayDlg.h"

//qCC_gl
#include <ccGuiParameters.h>

MatrixDisplayDlg::MatrixDisplayDlg(QWidget* parent/*=0*/)
	: QWidget(parent)
	, Ui::MatrixDisplayDlg()
{
	setupUi(this);

	show();
}

void MatrixDisplayDlg::fillDialogWith(const ccGLMatrix& mat)
{
	int precision = ccGui::Parameters().displayedNumPrecision;

	//display as 4x4 matrix
	maxTextEdit->setText(mat.toString(precision));

	//display as rotation vector/angle
	{
		PointCoordinateType angle_rad;
		CCVector3 axis3D, t3D;
		mat.getParameters(angle_rad, axis3D, t3D);

		fillDialogWith(CCVector3d::fromArray(axis3D.u),angle_rad,CCVector3d::fromArray(t3D.u),precision);
	}
}

void MatrixDisplayDlg::fillDialogWith(const ccGLMatrixd& mat)
{
	int precision = ccGui::Parameters().displayedNumPrecision;

	//display as 4x4 matrix
	maxTextEdit->setText(mat.toString(precision));

	//display as rotation vector/angle
	{
		double angle_rad;
		CCVector3d axis3D, t3D;
		mat.getParameters(angle_rad, axis3D, t3D);

		fillDialogWith(axis3D,angle_rad,t3D,precision);
	}
}

void MatrixDisplayDlg::fillDialogWith(const CCVector3d& axis, double angle_rad, const CCVector3d& T, int precision)
{
	//center position
	QString centerStr = QString("%0 ; %1 ; %2").arg(T.x,0,'f',precision).arg(T.y,0,'f',precision).arg(T.z,0,'f',precision);
	//rotation axis
	QString axisStr = QString("%0 ; %1 ; %2").arg(axis.x,0,'f',precision).arg(axis.y,0,'f',precision).arg(axis.z,0,'f',precision);
	//rotation angle
	QString angleStr = QString("%1 deg.").arg(angle_rad*CC_RAD_TO_DEG,0,'f',precision);

	axisLabel->setText(axisStr);
	angleLabel->setText(angleStr);
	centerLabel->setText(centerStr);
}

void MatrixDisplayDlg::clear()
{
	maxTextEdit->setText("Invalid transformation");
	axisLabel->setText(QString());
	angleLabel->setText(QString());
	centerLabel->setText(QString());
}
