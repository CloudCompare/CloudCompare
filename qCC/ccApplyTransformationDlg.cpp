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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#include "ccApplyTransformationDlg.h"

#include <QMessageBox>

static QString s_lastMatrix;
static bool s_inverseMatrix = false;

ccApplyTransformationDlg::ccApplyTransformationDlg(QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::ApplyTransformationDialog()
{
	setupUi(this);

	helpTextEdit->setVisible(false);

	if (!s_lastMatrix.isEmpty())
	{
		matrixTextEdit->clear();
		matrixTextEdit->appendPlainText(s_lastMatrix);

		inverseCheckBox->setChecked(s_inverseMatrix);
	}

	connect(buttonBox, SIGNAL(accepted()), this, SLOT(checkMatrixValidityAndAccept()));
}

ccGLMatrix ccApplyTransformationDlg::getTransformation() const
{
	assert(checkMatrixValidity());

	QStringList valuesStr = matrixTextEdit->toPlainText().split(QRegExp("\\s+"),QString::SkipEmptyParts);

	ccGLMatrix matrix;
	if (valuesStr.size() == 16)
	{
		float* matValues = matrix.data();
		for (int i=0;i<16;++i)
			matValues[i] = valuesStr[(i%4)*4+(i>>2)].toFloat();
	}

	if (inverseCheckBox->isChecked())
		matrix.invert();

	return matrix;
}

bool ccApplyTransformationDlg::checkMatrixValidity() const
{
	QString matrixStr = matrixTextEdit->toPlainText();
	QStringList valuesStr = matrixStr.split(QRegExp("\\s+"),QString::SkipEmptyParts);

	//check number of values
	if (valuesStr.size() != 16)
		return false;

	//check values
	for (int i=0;i<16;++i)
	{
		bool valid=true;
		valuesStr[i].toFloat(&valid);
		if (!valid)
			return false;
	}

	return true;
}

void ccApplyTransformationDlg::checkMatrixValidityAndAccept()
{
    if (!checkMatrixValidity())
    {
		QMessageBox::warning(this, "Invalid matrix", "Matrix is invalid. Make sure to only use white spaces or tabulations between the 16 elements");
		return;
    }

	accept();

	s_lastMatrix = matrixTextEdit->toPlainText();
	s_inverseMatrix = inverseCheckBox->isChecked();
}
