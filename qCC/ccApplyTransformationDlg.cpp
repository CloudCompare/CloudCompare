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

#include "ccApplyTransformationDlg.h"

//Local
#include "ccPersistentSettings.h"
#include "ccAskTwoDoubleValuesDlg.h"
#include "mainwindow.h"
#include "ui_dipDirTransformationDlg.h"

//qCC_db
#include <ccFileUtils.h>
#include <ccNormalVectors.h>

//Qt
#include <QMessageBox>
#include <QSettings>
#include <QFileDialog>
#include <QFileInfo>
#include <QClipboard>

//CCLib
#include <CCConst.h>

static QString s_lastMatrix("1.00000000 0.00000000 0.00000000 0.00000000\n0.00000000 1.00000000 0.00000000 0.00000000\n0.00000000 0.00000000 1.00000000 0.00000000\n0.00000000 0.00000000 0.00000000 1.00000000");
static bool s_inverseMatrix = false;
static int s_currentFormIndex = 0;

//! Dialog to define a dip / dip dir. transformation
class DipDirTransformationDialog : public QDialog, public Ui::DipDirTransformationDialog
{
	Q_OBJECT
	
public:

	DipDirTransformationDialog(QWidget* parent = 0) : QDialog(parent) { setupUi(this); }
};

ccApplyTransformationDlg::ccApplyTransformationDlg(QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::ApplyTransformationDialog()
{
	setupUi(this);

	helpTextEdit->setVisible(false);

	//restore last state
	matrixTextEdit->setPlainText(s_lastMatrix);
	inverseCheckBox->setChecked(s_inverseMatrix);
	onMatrixTextChange(); //provoke the update of the other forms
	tabWidget->setCurrentIndex(s_currentFormIndex);

	connect(buttonBox,				SIGNAL(accepted()),							this,	SLOT(checkMatrixValidityAndAccept()));
	connect(buttonBox,				SIGNAL(clicked(QAbstractButton*)),			this,	SLOT(buttonClicked(QAbstractButton*)));

	connect(matrixTextEdit,			SIGNAL(textChanged()),			this,	SLOT(onMatrixTextChange()));
	connect(fromFileToolButton,		SIGNAL(clicked()),				this,	SLOT(loadFromASCIIFile()));
	connect(fromClipboardToolButton, SIGNAL(clicked()),				this,	SLOT(loadFromClipboard()));
	connect(fromDipDipDirToolButton, SIGNAL(clicked()),				this,	SLOT(initFromDipAndDipDir()));

	connect(rxAxisDoubleSpinBox,	SIGNAL(valueChanged(double)),	this,	SLOT(onRotAngleValueChanged(double)));
	connect(ryAxisDoubleSpinBox,	SIGNAL(valueChanged(double)),	this,	SLOT(onRotAngleValueChanged(double)));
	connect(rzAxisDoubleSpinBox,	SIGNAL(valueChanged(double)),	this,	SLOT(onRotAngleValueChanged(double)));
	connect(rAngleDoubleSpinBox,	SIGNAL(valueChanged(double)),	this,	SLOT(onRotAngleValueChanged(double)));
	connect(txAxisDoubleSpinBox,	SIGNAL(valueChanged(double)),	this,	SLOT(onRotAngleValueChanged(double)));
	connect(tyAxisDoubleSpinBox,	SIGNAL(valueChanged(double)),	this,	SLOT(onRotAngleValueChanged(double)));
	connect(tzAxisDoubleSpinBox,	SIGNAL(valueChanged(double)),	this,	SLOT(onRotAngleValueChanged(double)));

	connect(ePhiDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,	SLOT(onEulerValueChanged(double)));
	connect(eThetaDoubleSpinBox,	SIGNAL(valueChanged(double)),	this,	SLOT(onEulerValueChanged(double)));
	connect(ePsiDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,	SLOT(onEulerValueChanged(double)));
	connect(etxAxisDoubleSpinBox,	SIGNAL(valueChanged(double)),	this,	SLOT(onEulerValueChanged(double)));
	connect(etyAxisDoubleSpinBox,	SIGNAL(valueChanged(double)),	this,	SLOT(onEulerValueChanged(double)));
	connect(etzAxisDoubleSpinBox,	SIGNAL(valueChanged(double)),	this,	SLOT(onEulerValueChanged(double)));
}

void ccApplyTransformationDlg::onMatrixTextChange()
{
	QString text = matrixTextEdit->toPlainText();
	if (text.contains("["))
	{
		//automatically remove anything between square brackets
		static const QRegExp squareBracketsFilter("\\[([^]]+)\\]");
		text.replace(squareBracketsFilter,"");
		matrixTextEdit->blockSignals(true);
		matrixTextEdit->setPlainText(text);
		matrixTextEdit->blockSignals(false);
	}

	bool valid = false;
	ccGLMatrix mat = ccGLMatrix::FromString(text,valid);
	if (valid)
		updateAll(mat, false, true, true); //no need to update the current form
}

void ccApplyTransformationDlg::onRotAngleValueChanged(double)
{
	PointCoordinateType alpha = 0;
	CCVector3 axis,t;

	axis.x	= static_cast<PointCoordinateType>(rxAxisDoubleSpinBox->value());
	axis.y	= static_cast<PointCoordinateType>(ryAxisDoubleSpinBox->value());
	axis.z	= static_cast<PointCoordinateType>(rzAxisDoubleSpinBox->value());
	alpha	= static_cast<PointCoordinateType>(rAngleDoubleSpinBox->value() * CC_DEG_TO_RAD);
	t.x		= static_cast<PointCoordinateType>(txAxisDoubleSpinBox->value());
	t.y		= static_cast<PointCoordinateType>(tyAxisDoubleSpinBox->value());
	t.z		= static_cast<PointCoordinateType>(tzAxisDoubleSpinBox->value());

	ccGLMatrix mat;
	mat.initFromParameters(alpha,axis,t);

	updateAll(mat, true, false, true); //no need to update the current form
}

void ccApplyTransformationDlg::onEulerValueChanged(double)
{
	PointCoordinateType phi,theta,psi = 0;
	CCVector3 t;

	phi		= static_cast<PointCoordinateType>(ePhiDoubleSpinBox->value() * CC_DEG_TO_RAD);
	theta	= static_cast<PointCoordinateType>(eThetaDoubleSpinBox->value() * CC_DEG_TO_RAD);
	psi		= static_cast<PointCoordinateType>(ePsiDoubleSpinBox->value() * CC_DEG_TO_RAD);
	t.x		= static_cast<PointCoordinateType>(etxAxisDoubleSpinBox->value());
	t.y		= static_cast<PointCoordinateType>(etyAxisDoubleSpinBox->value());
	t.z		= static_cast<PointCoordinateType>(etzAxisDoubleSpinBox->value());

	ccGLMatrix mat;
	mat.initFromParameters(phi,theta,psi,t);

	updateAll(mat, true, true, false); //no need to update the current form
}

void ccApplyTransformationDlg::updateAll(const ccGLMatrix& mat, bool textForm/*=true*/, bool axisAngleForm/*=true*/, bool eulerForm/*=true*/)
{
	if (textForm)
	{
		QString matText = mat.toString();
		matrixTextEdit->blockSignals(true);
		matrixTextEdit->setPlainText(matText);
		matrixTextEdit->blockSignals(false);
	}

	if (axisAngleForm)
	{
		rxAxisDoubleSpinBox->blockSignals(true);
		ryAxisDoubleSpinBox->blockSignals(true);
		rzAxisDoubleSpinBox->blockSignals(true);
		rAngleDoubleSpinBox->blockSignals(true);
		txAxisDoubleSpinBox->blockSignals(true);
		tyAxisDoubleSpinBox->blockSignals(true);
		tzAxisDoubleSpinBox->blockSignals(true);

		PointCoordinateType alpha = 0;
		CCVector3 axis,t;
		mat.getParameters(alpha,axis,t);

		rxAxisDoubleSpinBox->setValue(axis.x);
		ryAxisDoubleSpinBox->setValue(axis.y);
		rzAxisDoubleSpinBox->setValue(axis.z);
		rAngleDoubleSpinBox->setValue(alpha * CC_RAD_TO_DEG);
		txAxisDoubleSpinBox->setValue(t.x);
		tyAxisDoubleSpinBox->setValue(t.y);
		tzAxisDoubleSpinBox->setValue(t.z);

		rxAxisDoubleSpinBox->blockSignals(false);
		ryAxisDoubleSpinBox->blockSignals(false);
		rzAxisDoubleSpinBox->blockSignals(false);
		rAngleDoubleSpinBox->blockSignals(false);
		txAxisDoubleSpinBox->blockSignals(false);
		tyAxisDoubleSpinBox->blockSignals(false);
		tzAxisDoubleSpinBox->blockSignals(false);
	}

	if (eulerForm)
	{
		ePhiDoubleSpinBox   ->blockSignals(true);
		eThetaDoubleSpinBox ->blockSignals(true);
		ePsiDoubleSpinBox   ->blockSignals(true);
		etxAxisDoubleSpinBox->blockSignals(true);
		etyAxisDoubleSpinBox->blockSignals(true);
		etzAxisDoubleSpinBox->blockSignals(true);

		PointCoordinateType phi,theta,psi = 0;
		CCVector3 t;
		mat.getParameters(phi,theta,psi,t);

		ePhiDoubleSpinBox   ->setValue(phi * CC_RAD_TO_DEG);
		eThetaDoubleSpinBox ->setValue(theta * CC_RAD_TO_DEG);
		ePsiDoubleSpinBox   ->setValue(psi * CC_RAD_TO_DEG);
		etxAxisDoubleSpinBox->setValue(t.x);
		etyAxisDoubleSpinBox->setValue(t.y);
		etzAxisDoubleSpinBox->setValue(t.z);

		ePhiDoubleSpinBox   ->blockSignals(false);
		eThetaDoubleSpinBox ->blockSignals(false);
		ePsiDoubleSpinBox   ->blockSignals(false);
		etxAxisDoubleSpinBox->blockSignals(false);
		etyAxisDoubleSpinBox->blockSignals(false);
		etzAxisDoubleSpinBox->blockSignals(false);
	}
}

ccGLMatrixd ccApplyTransformationDlg::getTransformation() const
{
	//get current input matrix text
	QString matText = matrixTextEdit->toPlainText();
	//convert it to a ccGLMatrix
	bool valid = false;
	ccGLMatrixd mat = ccGLMatrixd::FromString(matText,valid);
	assert(valid);
	//eventually invert it if necessary
	if (inverseCheckBox->isChecked())
	{
		mat.invert();
	}

	return mat;
}

void ccApplyTransformationDlg::checkMatrixValidityAndAccept()
{
	//get current input matrix text
	QString matText = matrixTextEdit->toPlainText();
	//convert it to a ccGLMatrix
	bool valid = false;
	ccGLMatrix mat = ccGLMatrix::FromString(matText,valid);

	if (!valid)
	{
		QMessageBox::warning(this, "Invalid matrix", "Matrix is invalid. Make sure to only use white spaces or tabulations between the 16 elements");
		return;
	}

	accept();

	s_lastMatrix = matrixTextEdit->toPlainText();
	s_inverseMatrix = inverseCheckBox->isChecked();
	s_currentFormIndex = tabWidget->currentIndex();
}

void ccApplyTransformationDlg::loadFromASCIIFile()
{
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString inputFilename = QFileDialog::getOpenFileName(this, "Select input file", currentPath, "*.txt");
	if (inputFilename.isEmpty())
		return;

	ccGLMatrixd mat;
	if (mat.fromAsciiFile(inputFilename))
	{
		matrixTextEdit->setPlainText(mat.toString());
	}
	else
	{
		ccLog::Error(QString("Failed to load file '%1'").arg(inputFilename));
	}

	//save last loading location
	settings.setValue(ccPS::CurrentPath(), QFileInfo(inputFilename).absolutePath());
	settings.endGroup();
}

void ccApplyTransformationDlg::loadFromClipboard()
{
	QClipboard* clipboard = QApplication::clipboard();
	if (clipboard)
	{
		QString clipText = clipboard->text();
		if (!clipText.isEmpty())
			matrixTextEdit->setPlainText(clipText);
		else
			ccLog::Warning("[ccApplyTransformationDlg] Clipboard is empty");
	}
}

void ccApplyTransformationDlg::initFromDipAndDipDir()
{
	static double s_dip_deg = 0.0;
	static double s_dipDir_deg = 0.0;
	static bool s_rotateAboutCenter = false;
	DipDirTransformationDialog dddDlg(this);
	dddDlg.dipDoubleSpinBox->setValue(s_dip_deg);
	dddDlg.dipDirDoubleSpinBox->setValue(s_dipDir_deg);
	dddDlg.rotateAboutCenterCheckBox->setChecked(s_rotateAboutCenter);

	if (!dddDlg.exec())
	{
		return;
	}

	s_dip_deg = dddDlg.dipDoubleSpinBox->value();
	s_dipDir_deg = dddDlg.dipDirDoubleSpinBox->value();
	s_rotateAboutCenter = dddDlg.rotateAboutCenterCheckBox->isChecked();

	//resulting normal vector
	CCVector3 Nd = ccNormalVectors::ConvertDipAndDipDirToNormal(static_cast<PointCoordinateType>(s_dip_deg), static_cast<PointCoordinateType>(s_dipDir_deg));
	//corresponding rotation (assuming we start from (0, 0, 1))

	ccGLMatrix trans = ccGLMatrix::FromToRotation(CCVector3(0, 0, 1), Nd);

	if (s_rotateAboutCenter && MainWindow::TheInstance())
	{
		const ccHObject::Container& selectedEntities = MainWindow::TheInstance()->getSelectedEntities();
		ccBBox box;
		for (ccHObject* obj : selectedEntities)
		{
			box += obj->getBB_recursive();
		}

		if (box.isValid())
		{
			CCVector3 C = box.getCenter();
			ccGLMatrix shiftToCenter;
			shiftToCenter.setTranslation(-C);
			ccGLMatrix backToOrigin;
			backToOrigin.setTranslation(C);
			trans = backToOrigin * trans * shiftToCenter;
		}
	}

	updateAll(trans, true, true, true);
}

void ccApplyTransformationDlg::buttonClicked(QAbstractButton* button)
{
	if (buttonBox->buttonRole(button) == QDialogButtonBox::ResetRole)
	{
		updateAll(ccGLMatrix(), true, true, true);
		inverseCheckBox->setChecked(false);
	}
}

#include "ccApplyTransformationDlg.moc"
