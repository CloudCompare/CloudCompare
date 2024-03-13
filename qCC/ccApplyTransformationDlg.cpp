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
#include "mainwindow.h"
#include "ccUtils.h"
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

//CCCoreLib
#include <CCConst.h>

static QString s_lastMatrix("1.00000000 0.00000000 0.00000000 0.00000000\n0.00000000 1.00000000 0.00000000 0.00000000\n0.00000000 0.00000000 1.00000000 0.00000000\n0.00000000 0.00000000 0.00000000 1.00000000");
static bool s_inverseMatrix = false;
static bool s_applyToGlobal = false;
static int s_currentFormIndex = 0;

//! Dialog to define a dip / dip dir. transformation
class DipDirTransformationDialog : public QDialog, public Ui::DipDirTransformationDialog
{
	Q_OBJECT
	
public:

	DipDirTransformationDialog(QWidget* parent = nullptr) : QDialog(parent) { setupUi(this); }
};

ccApplyTransformationDlg::ccApplyTransformationDlg(QWidget* parent/*=nullptr*/)
	: QDialog(parent)
	, Ui::ApplyTransformationDialog()
{
	setupUi(this);

	helpTextEdit->setVisible(false);

	//restore last state
	matrixTextEdit->setPlainText(s_lastMatrix);
	inverseCheckBox->setChecked(s_inverseMatrix);
	applyToGlobalCheckBox->setChecked(s_applyToGlobal);
	onMatrixTextChange(); //provoke the update of the other forms
	tabWidget->setCurrentIndex(s_currentFormIndex);

	connect(buttonBox,				 &QDialogButtonBox::accepted,	this, &ccApplyTransformationDlg::checkMatrixValidityAndAccept);
	connect(buttonBox,				 &QDialogButtonBox::clicked,	this, &ccApplyTransformationDlg::buttonClicked);

	connect(matrixTextEdit,			 &QPlainTextEdit::textChanged,	this, &ccApplyTransformationDlg::onMatrixTextChange);
	connect(fromFileToolButton,		 &QToolButton::clicked,			this, &ccApplyTransformationDlg::loadFromASCIIFile);
	connect(fromClipboardToolButton, &QToolButton::clicked,			this, &ccApplyTransformationDlg::loadFromClipboard);
	connect(fromDipDipDirToolButton, &QToolButton::clicked,			this, &ccApplyTransformationDlg::initFromDipAndDipDir);

	// Axis and angle
	connect(rxAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onRotAngleValueChanged);
	connect(ryAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onRotAngleValueChanged);
	connect(rzAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onRotAngleValueChanged);
	connect(rAngleDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onRotAngleValueChanged);
	connect(txAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onRotAngleValueChanged);
	connect(tyAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onRotAngleValueChanged);
	connect(tzAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onRotAngleValueChanged);
	connect(scaleDoubleSpinBox,			qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onRotAngleValueChanged);
	connect(setIAxisToolButton,			&QToolButton::clicked,								[this]	{rxAxisDoubleSpinBox->setValue(1.0); ryAxisDoubleSpinBox->setValue(0.0); rzAxisDoubleSpinBox->setValue(0.0); });
	connect(setJAxisToolButton,			&QToolButton::clicked,								[this]	{rxAxisDoubleSpinBox->setValue(0.0); ryAxisDoubleSpinBox->setValue(1.0); rzAxisDoubleSpinBox->setValue(0.0); });
	connect(setKAxisToolButton,			&QToolButton::clicked,								[this]	{rxAxisDoubleSpinBox->setValue(0.0); ryAxisDoubleSpinBox->setValue(0.0); rzAxisDoubleSpinBox->setValue(1.0); });
	connect(pasteAxisToolButton,		&QToolButton::clicked,								this,	&ccApplyTransformationDlg::axisFromClipboard);
	connect(pasteTransToolButton,		&QToolButton::clicked,								this,	&ccApplyTransformationDlg::transFromClipboard);

	// Euler angles
	connect(ePhiDoubleSpinBox,			qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onEulerValueChanged);
	connect(eThetaDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onEulerValueChanged);
	connect(ePsiDoubleSpinBox,			qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onEulerValueChanged);
	connect(etxAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onEulerValueChanged);
	connect(etyAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onEulerValueChanged);
	connect(etzAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onEulerValueChanged);
	connect(eScaleDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccApplyTransformationDlg::onEulerValueChanged);
	connect(pasteEulerAnglesToolButton,	&QToolButton::clicked,								this,	&ccApplyTransformationDlg::eulerAnglesFromClipboard);
	connect(pasteEulerTransToolButton,	&QToolButton::clicked,								this,	&ccApplyTransformationDlg::eulerTransFromClipboard);

	// From-->to axes
	connect(fromXAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccApplyTransformationDlg::onFromToValueChanged);
	connect(fromYAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccApplyTransformationDlg::onFromToValueChanged);
	connect(fromZAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccApplyTransformationDlg::onFromToValueChanged);
	connect(toXAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccApplyTransformationDlg::onFromToValueChanged);
	connect(toYAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccApplyTransformationDlg::onFromToValueChanged);
	connect(toZAxisDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccApplyTransformationDlg::onFromToValueChanged);
	connect(fromToTxAxisDoubleSpinBox,	qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccApplyTransformationDlg::onFromToValueChanged);
	connect(fromToTyAxisDoubleSpinBox,	qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccApplyTransformationDlg::onFromToValueChanged);
	connect(fromToTzAxisDoubleSpinBox,	qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccApplyTransformationDlg::onFromToValueChanged);
	connect(fromToScaleDoubleSpinBox,	qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccApplyTransformationDlg::onFromToValueChanged);
	connect(pasteFromAxisToolButton,	&QToolButton::clicked,								this, &ccApplyTransformationDlg::fromAxisFromClipboard);
	connect(pasteToAxisToolButton,		&QToolButton::clicked,								this, &ccApplyTransformationDlg::toAxisFromClipboard);
	connect(pasteFromToTransToolButton,	&QToolButton::clicked,								this, &ccApplyTransformationDlg::fromToTransFromClipboard);

	connect(setFromAxisIToolButton, &QToolButton::clicked, [this] {fromXAxisDoubleSpinBox->setValue(1.0); fromYAxisDoubleSpinBox->setValue(0.0); fromZAxisDoubleSpinBox->setValue(0.0); });
	connect(setFromAxisJToolButton, &QToolButton::clicked, [this] {fromXAxisDoubleSpinBox->setValue(0.0); fromYAxisDoubleSpinBox->setValue(1.0); fromZAxisDoubleSpinBox->setValue(0.0); });
	connect(setFromAxisKToolButton, &QToolButton::clicked, [this] {fromXAxisDoubleSpinBox->setValue(0.0); fromYAxisDoubleSpinBox->setValue(0.0); fromZAxisDoubleSpinBox->setValue(1.0); });

	connect(setToAxisIToolButton, &QToolButton::clicked, [this] {toXAxisDoubleSpinBox->setValue(1.0); toYAxisDoubleSpinBox->setValue(0.0); toZAxisDoubleSpinBox->setValue(0.0); });
	connect(setToAxisJToolButton, &QToolButton::clicked, [this] {toXAxisDoubleSpinBox->setValue(0.0); toYAxisDoubleSpinBox->setValue(1.0); toZAxisDoubleSpinBox->setValue(0.0); });
	connect(setToAxisKToolButton, &QToolButton::clicked, [this] {toXAxisDoubleSpinBox->setValue(0.0); toYAxisDoubleSpinBox->setValue(0.0); toZAxisDoubleSpinBox->setValue(1.0); });

}

void ccApplyTransformationDlg::onMatrixTextChange()
{
	QString text = matrixTextEdit->toPlainText();
	if (text.contains("["))
	{
		//automatically remove anything between square brackets
		static const QRegExp squareBracketsFilter("\\[([^]]+)\\]");
		text.replace(squareBracketsFilter, "");
		matrixTextEdit->blockSignals(true);
		matrixTextEdit->setPlainText(text);
		matrixTextEdit->blockSignals(false);
	}

	bool valid = false;
	ccGLMatrixd mat = ccGLMatrixd::FromString(text, valid);
	if (valid)
	{
		updateAll(mat, false, true, true, true); //no need to update the current form
	}
}

void ccApplyTransformationDlg::onRotAngleValueChanged(double)
{
	double alpha = 0.0;
	CCVector3d axis(0.0, 0.0, 1.0);
	CCVector3d t(0.0, 0.0, 0.0);
	double scale = 1.0;

	axis.x	= rxAxisDoubleSpinBox->value();
	axis.y	= ryAxisDoubleSpinBox->value();
	axis.z	= rzAxisDoubleSpinBox->value();
	alpha	= CCCoreLib::DegreesToRadians(rAngleDoubleSpinBox->value());
	t.x		= txAxisDoubleSpinBox->value();
	t.y		= tyAxisDoubleSpinBox->value();
	t.z		= tzAxisDoubleSpinBox->value();
	scale	= scaleDoubleSpinBox->value();

	ccGLMatrixd mat;
	mat.initFromParameters(alpha, axis, t);
	if (scale != 1)
	{
		mat.scaleRotation(scale);
	}

	updateAll(mat, true, false, true, true); //no need to update the current form
}

void ccApplyTransformationDlg::onEulerValueChanged(double)
{
	double phi = 0.0;
	double theta = 0.0;
	double psi = 0.0;
	CCVector3d t(0.0, 0.0, 0.0);
	double scale = 1.0;

	phi		= CCCoreLib::DegreesToRadians(ePhiDoubleSpinBox->value());
	theta	= CCCoreLib::DegreesToRadians(eThetaDoubleSpinBox->value());
	psi		= CCCoreLib::DegreesToRadians(ePsiDoubleSpinBox->value());
	t.x		= etxAxisDoubleSpinBox->value();
	t.y		= etyAxisDoubleSpinBox->value();
	t.z		= etzAxisDoubleSpinBox->value();
	scale	= eScaleDoubleSpinBox->value();

	ccGLMatrixd mat;
	mat.initFromParameters(phi, theta, psi, t);
	if (scale != 1)
	{
		mat.scaleRotation(scale);
	}

	updateAll(mat, true, true, false, true); //no need to update the current form
}

void ccApplyTransformationDlg::onFromToValueChanged(double)
{
	CCVector3d fromAxis(0.0, 0.0, 1.0), toAxis(0.0, 0.0, 1.0);
	CCVector3d t(0.0, 0.0, 0.0);
	double scale = 1.0;

	fromAxis.x	= fromXAxisDoubleSpinBox->value();
	fromAxis.y	= fromYAxisDoubleSpinBox->value();
	fromAxis.z	= fromZAxisDoubleSpinBox->value();
	toAxis.x	= toXAxisDoubleSpinBox->value();
	toAxis.y	= toYAxisDoubleSpinBox->value();
	toAxis.z	= toZAxisDoubleSpinBox->value();
	t.x			= fromToTxAxisDoubleSpinBox->value();
	t.y			= fromToTyAxisDoubleSpinBox->value();
	t.z			= fromToTzAxisDoubleSpinBox->value();
	scale		= fromToScaleDoubleSpinBox->value();

	fromAxis.normalize();
	toAxis.normalize();
	ccGLMatrixd mat = ccGLMatrixd::FromToRotation(fromAxis, toAxis);
	mat.setTranslation(t);
	if (scale != 1.0)
	{
		mat.scaleRotation(scale);
	}

	updateAll(mat, true, true, true, false); //no need to update the current form

}

void ccApplyTransformationDlg::updateAll(	const ccGLMatrixd& mat,
											bool textForm/*=true*/,
											bool axisAngleForm/*=true*/,
											bool eulerForm/*=true*/,
											bool fromToForm/*=true*/ )
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
		scaleDoubleSpinBox ->blockSignals(true);

		double alpha = 0.0;
		CCVector3d axis(0.0, 0.0, 1.0);
		CCVector3d t(0.0, 0.0, 0.0);
		double scale = 1.0;
		mat.getParameters(alpha, axis, t, &scale);

		rxAxisDoubleSpinBox->setValue(axis.x);
		ryAxisDoubleSpinBox->setValue(axis.y);
		rzAxisDoubleSpinBox->setValue(axis.z);
		rAngleDoubleSpinBox->setValue(CCCoreLib::RadiansToDegrees(alpha));
		txAxisDoubleSpinBox->setValue(t.x);
		tyAxisDoubleSpinBox->setValue(t.y);
		tzAxisDoubleSpinBox->setValue(t.z);
		scaleDoubleSpinBox ->setValue(scale);

		rxAxisDoubleSpinBox->blockSignals(false);
		ryAxisDoubleSpinBox->blockSignals(false);
		rzAxisDoubleSpinBox->blockSignals(false);
		rAngleDoubleSpinBox->blockSignals(false);
		txAxisDoubleSpinBox->blockSignals(false);
		tyAxisDoubleSpinBox->blockSignals(false);
		tzAxisDoubleSpinBox->blockSignals(false);
		scaleDoubleSpinBox ->blockSignals(false);
	}

	if (eulerForm)
	{
		ePhiDoubleSpinBox   ->blockSignals(true);
		eThetaDoubleSpinBox ->blockSignals(true);
		ePsiDoubleSpinBox   ->blockSignals(true);
		etxAxisDoubleSpinBox->blockSignals(true);
		etyAxisDoubleSpinBox->blockSignals(true);
		etzAxisDoubleSpinBox->blockSignals(true);
		eScaleDoubleSpinBox ->blockSignals(true);

		double phi = 0.0;
		double theta = 0.0;
		double psi = 0.0;
		CCVector3d t(0.0, 0.0, 0.0);
		double scale = 1.0;
		mat.getParameters(phi, theta, psi, t, &scale);

		ePhiDoubleSpinBox   ->setValue(CCCoreLib::RadiansToDegrees(phi));
		eThetaDoubleSpinBox ->setValue(CCCoreLib::RadiansToDegrees(theta));
		ePsiDoubleSpinBox   ->setValue(CCCoreLib::RadiansToDegrees(psi));
		etxAxisDoubleSpinBox->setValue(t.x);
		etyAxisDoubleSpinBox->setValue(t.y);
		etzAxisDoubleSpinBox->setValue(t.z);
		eScaleDoubleSpinBox ->setValue(scale);

		ePhiDoubleSpinBox   ->blockSignals(false);
		eThetaDoubleSpinBox ->blockSignals(false);
		ePsiDoubleSpinBox   ->blockSignals(false);
		etxAxisDoubleSpinBox->blockSignals(false);
		etyAxisDoubleSpinBox->blockSignals(false);
		etzAxisDoubleSpinBox->blockSignals(false);
		eScaleDoubleSpinBox ->blockSignals(false);
	}

	if (fromToForm)
	{
		fromXAxisDoubleSpinBox   ->blockSignals(true);
		fromYAxisDoubleSpinBox   ->blockSignals(true);
		fromZAxisDoubleSpinBox   ->blockSignals(true);
		toXAxisDoubleSpinBox     ->blockSignals(true);
		toYAxisDoubleSpinBox     ->blockSignals(true);
		toZAxisDoubleSpinBox     ->blockSignals(true);
		fromToTxAxisDoubleSpinBox->blockSignals(true);
		fromToTyAxisDoubleSpinBox->blockSignals(true);
		fromToTzAxisDoubleSpinBox->blockSignals(true);
		fromToScaleDoubleSpinBox ->blockSignals(true);

		CCVector3d from(0.0, 0.0, 1.0);
		CCVector3d to = from;
		mat.applyRotation(to);
		double scale = mat.getColumnAsVec3D(0).norm();
		CCVector3d t = mat.getTranslationAsVec3D();

		fromXAxisDoubleSpinBox   ->setValue(from.x);
		fromYAxisDoubleSpinBox   ->setValue(from.y);
		fromZAxisDoubleSpinBox   ->setValue(from.z);
		toXAxisDoubleSpinBox     ->setValue(to.x);
		toYAxisDoubleSpinBox     ->setValue(to.y);
		toZAxisDoubleSpinBox     ->setValue(to.z);
		fromToTxAxisDoubleSpinBox->setValue(t.x);
		fromToTyAxisDoubleSpinBox->setValue(t.y);
		fromToTzAxisDoubleSpinBox->setValue(t.z);
		fromToScaleDoubleSpinBox ->setValue(scale);

		fromXAxisDoubleSpinBox   ->blockSignals(false);
		fromYAxisDoubleSpinBox   ->blockSignals(false);
		fromZAxisDoubleSpinBox   ->blockSignals(false);
		toXAxisDoubleSpinBox     ->blockSignals(false);
		toYAxisDoubleSpinBox     ->blockSignals(false);
		toZAxisDoubleSpinBox     ->blockSignals(false);
		fromToTxAxisDoubleSpinBox->blockSignals(false);
		fromToTyAxisDoubleSpinBox->blockSignals(false);
		fromToTzAxisDoubleSpinBox->blockSignals(false);
		fromToScaleDoubleSpinBox ->blockSignals(false);
	}
}

ccGLMatrixd ccApplyTransformationDlg::getTransformation(bool& applyToGlobal) const
{
	//get current input matrix text
	QString matText = matrixTextEdit->toPlainText();
	//convert it to a ccGLMatrix
	bool valid = false;
	ccGLMatrixd mat = ccGLMatrixd::FromString(matText, valid);
	assert(valid);
	//eventually invert it if necessary
	if (inverseCheckBox->isChecked())
	{
		mat.invert();
	}

	applyToGlobal = applyToGlobalCheckBox->isChecked();

	return mat;
}

void ccApplyTransformationDlg::checkMatrixValidityAndAccept()
{
	//get current input matrix text
	QString matText = matrixTextEdit->toPlainText();
	//convert it to a ccGLMatrix
	bool valid = false;
	ccGLMatrix mat = ccGLMatrix::FromString(matText, valid);

	if (!valid)
	{
		QMessageBox::warning(this, "Invalid matrix", "Matrix is invalid. Make sure to only use white spaces or tabulations between the 16 elements");
		return;
	}

	accept();

	s_lastMatrix = matrixTextEdit->toPlainText();
	s_inverseMatrix = inverseCheckBox->isChecked();
	s_applyToGlobal = applyToGlobalCheckBox->isChecked();
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
		{
			matrixTextEdit->setPlainText(clipText);
		}
		else
		{
			ccLog::Warning("[ccApplyTransformationDlg] Clipboard is empty");
		}
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
	CCVector3d Nd = ccNormalVectors::ConvertDipAndDipDirToNormal(s_dip_deg, s_dipDir_deg);
	//corresponding rotation (assuming we start from (0, 0, 1))

	ccGLMatrixd trans = ccGLMatrixd::FromToRotation(CCVector3d(0.0, 0.0, 1.0), Nd);

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
			CCVector3d C = box.getCenter().toDouble();
			ccGLMatrixd shiftToCenter;
			shiftToCenter.setTranslation(-C);
			ccGLMatrixd backToOrigin;
			backToOrigin.setTranslation(C);
			trans = backToOrigin * trans * shiftToCenter;
		}
	}

	updateAll(trans, true, true, true, true);
}

void ccApplyTransformationDlg::buttonClicked(QAbstractButton* button)
{
	if (buttonBox->buttonRole(button) == QDialogButtonBox::ResetRole)
	{
		updateAll({}, true, true, true, true);
		inverseCheckBox->setChecked(false);
	}
}

void ccApplyTransformationDlg::axisFromClipboard()
{
	CCVector3d vector;
	if (ccUtils::GetVectorFromClipboard(vector))
	{
		rxAxisDoubleSpinBox->setValue(vector.x);
		ryAxisDoubleSpinBox->setValue(vector.y);
		rzAxisDoubleSpinBox->setValue(vector.z);
	}
}

void ccApplyTransformationDlg::transFromClipboard()
{
	CCVector3d vector;
	if (ccUtils::GetVectorFromClipboard(vector))
	{
		txAxisDoubleSpinBox->setValue(vector.x);
		tyAxisDoubleSpinBox->setValue(vector.y);
		tzAxisDoubleSpinBox->setValue(vector.z);
	}
}

void ccApplyTransformationDlg::eulerAnglesFromClipboard()
{
	CCVector3d vector;
	if (ccUtils::GetVectorFromClipboard(vector))
	{
		ePhiDoubleSpinBox->setValue(vector.x);
		eThetaDoubleSpinBox->setValue(vector.y);
		ePsiDoubleSpinBox->setValue(vector.z);
	}
}

void ccApplyTransformationDlg::eulerTransFromClipboard()
{
	CCVector3d vector;
	if (ccUtils::GetVectorFromClipboard(vector))
	{
		etxAxisDoubleSpinBox->setValue(vector.x);
		etyAxisDoubleSpinBox->setValue(vector.y);
		etzAxisDoubleSpinBox->setValue(vector.z);
	}
}

void ccApplyTransformationDlg::fromAxisFromClipboard()
{
	CCVector3d vector;
	if (ccUtils::GetVectorFromClipboard(vector))
	{
		fromXAxisDoubleSpinBox->setValue(vector.x);
		fromYAxisDoubleSpinBox->setValue(vector.y);
		fromZAxisDoubleSpinBox->setValue(vector.z);
	}
}

void ccApplyTransformationDlg::toAxisFromClipboard()
{
	CCVector3d vector;
	if (ccUtils::GetVectorFromClipboard(vector))
	{
		toXAxisDoubleSpinBox->setValue(vector.x);
		toYAxisDoubleSpinBox->setValue(vector.y);
		toZAxisDoubleSpinBox->setValue(vector.z);
	}
}

void ccApplyTransformationDlg::fromToTransFromClipboard()
{
	CCVector3d vector;
	if (ccUtils::GetVectorFromClipboard(vector))
	{
		fromToTxAxisDoubleSpinBox->setValue(vector.x);
		fromToTyAxisDoubleSpinBox->setValue(vector.y);
		fromToTzAxisDoubleSpinBox->setValue(vector.z);
	}
}

#include "ccApplyTransformationDlg.moc"
