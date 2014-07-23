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

#include "ccBoundingBoxEditorDlg.h"
#include <limits>

//Box state at last dialog execution
static ccBBox s_lastBBox;

ccBoundingBoxEditorDlg::ccBoundingBoxEditorDlg(QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::BoundingBoxEditorDialog()
	, m_baseBoxIsMinimal(false)
	, m_showInclusionWarning(true)
{
	setupUi(this);

	setWindowFlags(Qt::Tool);

	xDoubleSpinBox->setMinimum(-1.0e9);
	yDoubleSpinBox->setMinimum(-1.0e9);
	zDoubleSpinBox->setMinimum(-1.0e9);
	xDoubleSpinBox->setMaximum( 1.0e9);
	yDoubleSpinBox->setMaximum( 1.0e9);
	zDoubleSpinBox->setMaximum( 1.0e9);

	dxDoubleSpinBox->setMinimum(  0.0);
	dyDoubleSpinBox->setMinimum(  0.0);
	dzDoubleSpinBox->setMinimum(  0.0);
	dxDoubleSpinBox->setMaximum(1.0e9);
	dyDoubleSpinBox->setMaximum(1.0e9);
	dzDoubleSpinBox->setMaximum(1.0e9);

	connect(pointTypeComboBox,	SIGNAL(currentIndexChanged(int)),	this,	SLOT(reflectChanges(int)));
	connect(keepSquareCheckBox,	SIGNAL(toggled(bool)),				this,	SLOT(squareModeActivated(bool)));
	connect(okPushButton,		SIGNAL(clicked()),					this,	SLOT(saveBoxAndAccept()));
	connect(cancelPushButton,	SIGNAL(clicked()),					this,	SLOT(cancel()));
	connect(defaultPushButton,	SIGNAL(clicked()),					this,	SLOT(resetToDefault()));
	connect(lastPushButton,		SIGNAL(clicked()),					this,	SLOT(resetToLast()));

	connect(xDoubleSpinBox,		SIGNAL(valueChanged(double)),		this,	SLOT(updateCurrentBBox(double)));	
	connect(yDoubleSpinBox,		SIGNAL(valueChanged(double)),		this,	SLOT(updateCurrentBBox(double)));	
	connect(zDoubleSpinBox,		SIGNAL(valueChanged(double)),		this,	SLOT(updateCurrentBBox(double)));	

	connect(dxDoubleSpinBox,	SIGNAL(valueChanged(double)),		this,	SLOT(updateXWidth(double)));	
	connect(dyDoubleSpinBox,	SIGNAL(valueChanged(double)),		this,	SLOT(updateYWidth(double)));	
	connect(dzDoubleSpinBox,	SIGNAL(valueChanged(double)),		this,	SLOT(updateZWidth(double)));	

	defaultPushButton->setVisible(false);
	lastPushButton->setVisible(s_lastBBox.isValid());
	checkBaseInclusion();
}

//Helper
void MakeSquare(ccBBox& box, int pivotType, int defaultDim = -1)
{
	assert(defaultDim<3);
	assert(pivotType>=0 && pivotType<3);

	CCVector3 W = box.getDiagVec();
	if (W.x != W.y || W.x != W.z)
	{
		if (defaultDim < 0)
		{
			//we take the largest one!
			defaultDim = 0;
			if (W.u[1]>W.u[defaultDim])
				defaultDim = 1;
			if (W.u[2]>W.u[defaultDim])
				defaultDim = 2;
		}

		CCVector3 newW(W.u[defaultDim],W.u[defaultDim],W.u[defaultDim]);
		switch(pivotType)
		{
		case 0: //min corner
			{
				CCVector3 A = box.minCorner();
				box = ccBBox(A,A+newW);
			}
			break;
		case 1: //center
			{
				CCVector3 C = box.getCenter();
				box = ccBBox(C-newW/2.0,C+newW/2.0);
			}
			break;
		case 2: //max corner
			{
				CCVector3 B = box.maxCorner();
				box = ccBBox(B-newW,B);
			}
			break;
		}
	}
}

bool ccBoundingBoxEditorDlg::keepSquare() const
{
	return keepSquareCheckBox->isChecked();
}

void ccBoundingBoxEditorDlg::forceKeepSquare(bool state)
{
	if (state)
		keepSquareCheckBox->setChecked(true);
	keepSquareCheckBox->setDisabled(state);
}

void ccBoundingBoxEditorDlg::squareModeActivated(bool state)
{
	if (state)
	{
		MakeSquare(m_currentBBox,pointTypeComboBox->currentIndex());
		reflectChanges();
	}
}

void ccBoundingBoxEditorDlg::set2DMode(bool state, unsigned char dim)
{
	bool hideX = (state && dim == 0);
	bool hideY = (state && dim == 1);
	bool hideZ = (state && dim == 2);

	xDoubleSpinBox->setHidden(hideX);
	dxDoubleSpinBox->setHidden(hideX);
	xLabel->setHidden(hideX);

	yDoubleSpinBox->setHidden(hideY);
	dyDoubleSpinBox->setHidden(hideY);
	yLabel->setHidden(hideY);

	zDoubleSpinBox->setHidden(hideZ);
	dzDoubleSpinBox->setHidden(hideZ);
	zLabel->setHidden(hideZ);
}

void ccBoundingBoxEditorDlg::setBaseBBox(const ccBBox& box, bool isMinimal/*=true*/)
{
	//set new default one
	m_initBBox = m_baseBBox = box;
	m_baseBoxIsMinimal = isMinimal;

	defaultPushButton->setVisible(m_baseBBox.isValid());

	resetToDefault();
}

void ccBoundingBoxEditorDlg::checkBaseInclusion()
{
	bool exclude = false;
	if (m_baseBBox.isValid())
	{
		exclude = !m_currentBBox.contains(m_baseBBox.minCorner()) || !m_currentBBox.contains(m_baseBBox.maxCorner());
	}

	warningLabel->setVisible(m_showInclusionWarning && exclude);
	okPushButton->setEnabled(!m_baseBoxIsMinimal || !exclude);
}

void ccBoundingBoxEditorDlg::resetToDefault()
{
	m_currentBBox = m_baseBBox;
	
	if (keepSquare())
		squareModeActivated(true); //will call reflectChanges
	else
		reflectChanges();

	checkBaseInclusion();
}

void ccBoundingBoxEditorDlg::resetToLast()
{
	m_currentBBox = s_lastBBox;
	
	if (keepSquare())
		squareModeActivated(true); //will call reflectChanges
	else
		reflectChanges();

	checkBaseInclusion();
}

void ccBoundingBoxEditorDlg::saveBoxAndAccept()
{
	s_lastBBox = m_currentBBox;
	
	accept();
}

int	ccBoundingBoxEditorDlg::exec()
{
	//backup current box
	m_initBBox = m_currentBBox;

	//call 'true' exec
	return QDialog::exec();
}

void ccBoundingBoxEditorDlg::cancel()
{
	//restore init. box
	m_currentBBox = m_initBBox;

	reject();
}

void ccBoundingBoxEditorDlg::updateXWidth(double value)
{
	updateCurrentBBox(value);
	if (keepSquare())
	{
		MakeSquare(m_currentBBox,pointTypeComboBox->currentIndex(),0);
		reflectChanges();
		//base box (if valid) should always be included!
		if (m_baseBBox.isValid())
			checkBaseInclusion();
	}
}

void ccBoundingBoxEditorDlg::updateYWidth(double value)
{
	updateCurrentBBox(value);
	if (keepSquare())
	{
		MakeSquare(m_currentBBox,pointTypeComboBox->currentIndex(),1);
		reflectChanges();
		//base box (if valid) should always be included!
		if (m_baseBBox.isValid())
			checkBaseInclusion();
	}
}

void ccBoundingBoxEditorDlg::updateZWidth(double value)
{
	updateCurrentBBox(value);
	if (keepSquare())
	{
		MakeSquare(m_currentBBox,pointTypeComboBox->currentIndex(),2);
		reflectChanges();
		//base box (if valid) should always be included!
		if (m_baseBBox.isValid())
			checkBaseInclusion();
	}
}

void ccBoundingBoxEditorDlg::updateCurrentBBox(double dummy)
{
	CCVector3 A(static_cast<PointCoordinateType>(xDoubleSpinBox->value()),
				static_cast<PointCoordinateType>(yDoubleSpinBox->value()),
				static_cast<PointCoordinateType>(zDoubleSpinBox->value()));
	CCVector3 W(static_cast<PointCoordinateType>(dxDoubleSpinBox->value()),
				static_cast<PointCoordinateType>(dyDoubleSpinBox->value()),
				static_cast<PointCoordinateType>(dzDoubleSpinBox->value()));

	switch (pointTypeComboBox->currentIndex())
	{
	case 0: //A = min corner
		m_currentBBox = ccBBox(A,A+W);
		break;
	case 1: //A = center
		m_currentBBox = ccBBox(A-W/2.0,A+W/2.0);
		break;
	case 2: //A = max corner
		m_currentBBox = ccBBox(A-W,A);
		break;
	default:
		assert(false);
		return;
	}

	//base box (if valid) should always be included!
	if (m_baseBBox.isValid())
		checkBaseInclusion();
}

void ccBoundingBoxEditorDlg::reflectChanges(int dummy)
{
	//left column
	{
		xDoubleSpinBox->blockSignals(true);
		yDoubleSpinBox->blockSignals(true);
		zDoubleSpinBox->blockSignals(true);

		switch (pointTypeComboBox->currentIndex())
		{
		case 0: //A = min corner
			{
				const CCVector3& A = m_currentBBox.minCorner();
				xDoubleSpinBox->setValue(A.x);
				yDoubleSpinBox->setValue(A.y);
				zDoubleSpinBox->setValue(A.z);
			}
			break;
		case 1: //A = center
			{
				CCVector3 C = m_currentBBox.getCenter();
				xDoubleSpinBox->setValue(C.x);
				yDoubleSpinBox->setValue(C.y);
				zDoubleSpinBox->setValue(C.z);
			}
			break;
		case 2: //A = max corner
			{
				const CCVector3& B = m_currentBBox.maxCorner();
				xDoubleSpinBox->setValue(B.x);
				yDoubleSpinBox->setValue(B.y);
				zDoubleSpinBox->setValue(B.z);
			}
			break;
		default:
			assert(false);
			return;
		}

		xDoubleSpinBox->blockSignals(false);
		yDoubleSpinBox->blockSignals(false);
		zDoubleSpinBox->blockSignals(false);
	}

	//right column
	{
		dxDoubleSpinBox->blockSignals(true);
		dyDoubleSpinBox->blockSignals(true);
		dzDoubleSpinBox->blockSignals(true);

		CCVector3 W = m_currentBBox.getDiagVec();
		//if 'square mode' is on, all width values should be the same!
		assert(!keepSquare() || fabs(W.x-W.y) < 1.0e-6 && fabs(W.x-W.z) < 1.0e-6);
		dxDoubleSpinBox->setValue(W.x);
		dyDoubleSpinBox->setValue(W.y);
		dzDoubleSpinBox->setValue(W.z);

		dxDoubleSpinBox->blockSignals(false);
		dyDoubleSpinBox->blockSignals(false);
		dzDoubleSpinBox->blockSignals(false);
	}
}
