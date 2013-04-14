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

#include "ccColorScaleEditorDlg.h"

//local
#include "ccColorScaleEditorWidget.h"
#include "ccDisplayOptionsDlg.h"

//qCC_db
#include <ccColorScalesManager.h>
#include <ccScalarField.h>

//Qt
#include <QHBoxLayout>
#include <QColorDialog>
#include <QMessageBox>
#include <QInputDialog>

//System
#include <assert.h>

ccColorScaleEditorDialog::ccColorScaleEditorDialog(ccColorScale::Shared currentScale/*=0*/, QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::ColorScaleEditorDlg()
	, m_colorScale(currentScale)
	, m_scaleWidget(new ccColorScaleEditorWidget(this,Qt::Horizontal))
	, m_associatedSF(0)
	, m_modified(false)
{
    setupUi(this);

	colorScaleEditorFrame->setLayout(new QHBoxLayout());
	colorScaleEditorFrame->layout()->setContentsMargins(0,0,0,0);
	colorScaleEditorFrame->layout()->addWidget(m_scaleWidget);

	//main combo box
	connect(rampComboBox, SIGNAL(activated(int)), this, SLOT(colorScaleChanged(int)));

	//upper buttons
	connect(renameToolButton,		SIGNAL(clicked()),				this,	SLOT(renameCurrentScale()));
	connect(saveToolButton,			SIGNAL(clicked()),				this,	SLOT(saveCurrentScale()));
	connect(deleteToolButton,		SIGNAL(clicked()),				this,	SLOT(deleteCurrentScale()));
	connect(copyToolButton,			SIGNAL(clicked()),				this,	SLOT(copyCurrentScale()));
	connect(newToolButton,			SIGNAL(clicked()),				this,	SLOT(createNewScale()));

	//scale widget
	connect(m_scaleWidget,			SIGNAL(stepSelected(int)),		this,	SLOT(onStepSelected(int)));
	connect(m_scaleWidget,			SIGNAL(stepModified(int)),		this,	SLOT(onStepModified(int)));

	//slider editor
	connect(deleteSliderToolButton,	SIGNAL(clicked()),				this,	SLOT(deletecSelectedStep()));
	connect(colorToolButton,		SIGNAL(clicked()),				this,	SLOT(changeSelectedStepColor()));
	connect(valueDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,	SLOT(changeSelectedStepValue(double)));

	//close button
	connect(closePushButton,		SIGNAL(clicked()),				this,	SLOT(onClose()));

	//TODO FIXME
	scaleModeComboBox->setEnabled(false);


	//populate main combox box with all known scales
	updateMainComboBox();

	if (!m_colorScale)
		m_colorScale = ccColorScalesManager::GetDefaultScale();

	setActiveScale(m_colorScale);
}

ccColorScaleEditorDialog::~ccColorScaleEditorDialog()
{
}

void ccColorScaleEditorDialog::updateMainComboBox()
{
	ccColorScalesManager* csManager = ccColorScalesManager::GetUniqueInstance();
	if (!csManager)
	{
		assert(false);
		return;
	}

	rampComboBox->blockSignals(true);
	rampComboBox->clear();

	//populate combo box with scale names (and UUID)
	assert(csManager);
	for (ccColorScalesManager::ScalesMap::const_iterator it = csManager->map().begin(); it != csManager->map().end(); ++it)
		rampComboBox->addItem((*it)->getName(),(*it)->getUuid());

	//find the currently selected scale in the new 'list'
	int pos = -1;
	if (m_colorScale)
	{
		pos = rampComboBox->findData(m_colorScale->getUuid());
		if (pos < 0) //the current color scale has disappeared?!
			m_colorScale = ccColorScale::Shared(0);
	}
	rampComboBox->setCurrentIndex(pos);

	rampComboBox->blockSignals(false);
}

void ccColorScaleEditorDialog::colorScaleChanged(int pos)
{
	QString UUID = rampComboBox->itemData(pos).toString();
	ccColorScale::Shared colorScale = ccColorScalesManager::GetUniqueInstance()->getScale(UUID);

	setActiveScale(colorScale);
}

void ccColorScaleEditorDialog::setModified(bool state)
{
	m_modified = state;
	saveToolButton->setEnabled(m_modified);
}

bool ccColorScaleEditorDialog::canChangeCurrentScale()
{
	if (!m_colorScale || !m_modified)
		return true;
	
	///ask the user if we should save the current scale?
	QMessageBox::StandardButton button = QMessageBox::warning(this,"Current scale has been modified", "Do you want to save modifications?", QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Cancel);
	if (button == QMessageBox::Yes)
	{
		saveCurrentScale();
	}
	else if (button == QMessageBox::Cancel)
	{
		return false;
	}
	return true;
}

void ccColorScaleEditorDialog::setActiveScale(ccColorScale::Shared currentScale)
{
	//we are gonna change the current scale while it has been modified
	if (m_colorScale != currentScale)
	{
		if (!canChangeCurrentScale())
		{
			//restore old combo-box state
			int pos = rampComboBox->findData(m_colorScale->getUuid());
			if (pos >= 0)
			{
				rampComboBox->blockSignals(true);
				rampComboBox->setCurrentIndex(pos);
				rampComboBox->blockSignals(false);
			}
			else
			{
				assert(false);
			}
			//stop process
			return;
		}
	}

	m_colorScale = currentScale;
	setModified(false);

	//make sure combo-box is up to date
	{
		int pos = rampComboBox->findData(m_colorScale->getUuid());
		if (pos >= 0)
		{
			rampComboBox->blockSignals(true);
			rampComboBox->setCurrentIndex(pos);
			rampComboBox->blockSignals(false);
		}
	}

	//setup dialog components
	bool isLocked = !m_colorScale || m_colorScale->isLocked();
	colorScaleParametersFrame->setEnabled(!isLocked);
	lockWarningLabel->setVisible(isLocked);
	selectedSliderGroupBox->setEnabled(!isLocked);
	m_scaleWidget->setEnabled(!isLocked);

	bool isRelative = !m_colorScale || m_colorScale->isRelative();
	scaleModeComboBox->setCurrentIndex(isRelative ? 0 : 1);
	valueDoubleSpinBox->setSuffix(isRelative ? QString(" %") : QString());
	if (isRelative)
		valueDoubleSpinBox->setRange(0.0,100.0); //between 0 and 100%
	else
		valueDoubleSpinBox->setRange(-DBL_MAX,DBL_MAX);

	m_scaleWidget->importColorScale(m_colorScale);

	onStepSelected(-1);
}

void ccColorScaleEditorDialog::onStepSelected(int index)
{
	selectedSliderGroupBox->setEnabled(/*m_colorScale && !m_colorScale->isLocked() && */index >= 0);

	deleteSliderToolButton->setEnabled(index >= 1 && index+1 < m_scaleWidget->getStepCount()); //don't delete the first and last steps!

	if (index < 0)
	{
		valueDoubleSpinBox->blockSignals(true);
		valueDoubleSpinBox->setValue(0.0);
		valueDoubleSpinBox->blockSignals(false);
		ccDisplayOptionsDlg::SetButtonColor(colorToolButton,Qt::gray);
		valueLabel->setVisible(false);
	}
	else
	{
		bool modified = m_modified; //save 'modified' state before calling onStepModified (which will force it to true)
		onStepModified(index);
		setModified(modified); //restore true 'modified' state
	}
}

void ccColorScaleEditorDialog::onStepModified(int index)
{
	if (index < 0)
		return;

	const ColorScaleElementSlider* slider = m_scaleWidget->getStep(index);
	assert(slider);

    ccDisplayOptionsDlg::SetButtonColor(colorToolButton,slider->getColor());
	if (m_colorScale)
	{
		if (m_colorScale->isRelative())
		{
			double relativePos = slider->getValue();
			valueDoubleSpinBox->blockSignals(true);
			valueDoubleSpinBox->setValue(relativePos*100.0);
			valueDoubleSpinBox->blockSignals(false);
			if (m_associatedSF)
			{
				//compute corresponding scalar value for associated SF
				double actualValue = m_associatedSF->getMinSaturation() + relativePos * (m_associatedSF->getMaxSaturation() - m_associatedSF->getMinSaturation());
				valueLabel->setText(QString("(%1)").arg(actualValue));
				valueLabel->setVisible(true);
			}
			else
			{
				valueLabel->setVisible(false);
			}

			valueDoubleSpinBox->setEnabled(index > 0 && index < m_scaleWidget->getStepCount()-1);
		}
		else
		{
			double absoluteValue = slider->getValue();
			valueDoubleSpinBox->blockSignals(true);
			valueDoubleSpinBox->setValue(absoluteValue);
			valueDoubleSpinBox->blockSignals(false);

			if (m_scaleWidget->getStepCount()>1)
			{
				double minVal = m_scaleWidget->getStep(0)->getValue();
				double maxVal = m_scaleWidget->getStep(m_scaleWidget->getStepCount()-1)->getValue();
				double relativePos = (absoluteValue-minVal)/(maxVal-minVal);
				valueLabel->setText(QString("(%1 %%)").arg(relativePos*100.0));
				valueLabel->setVisible(true);
			}
			else
			{
				valueLabel->setVisible(false);
			}

			valueDoubleSpinBox->setEnabled(true);
		}

		setModified(true);

	}		
}

void ccColorScaleEditorDialog::deletecSelectedStep()
{
	int selectedIndex = m_scaleWidget->getSelectedStepIndex();
	if (selectedIndex >= 1 && selectedIndex+1 < m_scaleWidget->getStepCount()) //don't delete the first and last steps!
	{
		m_scaleWidget->deleteStep(selectedIndex);
		setModified(true);
	}
}

void ccColorScaleEditorDialog::changeSelectedStepColor()
{
	int selectedIndex = m_scaleWidget->getSelectedStepIndex();
	if (selectedIndex<0)
		return;

	const ColorScaleElementSlider* slider = m_scaleWidget->getStep(selectedIndex);
	assert(slider);

	QColor newCol = QColorDialog::getColor(slider->getColor(), this);
    if (newCol.isValid())
    {
		//eventually onStepModified will be called (and thus m_modified will be updated)
		m_scaleWidget->setStepColor(selectedIndex,newCol);
    }
}

void ccColorScaleEditorDialog::changeSelectedStepValue(double value)
{
	int selectedIndex = m_scaleWidget->getSelectedStepIndex();
	if (selectedIndex<0)
		return;

	const ColorScaleElementSlider* slider = m_scaleWidget->getStep(selectedIndex);
	assert(slider);

	bool isRelative = !m_colorScale || m_colorScale->isRelative();
	if (isRelative)
		value /= 100.0;

	//eventually onStepModified will be called (and thus m_modified will be updated)
	m_scaleWidget->setStepValue(selectedIndex,value);
}

void ccColorScaleEditorDialog::copyCurrentScale()
{
	if (!m_colorScale)
	{
		assert(false);
		return;
	}
	
	ccColorScale::Shared scale(new ccColorScale(m_colorScale->getName()+QString("_copy"),QString(),m_colorScale->isRelative()));
	m_scaleWidget->exportColorScale(scale);

	ccColorScalesManager* csManager = ccColorScalesManager::GetUniqueInstance();
	assert(csManager);
	if (csManager)
		csManager->addScale(scale);

	updateMainComboBox();

	setActiveScale(scale);
}

void ccColorScaleEditorDialog::saveCurrentScale()
{
	if (!m_colorScale || m_colorScale->isLocked())
	{
		assert(false);
		return;
	}

	m_scaleWidget->exportColorScale(m_colorScale);

	setModified(false);
}

void ccColorScaleEditorDialog::renameCurrentScale()
{
	if (!m_colorScale || m_colorScale->isLocked())
	{
		assert(false);
		return;
	}

	QString newName = QInputDialog::getText(this,"Scale name","Name",QLineEdit::Normal,m_colorScale->getName());
	if (!newName.isNull())
	{
		m_colorScale->setName(newName);
		//position in combo box
		int pos = rampComboBox->findData(m_colorScale->getUuid());
		if (pos >= 0)
			//update combo box entry name
			rampComboBox->setItemText(pos,newName);
	}
}

void ccColorScaleEditorDialog::deleteCurrentScale()
{
	if (!m_colorScale || m_colorScale->isLocked())
	{
		assert(false);
		return;
	}

	//ask for confirmation
	if (QMessageBox::warning(	this,
								"Delete scale",
								"Are you sure?",
								QMessageBox::Yes | QMessageBox::No,
								QMessageBox::No) == QMessageBox::No)
	{
		return;
	}

	//backup current scale
	ccColorScale::Shared colorScaleToDelete = m_colorScale;
	setModified(false); //cancel any modification

	//change it by the default one
	setActiveScale(ccColorScalesManager::GetDefaultScale());

	ccColorScalesManager* csManager = ccColorScalesManager::GetUniqueInstance();
	assert(csManager);
	if (csManager)
		csManager->removeScale(colorScaleToDelete->getUuid());

	updateMainComboBox();
}

void ccColorScaleEditorDialog::createNewScale()
{
	ccColorScale::Shared scale(new ccColorScale("New scale"));
	scale->insert(ccColorScaleElement(0.0,Qt::blue),false);
	scale->insert(ccColorScaleElement(1.0,Qt::red),true);

	ccColorScalesManager* csManager = ccColorScalesManager::GetUniqueInstance();
	assert(csManager);
	if (csManager)
		csManager->addScale(scale);

	updateMainComboBox();
	
	setActiveScale(scale);
}

void ccColorScaleEditorDialog::onClose()
{
	if (canChangeCurrentScale())
	{
		accept();
	}
}
