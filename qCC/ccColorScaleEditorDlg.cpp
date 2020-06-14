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

#include "ccColorScaleEditorDlg.h"

//local
#include "ccColorScaleEditorWidget.h"
#include "ccPersistentSettings.h"

//common
#include <ccMainAppInterface.h>
#include <ccQtHelpers.h>

//qCC_db
#include <ccColorScalesManager.h>
#include <ccFileUtils.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>

//Qt
#include <QColorDialog>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QSettings>
#include <QUuid>

//System
#include <cassert>

static char s_defaultEmptyCustomListText[] = "(auto)";

ccColorScaleEditorDialog::ccColorScaleEditorDialog(	ccColorScalesManager* manager,
													ccMainAppInterface* mainApp,
													ccColorScale::Shared currentScale/*=0*/,
													QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::ColorScaleEditorDlg()
	, m_manager(manager)
	, m_colorScale(currentScale)
	, m_scaleWidget(new ccColorScaleEditorWidget(this,Qt::Horizontal))
	, m_associatedSF(nullptr)
	, m_modified(false)
	, m_minAbsoluteVal(0.0)
	, m_maxAbsoluteVal(1.0)
	, m_mainApp(mainApp)
{
	assert(m_manager);

	setupUi(this);

	colorScaleEditorFrame->setLayout(new QHBoxLayout());
	colorScaleEditorFrame->layout()->setContentsMargins(0,0,0,0);
	colorScaleEditorFrame->layout()->addWidget(m_scaleWidget);

	//main combo box
	connect(rampComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::activated), this, &ccColorScaleEditorDialog::colorScaleChanged);

	//import/export buttons
	connect(exportToolButton,		&QToolButton::clicked,				this,	&ccColorScaleEditorDialog::exportCurrentScale);
	connect(importToolButton,		&QToolButton::clicked,				this,	&ccColorScaleEditorDialog::importScale);

	//upper buttons
	connect(renameToolButton,		&QToolButton::clicked,				this,	&ccColorScaleEditorDialog::renameCurrentScale);
	connect(saveToolButton,			&QToolButton::clicked,				this,	&ccColorScaleEditorDialog::saveCurrentScale);
	connect(deleteToolButton,		&QToolButton::clicked,				this,	&ccColorScaleEditorDialog::deleteCurrentScale);
	connect(copyToolButton,			&QToolButton::clicked,				this,	&ccColorScaleEditorDialog::copyCurrentScale);
	connect(newToolButton,			&QToolButton::clicked,				this,	&ccColorScaleEditorDialog::createNewScale);
	connect(scaleModeComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::activated),			this,	&ccColorScaleEditorDialog::relativeModeChanged);

	//scale widget
	connect(m_scaleWidget,		&ccColorScaleEditorWidget::stepSelected,this,	&ccColorScaleEditorDialog::onStepSelected);
	connect(m_scaleWidget,		&ccColorScaleEditorWidget::stepModified,this,	&ccColorScaleEditorDialog::onStepModified);

	//slider editor
	connect(deleteSliderToolButton,	&QToolButton::clicked,				this,	&ccColorScaleEditorDialog::deletecSelectedStep);
	connect(colorToolButton,		&QToolButton::clicked,				this,	&ccColorScaleEditorDialog::changeSelectedStepColor);
	connect(valueDoubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this, &ccColorScaleEditorDialog::changeSelectedStepValue);

	//labels list widget
	connect(customLabelsGroupBox, &QGroupBox::toggled,					this, &ccColorScaleEditorDialog::toggleCustomLabelsList);
	connect(customLabelsPlainTextEdit, &QPlainTextEdit::textChanged,	this, &ccColorScaleEditorDialog::onCustomLabelsListChanged);

	//apply button
	connect(applyPushButton, &QPushButton::clicked,				this, &ccColorScaleEditorDialog::onApply);
	//close button
	connect(closePushButton, &QPushButton::clicked,				this, &ccColorScaleEditorDialog::onClose);

	//populate main combox box with all known scales
	updateMainComboBox();

	if (!m_colorScale)
		m_colorScale = m_manager->getDefaultScale(ccColorScalesManager::BGYR);

	setActiveScale(m_colorScale);
}

void ccColorScaleEditorDialog::setAssociatedScalarField(ccScalarField* sf)
{
	m_associatedSF = sf;
	if (m_associatedSF && (!m_colorScale || m_colorScale->isRelative())) //we only update those values if the current scale is not absolute!
	{
		m_minAbsoluteVal = m_associatedSF->getMin();
		m_maxAbsoluteVal = m_associatedSF->getMax();
	}
}

void ccColorScaleEditorDialog::updateMainComboBox()
{
	if (!m_manager)
	{
		assert(false);
		return;
	}

	rampComboBox->blockSignals(true);
	rampComboBox->clear();

	//populate combo box with scale names (and UUID)
	assert(m_manager);
	for (ccColorScalesManager::ScalesMap::const_iterator it = m_manager->map().constBegin(); it != m_manager->map().constEnd(); ++it)
		rampComboBox->addItem((*it)->getName(),(*it)->getUuid());

	//find the currently selected scale in the new 'list'
	int pos = -1;
	if (m_colorScale)
	{
		pos = rampComboBox->findData(m_colorScale->getUuid());
		if (pos < 0) //the current color scale has disappeared?!
			m_colorScale = ccColorScale::Shared(nullptr);
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

void ccColorScaleEditorDialog::relativeModeChanged(int value)
{
	setScaleModeToRelative(value == 0 ? true : false);

	setModified(true);
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

	if (m_colorScale->isLocked())
	{
		assert(false);
		return true;
	}
	
	///ask the user if we should save the current scale?
	QMessageBox::StandardButton button = QMessageBox::warning(this,
																"Current scale has been modified",
																"Do you want to save modifications?",
																QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel,
																QMessageBox::Cancel);
	if (button == QMessageBox::Yes)
	{
		if (!saveCurrentScale())
		{
			return false;
		}
	}
	else if (button == QMessageBox::Cancel)
	{
		return false;
	}
	return true;
}

bool ccColorScaleEditorDialog::isRelativeMode() const
{
	return (scaleModeComboBox->currentIndex() == 0 ? true : false);
}

void ccColorScaleEditorDialog::setActiveScale(ccColorScale::Shared currentScale)
{
	//the user wants to change the current scale while the it has been modified (and potentially not saved)
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
	{
		//locked state
		bool isLocked = !m_colorScale || m_colorScale->isLocked();
		colorScaleParametersFrame->setEnabled(!isLocked);
		exportToolButton->setEnabled(!isLocked);
		lockWarningLabel->setVisible(isLocked);
		selectedSliderGroupBox->setEnabled(!isLocked);
		m_scaleWidget->setEnabled(!isLocked);
		customLabelsGroupBox->blockSignals(true);
		customLabelsGroupBox->setEnabled(!isLocked);
		customLabelsGroupBox->blockSignals(false);

		//absolute or relative mode
		if (m_colorScale)
		{
			bool isRelative = m_colorScale->isRelative();
			if (!isRelative)
			{
				//absolute color scales defines their own boundaries
				m_colorScale->getAbsoluteBoundaries(m_minAbsoluteVal,m_maxAbsoluteVal);
			}
			setScaleModeToRelative(isRelative);
		}
		else
		{
			//shouldn't be accessible anyway....
			assert(isLocked == true);
			setScaleModeToRelative(false);
		}
	}

	//custom labels
	{
		ccColorScale::LabelSet& customLabels = m_colorScale->customLabels();
		if (customLabels.empty())
		{
			customLabelsPlainTextEdit->blockSignals(true);
			customLabelsPlainTextEdit->setPlainText(s_defaultEmptyCustomListText);
			customLabelsPlainTextEdit->blockSignals(false);
		}
		else
		{
			QString text;
			size_t index = 0;
			for (ccColorScale::LabelSet::const_iterator it=customLabels.begin(); it!=customLabels.end(); ++it, ++index)
			{
				if (index != 0)
					text += QString("\n");
				text += QString::number(*it,'f',6);
			}
			customLabelsPlainTextEdit->blockSignals(true);
			customLabelsPlainTextEdit->setPlainText(text);
			customLabelsPlainTextEdit->blockSignals(false);

		}
		customLabelsGroupBox->blockSignals(true);
		customLabelsGroupBox->setChecked(!customLabels.empty());
		customLabelsGroupBox->blockSignals(false);
	}

	m_scaleWidget->importColorScale(m_colorScale);

	onStepSelected(-1);
}

void ccColorScaleEditorDialog::setScaleModeToRelative(bool isRelative)
{
	scaleModeComboBox->setCurrentIndex(isRelative ? 0 : 1);
	valueDoubleSpinBox->setSuffix(isRelative ? QString(" %") : QString());
	valueDoubleSpinBox->blockSignals(true);
	if (isRelative)
		valueDoubleSpinBox->setRange(0.0, 100.0); //between 0 and 100%
	else
		valueDoubleSpinBox->setRange(-1.0e9, 1.0e9);
	valueDoubleSpinBox->blockSignals(false);

	//update selected slider frame
	int selectedIndex = (m_scaleWidget ? m_scaleWidget->getSelectedStepIndex() : -1);
	onStepModified(selectedIndex);
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
		ccQtHelpers::SetButtonColor(colorToolButton,Qt::gray);
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
	if (index < 0 || index >= m_scaleWidget->getStepCount())
		return;

	const ColorScaleElementSlider* slider = m_scaleWidget->getStep(index);
	assert(slider);

	ccQtHelpers::SetButtonColor(colorToolButton,slider->getColor());
	if (m_colorScale)
	{
		const double relativePos = slider->getRelativePos();
		if (isRelativeMode())
		{
			valueDoubleSpinBox->blockSignals(true);
			valueDoubleSpinBox->setValue(relativePos*100.0);
			valueDoubleSpinBox->blockSignals(false);
			if (m_associatedSF)
			{
				//compute corresponding scalar value for associated SF
				double actualValue = m_associatedSF->getMin() + relativePos * (m_associatedSF->getMax() - m_associatedSF->getMin());
				valueLabel->setText(QString("(%1)").arg(actualValue));
				valueLabel->setVisible(true);
			}
			else
			{
				valueLabel->setVisible(false);
			}

			//can't change min and max boundaries in 'relative' mode!
			valueDoubleSpinBox->setEnabled(index > 0 && index < m_scaleWidget->getStepCount()-1);
		}
		else
		{
			//compute corresponding 'absolute' value from current dialog boundaries
			double absoluteValue = m_minAbsoluteVal + relativePos * (m_maxAbsoluteVal - m_minAbsoluteVal);
			
			valueDoubleSpinBox->blockSignals(true);
			valueDoubleSpinBox->setValue(absoluteValue);
			valueDoubleSpinBox->blockSignals(false);
			valueDoubleSpinBox->setEnabled(true);

			//display corresponding relative position as well
			valueLabel->setText(QString("(%1 %)").arg(relativePos*100.0));
			valueLabel->setVisible(true);
		}

		setModified(true);

	}		
}

void ccColorScaleEditorDialog::deletecSelectedStep()
{
	int selectedIndex = m_scaleWidget->getSelectedStepIndex();
	if (selectedIndex >= 1 && selectedIndex+1 < m_scaleWidget->getStepCount()) //never delete the first and last steps!
	{
		m_scaleWidget->deleteStep(selectedIndex);
		setModified(true);
	}
}

void ccColorScaleEditorDialog::changeSelectedStepColor()
{
	int selectedIndex = m_scaleWidget->getSelectedStepIndex();
	if (selectedIndex < 0)
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
	if (!m_scaleWidget)
		return;

	int selectedIndex = m_scaleWidget->getSelectedStepIndex();
	if (selectedIndex < 0)
		return;

	const ColorScaleElementSlider* slider = m_scaleWidget->getStep(selectedIndex);
	assert(slider);

	bool relativeMode = isRelativeMode();
	if (relativeMode)
	{
		assert(selectedIndex != 0 && selectedIndex+1 < m_scaleWidget->getStepCount());

		value /= 100.0; //from percentage to relative position
		assert(value >= 0.0 && value <= 1.0);

		//eventually onStepModified will be called (and thus m_modified will be updated)
		m_scaleWidget->setStepRelativePosition(selectedIndex,value);
	}
	else //absolute scale mode
	{
		//we build up the new list based on absolute values
		SharedColorScaleElementSliders newSliders(new ColorScaleElementSliders());
		{
			for (int i=0;i<m_scaleWidget->getStepCount();++i)
			{
				const ColorScaleElementSlider* slider = m_scaleWidget->getStep(i);
				double absolutePos = (i == selectedIndex ? value : m_minAbsoluteVal + slider->getRelativePos() * (m_maxAbsoluteVal - m_minAbsoluteVal));
				newSliders->push_back(new ColorScaleElementSlider(absolutePos,slider->getColor()));
			}
		}

		//update min and max boundaries
		{
			newSliders->sort();
			m_minAbsoluteVal = newSliders->front()->getRelativePos(); //absolute in fact!
			m_maxAbsoluteVal = newSliders->back()->getRelativePos(); //absolute in fact!
		}

		//convert absolute pos to relative ones
		int newSelectedIndex = -1;
		{
			double range = std::max(m_maxAbsoluteVal-m_minAbsoluteVal,1e-12);
			for (int i=0;i<newSliders->size();++i)
			{
				double absoluteVal = newSliders->at(i)->getRelativePos();
				if (absoluteVal == value)
					newSelectedIndex = i;
				double relativePos = (absoluteVal-m_minAbsoluteVal)/range;
				newSliders->at(i)->setRelativePos(relativePos);
			}
		}

		//update the whole scale with new sliders
		m_scaleWidget->setSliders(newSliders);

		m_scaleWidget->setSelectedStepIndex(newSelectedIndex,true);

		setModified(true);
	}
}

bool ccColorScaleEditorDialog::exportCustomLabelsList(ccColorScale::LabelSet& labels)
{
	assert(customLabelsGroupBox->isChecked());
	labels.clear();

	QString text = customLabelsPlainTextEdit->toPlainText();
	QStringList items = text.simplified().split(QChar(' '), QString::SkipEmptyParts);
	if (items.size() < 2)
	{
		assert(false);
		return false;
	}

	try
	{
		for (int i=0; i<items.size(); ++i)
		{
			bool ok;
			double d = items[i].toDouble(&ok);
			if (!ok)
			{
				return false;
			}
			labels.insert(d);
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory to save the custom labels!");
		labels.clear();
		return false;
	}

	return true;
}

bool ccColorScaleEditorDialog::checkCustomLabelsList(bool showWarnings)
{
	QString text = customLabelsPlainTextEdit->toPlainText();
	QStringList items = text.simplified().split(QChar(' '), QString::SkipEmptyParts);
	if (items.size() < 2)
	{
		if (showWarnings)
			ccLog::Error("Not enough labels defined (2 at least are required)");
		return false;
	}

	for (int i=0; i<items.size(); ++i)
	{
		bool ok;
		items[i].toDouble(&ok);
		if (!ok)
		{
			if (showWarnings)
				ccLog::Error(QString("Invalid label value: '%1'").arg(items[i]));
			return false;
		}
	}

	return true;
}

void ccColorScaleEditorDialog::onCustomLabelsListChanged()
{
	setModified(true);
}

void ccColorScaleEditorDialog::toggleCustomLabelsList(bool state)
{
	//custom list enable
	if (state)
	{
		QString previousText = customLabelsPlainTextEdit->toPlainText();
		//if the previous list was 'empty', we clear its (fake) content
		if (previousText == s_defaultEmptyCustomListText)
		{
			customLabelsPlainTextEdit->blockSignals(true);
			customLabelsPlainTextEdit->clear();
			customLabelsPlainTextEdit->blockSignals(false);
		}
	}
	else
	{
		if (!checkCustomLabelsList(false))
		{
			//if the text is invalid
			customLabelsPlainTextEdit->setPlainText(s_defaultEmptyCustomListText);
		}
	}
	setModified(true);
}

void ccColorScaleEditorDialog::copyCurrentScale()
{
	if (!m_colorScale)
	{
		assert(false);
		return;
	}
	
	ccColorScale::Shared scale = ccColorScale::Create(m_colorScale->getName()+QString("_copy"));
	if (!m_colorScale->isRelative())
	{
		double minVal = 0.0;
		double maxVal = 0.0;
		m_colorScale->getAbsoluteBoundaries(minVal,maxVal);
		scale->setAbsolute(minVal,maxVal);
	}
	m_scaleWidget->exportColorScale(scale);

	assert(m_manager);
	if (m_manager)
		m_manager->addScale(scale);

	updateMainComboBox();

	setActiveScale(scale);
}

bool ccColorScaleEditorDialog::saveCurrentScale()
{
	if (!m_colorScale || m_colorScale->isLocked())
	{
		assert(false);
		return false;
	}

	//check the custom labels
	if (customLabelsGroupBox->isChecked() && !checkCustomLabelsList(true))
	{
		//error message already issued
		return false;
	}
	
	m_scaleWidget->exportColorScale(m_colorScale);
	bool wasRelative = m_colorScale->isRelative();
	bool isRelative = isRelativeMode();
	if (isRelative)
		m_colorScale->setRelative();
	else
		m_colorScale->setAbsolute(m_minAbsoluteVal,m_maxAbsoluteVal);

	//DGM: warning, if the relative state has changed
	//we must update all the SFs currently relying on this scale!
	if ((!isRelative || isRelative != wasRelative) && m_mainApp && m_mainApp->dbRootObject())
	{
		ccHObject::Container clouds;
		m_mainApp->dbRootObject()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
		for (size_t i=0; i<clouds.size(); ++i)
		{
			ccPointCloud* cloud = static_cast<ccPointCloud*>(clouds[i]);
			for (unsigned j=0; j<cloud->getNumberOfScalarFields(); ++j)
			{
				ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(j));
				if (sf->getColorScale() == m_colorScale)
				{
					//trick: we unlink then re-link the color scale to update everything automatically
					sf->setColorScale(ccColorScale::Shared(nullptr));
					sf->setColorScale(m_colorScale);

					if (cloud->getCurrentDisplayedScalarField() == sf)
					{
						cloud->prepareDisplayForRefresh();
						if (cloud->getParent() && cloud->getParent()->isKindOf(CC_TYPES::MESH))
						{
							//for mesh vertices (just in case)
							cloud->getParent()->prepareDisplayForRefresh();
						}
					}
				}
			}
		}

		m_mainApp->refreshAll();
	}

	//save the custom labels
	if (customLabelsGroupBox->isChecked())
	{
		exportCustomLabelsList(m_colorScale->customLabels());
	}
	else
	{
		m_colorScale->customLabels().clear();
	}

	setModified(false);

	return true;
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

	int currentIndex = rampComboBox->currentIndex();
	if (currentIndex == 0)
		currentIndex = 1;
	else if (currentIndex > 0)
		--currentIndex;

	assert(m_manager);
	if (m_manager)
	{
		//activate the neighbor scale in the list
		ccColorScale::Shared nextScale = m_manager->getScale(rampComboBox->itemData(currentIndex).toString());
		setActiveScale(nextScale);

		m_manager->removeScale(colorScaleToDelete->getUuid());
	}

	updateMainComboBox();
}

void ccColorScaleEditorDialog::createNewScale()
{
	ccColorScale::Shared scale = ccColorScale::Create("New scale");

	//add default min and max steps
	scale->insert(ccColorScaleElement(0.0,Qt::blue),false);
	scale->insert(ccColorScaleElement(1.0,Qt::red),true);

	assert(m_manager);
	if (m_manager)
		m_manager->addScale(scale);

	updateMainComboBox();
	
	setActiveScale(scale);
}

void ccColorScaleEditorDialog::onApply()
{
	if (m_mainApp && canChangeCurrentScale())
	{
		if (m_associatedSF)
			m_associatedSF->setColorScale(m_colorScale);
		m_mainApp->redrawAll();
	}
}

void ccColorScaleEditorDialog::onClose()
{
	if (canChangeCurrentScale())
	{
		accept();
	}
}

void ccColorScaleEditorDialog::exportCurrentScale()
{
	if (!m_colorScale || m_colorScale->isLocked())
	{
		assert(false);
		return;
	}

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	//ask for a filename
	QString filename = QFileDialog::getSaveFileName(this,"Select output file",currentPath,"*.xml");
	if (filename.isEmpty())
	{
		//process cancelled by user
		return;
	}

	//save last saving location
	settings.setValue(ccPS::CurrentPath(),QFileInfo(filename).absolutePath());
	settings.endGroup();

	//try to save the file
	if (m_colorScale->saveAsXML(filename))
	{
		ccLog::Print(QString("[ColorScale] Scale '%1' successfully exported in '%2'").arg(m_colorScale->getName(),filename));
	}
}

void ccColorScaleEditorDialog::importScale()
{
	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::LoadFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	//ask for a filename
	QString filename = QFileDialog::getOpenFileName(this,"Select color scale file",currentPath,"*.xml");
	if (filename.isEmpty())
	{
		//process cancelled by user
		return;
	}

	//save last loading parameters
	settings.setValue(ccPS::CurrentPath(),QFileInfo(filename).absolutePath());
	settings.endGroup();

	//try to load the file
	ccColorScale::Shared scale = ccColorScale::LoadFromXML(filename);
	if (scale)
	{
		assert(m_manager);
		if (m_manager)
		{
			ccColorScale::Shared otherScale = m_manager->getScale(scale->getUuid());
			if (otherScale)
			{
				QString message = "A color scale with the same UUID";
				if (otherScale->getName() == scale->getName())
					message += QString(" and the same name (%1)").arg(scale->getName());
				message += " is already in store!";
				message += "\n";
				message += "Do you want to force the importation of this new scale? (a new UUID will be generated)";

				if (QMessageBox::question	(this,
											"UUID conflict",
											message,
											QMessageBox::Yes,
											QMessageBox::No) == QMessageBox::No)
				{
					ccLog::Warning("[ccColorScaleEditorDialog::importScale] Importation cancelled due to a conflicting UUID (color scale may already be in store)");
					return;
				}
				//generate a new UUID
				scale->setUuid(QUuid::createUuid().toString());
			}
			//now we can import the scale
			m_manager->addScale(scale);
			ccLog::Print(QString("[ccColorScaleEditorDialog::importScale] Color scale '%1' successfully imported").arg(scale->getName()));
		}

		updateMainComboBox();
	
		setActiveScale(scale);
	}
}
