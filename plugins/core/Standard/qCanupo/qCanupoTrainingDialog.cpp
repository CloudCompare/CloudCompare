//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCANUPO                       #
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
//#      COPYRIGHT: UEB (UNIVERSITE EUROPEENNE DE BRETAGNE) / CNRS         #
//#                                                                        #
//##########################################################################

#include "qCanupoTrainingDialog.h"

//local
#include "qCanupoTools.h"
#include "ccPointDescriptor.h"

//qCC_plugins
#include "ccMainAppInterface.h"

//qCC_db
#include <ccPointCloud.h>

//Qt
#include <QSettings>
#include <QMainWindow>
#include <QComboBox>
#include <QPushButton>
#include <QApplication>
#include <QThread>

//system
#include <limits>

qCanupoTrainingDialog::qCanupoTrainingDialog(ccMainAppInterface* app)
	: QDialog(app ? app->getMainWindow() : nullptr)
	, Ui::CanupoTrainingDialog()
	, m_app(app)
{
	setupUi(this);

	int maxThreadCount = QThread::idealThreadCount();
	maxThreadCountSpinBox->setRange(1, maxThreadCount);
	maxThreadCountSpinBox->setSuffix(QString(" / %1").arg(maxThreadCount));

	if (m_app)
	{
		//add list of clouds to the combo-boxes
		ccHObject::Container clouds;
		if (m_app->dbRootObject())
			m_app->dbRootObject()->filterChildren(clouds,true,CC_TYPES::POINT_CLOUD);

		unsigned cloudCount = 0;
		for (size_t i=0; i<clouds.size(); ++i)
		{
			if (clouds[i]->isA(CC_TYPES::POINT_CLOUD)) //as filterChildren only test 'isKindOf'
			{
				QString name = qCanupoTools::GetEntityName(clouds[i]);
				QVariant uniqueID(clouds[i]->getUniqueID());
				originCloudComboBox->addItem(name,uniqueID);
				class1CloudComboBox->addItem(name,uniqueID);
				class2CloudComboBox->addItem(name,uniqueID);
				evaluationCloudComboBox->addItem(name,uniqueID);
				++cloudCount;
			}
		}

		//if 3 clouds are loaded, then there's chances that the first one is the global  cloud!
		class1CloudComboBox->setCurrentIndex(cloudCount > 0 ? (cloudCount > 2 ? 1 : 0) : -1);
		class2CloudComboBox->setCurrentIndex(cloudCount > 1 ? (cloudCount > 2 ? 2 : 1) : -1);
		originCloudComboBox->setCurrentIndex(cloudCount > 2 ? 0 : -1);

		if (cloudCount < 2 && app)
			app->dispToConsole("You need at least 2 loaded clouds to train a classifier (one per class)",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}

	//add the list of available descriptors
	{
		paramComboBox->clear();
		unsigned count = ScaleParamsComputer::AvailableCount();
		for (unsigned i=0; i<count; ++i)
		{
			ScaleParamsComputer* scp = ScaleParamsComputer::GetByIndex(i);
			paramComboBox->addItem(scp->getName(),scp->getID());
		}
	}

	loadParamsFromPersistentSettings();

	connect(cloud1ClassSpinBox,		SIGNAL(valueChanged(int)),			this,	SLOT(onClassChanged(int)));
	connect(cloud2ClassSpinBox,		SIGNAL(valueChanged(int)),			this,	SLOT(onClassChanged(int)));
	connect(class1CloudComboBox,	SIGNAL(currentIndexChanged (int)),	this,	SLOT(onCloudChanged(int)));
	connect(class2CloudComboBox,	SIGNAL(currentIndexChanged (int)),	this,	SLOT(onCloudChanged(int)));

	onClassChanged(0);
	onCloudChanged(0);
}

bool qCanupoTrainingDialog::validParameters() const
{
	if (cloud1ClassSpinBox->value() == cloud2ClassSpinBox->value())
		return false;

	int c1 = class1CloudComboBox->currentIndex();
	int c2 = class2CloudComboBox->currentIndex();
	if (c1 < 0 || c2 < 0)
		return false;
	if (c1 == c2)
		return false;

	return true;
}

int qCanupoTrainingDialog::getMaxThreadCount() const
{
	return maxThreadCountSpinBox->value();
}

bool qCanupoTrainingDialog::getScales(std::vector<float>& scales) const
{
	scales.clear();

	try
	{
		if (scalesRampRadioButton->isChecked())
		{
			double maxScale = maxScaleDoubleSpinBox->value();
			double step = stepScaleDoubleSpinBox->value();
			double minScale	= minScaleDoubleSpinBox->value();
			if (maxScale < minScale || maxScale < 0 || step < 1.0e-6)
				return false;
			unsigned stepCount = static_cast<unsigned>( floor((maxScale-minScale)/step + 1.0e-6) ) + 1;
			scales.resize(stepCount);
			for (unsigned i=0; i<stepCount; ++i)
				scales[i] = static_cast<float>(maxScale - i*step);
		}
		else if (scalesListRadioButton->isChecked())
		{
			QStringList scaleList = scalesListLineEdit->text().split(' ',QString::SkipEmptyParts);
		
			int listSize = scaleList.size();
			scales.resize(listSize);
			for (int i=0; i<listSize; ++i)
			{
				bool ok = false;
				float f;
				f = scaleList[i].toFloat(&ok);
				if (!ok)
					return false;
				scales[i] = f;
			}
		}
		else
		{
			return false;
		}
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}

	return true;
}

unsigned qCanupoTrainingDialog::getDescriptorID() const
{
	//paramComboBox
	int currentIndex = paramComboBox->currentIndex();
	if (currentIndex < 0)
	{
		assert(false);
		return 0;
	}

	return paramComboBox->itemData(currentIndex).toUInt();
}

void qCanupoTrainingDialog::onClassChanged(int dummy)
{
	buttonBox->button(QDialogButtonBox::Ok)->setEnabled(validParameters());
}

void qCanupoTrainingDialog::onCloudChanged(int dummy)
{
	buttonBox->button(QDialogButtonBox::Ok)->setEnabled(validParameters());
}

ccPointCloud* qCanupoTrainingDialog::getClass1Cloud()
{
	//return the cloud currently selected in the combox box
	return qCanupoTools::GetCloudFromCombo(class1CloudComboBox, m_app->dbRootObject());
}

ccPointCloud* qCanupoTrainingDialog::getClass2Cloud()
{
	//return the cloud currently selected in the combox box
	return qCanupoTools::GetCloudFromCombo(class2CloudComboBox, m_app->dbRootObject());
}

ccPointCloud* qCanupoTrainingDialog::getOriginPointCloud()
{
	//return the cloud currently selected in the combox box
	return useOriginalCloudCheckBox->isChecked() ? qCanupoTools::GetCloudFromCombo(originCloudComboBox, m_app->dbRootObject()) : nullptr;
}

ccPointCloud* qCanupoTrainingDialog::getEvaluationCloud()
{
	//return the cloud currently selected in the combox box
	return evaluateParamsCheckBox->isChecked() ? qCanupoTools::GetCloudFromCombo(evaluationCloudComboBox, m_app->dbRootObject()) : nullptr;
}

void qCanupoTrainingDialog::loadParamsFromPersistentSettings()
{
	QSettings settings("qCanupo");
	settings.beginGroup("Training");

	//read out parameters
	double minScale = settings.value("MinScale",minScaleDoubleSpinBox->value()).toDouble();
	double step = settings.value("Step",stepScaleDoubleSpinBox->value()).toDouble();
	double maxScale = settings.value("MaxScale",maxScaleDoubleSpinBox->value()).toDouble();
	QString scalesList = settings.value("ScalesList",scalesListLineEdit->text()).toString();
	bool scalesRampEnabled = settings.value("ScalesRampEnabled",scalesRampRadioButton->isChecked()).toBool();

	unsigned maxPoints = settings.value("MaxPoints",maxPointsSpinBox->value()).toUInt();
	int classifParam = settings.value("ClassifParam",paramComboBox->currentIndex()).toInt();
	int maxThreadCount = settings.value("MaxThreadCount", maxThreadCountSpinBox->maximum()).toInt();

	//apply parameters

	minScaleDoubleSpinBox->setValue(minScale);
	stepScaleDoubleSpinBox->setValue(step);
	maxScaleDoubleSpinBox->setValue(maxScale);
	scalesListLineEdit->setText(scalesList);
	if (scalesRampEnabled)
		scalesRampRadioButton->setChecked(true);
	else
		scalesListRadioButton->setChecked(true);

	maxPointsSpinBox->setValue(maxPoints);
	paramComboBox->setCurrentIndex(classifParam);
	maxThreadCountSpinBox->setValue(maxThreadCount);
}

void qCanupoTrainingDialog::saveParamsToPersistentSettings()
{
	QSettings settings("qCanupo");
	settings.beginGroup("Training");

	//save parameters
	settings.setValue("MinScale",minScaleDoubleSpinBox->value());
	settings.setValue("Step",stepScaleDoubleSpinBox->value());
	settings.setValue("MaxScale",maxScaleDoubleSpinBox->value());
	settings.setValue("ScalesList",scalesListLineEdit->text());
	settings.setValue("ScalesRampEnabled",scalesRampRadioButton->isChecked());

	settings.setValue("MaxPoints",maxPointsSpinBox->value());
	settings.setValue("ClassifParam",paramComboBox->currentIndex());
	settings.setValue("MaxThreadCount", maxThreadCountSpinBox->value());
}
