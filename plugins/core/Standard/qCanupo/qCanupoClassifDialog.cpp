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

#include "qCanupoClassifDialog.h"

//local
#include "classifier.h"
#include "qCanupoTools.h"

//qCC_plugins
#include "ccMainAppInterface.h"

//qCC_db
#include <ccPointCloud.h>

//Qt
#include <QSettings>
#include <QMainWindow>
#include <QComboBox>
#include <QFileInfo>
#include <QFileDialog>
#include <QPushButton>
#include <QApplication>
#include <QThread>

qCanupoClassifDialog::qCanupoClassifDialog(ccPointCloud* cloud, ccMainAppInterface* app)
	: QDialog(app ? app->getMainWindow() : nullptr)
	, Ui::CanupoClassifDialog()
	, m_app(app)
	, m_cloud(cloud)
{
	setupUi(this);

#ifndef COMPILE_PRIVATE_CANUPO
	generateRoughnessSFsCheckBox->setVisible(false);
#endif

	int maxThreadCount = QThread::idealThreadCount();
	maxThreadCountSpinBox->setRange(1, maxThreadCount);
	maxThreadCountSpinBox->setSuffix(QString(" / %1").arg(maxThreadCount));

	loadParamsFromPersistentSettings();

	if (cloud)
	{
		//check if a the cloud has an active SF!
		useSFCheckBox->setEnabled(cloud->getCurrentDisplayedScalarField() != nullptr);
	}

	if (m_app)
	{
		//add list of clouds to the combo-boxes
		ccHObject::Container clouds;
		if (m_app->dbRootObject())
			m_app->dbRootObject()->filterChildren(clouds,true,CC_TYPES::POINT_CLOUD);

		for (size_t i=0; i<clouds.size(); ++i)
		{
			if (clouds[i]->isA(CC_TYPES::POINT_CLOUD)) //as filterChildren only test 'isKindOf'
				cpOtherCloudComboBox->addItem(qCanupoTools::GetEntityName(clouds[i]),QVariant(clouds[i]->getUniqueID()));
		}
	}

	connect(browseToolButton,		SIGNAL(clicked()), this, SLOT(browseClassifierFile()));
	connect(mscBrowseToolButton,	SIGNAL(clicked()), this, SLOT(browseMscFile()));

	buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
}

double qCanupoClassifDialog::getConfidenceTrehshold() const
{
	if (!useConfThresholdGroupBox->isChecked())
		return 0.0;

	//confidence threshold between 0 and 1
	assert(pokDoubleSpinBox->value() >= 0 && pokDoubleSpinBox->value() <= 1.0);
	return pokDoubleSpinBox->value();
}

bool qCanupoClassifDialog::useSF() const
{
	return useConfThresholdGroupBox->isChecked() && useSFCheckBox->isChecked();
}

int qCanupoClassifDialog::getMaxThreadCount() const
{
	return maxThreadCountSpinBox->value();
}

qCanupoClassifDialog::CORE_CLOUD_SOURCES qCanupoClassifDialog::getCorePointsCloudSource() const
{
	if (cpUseCloudRadioButton->isChecked())
	{
		return ORIGINAL;
	}
	else if (cpSubsampleRadioButton->isChecked())
	{
		return SUBSAMPLED;
	}
	else if (cpUseOtherCloudRadioButton->isChecked())
	{
		return OTHER;
	}
	else if (cpMscFileRadioButton->isChecked())
	{
		return MSC_FILE;
	}

	assert(false);
	return ORIGINAL;
}

QString qCanupoClassifDialog::getMscFilename() const
{
	return mscFileLineEdit->text();
}

ccPointCloud* qCanupoClassifDialog::getCorePointsCloud()
{
	if (cpUseCloudRadioButton->isChecked())
	{
		return m_cloud;
	}
	else if (cpUseOtherCloudRadioButton->isChecked())
	{
		//return the cloud currently selected in the combox box
		return qCanupoTools::GetCloudFromCombo(cpOtherCloudComboBox, m_app->dbRootObject());
	}
	else
	{
		return nullptr;
	}
}

void qCanupoClassifDialog::loadParamsFromPersistentSettings()
{
	QSettings settings("qCanupo");
	settings.beginGroup("Classif");

	//read out parameters
	//double minScale = settings.value("MinScale",minScaleDoubleSpinBox->value()).toDouble();
	//double step = settings.value("Step",stepScaleDoubleSpinBox->value()).toDouble();
	//double maxScale = settings.value("MaxScale",maxScaleDoubleSpinBox->value()).toDouble();

	double subsampleRadius = settings.value("SubsampleRadius",cpSubsamplingDoubleSpinBox->value()).toDouble();
	bool subsampleEnabled = settings.value("SubsampleEnabled",cpSubsampleRadioButton->isChecked()).toBool();

	QString currentPath = settings.value("CurrentPath",QApplication::applicationDirPath()).toString();
	QString mscCurrentPath = settings.value("MscCurrentPath",QApplication::applicationDirPath()).toString();

	bool useConfThreshold = settings.value("UseConfThreshold",useConfThresholdGroupBox->isChecked()).toBool();
	double pok = settings.value("Pok",pokDoubleSpinBox->value()).toDouble();
	bool useSF = settings.value("UseSF",useSFCheckBox->isChecked()).toBool();
	bool additionalSF = settings.value("AdditionalSF",generateAdditionalSFsCheckBox->isChecked()).toBool();
	bool roughnessSF = settings.value("RoughnessSF",generateRoughnessSFsCheckBox->isChecked()).toBool();
	int maxThreadCount = settings.value("MaxThreadCount", maxThreadCountSpinBox->maximum()).toInt();

	//apply parameters

	//minScaleDoubleSpinBox->setValue(minScale);
	//stepScaleDoubleSpinBox->setValue(step);
	//maxScaleDoubleSpinBox->setValue(maxScale);

	cpSubsamplingDoubleSpinBox->setValue(subsampleRadius);
	if (subsampleEnabled)
		cpSubsampleRadioButton->setChecked(true);
	
	classifFileLineEdit->setText(currentPath);
	mscFileLineEdit->setText(mscCurrentPath);

	useConfThresholdGroupBox->setChecked(useConfThreshold);
	pokDoubleSpinBox->setValue(pok);
	useSFCheckBox->setChecked(useSF);
	generateAdditionalSFsCheckBox->setChecked(additionalSF);
	generateRoughnessSFsCheckBox->setChecked(roughnessSF);
	maxThreadCountSpinBox->setValue(maxThreadCount);
}

void qCanupoClassifDialog::saveParamsToPersistentSettings()
{
	QSettings settings("qCanupo");
	settings.beginGroup("Classif");

	//save parameters
	//settings.setValue("MinScale",minScaleDoubleSpinBox->value());
	//settings.setValue("Step",stepScaleDoubleSpinBox->value());
	//settings.setValue("MaxScale",maxScaleDoubleSpinBox->value());

	settings.setValue("SubsampleRadius",cpSubsamplingDoubleSpinBox->value());
	settings.setValue("SubsampleEnabled",cpSubsampleRadioButton->isChecked());

	QString currentPath = QFileInfo(classifFileLineEdit->text()).absolutePath();
	settings.setValue("CurrentPath",currentPath);

	settings.setValue("UseConfThreshold",useConfThresholdGroupBox->isChecked());
	settings.setValue("Pok",pokDoubleSpinBox->value());
	settings.setValue("UseSF",useSFCheckBox->isChecked());
	settings.setValue("AdditionalSF",generateAdditionalSFsCheckBox->isChecked());
	settings.setValue("RoughnessSF",generateRoughnessSFsCheckBox->isChecked());

	settings.setValue("MaxThreadCount", maxThreadCountSpinBox->value());
}

void qCanupoClassifDialog::browseMscFile()
{
	//select file to open
	QSettings settings("qCanupo");
	settings.beginGroup("Classif");
	QString currentPath = settings.value("MscCurrentPath",mscFileLineEdit->text()).toString();

	QString filename = QFileDialog::getOpenFileName(this,"Load MSC file",currentPath,"*.msc");
	if (filename.isEmpty())
		return;

	//we update current file path
	mscFileLineEdit->setText(filename);
	currentPath = QFileInfo(filename).absolutePath();
	settings.setValue("MscCurrentPath",currentPath);
}

void qCanupoClassifDialog::browseClassifierFile()
{
	//select file to open
	QString filename;
	QSettings settings("qCanupo");
	settings.beginGroup("Classif");
	QString currentPath = settings.value("CurrentPath",classifFileLineEdit->text()).toString();

	filename = QFileDialog::getOpenFileName(this,"Load classifier file",currentPath,"*.prm");
	if (filename.isEmpty())
		return;

	//we update current file path
	classifFileLineEdit->setText(filename);
	currentPath = QFileInfo(filename).absolutePath();
	settings.setValue("CurrentPath",currentPath);

	//and we try to load the file header (to display some info)
	std::vector<Classifier> classifiers;
	std::vector<PointCoordinateType> scales;
	QString error;
	Classifier::FileHeader header;
	if (Classifier::Load(filename, classifiers, scales, error, &header, true))
	{
		//display classifier file description
		QStringList fileDesc;
		fileDesc << QString("File: %1").arg(QFileInfo(filename).fileName());
		fileDesc << QString("Classifier(s) in file: %1").arg(header.classifierCount);

		assert(header.descID != 0);
		ScaleParamsComputer* computer = ScaleParamsComputer::GetByID(header.descID);
		QString descriptorName = computer? computer->getName() : QString("INVALID");
		fileDesc << QString("Descriptor ID: %1 (%2)").arg(header.descID).arg(descriptorName);

		fileDesc << QString("Dimensions per scale: %1").arg(header.dimPerScale);
		fileDesc << QString("Scales: %1").arg(scales.size());
		QString scalesStr("    [ ");
		for (size_t i=0; i<scales.size(); ++i)
			scalesStr.append(QString("%1 ").arg(scales[scales.size()-1-i])); //reverse order as scales are sorted from biggest to smallest
		scalesStr.append("]");
		fileDesc << scalesStr;

		classifInfoTextEdit->setText(fileDesc.join("\n"));
	
		buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
	}
	else
	{
		classifInfoTextEdit->setText("Error: failed to open/read the input file");
		buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
	}
}
