//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qM3C2                       #
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
//#            COPYRIGHT: UNIVERSITE EUROPEENNE DE BRETAGNE                #
//#                                                                        #
//##########################################################################

#include "qM3C2Dialog.h"

//qCC
#include "../ccMainAppInterface.h"

//qCC_db
#include <ccPointCloud.h>

//Qt
#include <QMainWindow>
#include <QComboBox>
#include <QFileInfo>
#include <QFileDialog>
#include <QMessageBox>
#include <QThread>

/*** HELPERS ***/

static QString GetEntityName(ccHObject* obj)
{
	if (!obj)
	{
		assert(false);
		return QString();
	}

	QString name = obj->getName();
	if (name.isEmpty())
		name = "unnamed";
	name += QString(" [ID %1]").arg(obj->getUniqueID());

	return name;
}

static ccPointCloud* GetCloudFromCombo(QComboBox* comboBox, ccHObject* dbRoot)
{
	assert(comboBox && dbRoot);
	if (!comboBox || !dbRoot)
	{
		assert(false);
		return 0;
	}

	//return the cloud currently selected in the combox box
	int index = comboBox->currentIndex();
	if (index < 0)
	{
		assert(false);
		return 0;
	}
	assert(comboBox->userData(index));
	unsigned uniqueID = comboBox->itemData(index).toUInt();
	ccHObject* item = dbRoot->find(uniqueID);
	if (!item || !item->isA(CC_TYPES::POINT_CLOUD))
	{
		assert(false);
		return 0;
	}
	return static_cast<ccPointCloud*>(item);
}

/*** HELPERS (END) ***/

qM3C2Dialog::qM3C2Dialog(ccPointCloud* cloud1, ccPointCloud* cloud2, ccMainAppInterface* app)
	: QDialog(app ? app->getMainWindow() : 0)
	, Ui::M3C2Dialog()
	, m_app(app)
	, m_cloud1(0)
	, m_cloud2(0)
	, m_firstTimeInit(true)
{
	setupUi(this);

	int maxThreadCount = QThread::idealThreadCount();
	maxThreadCountSpinBox->setRange(1, maxThreadCount);
	maxThreadCountSpinBox->setSuffix(QString(" / %1").arg(maxThreadCount));

	connect(showCloud1CheckBox,			SIGNAL(toggled(bool)),	this, SLOT(setCloud1Visibility(bool)));
	connect(showCloud2CheckBox,			SIGNAL(toggled(bool)),	this, SLOT(setCloud2Visibility(bool)));
	connect(cpUseOtherCloudRadioButton,	SIGNAL(toggled(bool)),	this, SLOT(ifUseOtherCloudForCorePoints(bool)));

	connect(loadParamsToolButton,		SIGNAL(clicked()),		this, SLOT(loadParamsFromFile()));
	connect(saveParamsToolButton,		SIGNAL(clicked()),		this, SLOT(saveParamsToFile()));
	connect(swapCloudsToolButton,		SIGNAL(clicked()),		this, SLOT(swapClouds()));
	connect(guessParamsPushButton,		SIGNAL(clicked()),		this, SLOT(guessParamsSlow()));

	connect(projDestComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(projDestIndexChanged(int)));

	loadParamsFromPersistentSettings();

	setClouds(cloud1, cloud2);

	if (m_app)
	{
		//add list of clouds to the combo-boxes
		ccHObject::Container clouds;
		if (m_app->dbRootObject())
			m_app->dbRootObject()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD);

		for (size_t i = 0; i < clouds.size(); ++i)
		{
			if (clouds[i]->isA(CC_TYPES::POINT_CLOUD)) //as filterChildren only test 'isKindOf'
			{
				cpOtherCloudComboBox->addItem(GetEntityName(clouds[i]), QVariant(clouds[i]->getUniqueID()));
				normOriCloudComboBox->addItem(GetEntityName(clouds[i]), QVariant(clouds[i]->getUniqueID()));
			}
		}
	}
}

void qM3C2Dialog::swapClouds()
{
	setClouds(m_cloud2, m_cloud1);
}

void qM3C2Dialog::setClouds(ccPointCloud* cloud1, ccPointCloud* cloud2)
{
	if (!cloud1 || !cloud2)
	{
		assert(false);
		return;
	}

	m_cloud1 = cloud1;
	m_cloud2 = cloud2;

	//cloud #1
	cloud1LineEdit->setText(GetEntityName(cloud1));
	showCloud1CheckBox->blockSignals(true);
	showCloud1CheckBox->setChecked(cloud1->isVisible());
	showCloud1CheckBox->blockSignals(false);
	if (!cloud1->hasNormals())
		useCloud1NormalsCheckBox->setChecked(false);
	ifUseOtherCloudForCorePoints(cpUseOtherCloudRadioButton->isChecked()); //enables or disables 'useCloud1NormalsCheckBox'

	//cloud #2
	cloud2LineEdit->setText(GetEntityName(cloud2));
	showCloud2CheckBox->blockSignals(true);
	showCloud2CheckBox->setChecked(cloud2->isVisible());
	showCloud2CheckBox->blockSignals(false);

	if (m_firstTimeInit)
	{
		//on initialization, try to guess some parameters from the input clouds
		guessParams(true);
		m_firstTimeInit = false;
	}
}

void qM3C2Dialog::ifUseOtherCloudForCorePoints(bool state)
{
	useCloud1NormalsCheckBox->setEnabled(m_cloud1 && m_cloud1->hasNormals() && !state);
	normParamsFrame->setEnabled(!useCloud1NormalsCheckBox->isEnabled() || !useCloud1NormalsCheckBox->isChecked());
}

ccPointCloud* qM3C2Dialog::getCorePointsCloud()
{
	if (cpUseCloud1RadioButton->isChecked())
	{
		return m_cloud1;
	}
	else if (cpUseOtherCloudRadioButton->isChecked())
	{
		//return the cloud currently selected in the combox box
		return GetCloudFromCombo(cpOtherCloudComboBox, m_app->dbRootObject());
	}
	else
	{
		return 0;
	}
}

ccPointCloud* qM3C2Dialog::getNormalsOrientationCloud()
{
	if (normOriUseCloudRadioButton->isChecked())
	{
		//return the cloud currently selected in the combox box
		return GetCloudFromCombo(normOriCloudComboBox, m_app->dbRootObject());
	}
	else
	{
		return 0;
	}
}

void qM3C2Dialog::setCloud1Visibility(bool state)
{
	if (m_cloud1)
	{
		m_cloud1->setVisible(state);
		m_cloud1->prepareDisplayForRefresh();
	}
	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}
}

void qM3C2Dialog::setCloud2Visibility(bool state)
{
	if (m_cloud2)
	{
		m_cloud2->setVisible(state);
		m_cloud2->prepareDisplayForRefresh();
	}
	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}
}

qM3C2Normals::ComputationMode qM3C2Dialog::getNormalsComputationMode() const
{
	//special case
	if (useCloud1NormalsCheckBox->isEnabled() && useCloud1NormalsCheckBox->isChecked())
	{
		assert(m_cloud1 && m_cloud1->hasNormals());
		return qM3C2Normals::USE_CLOUD1_NORMALS;
	}
	else if (normMultiScaleRadioButton->isChecked())
		return qM3C2Normals::MULTI_SCALE_MODE;
	else if (normVertRadioButton->isChecked())
		return qM3C2Normals::VERT_MODE;
	else if (normHorizRadioButton->isChecked())
		return qM3C2Normals::HORIZ_MODE;
	else /*if (normDefaultRadioButton->isChecked())*/
		return qM3C2Normals::DEFAULT_MODE;
}

qM3C2Dialog::ExportOptions qM3C2Dialog::getExportOption() const
{
	switch (projDestComboBox->currentIndex())
	{
	case 0:
		return PROJECT_ON_CLOUD1;
	case 1:
		return PROJECT_ON_CLOUD2;
	case 2:
		return PROJECT_ON_CORE_POINTS;
	default:
		assert(false);
		break;
	}

	return PROJECT_ON_CORE_POINTS;
}

void qM3C2Dialog::projDestIndexChanged(int index)
{
	useOriginalCloudCheckBox->setEnabled(getExportOption() == PROJECT_ON_CORE_POINTS);
}

bool qM3C2Dialog::keepOriginalCloud() const
{
	return useOriginalCloudCheckBox->isEnabled() && useOriginalCloudCheckBox->isChecked();
}

int qM3C2Dialog::getMaxThreadCount() const
{
	return maxThreadCountSpinBox->value();
}

unsigned qM3C2Dialog::getMinPointsForStats(unsigned defaultValue/*=5*/) const
{
	return useMinPoints4StatCheckBox->isChecked() ? static_cast<unsigned>(std::max(0,minPoints4StatSpinBox->value())) : defaultValue;
}

void qM3C2Dialog::loadParamsFromPersistentSettings()
{
	QSettings settings("qM3C2");
	loadParamsFrom(settings);
}

void qM3C2Dialog::loadParamsFrom(const QSettings& settings)
{
	//read out parameters
	double normalScale = settings.value("NormalScale",normalScaleDoubleSpinBox->value()).toDouble();
	int normModeInt = settings.value("NormalMode",static_cast<int>(getNormalsComputationMode())).toInt();
	double normMinScale = settings.value("NormalMinScale",minScaleDoubleSpinBox->value()).toDouble();
	double normStep = settings.value("NormalStep",stepScaleDoubleSpinBox->value()).toDouble();
	double normMaxScale = settings.value("NormalMaxScale",maxScaleDoubleSpinBox->value()).toDouble();
	bool normUseCorePoints = settings.value("NormalUseCorePoints",normUseCorePointsCheckBox->isChecked()).toBool();
	int normPreferredOri = settings.value("NormalPreferedOri",normOriPreferredComboBox->currentIndex()).toInt();

	double seachScale = settings.value("SearchScale",cylDiameterDoubleSpinBox->value()).toDouble();
	double searchDepth = settings.value("SearchDepth",cylHalfHeightDoubleSpinBox->value()).toDouble();
	
	double subsampleRadius = settings.value("SubsampleRadius",cpSubsamplingDoubleSpinBox->value()).toDouble();
	bool subsampleEnabled = settings.value("SubsampleEnabled",cpSubsampleRadioButton->isChecked()).toBool();
	
	double registrationError = settings.value("RegistrationError",rmsDoubleSpinBox->value()).toDouble();
	bool registrationErrorEnabled = settings.value("RegistrationErrorEnabled",rmsCheckBox->isChecked()).toBool();

	bool useSinglePass4Depth = settings.value("UseSinglePass4Depth",useSinglePass4DepthCheckBox->isChecked()).toBool();
	bool positiveSearchOnly = settings.value("PositiveSearchOnly",positiveSearchOnlyCheckBox->isChecked()).toBool();
	bool useMedian = settings.value("UseMedian",useMedianCheckBox->isChecked()).toBool();
	
	bool useMinPoints4Stat = settings.value("UseMinPoints4Stat",useMinPoints4StatCheckBox->isChecked()).toBool();
	int minPoints4Stat = settings.value("MinPoints4Stat",minPoints4StatSpinBox->value()).toInt();

	int projDestIndex = settings.value("ProjDestIndex",projDestComboBox->currentIndex()).toInt();
	bool useOriginalCloud = settings.value("UseOriginalCloud",useOriginalCloudCheckBox->isChecked()).toBool();

	bool exportStdDevInfo = settings.value("ExportStdDevInfo",exportStdDevInfoCheckBox->isChecked()).toBool();
	bool exportDensityAtProjScale = settings.value("ExportDensityAtProjScale",exportDensityAtProjScaleCheckBox->isChecked()).toBool();

	int maxThreadCount = settings.value("MaxThreadCount", maxThreadCountSpinBox->maximum()).toInt();

	//apply parameters
	normalScaleDoubleSpinBox->setValue(normalScale);
	switch(normModeInt)
	{
	case qM3C2Normals::USE_CLOUD1_NORMALS:
		useCloud1NormalsCheckBox->setChecked(m_cloud1 && m_cloud1->hasNormals());
	case qM3C2Normals::DEFAULT_MODE:
		normDefaultRadioButton->setChecked(true);
		break;
	case qM3C2Normals::MULTI_SCALE_MODE:
		normMultiScaleRadioButton->setChecked(true);
		break;
	case qM3C2Normals::VERT_MODE:
		normVertRadioButton->setChecked(true);
		break;
	case qM3C2Normals::HORIZ_MODE:
		normHorizRadioButton->setChecked(true);
		break;
	default:
		//nothing to do
		break;
	}

	minScaleDoubleSpinBox->setValue(normMinScale);
	stepScaleDoubleSpinBox->setValue(normStep);
	maxScaleDoubleSpinBox->setValue(normMaxScale);
	normUseCorePointsCheckBox->setChecked(normUseCorePoints);
	normOriPreferredComboBox->setCurrentIndex(normPreferredOri);

	cylDiameterDoubleSpinBox->setValue(seachScale);
	cylHalfHeightDoubleSpinBox->setValue(searchDepth);
	
	cpSubsamplingDoubleSpinBox->setValue(subsampleRadius);
	if (subsampleEnabled)
	{
		cpSubsampleRadioButton->setChecked(true);
	}
	
	rmsCheckBox->setChecked(registrationErrorEnabled);
	rmsDoubleSpinBox->setValue(registrationError);

	useSinglePass4DepthCheckBox->setChecked(useSinglePass4Depth);
	positiveSearchOnlyCheckBox->setChecked(positiveSearchOnly);
	useMedianCheckBox->setChecked(useMedian);
	
	useMinPoints4StatCheckBox->setChecked(useMinPoints4Stat);
	minPoints4StatSpinBox->setValue(minPoints4Stat);

	projDestComboBox->setCurrentIndex(projDestIndex);
	useOriginalCloudCheckBox->setChecked(useOriginalCloud);

	exportStdDevInfoCheckBox->setChecked(exportStdDevInfo);
	exportDensityAtProjScaleCheckBox->setChecked(exportDensityAtProjScale);

	maxThreadCountSpinBox->setValue(maxThreadCount);
}

void qM3C2Dialog::saveParamsToPersistentSettings()
{
	QSettings settings("qM3C2");
	saveParamsTo(settings);
}

void qM3C2Dialog::saveParamsTo(QSettings& settings)
{
	//save parameters
	settings.setValue("NormalScale",normalScaleDoubleSpinBox->value());
	settings.setValue("NormalMode",static_cast<int>(getNormalsComputationMode()));
	settings.setValue("NormalMinScale",minScaleDoubleSpinBox->value());
	settings.setValue("NormalStep",stepScaleDoubleSpinBox->value());
	settings.setValue("NormalMaxScale",maxScaleDoubleSpinBox->value());
	settings.setValue("NormalUseCorePoints",normUseCorePointsCheckBox->isChecked());
	settings.setValue("NormalPreferedOri",normOriPreferredComboBox->currentIndex());

	settings.setValue("SearchScale",cylDiameterDoubleSpinBox->value());
	settings.setValue("SearchDepth",cylHalfHeightDoubleSpinBox->value());
	
	settings.setValue("SubsampleRadius",cpSubsamplingDoubleSpinBox->value());
	settings.setValue("SubsampleEnabled",cpSubsampleRadioButton->isChecked());
	
	settings.setValue("RegistrationError",rmsDoubleSpinBox->value());
	settings.setValue("RegistrationErrorEnabled",rmsCheckBox->isChecked());

	settings.setValue("UseSinglePass4Depth",useSinglePass4DepthCheckBox->isChecked());
	settings.setValue("PositiveSearchOnly",positiveSearchOnlyCheckBox->isChecked());
	settings.setValue("UseMedian",useMedianCheckBox->isChecked());
	
	settings.setValue("UseMinPoints4Stat",useMinPoints4StatCheckBox->isChecked());
	settings.setValue("MinPoints4Stat",minPoints4StatSpinBox->value());

	settings.setValue("ProjDestIndex",projDestComboBox->currentIndex());
	settings.setValue("UseOriginalCloud",useOriginalCloudCheckBox->isChecked());

	settings.setValue("ExportStdDevInfo",exportStdDevInfoCheckBox->isChecked());
	settings.setValue("ExportDensityAtProjScale",exportDensityAtProjScaleCheckBox->isChecked());

	settings.setValue("MaxThreadCount", maxThreadCountSpinBox->value());
}

void qM3C2Dialog::loadParamsFromFile()
{
	//select file to open
	QString filename;
	{
		QSettings settings("qM3C2");
		QString currentPath = settings.value("currentPath", QApplication::applicationDirPath()).toString();

		filename = QFileDialog::getOpenFileName(this, "Load M3C2 parameters", currentPath, "*.txt");
		if (filename.isEmpty())
			return;

		//we update current file path
		currentPath = QFileInfo(filename).absolutePath();
		settings.setValue("currentPath", currentPath);
	}

	//load file
	{
		QSettings fileSettings(filename, QSettings::IniFormat);
		//check validity
		if (!fileSettings.contains("M3C2VER"))
		{
			QMessageBox::critical(this, "Invalid file", "File doesn't seem to be a valid M3C2 parameters file ('M3C2VER' not found)!");
			return;
		}

		loadParamsFrom(fileSettings);
	}
}

void qM3C2Dialog::saveParamsToFile()
{
	//select file to save
	QString filename;
	{
		QSettings settings("qM3C2");
		QString currentPath = settings.value("currentPath", QApplication::applicationDirPath()).toString();

		filename = QFileDialog::getSaveFileName(this, "Save M3C2 parameters", currentPath + QString("/m3c2_params.txt"), "*.txt");
		if (filename.isEmpty())
			return;

		//we update current file path
		currentPath = QFileInfo(filename).absolutePath();
		settings.setValue("currentPath", currentPath);
	}

	//save file
	{
		QSettings fileSettings(filename, QSettings::IniFormat);
		//set version tag (mandatory for a valid parameters file!)
		fileSettings.setValue("M3C2VER", QVariant::fromValue<int>(1));
		saveParamsTo(fileSettings);
	}
}

void qM3C2Dialog::guessParams(bool fastMode/*=false*/)
{
	if (!m_cloud1 || !m_cloud2)
		return;

	unsigned minPoints4Stats = getMinPointsForStats() * 6; //see article: ideal = 30 while default = 5
	//Guessed parameters
	qM3C2Tools::GuessedParams params;
	params.preferredDimension = normOriPreferredComboBox->currentIndex();

	if (qM3C2Tools::GuessBestParams(m_cloud1, m_cloud2, minPoints4Stats, params, fastMode, m_app))
	{
		normalScaleDoubleSpinBox->setValue(params.normScale);
		cylDiameterDoubleSpinBox->setValue(params.projScale);
		cylHalfHeightDoubleSpinBox->setValue(params.projDepth);
		normOriPreferredComboBox->setCurrentIndex(params.preferredDimension);

		minScaleDoubleSpinBox->setValue(params.normScale / 2);
		stepScaleDoubleSpinBox->setValue(params.normScale / 2);
		maxScaleDoubleSpinBox->setValue(params.normScale * 2);

		cpSubsamplingDoubleSpinBox->setValue(params.projScale / 2);
	}
}
