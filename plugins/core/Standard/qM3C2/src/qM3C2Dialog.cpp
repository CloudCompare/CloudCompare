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
#include "ccMainAppInterface.h"

//qCC_db
#include <ccFileUtils.h>
#include <ccPointCloud.h>

//Qt
#include <QMainWindow>
#include <QComboBox>
#include <QFileInfo>
#include <QFileDialog>
#include <QMessageBox>
#include <QThread>

static bool s_firstTimeInit = true;

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
		return nullptr;
	}

	//return the cloud currently selected in the combox box
	int index = comboBox->currentIndex();
	if (index < 0)
	{
		assert(false);
		return nullptr;
	}
	assert(comboBox->itemData(index).isValid());
	unsigned uniqueID = comboBox->itemData(index).toUInt();
	ccHObject* item = dbRoot->find(uniqueID);
	if (!item || !item->isA(CC_TYPES::POINT_CLOUD))
	{
		assert(false);
		return nullptr;
	}
	return static_cast<ccPointCloud*>(item);
}

/*** HELPERS (END) ***/

qM3C2Dialog::qM3C2Dialog(ccPointCloud* cloud1, ccPointCloud* cloud2, ccMainAppInterface* app)
	: QDialog(app ? app->getMainWindow() : nullptr)
	, Ui::M3C2Dialog()
	, m_app(app)
	, m_cloud1(nullptr)
	, m_cloud2(nullptr)
	, m_corePointsCloud(nullptr)
{
	setupUi(this);

	int maxThreadCount = QThread::idealThreadCount();
	maxThreadCountSpinBox->setRange(1, maxThreadCount);
	maxThreadCountSpinBox->setSuffix(QString(" / %1").arg(maxThreadCount));

	connect(showCloud1CheckBox,			SIGNAL(toggled(bool)),	this, SLOT(setCloud1Visibility(bool)));
	connect(showCloud2CheckBox,			SIGNAL(toggled(bool)),	this, SLOT(setCloud2Visibility(bool)));

	connect(loadParamsToolButton,		SIGNAL(clicked()),		this, SLOT(loadParamsFromFile()));
	connect(saveParamsToolButton,		SIGNAL(clicked()),		this, SLOT(saveParamsToFile()));
	connect(swapCloudsToolButton,		SIGNAL(clicked()),		this, SLOT(swapClouds()));
	connect(guessParamsPushButton,		SIGNAL(clicked()),		this, SLOT(guessParamsSlow()));

	connect(projDestComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(projDestIndexChanged(int)));

	connect(cpOtherCloudComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateNormalComboBox()));
	connect(normalSourceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onUpdateNormalComboBoxChanged(int)));
	connect(cpUseCloud1RadioButton, SIGNAL(toggled(bool)), this, SLOT(updateNormalComboBox()));
	connect(cpSubsampleRadioButton, SIGNAL(toggled(bool)), this, SLOT(updateNormalComboBox()));
	connect(cpUseOtherCloudRadioButton, SIGNAL(toggled(bool)), this, SLOT(updateNormalComboBox()));

	loadParamsFromPersistentSettings();

	setClouds(cloud1, cloud2);

	if (m_app)
	{
		//add list of clouds to the combo-boxes
		ccHObject::Container clouds;
		if (m_app->dbRootObject())
		{
			m_app->dbRootObject()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD);
		}

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

bool PopulateSFCombo(QComboBox* combo, const ccPointCloud& cloud, int defaultFieldIndex = -1, QString defaultField = QString())
{
	unsigned sfCount = cloud.getNumberOfScalarFields();
	if (!combo || sfCount == 0)
	{
		assert(false);
		return false;
	}

	combo->clear();
	int selectedFieldIndex = -1;
	bool defaultFieldFound = false;
	for (unsigned i = 0; i < sfCount; ++i)
	{
		QString sfName = cloud.getScalarFieldName(i);
		combo->addItem(sfName);
		if (selectedFieldIndex < 0 && !defaultField.isEmpty())
		{
			if (sfName.contains(defaultField, Qt::CaseInsensitive))
			{
				selectedFieldIndex = static_cast<int>(i);
				defaultFieldFound = true;
			}
		}
	}

	if (selectedFieldIndex < 0)
	{
		selectedFieldIndex = defaultFieldIndex;
	}
	combo->setCurrentIndex(selectedFieldIndex);

	return defaultFieldFound;
}

bool PopulatePMFields(QComboBox* sx, QComboBox* sy, QComboBox* sz, const ccPointCloud& cloud)
{
	assert(sx && sy && sz);
	int sfCount = static_cast<int>(cloud.getNumberOfScalarFields());
	if (sfCount == 0)
	{
		assert(false);
		return false;
	}

	bool sxFound = PopulateSFCombo(sx, cloud, std::min<int>(sfCount, 0), "sx");
	bool syFound = PopulateSFCombo(sy, cloud, std::min<int>(sfCount, 1), "sy");
	bool szFound = PopulateSFCombo(sz, cloud, std::min<int>(sfCount, 2), "sz");

	return sxFound && syFound && szFound;
}

void qM3C2Dialog::setupPrecisionMapsTab()
{
	precisionMapsGroupBox->setEnabled(false);

	if (!m_cloud1 || !m_cloud2)
	{
		assert(false);
		return;
	}

	if (m_cloud1->hasScalarFields() && m_cloud2->hasScalarFields())
	{
		bool wasChecked = precisionMapsGroupBox->isChecked();
		bool auto1 = PopulatePMFields(c1SxComboBox, c1SyComboBox, c1SzComboBox, *m_cloud1);
		bool auto2 = PopulatePMFields(c2SxComboBox, c2SyComboBox, c2SzComboBox, *m_cloud2);
		precisionMapsGroupBox->setChecked(wasChecked && (auto1 && auto2));
		precisionMapsGroupBox->setEnabled(true);
	}
}

void qM3C2Dialog::swapClouds()
{
	setClouds(m_cloud2, m_cloud1);
	updateNormalComboBox();
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

	//cloud #2
	cloud2LineEdit->setText(GetEntityName(cloud2));
	showCloud2CheckBox->blockSignals(true);
	showCloud2CheckBox->setChecked(cloud2->isVisible());
	showCloud2CheckBox->blockSignals(false);

	if (s_firstTimeInit)
	{
		//on initialization, try to guess some parameters from the input clouds
		guessParams(true);
		s_firstTimeInit = false;
	}

	setupPrecisionMapsTab();
}

void qM3C2Dialog::onUpdateNormalComboBoxChanged(int)
{
	int selectedItem = normalSourceComboBox->currentIndex() >= 0 ? normalSourceComboBox->currentData().toInt() : -1;
	switch (selectedItem)
	{
	case qM3C2Normals::USE_CLOUD1_NORMALS:
	case qM3C2Normals::USE_CORE_POINTS_NORMALS:
		normParamsFrame->setEnabled(false);
		normalScaleDoubleSpinBox->setEnabled(false);
		break;
	default:
		normParamsFrame->setEnabled(true);
		normalScaleDoubleSpinBox->setEnabled(true);
		break;
	}
}

void qM3C2Dialog::updateNormalComboBox()
{
	int previouslySelectedItem = normalSourceComboBox->currentIndex() >= 0 ? normalSourceComboBox->currentData().toInt() : -1;
	int lastIndex = -1;
	normalSourceComboBox->clear();
	normalSourceComboBox->addItem("Compute normals (on core points)", QVariant(qM3C2Normals::DEFAULT_MODE));
	++lastIndex;
	//if (previouslySelectedItem == qM3C2Normals::DEFAULT_MODE)
	{
		normalSourceComboBox->setCurrentIndex(lastIndex); //default mode
	}
	
	if (m_cloud1 && m_cloud1->hasNormals())
	{
		normalSourceComboBox->addItem("Use cloud #1 normals", QVariant(qM3C2Normals::USE_CLOUD1_NORMALS));
		++lastIndex;
		if (previouslySelectedItem == qM3C2Normals::USE_CLOUD1_NORMALS || previouslySelectedItem < 0)
		{
			normalSourceComboBox->setCurrentIndex(lastIndex);
			previouslySelectedItem = qM3C2Normals::USE_CLOUD1_NORMALS;
		}
	}
	
	if (cpUseOtherCloudRadioButton->isChecked())
	{
		//return the cloud currently selected in the combox box
		ccPointCloud* otherCloud = GetCloudFromCombo(cpOtherCloudComboBox, m_app->dbRootObject());
		if (otherCloud && otherCloud->hasNormals())
		{
			normalSourceComboBox->addItem("Use core points normals", QVariant(qM3C2Normals::USE_CORE_POINTS_NORMALS));
			++lastIndex;
			if (previouslySelectedItem == qM3C2Normals::USE_CORE_POINTS_NORMALS || previouslySelectedItem < 0)
			{
				normalSourceComboBox->setCurrentIndex(lastIndex);
				previouslySelectedItem = qM3C2Normals::USE_CORE_POINTS_NORMALS;
			}
		}
	}
}

ccPointCloud* qM3C2Dialog::getCorePointsCloud() const
{
	if (m_corePointsCloud)
	{
		return m_corePointsCloud;
	}
	else if (cpUseCloud1RadioButton->isChecked())
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
		return nullptr;
	}
}

ccPointCloud* qM3C2Dialog::getNormalsOrientationCloud() const
{
	if (normOriUseCloudRadioButton->isChecked())
	{
		//return the cloud currently selected in the combox box
		return GetCloudFromCombo(normOriCloudComboBox, m_app->dbRootObject());
	}
	else
	{
		return nullptr;
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
	if (normalSourceComboBox->currentIndex() >= 0)
	{
		int selectedItem = normalSourceComboBox->currentData().toInt();
		if (selectedItem == qM3C2Normals::USE_CLOUD1_NORMALS)
		{
			assert(m_cloud1 && m_cloud1->hasNormals());
			return qM3C2Normals::USE_CLOUD1_NORMALS;
		}
		else if (selectedItem == qM3C2Normals::USE_CORE_POINTS_NORMALS)
		{
			return qM3C2Normals::USE_CORE_POINTS_NORMALS;
		}
	}

	//otherwise we are in the default mode
	if (normMultiScaleRadioButton->isChecked())
	{
		return qM3C2Normals::MULTI_SCALE_MODE;
	}
	else if (normVertRadioButton->isChecked())
	{
		return qM3C2Normals::VERT_MODE;
	}
	else if (normHorizRadioButton->isChecked())
	{
		return qM3C2Normals::HORIZ_MODE;
	}
	else /*if (normDefaultRadioButton->isChecked())*/
	{
		return qM3C2Normals::DEFAULT_MODE;
	}
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
	double normalScale = settings.value("NormalScale", normalScaleDoubleSpinBox->value()).toDouble();
	int normModeInt = settings.value("NormalMode", static_cast<int>(getNormalsComputationMode())).toInt();
	double normMinScale = settings.value("NormalMinScale", minScaleDoubleSpinBox->value()).toDouble();
	double normStep = settings.value("NormalStep", stepScaleDoubleSpinBox->value()).toDouble();
	double normMaxScale = settings.value("NormalMaxScale", maxScaleDoubleSpinBox->value()).toDouble();
	bool normUseCorePoints = settings.value("NormalUseCorePoints", normUseCorePointsCheckBox->isChecked()).toBool();
	int normPreferredOri = settings.value("NormalPreferedOri", normOriPreferredComboBox->currentIndex()).toInt();

	double seachScale = settings.value("SearchScale", cylDiameterDoubleSpinBox->value()).toDouble();
	double searchDepth = settings.value("SearchDepth", cylHalfHeightDoubleSpinBox->value()).toDouble();

	double subsampleRadius = settings.value("SubsampleRadius", cpSubsamplingDoubleSpinBox->value()).toDouble();
	bool subsampleEnabled = settings.value("SubsampleEnabled", cpSubsampleRadioButton->isChecked()).toBool();

	double registrationError = settings.value("RegistrationError", rmsDoubleSpinBox->value()).toDouble();
	bool registrationErrorEnabled = settings.value("RegistrationErrorEnabled", rmsCheckBox->isChecked()).toBool();

	bool useSinglePass4Depth = settings.value("UseSinglePass4Depth", useSinglePass4DepthCheckBox->isChecked()).toBool();
	bool positiveSearchOnly = settings.value("PositiveSearchOnly", positiveSearchOnlyCheckBox->isChecked()).toBool();
	bool useMedian = settings.value("UseMedian", useMedianCheckBox->isChecked()).toBool();

	bool useMinPoints4Stat = settings.value("UseMinPoints4Stat", useMinPoints4StatCheckBox->isChecked()).toBool();
	int minPoints4Stat = settings.value("MinPoints4Stat", minPoints4StatSpinBox->value()).toInt();

	int projDestIndex = settings.value("ProjDestIndex", projDestComboBox->currentIndex()).toInt();
	bool useOriginalCloud = settings.value("UseOriginalCloud", useOriginalCloudCheckBox->isChecked()).toBool();

	bool exportStdDevInfo = settings.value("ExportStdDevInfo", exportStdDevInfoCheckBox->isChecked()).toBool();
	bool exportDensityAtProjScale = settings.value("ExportDensityAtProjScale", exportDensityAtProjScaleCheckBox->isChecked()).toBool();

	int maxThreadCount = settings.value("MaxThreadCount", maxThreadCountSpinBox->maximum()).toInt();

	bool usePrecisionMaps = settings.value("UsePrecisionMaps", precisionMapsGroupBox->isChecked()).toBool();
	double pm1Scale = settings.value("PM1Scale", pm1ScaleDoubleSpinBox->value()).toDouble();
	double pm2Scale = settings.value("PM2Scale", pm2ScaleDoubleSpinBox->value()).toDouble();

	//apply parameters
	normalScaleDoubleSpinBox->setValue(normalScale);
	switch(normModeInt)
	{
	case qM3C2Normals::USE_CLOUD1_NORMALS:
	case qM3C2Normals::USE_CORE_POINTS_NORMALS:
	{
		bool found = false;
		for (int i = 0; i < normalSourceComboBox->count(); ++i)
		{
			if (normalSourceComboBox->itemData(i) == normModeInt)
			{
				normalSourceComboBox->setCurrentIndex(i);
				found = true;
				break;
			}
		}
		if (!found)
		{
			ccLog::Warning("Can't restore the previous normal computation method (cloud #1 or core points has no normals)");
		}
	}
	break;
	
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
		cpSubsampleRadioButton->setChecked(true);
	else
		cpUseCloud1RadioButton->setChecked(true);

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

	precisionMapsGroupBox->setChecked(usePrecisionMaps);
	pm1ScaleDoubleSpinBox->setValue(pm1Scale);
	pm2ScaleDoubleSpinBox->setValue(pm2Scale);
}

void qM3C2Dialog::saveParamsToPersistentSettings()
{
	QSettings settings("qM3C2");
	saveParamsTo(settings);
}

void qM3C2Dialog::saveParamsTo(QSettings& settings)
{
	//save parameters
	settings.setValue("NormalScale", normalScaleDoubleSpinBox->value());
	settings.setValue("NormalMode", static_cast<int>(getNormalsComputationMode()));
	settings.setValue("NormalMinScale", minScaleDoubleSpinBox->value());
	settings.setValue("NormalStep", stepScaleDoubleSpinBox->value());
	settings.setValue("NormalMaxScale", maxScaleDoubleSpinBox->value());
	settings.setValue("NormalUseCorePoints", normUseCorePointsCheckBox->isChecked());
	settings.setValue("NormalPreferedOri", normOriPreferredComboBox->currentIndex());

	settings.setValue("SearchScale", cylDiameterDoubleSpinBox->value());
	settings.setValue("SearchDepth", cylHalfHeightDoubleSpinBox->value());

	settings.setValue("SubsampleRadius", cpSubsamplingDoubleSpinBox->value());
	settings.setValue("SubsampleEnabled", cpSubsampleRadioButton->isChecked());

	settings.setValue("RegistrationError", rmsDoubleSpinBox->value());
	settings.setValue("RegistrationErrorEnabled", rmsCheckBox->isChecked());

	settings.setValue("UseSinglePass4Depth", useSinglePass4DepthCheckBox->isChecked());
	settings.setValue("PositiveSearchOnly", positiveSearchOnlyCheckBox->isChecked());
	settings.setValue("UseMedian", useMedianCheckBox->isChecked());

	settings.setValue("UseMinPoints4Stat", useMinPoints4StatCheckBox->isChecked());
	settings.setValue("MinPoints4Stat", minPoints4StatSpinBox->value());

	settings.setValue("ProjDestIndex", projDestComboBox->currentIndex());
	settings.setValue("UseOriginalCloud", useOriginalCloudCheckBox->isChecked());

	settings.setValue("ExportStdDevInfo", exportStdDevInfoCheckBox->isChecked());
	settings.setValue("ExportDensityAtProjScale", exportDensityAtProjScaleCheckBox->isChecked());

	settings.setValue("MaxThreadCount", maxThreadCountSpinBox->value());

	settings.setValue("UsePrecisionMaps", precisionMapsGroupBox->isChecked());
	settings.setValue("PM1Scale", pm1ScaleDoubleSpinBox->value());
	settings.setValue("PM2Scale", pm2ScaleDoubleSpinBox->value());
}

void qM3C2Dialog::loadParamsFromFile()
{
	//select file to open
	QString filename;
	{
		QSettings settings("qM3C2");
		QString currentPath = settings.value("currentPath", ccFileUtils::defaultDocPath()).toString();

		filename = QFileDialog::getOpenFileName(this, "Load M3C2 parameters", currentPath, "*.txt");
		if (filename.isEmpty())
			return;

		//we update current file path
		currentPath = QFileInfo(filename).absolutePath();
		settings.setValue("currentPath", currentPath);
	}

	loadParamsFromFile(filename);
}

bool qM3C2Dialog::loadParamsFromFile(QString filename)
{
	QSettings fileSettings(filename, QSettings::IniFormat);
	//check validity
	if (!fileSettings.contains("M3C2VER"))
	{
		QMessageBox::critical(this, "Invalid file", "File doesn't seem to be a valid M3C2 parameters file ('M3C2VER' not found)!");
		return false;
	}

	loadParamsFrom(fileSettings);

	return true;
}

void qM3C2Dialog::saveParamsToFile()
{
	//select file to save
	QString filename;
	{
		QSettings settings("qM3C2");
		QString currentPath = settings.value("currentPath", ccFileUtils::defaultDocPath()).toString();

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
