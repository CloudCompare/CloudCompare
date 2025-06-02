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

#include "ccSubsamplingDlg.h"
#include "ui_subsamplingDlg.h"

//CCCoreLib
#include <CloudSamplingTools.h>
#include <ScalarField.h>

//qCC_db
#include <ccGenericPointCloud.h>

//Qt
#include <QSettings>

#include "ccPointCloud.h"
#include "ccScalarField.h"

//Exponent of the 'log' scale used for 'SPATIAL' interval
static const double SPACE_RANGE_EXPONENT = 0.05;

ccSubsamplingDlg::ccSubsamplingDlg(unsigned maxPointCount, double maxCloudRadius, QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, m_maxPointCount(maxPointCount)
	, m_maxRadius(maxCloudRadius)
	, m_sfModEnabled(false)
	, m_sfMin(0)
	, m_sfMax(0)
	, m_ui( new Ui::SubsamplingDialog )
{
	m_ui->setupUi(this);

	m_ui->samplingMethodComboBox->addItem( tr( "Random" ) );
	m_ui->samplingMethodComboBox->addItem( tr( "Random (%)" ) );
	m_ui->samplingMethodComboBox->addItem( tr( "Spatial" ) );
	m_ui->samplingMethodComboBox->addItem( tr( "Octree" ) );

	connect(m_ui->slider,					&QSlider::sliderMoved,								this, &ccSubsamplingDlg::sliderMoved);
	connect(m_ui->valueDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccSubsamplingDlg::valueChanged);
	connect(m_ui->samplingMethodComboBox,	qOverload<int>(&QComboBox::currentIndexChanged),	this, &ccSubsamplingDlg::changeSamplingMethod);

	m_ui->samplingMethodComboBox->setCurrentIndex(SPATIAL);
	sliderMoved(m_ui->slider->sliderPosition());

	// Init the 'last used values' (used when switching from one method to another)
	m_lastUsedValues[RANDOM] = static_cast<double>(maxPointCount);
	m_lastUsedValues[RANDOM_PERCENT] = 100.0;
	m_lastUsedValues[SPATIAL] = maxCloudRadius;
	m_lastUsedValues[OCTREE] = static_cast<double>(CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL);
}

ccSubsamplingDlg::~ccSubsamplingDlg()
{
	delete m_ui;
}

CCCoreLib::ReferenceCloud* ccSubsamplingDlg::getSampledCloud(ccGenericPointCloud* cloud, CCCoreLib::GenericProgressCallback* progressCb/*=nullptr*/) const {
	CCCoreLib::ReferenceCloud *result = nullptr;

	try {
		result = DoSubSample(subSamplingParams(), cloud, progressCb);
	} catch (std::exception& e) {
		ccLog::Warning(QString("[ccSubsamplingDlg::getSampledCloud] Failed to subsample %1: %2!").arg(cloud->getName(), e.what()));
	}
	return result;
}


bool SubsamplingParams::Spatial::tryEnableSfModulation(const ccScalarField& sf) {
	sfMin = CCCoreLib::NAN_VALUE;
	sfMax = CCCoreLib::NAN_VALUE;
	if (!ccScalarField::ValidValue(sfMin) || sfMin > sf.getMin())
		sfMin = sf.getMin();
	if (!ccScalarField::ValidValue(sfMax) || sfMax < sf.getMax())
		sfMax = sf.getMax();

	if (!ccScalarField::ValidValue(sfMin) || !ccScalarField::ValidValue(sfMax))
	{
		// warn the user, don't use 'Use Active SF' and keep going
		ccLog::Warning(QObject::tr("\tCan't use 'scalar field modulation': scalar field '%1' has invalid min/max values.").arg(QString::fromStdString(sf.getName())));
		modulationEnabled = false;
	} else {
		modulationEnabled = true;
	}
	return modulationEnabled;
}


CCCoreLib::ReferenceCloud* DoSubSample(const SubsamplingParams &params, ccGenericPointCloud *cloud, CCCoreLib::GenericProgressCallback *progressCb /*=nullptr*/) {
	if (!cloud || cloud->size() == 0)
	{
		throw std::runtime_error("Invalid input cloud!");
	}

	switch (params.method)
	{
		case SubsamplingParams::Method::Random:
		{
			return CCCoreLib::CloudSamplingTools::subsampleCloudRandomly(	cloud,
																			params.randomCount,
																			progressCb);
		}
	case SubsamplingParams::Method::RandomPercent:
		{
			unsigned count = cloud->size();
			count = static_cast<unsigned>(count * params.randomPercent);
			return CCCoreLib::CloudSamplingTools::subsampleCloudRandomly(	cloud,
																			count,
																			progressCb);
		}
	case SubsamplingParams::Method::Spatial:
		{
			ccOctree::Shared octree = cloud->getOctree();
			if (!octree && params.spatial.useOctree)
			{
				octree = cloud->computeOctree(progressCb);

				if (!octree) {
					throw std::runtime_error("Failed to compute octree!");
				}
			}

			return CCCoreLib::CloudSamplingTools::resampleCloudSpatially(	cloud,
																			params.spatial.minDist,
																			params.spatial.modulationParams(),
																			octree.data(),
																			progressCb);
		}
	case SubsamplingParams::Method::Octree:
		{
			ccOctree::Shared octree = cloud->getOctree();
			if (!octree) {
				octree = cloud->computeOctree(progressCb);

				if (!octree) {
					throw std::runtime_error("Failed to compute octree!");
				}
			}
			const unsigned char level = params.octreeLevel;
			if (level > CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL) {
				throw std::runtime_error("Octree level out of range!");
			}
			return CCCoreLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(	cloud,
																					level,
																					CCCoreLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
																					progressCb,
																					octree.data());
		}
	}

	// unhandled subsampling method
	assert(false);
	return nullptr;
}

SubsamplingParams ccSubsamplingDlg::subSamplingParams() const {
	SubsamplingParams params = { };

	switch (m_ui->samplingMethodComboBox->currentIndex())
	{
	case RANDOM:
		{
			assert(m_ui->valueDoubleSpinBox->value() >= 0);
			const auto count = static_cast<unsigned>(m_ui->valueDoubleSpinBox->value());
			params.method = SubsamplingParams::Method::Random;
			params.randomCount = count;
			return params;
		}
	case RANDOM_PERCENT:
		{
			assert(m_ui->valueDoubleSpinBox->value() >= 0);
			params.method = SubsamplingParams::Method::RandomPercent;
			params.randomPercent = m_ui->valueDoubleSpinBox->value();
			return params;
		}
	case SPATIAL: {
		params.method = SubsamplingParams::Method::Spatial;
		params.spatial.minDist = m_ui->valueDoubleSpinBox->value();
		params.spatial.useOctree = true;
		params.spatial.modulationEnabled = m_ui->sfGroupBox->isEnabled() && m_ui->sfGroupBox->isChecked();
		if (params.spatial.modulationEnabled) {
			params.spatial.sfMax = m_sfMax;
			params.spatial.sfMin = m_sfMin;
			params.spatial.sfMinSpacing = m_ui->minSFSpacingDoubleSpinBox->value();
			params.spatial.sfMaxSpacing = m_ui->maxSFSpacingDoubleSpinBox->value();
		}
		return params;
	}
	case OCTREE:
		{
			params.method = SubsamplingParams::Method::Octree;
			assert(m_ui->valueDoubleSpinBox->value() >= 0);
			const auto level = static_cast<unsigned char>(m_ui->valueDoubleSpinBox->value());
			assert(level <= CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL);
			params.octreeLevel = level;
			return params;
		}
	}

	return params;
}


void ccSubsamplingDlg::updateLabels()
{
	switch(m_ui->samplingMethodComboBox->currentIndex())
	{
	case RANDOM:
		m_ui->labelSliderMin->setText( tr( "none" ) );
		m_ui->labelSliderMax->setText( tr( "all" ) );
		m_ui->valueLabel->setText( tr( "remaining points" ) );
		break;
	case RANDOM_PERCENT:
		m_ui->labelSliderMin->setText( tr( "none" ) );
		m_ui->labelSliderMax->setText( tr( "all" ) );
		m_ui->valueLabel->setText(tr("remaining points") + "(%)" );
		break;
	case SPATIAL:
		m_ui->labelSliderMin->setText( tr( "large" ) );
		m_ui->labelSliderMax->setText( tr( "small" ) );
		m_ui->valueLabel->setText( tr( "min. space between points" ) );
		break;
	case OCTREE:
		m_ui->labelSliderMin->setText( tr( "min" ) );
		m_ui->labelSliderMax->setText( tr( "max" ) );
		m_ui->valueLabel->setText( tr( "subdivision level" ) );
		break;
	default:
		break;
	}
}

void ccSubsamplingDlg::sliderMoved(int sliderPos)
{
	double sliderRange = static_cast<double>(m_ui->slider->maximum() - m_ui->slider->minimum());
	double rate = (sliderPos - m_ui->slider->minimum()) / sliderRange;
	if (m_ui->samplingMethodComboBox->currentIndex() == SPATIAL)
	{
		rate = pow(rate, SPACE_RANGE_EXPONENT);
		rate = 1.0 - rate;
	}

	double valueRange = static_cast<double>(m_ui->valueDoubleSpinBox->maximum() - m_ui->valueDoubleSpinBox->minimum());
	double newValue = m_ui->valueDoubleSpinBox->minimum() + rate * valueRange;
	m_ui->valueDoubleSpinBox->setValue(newValue);
}

void ccSubsamplingDlg::valueChanged(double value)
{
	double valueRange = static_cast<double>(m_ui->valueDoubleSpinBox->maximum() - m_ui->valueDoubleSpinBox->minimum());
	double rate = (value - m_ui->valueDoubleSpinBox->minimum()) / valueRange;

	if (m_ui->samplingMethodComboBox->currentIndex() == SPATIAL)
	{
		rate = 1.0 - rate;
		rate = pow(rate, 1.0 / SPACE_RANGE_EXPONENT);

		if (m_sfModEnabled && !m_ui->sfGroupBox->isChecked())
		{
			m_ui->minSFSpacingDoubleSpinBox->setValue(value);
			m_ui->maxSFSpacingDoubleSpinBox->setValue(value);
		}
	}

	double sliderRange = static_cast<double>(m_ui->slider->maximum() - m_ui->slider->minimum());
	int newSliderPos = m_ui->slider->minimum() + static_cast<int>(rate * sliderRange);

	//remember the last used value
	m_lastUsedValues[m_ui->samplingMethodComboBox->currentIndex()] = value;

	m_ui->slider->blockSignals(true);
	m_ui->slider->setSliderPosition(newSliderPos);
	m_ui->slider->blockSignals(false);
}

void ccSubsamplingDlg::changeSamplingMethod(int index)
{
	m_ui->sfGroupBox->setEnabled(false);

	//update the labels
	m_ui->valueDoubleSpinBox->blockSignals(true);
	switch (index)
	{
	case RANDOM:
		{
			m_ui->valueDoubleSpinBox->setDecimals(0);
			m_ui->valueDoubleSpinBox->setMinimum(1.0);
			m_ui->valueDoubleSpinBox->setMaximum(static_cast<double>(m_maxPointCount));
			m_ui->valueDoubleSpinBox->setSingleStep(1.0);
			m_ui->valueDoubleSpinBox->setEnabled(true);
			m_ui->valueDoubleSpinBox->setSuffix(QString());
	}
		break;
	case RANDOM_PERCENT:
		{
			m_ui->valueDoubleSpinBox->setDecimals(3);
			m_ui->valueDoubleSpinBox->setSuffix("%");
			m_ui->valueDoubleSpinBox->setMinimum(0.0);
			m_ui->valueDoubleSpinBox->setMaximum(100.0);
			m_ui->valueDoubleSpinBox->setSingleStep(1.0);
			m_ui->valueDoubleSpinBox->setEnabled(true);
		}
		break;
	case SPATIAL:
		{
			m_ui->valueDoubleSpinBox->setDecimals(4);
			m_ui->valueDoubleSpinBox->setMinimum(0.0);
			m_ui->valueDoubleSpinBox->setMaximum(m_maxRadius);
			double step = m_maxRadius / 1000.0;
			m_ui->valueDoubleSpinBox->setSingleStep(step);
			m_ui->minSFSpacingDoubleSpinBox->setMaximum(m_maxRadius);
			m_ui->minSFSpacingDoubleSpinBox->setSingleStep(step);
			m_ui->maxSFSpacingDoubleSpinBox->setMaximum(m_maxRadius);
			m_ui->maxSFSpacingDoubleSpinBox->setSingleStep(step);
			m_ui->sfGroupBox->setEnabled(m_sfModEnabled);
			m_ui->valueDoubleSpinBox->setDisabled(m_ui->sfGroupBox->isEnabled() && m_ui->sfGroupBox->isChecked());
			m_ui->valueDoubleSpinBox->setSuffix(QString());
	}
		break;
	case OCTREE:
		{
			m_ui->valueDoubleSpinBox->setDecimals(0);
			m_ui->valueDoubleSpinBox->setMinimum(1.0);
			m_ui->valueDoubleSpinBox->setMaximum(static_cast<double>(CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL));
			m_ui->valueDoubleSpinBox->setSingleStep(1.0);
			m_ui->valueDoubleSpinBox->setEnabled(true);
			m_ui->valueDoubleSpinBox->setSuffix(QString());
	}
		break;
	default:
		break;
	}

	m_ui->valueDoubleSpinBox->setValue(m_lastUsedValues[index]);

	m_ui->valueDoubleSpinBox->blockSignals(false);

	// force the updates of the labels and the sliders
	updateLabels();
	valueChanged(m_ui->valueDoubleSpinBox->value());
}

void ccSubsamplingDlg::enableSFModulation(ScalarType sfMin, ScalarType sfMax)
{
	m_sfModEnabled = CCCoreLib::ScalarField::ValidValue(sfMin) && CCCoreLib::ScalarField::ValidValue(sfMax);
	if (!m_sfModEnabled)
	{
		ccLog::Warning("[ccSubsamplingDlg::enableSFModulation] Invalid input SF values");
		return;
	}

	if (sfMin == sfMax)
	{
		m_sfModEnabled = false;
		ccLog::Warning("[ccSubsamplingDlg::enableSFModulation] Can't modulate sampling if all scalar values are the same");
		return;
	}

	m_sfMin = sfMin;
	m_sfMax = sfMax;

	m_ui->sfGroupBox->setEnabled(m_ui->samplingMethodComboBox->currentIndex() == SPATIAL);
	m_ui->minSFlabel->setText(QString::number(sfMin));
	m_ui->maxSFlabel->setText(QString::number(sfMax));
}

void ccSubsamplingDlg::saveToPersistentSettings() const
{
	QSettings settings;
	settings.beginGroup("SubsamplingDialog");
	{
		settings.setValue("method", m_ui->samplingMethodComboBox->currentIndex());
		settings.setValue("value", m_ui->valueDoubleSpinBox->value());
		settings.setValue("useActiveSF", m_ui->sfGroupBox->isChecked());
		settings.setValue("minSFRatio", m_ui->minSFSpacingDoubleSpinBox->value());
		settings.setValue("maxSFRatio", m_ui->maxSFSpacingDoubleSpinBox->value());
	}
	settings.endGroup();
}

void ccSubsamplingDlg::loadFromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("SubsamplingDialog");
	{
		int methodIndex = settings.value("method", m_ui->samplingMethodComboBox->currentIndex()).toInt();
		double value = settings.value("value", m_ui->valueDoubleSpinBox->value()).toDouble();
		bool useActiveSF = settings.value("useActiveSF", m_ui->sfGroupBox->isChecked()).toBool();
		double minSFRatio = settings.value("minSFRatio", m_ui->minSFSpacingDoubleSpinBox->value()).toDouble();
		double maxSFRatio = settings.value("maxSFRatio", m_ui->maxSFSpacingDoubleSpinBox->value()).toDouble();

		// force the update of the dialog
		m_ui->samplingMethodComboBox->blockSignals(true);
		m_ui->samplingMethodComboBox->setCurrentIndex(methodIndex);
		m_ui->samplingMethodComboBox->blockSignals(false);
		changeSamplingMethod(methodIndex);

		m_ui->valueDoubleSpinBox->setValue(value);
		m_ui->sfGroupBox->setChecked(useActiveSF);
		m_ui->minSFSpacingDoubleSpinBox->setValue(minSFRatio);
		m_ui->maxSFSpacingDoubleSpinBox->setValue(maxSFRatio);
	}
	settings.endGroup();
}
