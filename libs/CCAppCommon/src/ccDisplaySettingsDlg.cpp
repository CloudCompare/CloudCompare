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

#include "ccDisplaySettingsDlg.h"
#include "ccApplicationBase.h"

#include "ui_displaySettingsDlg.h"

//local
#include "ccQtHelpers.h"
#include "ccPersistentSettings.h"

#include <ccLog.h>

//Qt
#include <QColorDialog>
#include <QStyleFactory>

#include <cassert>
#include <QSettings>

//Default 'min cloud size' for LoD  when VBOs are activated
constexpr double s_defaultMaxVBOCloudSizeM = 50.0;

ccDisplaySettingsDlg::ccDisplaySettingsDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, m_ui( new Ui::DisplaySettingsDlg )
	, m_defaultAppStyleIndex(-1)
{
	m_ui->setupUi(this);

	connect(m_ui->ambientColorButton,		&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::changeLightAmbientColor);
	connect(m_ui->diffuseColorButton,		&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::changeLightDiffuseColor);
	connect(m_ui->specularColorButton,		&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::changeLightSpecularColor);
	connect(m_ui->meshBackColorButton,		&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::changeMeshBackDiffuseColor);
	connect(m_ui->meshSpecularColorButton,	&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::changeMeshSpecularColor);
	connect(m_ui->meshFrontColorButton,		&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::changeMeshFrontDiffuseColor);
	connect(m_ui->bbColorButton,			&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::changeBBColor);
	connect(m_ui->bkgColorButton,			&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::changeBackgroundColor);
	connect(m_ui->labelBkgColorButton,		&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::changeLabelBackgroundColor);
	connect(m_ui->labelMarkerColorButton,	&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::changeLabelMarkerColor);
	connect(m_ui->pointsColorButton,		&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::changePointsColor);
	connect(m_ui->textColorButton,			&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::changeTextColor);

	connect(m_ui->doubleSidedCheckBox,             &QCheckBox::toggled, this, [&](bool state) { m_parameters.lightDoubleSided = state; });
	connect(m_ui->enableGradientCheckBox,          &QCheckBox::toggled, this, [&](bool state) { m_parameters.drawBackgroundGradient = state; });
	connect(m_ui->showCrossCheckBox,               &QCheckBox::toggled, this, [&](bool state) { m_parameters.displayCross = state; });
	connect(m_ui->colorScaleShowHistogramCheckBox, &QCheckBox::toggled, this, [&](bool state) { m_parameters.colorScaleShowHistogram = state; });
	connect(m_ui->useColorScaleShaderCheckBox,     &QCheckBox::toggled, this, [&](bool state) { m_parameters.colorScaleUseShader = state; });
	connect(m_ui->decimateMeshBox,                 &QCheckBox::toggled, this, [&](bool state) { m_parameters.decimateMeshOnMove = state; });
	connect(m_ui->decimateCloudBox,                &QCheckBox::toggled, this, [&](bool state) { m_parameters.decimateCloudOnMove = state; });
	connect(m_ui->drawRoundedPointsCheckBox,       &QCheckBox::toggled, this, [&](bool state) { m_parameters.drawRoundedPoints = state; });
	connect(m_ui->singleClickPickingCheckBox,	   &QCheckBox::toggled, this, [&](bool state) { m_parameters.singleClickPicking = state; });
	connect(m_ui->autoDisplayNormalsCheckBox,      &QCheckBox::toggled, this, [&](bool state) { m_options.normalsDisplayedByDefault = state; });
	connect(m_ui->useNativeDialogsCheckBox,        &QCheckBox::toggled, this, [&](bool state) { m_options.useNativeDialogs = state; });
	connect(m_ui->confirmQuitCheckBox,             &QCheckBox::toggled, this, [&](bool state) { m_options.confirmQuit = state; });

	connect(m_ui->useVBOCheckBox,	&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::changeVBOUsage);

	connect(m_ui->colorRampWidthSpinBox,	qOverload<int>(&QSpinBox::valueChanged), this, &ccDisplaySettingsDlg::changeColorScaleRampWidth);

	connect(m_ui->defaultFontSizeSpinBox,	qOverload<int>(&QSpinBox::valueChanged), this, &ccDisplaySettingsDlg::changeDefaultFontSize);
	connect(m_ui->labelFontSizeSpinBox,		qOverload<int>(&QSpinBox::valueChanged), this, &ccDisplaySettingsDlg::changeLabelFontSize);
	connect(m_ui->numberPrecisionSpinBox,	qOverload<int>(&QSpinBox::valueChanged), this, &ccDisplaySettingsDlg::changeNumberPrecision);
	connect(m_ui->labelOpacitySpinBox,		qOverload<int>(&QSpinBox::valueChanged), this, &ccDisplaySettingsDlg::changeLabelOpacity);
	connect(m_ui->labelMarkerSizeSpinBox,	qOverload<int>(&QSpinBox::valueChanged), this, &ccDisplaySettingsDlg::changeLabelMarkerSize);

	connect(m_ui->zoomSpeedDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ccDisplaySettingsDlg::changeZoomSpeed);
	connect(m_ui->maxCloudSizeDoubleSpinBox,	qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ccDisplaySettingsDlg::changeMaxCloudSize);
	connect(m_ui->maxMeshSizeDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ccDisplaySettingsDlg::changeMaxMeshSize);

	connect(m_ui->autoComputeOctreeComboBox,	qOverload<int>(&QComboBox::currentIndexChanged), this, &ccDisplaySettingsDlg::changeAutoComputeOctreeOption);
	connect(m_ui->pickingCursorComboBox,		qOverload<int>(&QComboBox::currentIndexChanged), this, &ccDisplaySettingsDlg::changePickingCursor);
	connect(m_ui->logVerbosityComboBox,			qOverload<int>(&QComboBox::currentIndexChanged), this, &ccDisplaySettingsDlg::changeLogVerbosity);

	connect(m_ui->okButton,		&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::doAccept);
	connect(m_ui->applyButton,	&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::apply);
	connect(m_ui->resetButton,	&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::reset);
	connect(m_ui->cancelButton,	&QAbstractButton::clicked,	this, &ccDisplaySettingsDlg::doReject);

	// fill the application style combo-box
	{
		// Store the default style (= the active one when the dialog is first shown)
		// Which is necessarily the key currently stored in persistent settings
		// since this is saved each time the current setting is selected,
		// and since it's the first time the user opens the settings for this session,
		// then, the style was not changed.
		QSettings settings;
		settings.beginGroup(ccPS::AppStyle());
		const QString defaultStyleName = settings.value("style").toString();
		settings.endGroup();

		// fill the combo-box
		QStringList appStyles = QStyleFactory::keys();
		for (const auto & style : appStyles)
		{
			m_ui->appStyleComboBox->addItem(style);
		}

		m_ui->appStyleComboBox->addItem(QStringLiteral("QDarkStyleSheet::Light"));
		m_ui->appStyleComboBox->addItem(QStringLiteral("QDarkStyleSheet::Dark"));

		for (int i = 0; i < m_ui->appStyleComboBox->count(); ++i)
		{
			if (m_ui->appStyleComboBox->itemText(i).compare(defaultStyleName, Qt::CaseInsensitive) == 0)
			{
				m_defaultAppStyleIndex = i;
			}
		}

		m_ui->appStyleComboBox->setCurrentIndex(m_defaultAppStyleIndex);
	}

	m_oldParameters = m_parameters = ccGui::Parameters();
	m_oldOptions = m_options = ccOptions::Instance();

	refresh();

	setUpdatesEnabled(true);
}

ccDisplaySettingsDlg::~ccDisplaySettingsDlg()
{
	delete m_ui;
	m_ui = nullptr;
}

void ccDisplaySettingsDlg::refresh()
{
	// Parameters
	{
		const ccColor::Rgbaf& ac = m_parameters.lightAmbientColor;
		m_lightAmbientColor.setRgbF(ac.r, ac.g, ac.b, ac.a);
		ccQtHelpers::SetButtonColor(m_ui->ambientColorButton, m_lightAmbientColor);

		const ccColor::Rgbaf& dc = m_parameters.lightDiffuseColor;
		m_lightDiffuseColor.setRgbF(dc.r, dc.g, dc.b, dc.a);
		ccQtHelpers::SetButtonColor(m_ui->diffuseColorButton, m_lightDiffuseColor);

		const ccColor::Rgbaf& sc = m_parameters.lightSpecularColor;
		m_lightSpecularColor.setRgbF(sc.r, sc.g, sc.b, sc.a);
		ccQtHelpers::SetButtonColor(m_ui->specularColorButton, m_lightSpecularColor);

		const ccColor::Rgbaf& mbc = m_parameters.meshBackDiff;
		m_meshBackDiff.setRgbF(mbc.r, mbc.g, mbc.b, mbc.a);
		ccQtHelpers::SetButtonColor(m_ui->meshBackColorButton, m_meshBackDiff);

		const ccColor::Rgbaf& mspec = m_parameters.meshSpecular;
		m_meshSpecularColor.setRgbF(mspec.r, mspec.g, mspec.b, mspec.a);
		ccQtHelpers::SetButtonColor(m_ui->meshSpecularColorButton, m_meshSpecularColor);

		const ccColor::Rgbaf& mfc = m_parameters.meshFrontDiff;
		m_meshFrontDiff.setRgbF(mfc.r, mfc.g, mfc.b, mfc.a);
		ccQtHelpers::SetButtonColor(m_ui->meshFrontColorButton, m_meshFrontDiff);

		const ccColor::Rgba& bbc = m_parameters.bbDefaultCol;
		m_bbDefaultCol.setRgb(bbc.r, bbc.g, bbc.b, bbc.a);
		ccQtHelpers::SetButtonColor(m_ui->bbColorButton, m_bbDefaultCol);

		const ccColor::Rgbub& bgc = m_parameters.backgroundCol;
		m_backgroundCol.setRgb(bgc.r, bgc.g, bgc.b);
		ccQtHelpers::SetButtonColor(m_ui->bkgColorButton, m_backgroundCol);

		const ccColor::Rgba& lblbc = m_parameters.labelBackgroundCol;
		m_labelBackgroundCol.setRgb(lblbc.r, lblbc.g, lblbc.b, lblbc.a);
		ccQtHelpers::SetButtonColor(m_ui->labelBkgColorButton, m_labelBackgroundCol);

		const ccColor::Rgba& lblmc = m_parameters.labelMarkerCol;
		m_labelMarkerCol.setRgb(lblmc.r, lblmc.g, lblmc.b, lblmc.a);
		ccQtHelpers::SetButtonColor(m_ui->labelMarkerColorButton, m_labelMarkerCol);

		const ccColor::Rgba& pdc = m_parameters.pointsDefaultCol;
		m_pointsDefaultCol.setRgb(pdc.r, pdc.g, pdc.b, pdc.a);
		ccQtHelpers::SetButtonColor(m_ui->pointsColorButton, m_pointsDefaultCol);

		const ccColor::Rgba& tdc = m_parameters.textDefaultCol;
		m_textDefaultCol.setRgb(tdc.r, tdc.g, tdc.b, tdc.a);
		ccQtHelpers::SetButtonColor(m_ui->textColorButton, m_textDefaultCol);

		m_ui->doubleSidedCheckBox->setChecked(m_parameters.lightDoubleSided);
		m_ui->enableGradientCheckBox->setChecked(m_parameters.drawBackgroundGradient);
		m_ui->decimateMeshBox->setChecked(m_parameters.decimateMeshOnMove);
		m_ui->maxMeshSizeDoubleSpinBox->setValue(m_parameters.minLoDMeshSize / 1000000.0);
		m_ui->decimateCloudBox->setChecked(m_parameters.decimateCloudOnMove);
		m_ui->drawRoundedPointsCheckBox->setChecked(m_parameters.drawRoundedPoints);
		m_ui->maxCloudSizeDoubleSpinBox->setValue(m_parameters.minLoDCloudSize / 1000000.0);
		m_ui->useVBOCheckBox->setChecked(m_parameters.useVBOs);
		m_ui->showCrossCheckBox->setChecked(m_parameters.displayCross);
		m_ui->singleClickPickingCheckBox->setChecked(m_parameters.singleClickPicking);

		m_ui->colorScaleShowHistogramCheckBox->setChecked(m_parameters.colorScaleShowHistogram);
		m_ui->useColorScaleShaderCheckBox->setChecked(m_parameters.colorScaleUseShader);
		m_ui->useColorScaleShaderCheckBox->setEnabled(m_parameters.colorScaleShaderSupported);
		m_ui->colorRampWidthSpinBox->setValue(m_parameters.colorScaleRampWidth);

		m_ui->defaultFontSizeSpinBox->setValue(m_parameters.defaultFontSize);
		m_ui->labelFontSizeSpinBox->setValue(m_parameters.labelFontSize);
		m_ui->numberPrecisionSpinBox->setValue(m_parameters.displayedNumPrecision);
		m_ui->labelOpacitySpinBox->setValue(m_parameters.labelOpacity);
		m_ui->labelMarkerSizeSpinBox->setValue(m_parameters.labelMarkerSize);

		m_ui->zoomSpeedDoubleSpinBox->setValue(m_parameters.zoomSpeed);

		m_ui->autoComputeOctreeComboBox->setCurrentIndex(m_parameters.autoComputeOctree);

		switch (m_parameters.pickingCursorShape)
		{
		default:
			assert(false);
		case Qt::CrossCursor:
			m_ui->pickingCursorComboBox->setCurrentIndex(0);
			break;
		case Qt::PointingHandCursor:
			m_ui->pickingCursorComboBox->setCurrentIndex(1);
			break;
		}

		m_ui->logVerbosityComboBox->setCurrentIndex(std::min(static_cast<int>(m_parameters.logVerbosityLevel), static_cast<int>(ccLog::LOG_WARNING)));
	}

	// Options
	{
		m_ui->autoDisplayNormalsCheckBox->setChecked(m_options.normalsDisplayedByDefault);
		m_ui->useNativeDialogsCheckBox->setChecked(m_options.useNativeDialogs);
		m_ui->confirmQuitCheckBox->setChecked(m_options.confirmQuit);
	}

	update();
}

void ccDisplaySettingsDlg::changeLightDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(m_lightDiffuseColor, this);
	if (!newCol.isValid())
		return;

	m_lightDiffuseColor = newCol;
	ccQtHelpers::SetButtonColor(m_ui->diffuseColorButton, m_lightDiffuseColor);
	m_parameters.lightDiffuseColor = ccColor::FromQColoraf(m_lightDiffuseColor);
}

void ccDisplaySettingsDlg::changeLightAmbientColor()
{
	QColor newCol = QColorDialog::getColor(m_lightAmbientColor, this);
	if (!newCol.isValid())
		return;

	m_lightAmbientColor = newCol;
	ccQtHelpers::SetButtonColor(m_ui->ambientColorButton, m_lightAmbientColor);
	m_parameters.lightAmbientColor = ccColor::FromQColoraf(m_lightAmbientColor);

	update();
}

void ccDisplaySettingsDlg::changeLightSpecularColor()
{
	QColor newCol = QColorDialog::getColor(m_lightSpecularColor, this);
	if (!newCol.isValid())
		return;

	m_lightSpecularColor = newCol;
	ccQtHelpers::SetButtonColor(m_ui->specularColorButton, m_lightSpecularColor);
	m_parameters.lightSpecularColor = ccColor::FromQColoraf(m_lightSpecularColor);

	update();
}

void ccDisplaySettingsDlg::changeMeshFrontDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(m_meshFrontDiff, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	m_meshFrontDiff = newCol;
	ccQtHelpers::SetButtonColor(m_ui->meshFrontColorButton, m_meshFrontDiff);

	m_parameters.meshFrontDiff = ccColor::FromQColoraf(m_meshFrontDiff);

	update();
}

void ccDisplaySettingsDlg::changeMeshBackDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(m_meshBackDiff, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	m_meshBackDiff = newCol;
	ccQtHelpers::SetButtonColor(m_ui->meshBackColorButton, m_meshBackDiff);
	m_parameters.meshBackDiff = ccColor::FromQColoraf(m_meshBackDiff);

	update();
}

void ccDisplaySettingsDlg::changeMeshSpecularColor()
{
	QColor newCol = QColorDialog::getColor(m_meshSpecularColor, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	m_meshSpecularColor = newCol;
	ccQtHelpers::SetButtonColor(m_ui->meshSpecularColorButton, m_meshSpecularColor);
	m_parameters.meshSpecular = ccColor::FromQColoraf(m_meshSpecularColor);

	update();
}

void ccDisplaySettingsDlg::changePointsColor()
{
	QColor newCol = QColorDialog::getColor(m_pointsDefaultCol, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	m_pointsDefaultCol = newCol;
	ccQtHelpers::SetButtonColor(m_ui->pointsColorButton, m_pointsDefaultCol);
	m_parameters.pointsDefaultCol = ccColor::FromQColora(m_pointsDefaultCol);

	update();
}

void ccDisplaySettingsDlg::changeBBColor()
{
	QColor newCol = QColorDialog::getColor(m_bbDefaultCol, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	m_bbDefaultCol = newCol;
	ccQtHelpers::SetButtonColor(m_ui->bbColorButton, m_bbDefaultCol);
	m_parameters.bbDefaultCol = ccColor::FromQColora(m_bbDefaultCol);

	update();
}

void ccDisplaySettingsDlg::changeTextColor()
{
	QColor newCol = QColorDialog::getColor(m_textDefaultCol, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	m_textDefaultCol = newCol;
	ccQtHelpers::SetButtonColor(m_ui->textColorButton, m_textDefaultCol);
	m_parameters.textDefaultCol = ccColor::FromQColora(m_textDefaultCol);

	update();
}

void ccDisplaySettingsDlg::changeBackgroundColor()
{
	QColor newCol = QColorDialog::getColor(m_backgroundCol, this);
	if (!newCol.isValid())
		return;

	m_backgroundCol = newCol;
	ccQtHelpers::SetButtonColor(m_ui->bkgColorButton, m_backgroundCol);
	m_parameters.backgroundCol = ccColor::FromQColor(m_backgroundCol);

	update();
}

void ccDisplaySettingsDlg::changeLabelBackgroundColor()
{
	QColor newCol = QColorDialog::getColor(m_labelBackgroundCol, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	m_labelBackgroundCol = newCol;
	ccQtHelpers::SetButtonColor(m_ui->labelBkgColorButton, m_labelBackgroundCol);
	m_parameters.labelBackgroundCol = ccColor::FromQColora(m_labelBackgroundCol);

	update();
}

void ccDisplaySettingsDlg::changeLabelMarkerColor()
{
	QColor newCol = QColorDialog::getColor(m_labelMarkerCol, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	m_labelMarkerCol = newCol;
	ccQtHelpers::SetButtonColor(m_ui->labelMarkerColorButton, m_labelMarkerCol);
	m_parameters.labelMarkerCol = ccColor::FromQColora(m_labelMarkerCol);

	update();
}

void ccDisplaySettingsDlg::changeMaxMeshSize(double val)
{
	m_parameters.minLoDMeshSize = static_cast<unsigned>(val * 1000000);
}

void ccDisplaySettingsDlg::changeMaxCloudSize(double val)
{
	m_parameters.minLoDCloudSize = static_cast<unsigned>(val * 1000000);
}

void ccDisplaySettingsDlg::changeVBOUsage()
{
	m_parameters.useVBOs = m_ui->useVBOCheckBox->isChecked();
	if (m_parameters.useVBOs && m_ui->maxCloudSizeDoubleSpinBox->value() < s_defaultMaxVBOCloudSizeM)
	{
		m_ui->maxCloudSizeDoubleSpinBox->setValue(s_defaultMaxVBOCloudSizeM);
	}
}

void ccDisplaySettingsDlg::changePickingCursor(int index)
{
	switch (index)
	{
	case 0:
		m_parameters.pickingCursorShape = Qt::CrossCursor;
		break;
	case 1:
		m_parameters.pickingCursorShape = Qt::PointingHandCursor;
		break;
	default:
		assert(false);
		break;
	}
}

void ccDisplaySettingsDlg::changeLogVerbosity(int index)
{
	if (index >= 0 && index < ccLog::LOG_ERROR)
	{
		m_parameters.logVerbosityLevel = static_cast<ccLog::MessageLevelFlags>(index);
	}
	else
	{
		// unexepected value
		assert(false);
	}
}


void ccDisplaySettingsDlg::changeColorScaleRampWidth(int val)
{
	if (val < 2)
		return;
	m_parameters.colorScaleRampWidth = static_cast<unsigned>(val);
}

void ccDisplaySettingsDlg::changeDefaultFontSize(int val)
{
	if (val < 0)
		return;
	m_parameters.defaultFontSize = static_cast<unsigned>(val);
}

void ccDisplaySettingsDlg::changeLabelFontSize(int val)
{
	if (val < 0)
		return;
	m_parameters.labelFontSize = static_cast<unsigned>(val);
}

void ccDisplaySettingsDlg::changeNumberPrecision(int val)
{
	if (val < 0)
		return;
	m_parameters.displayedNumPrecision = static_cast<unsigned>(val);
}

void ccDisplaySettingsDlg::changeZoomSpeed(double val)
{
	m_parameters.zoomSpeed = val;
}

void ccDisplaySettingsDlg::changeAutoComputeOctreeOption(int index)
{
	assert(index >= 0 && index < 3);
	m_parameters.autoComputeOctree = static_cast<ccGui::ParamStruct::ComputeOctreeForPicking>(index);
}

void ccDisplaySettingsDlg::changeLabelOpacity(int val)
{
	if (val < 0 || val > 100)
		return;
	m_parameters.labelOpacity = static_cast<unsigned>(val);
}

void ccDisplaySettingsDlg::changeLabelMarkerSize(int val)
{
	if (val <= 0)
		return;

	m_parameters.labelMarkerSize = static_cast<unsigned>(val);
}

void ccDisplaySettingsDlg::doReject()
{
	ccGui::Set(m_oldParameters);
	ccOptions::Set(m_oldOptions);

	Q_EMIT aspectHasChanged();

	reject();
}

void ccDisplaySettingsDlg::reset()
{
	m_parameters.reset();
	m_options.reset();
	m_ui->appStyleComboBox->setCurrentIndex(m_defaultAppStyleIndex);

	refresh();
}

void ccDisplaySettingsDlg::apply()
{
	ccGui::Set(m_parameters);
	ccOptions::Set(m_options);

	// application style
	{
		QString style = m_ui->appStyleComboBox->currentText();
		ccApp->setAppStyle(style);
	}

	// log verbosity
	if (ccLog::VerbosityLevel() != m_parameters.logVerbosityLevel)
	{
		ccLog::SetVerbosityLevel(m_parameters.logVerbosityLevel);
		ccLog::Print(QString("New log verbosity level: %1").arg(m_parameters.logVerbosityLevel));
	}

	Q_EMIT aspectHasChanged();
}

void ccDisplaySettingsDlg::doAccept()
{
	apply();

	m_parameters.toPersistentSettings();
	m_options.toPersistentSettings();

	accept();
}
