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

#include "ccDisplayOptionsDlg.h"
#include "ui_displayOptionsDlg.h"

//local
#include "ccQtHelpers.h"

//Qt
#include <QColorDialog>

#include <cassert>

//Default 'min cloud size' for LoD  when VBOs are activated
constexpr double s_defaultMaxVBOCloudSizeM = 50.0;

ccDisplayOptionsDlg::ccDisplayOptionsDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, m_ui( new Ui::DisplayOptionsDlg )
{
	m_ui->setupUi(this);

	connect(m_ui->ambientColorButton,		&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::changeLightAmbientColor);
	connect(m_ui->diffuseColorButton,		&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::changeLightDiffuseColor);
	connect(m_ui->specularColorButton,		&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::changeLightSpecularColor);
	connect(m_ui->meshBackColorButton,		&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::changeMeshBackDiffuseColor);
	connect(m_ui->meshSpecularColorButton,	&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::changeMeshSpecularColor);
	connect(m_ui->meshFrontColorButton,		&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::changeMeshFrontDiffuseColor);
	connect(m_ui->bbColorButton,			&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::changeBBColor);
	connect(m_ui->bkgColorButton,			&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::changeBackgroundColor);
	connect(m_ui->labelBkgColorButton,		&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::changeLabelBackgroundColor);
	connect(m_ui->labelMarkerColorButton,	&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::changeLabelMarkerColor);
	connect(m_ui->pointsColorButton,		&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::changePointsColor);
	connect(m_ui->textColorButton,			&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::changeTextColor);

	connect(m_ui->doubleSidedCheckBox,             &QCheckBox::toggled, this, [&](bool state) { parameters.lightDoubleSided = state; });
	connect(m_ui->enableGradientCheckBox,          &QCheckBox::toggled, this, [&](bool state) { parameters.drawBackgroundGradient = state; });
	connect(m_ui->showCrossCheckBox,               &QCheckBox::toggled, this, [&](bool state) { parameters.displayCross = state; });
	connect(m_ui->colorScaleShowHistogramCheckBox, &QCheckBox::toggled, this, [&](bool state) { parameters.colorScaleShowHistogram = state; });
	connect(m_ui->useColorScaleShaderCheckBox,     &QCheckBox::toggled, this, [&](bool state) { parameters.colorScaleUseShader = state; });
	connect(m_ui->decimateMeshBox,                 &QCheckBox::toggled, this, [&](bool state) { parameters.decimateMeshOnMove = state; });
	connect(m_ui->decimateCloudBox,                &QCheckBox::toggled, this, [&](bool state) { parameters.decimateCloudOnMove = state; });
	connect(m_ui->drawRoundedPointsCheckBox,       &QCheckBox::toggled, this, [&](bool state) { parameters.drawRoundedPoints = state; });
	connect(m_ui->autoDisplayNormalsCheckBox,      &QCheckBox::toggled, this, [&](bool state) { options.normalsDisplayedByDefault = state; });
	connect(m_ui->useNativeDialogsCheckBox,        &QCheckBox::toggled, this, [&](bool state) { options.useNativeDialogs = state; });

	connect(m_ui->useVBOCheckBox,	&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::changeVBOUsage);

	connect(m_ui->colorRampWidthSpinBox,	static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccDisplayOptionsDlg::changeColorScaleRampWidth);

	connect(m_ui->defaultFontSizeSpinBox,	static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccDisplayOptionsDlg::changeDefaultFontSize);
	connect(m_ui->labelFontSizeSpinBox,		static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccDisplayOptionsDlg::changeLabelFontSize);
	connect(m_ui->numberPrecisionSpinBox,	static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccDisplayOptionsDlg::changeNumberPrecision);
	connect(m_ui->labelOpacitySpinBox,		static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccDisplayOptionsDlg::changeLabelOpacity);
	connect(m_ui->labelMarkerSizeSpinBox,	static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ccDisplayOptionsDlg::changeLabelMarkerSize);

	connect(m_ui->zoomSpeedDoubleSpinBox,		static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccDisplayOptionsDlg::changeZoomSpeed);
	connect(m_ui->maxCloudSizeDoubleSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccDisplayOptionsDlg::changeMaxCloudSize);
	connect(m_ui->maxMeshSizeDoubleSpinBox,		static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccDisplayOptionsDlg::changeMaxMeshSize);

	connect(m_ui->autoComputeOctreeComboBox,	static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ccDisplayOptionsDlg::changeAutoComputeOctreeOption);

	connect(m_ui->okButton,		&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::doAccept);
	connect(m_ui->applyButton,	&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::apply);
	connect(m_ui->resetButton,	&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::reset);
	connect(m_ui->cancelButton,	&QAbstractButton::clicked,	this, &ccDisplayOptionsDlg::doReject);

	oldParameters = parameters = ccGui::Parameters();
	oldOptions = options = ccOptions::Instance();

	refresh();

	setUpdatesEnabled(true);
}

ccDisplayOptionsDlg::~ccDisplayOptionsDlg()
{
	delete m_ui;
	m_ui = nullptr;
}

void ccDisplayOptionsDlg::refresh()
{
	const ccColor::Rgbaf& ac = parameters.lightAmbientColor;
	lightAmbientColor.setRgbF(ac.r, ac.g, ac.b, ac.a);
	ccQtHelpers::SetButtonColor(m_ui->ambientColorButton, lightAmbientColor);

	const ccColor::Rgbaf& dc = parameters.lightDiffuseColor;
	lightDiffuseColor.setRgbF(dc.r, dc.g, dc.b, dc.a);
	ccQtHelpers::SetButtonColor(m_ui->diffuseColorButton, lightDiffuseColor);

	const ccColor::Rgbaf& sc = parameters.lightSpecularColor;
	lightSpecularColor.setRgbF(sc.r, sc.g, sc.b, sc.a);
	ccQtHelpers::SetButtonColor(m_ui->specularColorButton, lightSpecularColor);

	const ccColor::Rgbaf& mbc = parameters.meshBackDiff;
	meshBackDiff.setRgbF(mbc.r, mbc.g, mbc.b, mbc.a);
	ccQtHelpers::SetButtonColor(m_ui->meshBackColorButton, meshBackDiff);

	const ccColor::Rgbaf& mspec = parameters.meshSpecular;
	meshSpecularColor.setRgbF(mspec.r, mspec.g, mspec.b, mspec.a);
	ccQtHelpers::SetButtonColor(m_ui->meshSpecularColorButton, meshSpecularColor);

	const ccColor::Rgbaf& mfc = parameters.meshFrontDiff;
	meshFrontDiff.setRgbF(mfc.r, mfc.g, mfc.b, mfc.a);
	ccQtHelpers::SetButtonColor(m_ui->meshFrontColorButton, meshFrontDiff);

	const ccColor::Rgba& bbc = parameters.bbDefaultCol;
	bbDefaultCol.setRgb(bbc.r, bbc.g, bbc.b, bbc.a);
	ccQtHelpers::SetButtonColor(m_ui->bbColorButton, bbDefaultCol);

	const ccColor::Rgbub& bgc = parameters.backgroundCol;
	backgroundCol.setRgb(bgc.r, bgc.g, bgc.b);
	ccQtHelpers::SetButtonColor(m_ui->bkgColorButton, backgroundCol);

	const ccColor::Rgba& lblbc = parameters.labelBackgroundCol;
	labelBackgroundCol.setRgb(lblbc.r, lblbc.g, lblbc.b, lblbc.a);
	ccQtHelpers::SetButtonColor(m_ui->labelBkgColorButton, labelBackgroundCol);

	const ccColor::Rgba& lblmc = parameters.labelMarkerCol;
	labelMarkerCol.setRgb(lblmc.r, lblmc.g, lblmc.b, lblmc.a);
	ccQtHelpers::SetButtonColor(m_ui->labelMarkerColorButton, labelMarkerCol);

	const ccColor::Rgba& pdc = parameters.pointsDefaultCol;
	pointsDefaultCol.setRgb(pdc.r, pdc.g, pdc.b, pdc.a);
	ccQtHelpers::SetButtonColor(m_ui->pointsColorButton, pointsDefaultCol);

	const ccColor::Rgba& tdc = parameters.textDefaultCol;
	textDefaultCol.setRgb(tdc.r, tdc.g, tdc.b, tdc.a);
	ccQtHelpers::SetButtonColor(m_ui->textColorButton, textDefaultCol);

	m_ui->doubleSidedCheckBox->setChecked(parameters.lightDoubleSided);
	m_ui->enableGradientCheckBox->setChecked(parameters.drawBackgroundGradient);
	m_ui->decimateMeshBox->setChecked(parameters.decimateMeshOnMove);
	m_ui->maxMeshSizeDoubleSpinBox->setValue(parameters.minLoDMeshSize / 1000000.0);
	m_ui->decimateCloudBox->setChecked(parameters.decimateCloudOnMove);
	m_ui->drawRoundedPointsCheckBox->setChecked(parameters.drawRoundedPoints);
	m_ui->maxCloudSizeDoubleSpinBox->setValue(parameters.minLoDCloudSize / 1000000.0);
	m_ui->useVBOCheckBox->setChecked(parameters.useVBOs);
	m_ui->showCrossCheckBox->setChecked(parameters.displayCross);

	m_ui->colorScaleShowHistogramCheckBox->setChecked(parameters.colorScaleShowHistogram);
	m_ui->useColorScaleShaderCheckBox->setChecked(parameters.colorScaleUseShader);
	m_ui->useColorScaleShaderCheckBox->setEnabled(parameters.colorScaleShaderSupported);
	m_ui->colorRampWidthSpinBox->setValue(parameters.colorScaleRampWidth);

	m_ui->defaultFontSizeSpinBox->setValue(parameters.defaultFontSize);
	m_ui->labelFontSizeSpinBox->setValue(parameters.labelFontSize);
	m_ui->numberPrecisionSpinBox->setValue(parameters.displayedNumPrecision);
	m_ui->labelOpacitySpinBox->setValue(parameters.labelOpacity);
	m_ui->labelMarkerSizeSpinBox->setValue(parameters.labelMarkerSize);

	m_ui->zoomSpeedDoubleSpinBox->setValue(parameters.zoomSpeed);
	
	m_ui->autoComputeOctreeComboBox->setCurrentIndex(parameters.autoComputeOctree);

	m_ui->autoDisplayNormalsCheckBox->setChecked(options.normalsDisplayedByDefault);
	m_ui->useNativeDialogsCheckBox->setChecked(options.useNativeDialogs);

	update();
}

void ccDisplayOptionsDlg::changeLightDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(lightDiffuseColor, this);
	if (!newCol.isValid())
		return;

	lightDiffuseColor = newCol;
	ccQtHelpers::SetButtonColor(m_ui->diffuseColorButton, lightDiffuseColor);
	parameters.lightDiffuseColor = ccColor::FromQColoraf(lightDiffuseColor);
}

void ccDisplayOptionsDlg::changeLightAmbientColor()
{
	QColor newCol = QColorDialog::getColor(lightAmbientColor, this);
	if (!newCol.isValid())
		return;

	lightAmbientColor = newCol;
	ccQtHelpers::SetButtonColor(m_ui->ambientColorButton, lightAmbientColor);
	parameters.lightAmbientColor = ccColor::FromQColoraf(lightAmbientColor);

	update();
}

void ccDisplayOptionsDlg::changeLightSpecularColor()
{
	QColor newCol = QColorDialog::getColor(lightSpecularColor, this);
	if (!newCol.isValid())
		return;

	lightSpecularColor = newCol;
	ccQtHelpers::SetButtonColor(m_ui->specularColorButton, lightSpecularColor);
	parameters.lightSpecularColor = ccColor::FromQColoraf(lightSpecularColor);

	update();
}

void ccDisplayOptionsDlg::changeMeshFrontDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(meshFrontDiff, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	meshFrontDiff = newCol;
	ccQtHelpers::SetButtonColor(m_ui->meshFrontColorButton, meshFrontDiff);

	parameters.meshFrontDiff = ccColor::FromQColoraf(meshFrontDiff);

	update();
}

void ccDisplayOptionsDlg::changeMeshBackDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(meshBackDiff, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	meshBackDiff = newCol;
	ccQtHelpers::SetButtonColor(m_ui->meshBackColorButton, meshBackDiff);
	parameters.meshBackDiff = ccColor::FromQColoraf(meshBackDiff);

	update();
}

void ccDisplayOptionsDlg::changeMeshSpecularColor()
{
	QColor newCol = QColorDialog::getColor(meshSpecularColor, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	meshSpecularColor = newCol;
	ccQtHelpers::SetButtonColor(m_ui->meshSpecularColorButton, meshSpecularColor);
	parameters.meshSpecular = ccColor::FromQColoraf(meshSpecularColor);

	update();
}

void ccDisplayOptionsDlg::changePointsColor()
{
	QColor newCol = QColorDialog::getColor(pointsDefaultCol, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	pointsDefaultCol = newCol;
	ccQtHelpers::SetButtonColor(m_ui->pointsColorButton, pointsDefaultCol);
	parameters.pointsDefaultCol = ccColor::FromQColora(pointsDefaultCol);

	update();
}

void ccDisplayOptionsDlg::changeBBColor()
{
	QColor newCol = QColorDialog::getColor(bbDefaultCol, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	bbDefaultCol = newCol;
	ccQtHelpers::SetButtonColor(m_ui->bbColorButton, bbDefaultCol);
	parameters.bbDefaultCol = ccColor::FromQColora(bbDefaultCol);

	update();
}

void ccDisplayOptionsDlg::changeTextColor()
{
	QColor newCol = QColorDialog::getColor(textDefaultCol, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	textDefaultCol = newCol;
	ccQtHelpers::SetButtonColor(m_ui->textColorButton, textDefaultCol);
	parameters.textDefaultCol = ccColor::FromQColora(textDefaultCol);

	update();
}

void ccDisplayOptionsDlg::changeBackgroundColor()
{
	QColor newCol = QColorDialog::getColor(backgroundCol, this);
	if (!newCol.isValid())
		return;

	backgroundCol = newCol;
	ccQtHelpers::SetButtonColor(m_ui->bkgColorButton, backgroundCol);
	parameters.backgroundCol = ccColor::FromQColor(backgroundCol);

	update();
}

void ccDisplayOptionsDlg::changeLabelBackgroundColor()
{
	QColor newCol = QColorDialog::getColor(labelBackgroundCol, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	labelBackgroundCol = newCol;
	ccQtHelpers::SetButtonColor(m_ui->labelBkgColorButton, labelBackgroundCol);
	parameters.labelBackgroundCol = ccColor::FromQColora(labelBackgroundCol);

	update();
}

void ccDisplayOptionsDlg::changeLabelMarkerColor()
{
	QColor newCol = QColorDialog::getColor(labelMarkerCol, this, QString(), QColorDialog::ShowAlphaChannel);
	if (!newCol.isValid())
		return;

	labelMarkerCol = newCol;
	ccQtHelpers::SetButtonColor(m_ui->labelMarkerColorButton, labelMarkerCol);

	parameters.labelMarkerCol = ccColor::FromQColora(labelMarkerCol);

	update();
}

void ccDisplayOptionsDlg::changeMaxMeshSize(double val)
{
	parameters.minLoDMeshSize = static_cast<unsigned>(val * 1000000);
}

void ccDisplayOptionsDlg::changeMaxCloudSize(double val)
{
	parameters.minLoDCloudSize = static_cast<unsigned>(val * 1000000);
}

void ccDisplayOptionsDlg::changeVBOUsage()
{
	parameters.useVBOs = m_ui->useVBOCheckBox->isChecked();
	if (parameters.useVBOs && m_ui->maxCloudSizeDoubleSpinBox->value() < s_defaultMaxVBOCloudSizeM)
	{
		m_ui->maxCloudSizeDoubleSpinBox->setValue(s_defaultMaxVBOCloudSizeM);
	}
}

void ccDisplayOptionsDlg::changeColorScaleRampWidth(int val)
{
	if (val < 2)
		return;
	parameters.colorScaleRampWidth = static_cast<unsigned>(val);
}

void ccDisplayOptionsDlg::changeDefaultFontSize(int val)
{
	if (val < 0)
		return;
	parameters.defaultFontSize = static_cast<unsigned>(val);
}

void ccDisplayOptionsDlg::changeLabelFontSize(int val)
{
	if (val < 0)
		return;
	parameters.labelFontSize = static_cast<unsigned>(val);
}

void ccDisplayOptionsDlg::changeNumberPrecision(int val)
{
	if (val < 0)
		return;
	parameters.displayedNumPrecision = static_cast<unsigned>(val);
}

void ccDisplayOptionsDlg::changeZoomSpeed(double val)
{
	parameters.zoomSpeed = val;
}

void ccDisplayOptionsDlg::changeAutoComputeOctreeOption(int index)
{
	assert(index >= 0 && index < 3);
	parameters.autoComputeOctree = static_cast<ccGui::ParamStruct::ComputeOctreeForPicking>(index);
}

void ccDisplayOptionsDlg::changeLabelOpacity(int val)
{
	if (val < 0 || val > 100)
		return;
	parameters.labelOpacity = static_cast<unsigned>(val);
}

void ccDisplayOptionsDlg::changeLabelMarkerSize(int val)
{
	if (val <= 0)
		return;

	parameters.labelMarkerSize = static_cast<unsigned>(val);
}

void ccDisplayOptionsDlg::doReject()
{
	ccGui::Set(oldParameters);
	ccOptions::Set(oldOptions);

	emit aspectHasChanged();

	reject();
}

void ccDisplayOptionsDlg::reset()
{
	parameters.reset();
	options.reset();
	refresh();
}

void ccDisplayOptionsDlg::apply()
{
	ccGui::Set(parameters);
	ccOptions::Set(options);

	emit aspectHasChanged();
}

void ccDisplayOptionsDlg::doAccept()
{
	apply();

	parameters.toPersistentSettings();
	options.toPersistentSettings();

	accept();
}
