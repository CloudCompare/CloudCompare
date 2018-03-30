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

//local
#include "ccQtHelpers.h"

//Qt
#include <QColor>
#include <QColorDialog>

//Default 'min cloud size' for LoD  when VBOs are activated
static const double s_defaultMaxVBOCloudSizeM = 50.0;

ccDisplayOptionsDlg::ccDisplayOptionsDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::DisplayOptionsDlg()
{
	setupUi(this);

	connect(ambientColorButton,              SIGNAL(clicked()),         this, SLOT(changeLightAmbientColor()));
	connect(diffuseColorButton,              SIGNAL(clicked()),         this, SLOT(changeLightDiffuseColor()));
	connect(specularColorButton,             SIGNAL(clicked()),         this, SLOT(changeLightSpecularColor()));
	connect(meshBackColorButton,             SIGNAL(clicked()),         this, SLOT(changeMeshBackDiffuseColor()));
	connect(meshSpecularColorButton,         SIGNAL(clicked()),         this, SLOT(changeMeshSpecularColor()));
	connect(meshFrontColorButton,            SIGNAL(clicked()),         this, SLOT(changeMeshFrontDiffuseColor()));
	connect(bbColorButton,                   SIGNAL(clicked()),         this, SLOT(changeBBColor()));
	connect(bkgColorButton,                  SIGNAL(clicked()),         this, SLOT(changeBackgroundColor()));
	connect(labelBkgColorButton,             SIGNAL(clicked()),         this, SLOT(changeLabelBackgroundColor()));
	connect(labelMarkerColorButton,          SIGNAL(clicked()),         this, SLOT(changeLabelMarkerColor()));
	connect(pointsColorButton,               SIGNAL(clicked()),         this, SLOT(changePointsColor()));
	connect(textColorButton,                 SIGNAL(clicked()),         this, SLOT(changeTextColor()));

	connect(doubleSidedCheckBox,             &QCheckBox::toggled, this, [&](bool state) { parameters.lightDoubleSided = state; });
	connect(enableGradientCheckBox,          &QCheckBox::toggled, this, [&](bool state) { parameters.drawBackgroundGradient = state; });
	connect(showCrossCheckBox,               &QCheckBox::toggled, this, [&](bool state) { parameters.displayCross = state; });
	connect(colorScaleShowHistogramCheckBox, &QCheckBox::toggled, this, [&](bool state) { parameters.colorScaleShowHistogram = state; });
	connect(useColorScaleShaderCheckBox,     &QCheckBox::toggled, this, [&](bool state) { parameters.colorScaleUseShader = state; });
	connect(decimateMeshBox,                 &QCheckBox::toggled, this, [&](bool state) { parameters.decimateMeshOnMove = state; });
	connect(decimateCloudBox,                &QCheckBox::toggled, this, [&](bool state) { parameters.decimateCloudOnMove = state; });
	connect(drawRoundedPointsCheckBox,       &QCheckBox::toggled, this, [&](bool state) { parameters.drawRoundedPoints = state; });
	connect(autoDisplayNormalsCheckBox,      &QCheckBox::toggled, this, [&](bool state) { options.normalsDisplayedByDefault = state; });
	connect(useNativeDialogsCheckBox,        &QCheckBox::toggled, this, [&](bool state) { options.useNativeDialogs = state; });

	connect(useVBOCheckBox,                  SIGNAL(clicked()),         this, SLOT(changeVBOUsage()));

	connect(colorRampWidthSpinBox,           SIGNAL(valueChanged(int)), this, SLOT(changeColorScaleRampWidth(int)));

	connect(defaultFontSizeSpinBox,          SIGNAL(valueChanged(int)), this, SLOT(changeDefaultFontSize(int)));
	connect(labelFontSizeSpinBox,            SIGNAL(valueChanged(int)), this, SLOT(changeLabelFontSize(int)));
	connect(numberPrecisionSpinBox,          SIGNAL(valueChanged(int)), this, SLOT(changeNumberPrecision(int)));
	connect(labelOpacitySpinBox,             SIGNAL(valueChanged(int)), this, SLOT(changeLabelOpacity(int)));
	connect(labelMarkerSizeSpinBox,          SIGNAL(valueChanged(int)), this, SLOT(changeLabelMarkerSize(int)));

	connect(zoomSpeedDoubleSpinBox,          SIGNAL(valueChanged(double)), this, SLOT(changeZoomSpeed(double)));
	connect(maxCloudSizeDoubleSpinBox,       SIGNAL(valueChanged(double)), this, SLOT(changeMaxCloudSize(double)));
	connect(maxMeshSizeDoubleSpinBox,        SIGNAL(valueChanged(double)), this, SLOT(changeMaxMeshSize(double)));

	connect(autoComputeOctreeComboBox,       SIGNAL(currentIndexChanged(int)), this, SLOT(changeAutoComputeOctreeOption(int)));

	connect(okButton,                        SIGNAL(clicked()),         this, SLOT(doAccept()));
	connect(applyButton,                     SIGNAL(clicked()),         this, SLOT(apply()));
	connect(resetButton,                     SIGNAL(clicked()),         this, SLOT(reset()));
	connect(cancelButton,                    SIGNAL(clicked()),         this, SLOT(doReject()));

	oldParameters = parameters = ccGui::Parameters();
	oldOptions = options = ccOptions::Instance();

	refresh();

	setUpdatesEnabled(true);
}

void ccDisplayOptionsDlg::refresh()
{
	const ccColor::Rgbaf& ac = parameters.lightAmbientColor;
	lightAmbientColor.setRgbF(ac.r, ac.g, ac.b, ac.a);
	ccQtHelpers::SetButtonColor(ambientColorButton, lightAmbientColor);

	const ccColor::Rgbaf& dc = parameters.lightDiffuseColor;
	lightDiffuseColor.setRgbF(dc.r, dc.g, dc.b, dc.a);
	ccQtHelpers::SetButtonColor(diffuseColorButton, lightDiffuseColor);

	const ccColor::Rgbaf& sc = parameters.lightSpecularColor;
	lightSpecularColor.setRgbF(sc.r, sc.g, sc.b, sc.a);
	ccQtHelpers::SetButtonColor(specularColorButton, lightSpecularColor);

	const ccColor::Rgbaf& mbc = parameters.meshBackDiff;
	meshBackDiff.setRgbF(mbc.r, mbc.g, mbc.b, mbc.a);
	ccQtHelpers::SetButtonColor(meshBackColorButton, meshBackDiff);

	const ccColor::Rgbaf& mspec = parameters.meshSpecular;
	meshSpecularColor.setRgbF(mspec.r, mspec.g, mspec.b, mspec.a);
	ccQtHelpers::SetButtonColor(meshSpecularColorButton, meshSpecularColor);

	const ccColor::Rgbaf& mfc = parameters.meshFrontDiff;
	meshFrontDiff.setRgbF(mfc.r, mfc.g, mfc.b, mfc.a);
	ccQtHelpers::SetButtonColor(meshFrontColorButton, meshFrontDiff);

	const ccColor::Rgbub& bbc = parameters.bbDefaultCol;
	bbDefaultCol.setRgb(bbc.r, bbc.g, bbc.b);
	ccQtHelpers::SetButtonColor(bbColorButton, bbDefaultCol);

	const ccColor::Rgbub& bgc = parameters.backgroundCol;
	backgroundCol.setRgb(bgc.r, bgc.g, bgc.b);
	ccQtHelpers::SetButtonColor(bkgColorButton, backgroundCol);

	const ccColor::Rgbub& lblbc = parameters.labelBackgroundCol;
	labelBackgroundCol.setRgb(lblbc.r, lblbc.g, lblbc.b);
	ccQtHelpers::SetButtonColor(labelBkgColorButton, labelBackgroundCol);

	const ccColor::Rgbub& lblmc = parameters.labelMarkerCol;
	labelMarkerCol.setRgb(lblmc.r, lblmc.g, lblmc.b);
	ccQtHelpers::SetButtonColor(labelMarkerColorButton, labelMarkerCol);

	const ccColor::Rgbub& pdc = parameters.pointsDefaultCol;
	pointsDefaultCol.setRgb(pdc.r, pdc.g, pdc.b);
	ccQtHelpers::SetButtonColor(pointsColorButton, pointsDefaultCol);

	const ccColor::Rgbub& tdc = parameters.textDefaultCol;
	textDefaultCol.setRgb(tdc.r, tdc.g, tdc.b);
	ccQtHelpers::SetButtonColor(textColorButton, textDefaultCol);

	doubleSidedCheckBox->setChecked(parameters.lightDoubleSided);
	enableGradientCheckBox->setChecked(parameters.drawBackgroundGradient);
	decimateMeshBox->setChecked(parameters.decimateMeshOnMove);
	maxMeshSizeDoubleSpinBox->setValue(parameters.minLoDMeshSize / 1000000.0);
	decimateCloudBox->setChecked(parameters.decimateCloudOnMove);
	drawRoundedPointsCheckBox->setChecked(parameters.drawRoundedPoints);
	maxCloudSizeDoubleSpinBox->setValue(parameters.minLoDCloudSize / 1000000.0);
	useVBOCheckBox->setChecked(parameters.useVBOs);
	showCrossCheckBox->setChecked(parameters.displayCross);

	colorScaleShowHistogramCheckBox->setChecked(parameters.colorScaleShowHistogram);
	useColorScaleShaderCheckBox->setChecked(parameters.colorScaleUseShader);
	useColorScaleShaderCheckBox->setEnabled(parameters.colorScaleShaderSupported);
	colorRampWidthSpinBox->setValue(parameters.colorScaleRampWidth);

	defaultFontSizeSpinBox->setValue(parameters.defaultFontSize);
	labelFontSizeSpinBox->setValue(parameters.labelFontSize);
	numberPrecisionSpinBox->setValue(parameters.displayedNumPrecision);
	labelOpacitySpinBox->setValue(parameters.labelOpacity);
	labelMarkerSizeSpinBox->setValue(parameters.labelMarkerSize);

	zoomSpeedDoubleSpinBox->setValue(parameters.zoomSpeed);
	
	autoComputeOctreeComboBox->setCurrentIndex(parameters.autoComputeOctree);

	autoDisplayNormalsCheckBox->setChecked(options.normalsDisplayedByDefault);
	useNativeDialogsCheckBox->setChecked(options.useNativeDialogs);

	update();
}

void ccDisplayOptionsDlg::changeLightDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(lightDiffuseColor, this);
	if (!newCol.isValid())
		return;

	lightDiffuseColor = newCol;
	ccQtHelpers::SetButtonColor(diffuseColorButton, lightDiffuseColor);
	parameters.lightDiffuseColor = ccColor::FromQColoraf(lightDiffuseColor);
}

void ccDisplayOptionsDlg::changeLightAmbientColor()
{
	QColor newCol = QColorDialog::getColor(lightAmbientColor, this);
	if (!newCol.isValid())
		return;

	lightAmbientColor = newCol;
	ccQtHelpers::SetButtonColor(ambientColorButton, lightAmbientColor);
	parameters.lightAmbientColor = ccColor::FromQColoraf(lightAmbientColor);

	update();
}

void ccDisplayOptionsDlg::changeLightSpecularColor()
{
	QColor newCol = QColorDialog::getColor(lightSpecularColor, this);
	if (!newCol.isValid())
		return;

	lightSpecularColor = newCol;
	ccQtHelpers::SetButtonColor(specularColorButton, lightSpecularColor);
	parameters.lightSpecularColor = ccColor::FromQColoraf(lightSpecularColor);

	update();
}

void ccDisplayOptionsDlg::changeMeshFrontDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(meshFrontDiff, this);
	if (!newCol.isValid())
		return;

	meshFrontDiff = newCol;
	ccQtHelpers::SetButtonColor(meshFrontColorButton, meshFrontDiff);

	parameters.meshFrontDiff = ccColor::FromQColoraf(meshFrontDiff);

	update();
}

void ccDisplayOptionsDlg::changeMeshBackDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(meshBackDiff, this);
	if (!newCol.isValid())
		return;

	meshBackDiff = newCol;
	ccQtHelpers::SetButtonColor(meshBackColorButton, meshBackDiff);
	parameters.meshBackDiff = ccColor::FromQColoraf(meshBackDiff);

	update();
}

void ccDisplayOptionsDlg::changeMeshSpecularColor()
{
	QColor newCol = QColorDialog::getColor(meshSpecularColor, this);
	if (!newCol.isValid())
		return;

	meshSpecularColor = newCol;
	ccQtHelpers::SetButtonColor(meshSpecularColorButton, meshSpecularColor);
	parameters.meshSpecular = ccColor::FromQColoraf(meshSpecularColor);

	update();
}

void ccDisplayOptionsDlg::changePointsColor()
{
	QColor newCol = QColorDialog::getColor(pointsDefaultCol, this);
	if (!newCol.isValid())
		return;

	pointsDefaultCol = newCol;
	ccQtHelpers::SetButtonColor(pointsColorButton, pointsDefaultCol);
	parameters.pointsDefaultCol = ccColor::FromQColor(pointsDefaultCol);

	update();
}

void ccDisplayOptionsDlg::changeBBColor()
{
	QColor newCol = QColorDialog::getColor(bbDefaultCol, this);
	if (!newCol.isValid())
		return;

	bbDefaultCol = newCol;
	ccQtHelpers::SetButtonColor(bbColorButton, bbDefaultCol);
	parameters.bbDefaultCol = ccColor::FromQColor(bbDefaultCol);

	update();
}

void ccDisplayOptionsDlg::changeTextColor()
{
	QColor newCol = QColorDialog::getColor(textDefaultCol, this);
	if (!newCol.isValid())
		return;

	textDefaultCol = newCol;
	ccQtHelpers::SetButtonColor(textColorButton, textDefaultCol);
	parameters.textDefaultCol = ccColor::FromQColor(textDefaultCol);

	update();
}

void ccDisplayOptionsDlg::changeBackgroundColor()
{
	QColor newCol = QColorDialog::getColor(backgroundCol, this);
	if (!newCol.isValid())
		return;

	backgroundCol = newCol;
	ccQtHelpers::SetButtonColor(bkgColorButton, backgroundCol);
	parameters.backgroundCol = ccColor::FromQColor(backgroundCol);

	update();
}

void ccDisplayOptionsDlg::changeLabelBackgroundColor()
{
	QColor newCol = QColorDialog::getColor(labelBackgroundCol, this);
	if (!newCol.isValid())
		return;

	labelBackgroundCol = newCol;
	ccQtHelpers::SetButtonColor(labelBkgColorButton, labelBackgroundCol);
	parameters.labelBackgroundCol = ccColor::FromQColor(labelBackgroundCol);

	update();
}

void ccDisplayOptionsDlg::changeLabelMarkerColor()
{
	QColor newCol = QColorDialog::getColor(labelMarkerCol, this);
	if (!newCol.isValid())
		return;

	labelMarkerCol = newCol;
	ccQtHelpers::SetButtonColor(labelMarkerColorButton, labelMarkerCol);

	parameters.labelMarkerCol = ccColor::FromQColor(labelMarkerCol);

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
	parameters.useVBOs = useVBOCheckBox->isChecked();
	if (parameters.useVBOs && maxCloudSizeDoubleSpinBox->value() < s_defaultMaxVBOCloudSizeM)
	{
		maxCloudSizeDoubleSpinBox->setValue(s_defaultMaxVBOCloudSizeM);
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
