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
	connect(enableGradientCheckBox,          SIGNAL(clicked()),         this, SLOT(changeBackgroundGradient()));
	connect(pointsColorButton,               SIGNAL(clicked()),         this, SLOT(changePointsColor()));
	connect(textColorButton,                 SIGNAL(clicked()),         this, SLOT(changeTextColor()));
	connect(decimateMeshBox,                 SIGNAL(clicked()),         this, SLOT(changeMeshDecimation()));
	connect(decimateCloudBox,                SIGNAL(clicked()),         this, SLOT(changeCloudDecimation()));
	connect(useVBOCheckBox,                  SIGNAL(clicked()),         this, SLOT(changeVBOUsage()));
	connect(showCrossCheckBox,               SIGNAL(clicked()),         this, SLOT(changeCrossDisplayed()));

	connect(colorScaleShowHistogramCheckBox, SIGNAL(clicked()),         this, SLOT(changeColorScaleShowHistogram()));
	connect(useColorScaleShaderCheckBox,     SIGNAL(clicked()),         this, SLOT(changeColorScaleUseShader()));
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

	refresh();

	setUpdatesEnabled(true);
}

void ccDisplayOptionsDlg::refresh()
{
	const ccColor::Rgbaf& ac = parameters.lightAmbientColor;
	lightAmbientColor.setRgbF(ac.r,ac.g,ac.b,ac.a);
	ccQtHelpers::SetButtonColor(ambientColorButton,lightAmbientColor);

	const ccColor::Rgbaf& dc = parameters.lightDiffuseColor;
	lightDiffuseColor.setRgbF(dc.r,dc.g,dc.b,dc.a);
	ccQtHelpers::SetButtonColor(diffuseColorButton,lightDiffuseColor);

	const ccColor::Rgbaf& sc = parameters.lightSpecularColor;
	lightSpecularColor.setRgbF(sc.r,sc.g,sc.b,sc.a);
	ccQtHelpers::SetButtonColor(specularColorButton,lightSpecularColor);

	const ccColor::Rgbaf& mbc = parameters.meshBackDiff;
	meshBackDiff.setRgbF(mbc.r,mbc.g,mbc.b,mbc.a);
	ccQtHelpers::SetButtonColor(meshBackColorButton,meshBackDiff);

	const ccColor::Rgbaf& mspec = parameters.meshSpecular;
	meshSpecularColor.setRgbF(mspec.r,mspec.g,mspec.b,mspec.a);
	ccQtHelpers::SetButtonColor(meshSpecularColorButton,meshSpecularColor);

	const ccColor::Rgbaf& mfc = parameters.meshFrontDiff;
	meshFrontDiff.setRgbF(mfc.r,mfc.g,mfc.b,mfc.a);
	ccQtHelpers::SetButtonColor(meshFrontColorButton,meshFrontDiff);

	const ccColor::Rgbub& bbc = parameters.bbDefaultCol;
	bbDefaultCol.setRgb(bbc.r,bbc.g,bbc.b);
	ccQtHelpers::SetButtonColor(bbColorButton,bbDefaultCol);

	const ccColor::Rgbub& bgc = parameters.backgroundCol;
	backgroundCol.setRgb(bgc.r,bgc.g,bgc.b);
	ccQtHelpers::SetButtonColor(bkgColorButton,backgroundCol);

	const ccColor::Rgbub& lblbc = parameters.labelBackgroundCol;
	labelBackgroundCol.setRgb(lblbc.r,lblbc.g,lblbc.b);
	ccQtHelpers::SetButtonColor(labelBkgColorButton,labelBackgroundCol);

	const ccColor::Rgbub& lblmc = parameters.labelMarkerCol;
	labelMarkerCol.setRgb(lblmc.r,lblmc.g,lblmc.b);
	ccQtHelpers::SetButtonColor(labelMarkerColorButton,labelMarkerCol);

	const ccColor::Rgbub& pdc = parameters.pointsDefaultCol;
	pointsDefaultCol.setRgb(pdc.r,pdc.g,pdc.b);
	ccQtHelpers::SetButtonColor(pointsColorButton,pointsDefaultCol);

	const ccColor::Rgbub& tdc = parameters.textDefaultCol;
	textDefaultCol.setRgb(tdc.r,tdc.g,tdc.b);
	ccQtHelpers::SetButtonColor(textColorButton,textDefaultCol);

	enableGradientCheckBox->setChecked(parameters.drawBackgroundGradient);
	decimateMeshBox->setChecked(parameters.decimateMeshOnMove);
	maxMeshSizeDoubleSpinBox->setValue(static_cast<double>(parameters.minLoDMeshSize)/1000000.0);
	decimateCloudBox->setChecked(parameters.decimateCloudOnMove);
	maxCloudSizeDoubleSpinBox->setValue(static_cast<double>(parameters.minLoDCloudSize)/1000000.0);
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

	update();
}

static void QColorToFloat(const QColor& col, ccColor::Rgbaf& rgba)
{
	rgba.r = static_cast<float>(col.redF  ());
	rgba.g = static_cast<float>(col.greenF());
	rgba.b = static_cast<float>(col.blueF ());
	rgba.a = static_cast<float>(col.alphaF());
}

void ccDisplayOptionsDlg::changeLightDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(lightDiffuseColor, this);
	if (!newCol.isValid())
		return;

	lightDiffuseColor = newCol;
	ccQtHelpers::SetButtonColor(diffuseColorButton,lightDiffuseColor);

	ccColor::Rgbaf rgba;
	QColorToFloat(lightDiffuseColor,rgba);

	parameters.lightDiffuseColor = rgba;
}

void ccDisplayOptionsDlg::changeLightAmbientColor()
{
	QColor newCol = QColorDialog::getColor(lightAmbientColor, this);
	if (!newCol.isValid())
		return;

	lightAmbientColor = newCol;
	ccQtHelpers::SetButtonColor(ambientColorButton,lightAmbientColor);

	ccColor::Rgbaf rgba;
	QColorToFloat(lightAmbientColor,rgba);
	parameters.lightAmbientColor = rgba;

	update();
}

void ccDisplayOptionsDlg::changeLightSpecularColor()
{
	QColor newCol = QColorDialog::getColor(lightSpecularColor, this);
	if (!newCol.isValid())
		return;

	lightSpecularColor = newCol;
	ccQtHelpers::SetButtonColor(specularColorButton,lightSpecularColor);

	ccColor::Rgbaf rgba;
	QColorToFloat(lightSpecularColor,rgba);
	parameters.lightSpecularColor = rgba;

	update();
}

void ccDisplayOptionsDlg::changeMeshFrontDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(meshFrontDiff, this);
	if (!newCol.isValid())
		return;

	meshFrontDiff = newCol;
	ccQtHelpers::SetButtonColor(meshFrontColorButton,meshFrontDiff);

	ccColor::Rgbaf rgba;
	QColorToFloat(meshFrontDiff,rgba);
	parameters.meshFrontDiff = rgba;

	update();
}

void ccDisplayOptionsDlg::changeMeshBackDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(meshBackDiff, this);
	if (!newCol.isValid())
		return;

	meshBackDiff = newCol;
	ccQtHelpers::SetButtonColor(meshBackColorButton,meshBackDiff);

	ccColor::Rgbaf rgba;
	QColorToFloat(meshBackDiff,rgba);
	parameters.meshBackDiff = rgba;

	update();
}

void ccDisplayOptionsDlg::changeMeshSpecularColor()
{
	QColor newCol = QColorDialog::getColor(meshSpecularColor, this);
	if (!newCol.isValid())
		return;

	meshSpecularColor = newCol;
	ccQtHelpers::SetButtonColor(meshSpecularColorButton,meshSpecularColor);

	ccColor::Rgbaf rgba;
	QColorToFloat(meshSpecularColor,rgba);
	parameters.meshSpecular = rgba;

	update();
}

void ccDisplayOptionsDlg::changePointsColor()
{
	QColor newCol = QColorDialog::getColor(pointsDefaultCol, this);
	if (!newCol.isValid())
		return;

	pointsDefaultCol = newCol;
	ccQtHelpers::SetButtonColor(pointsColorButton,pointsDefaultCol);

	parameters.pointsDefaultCol = ccColor::Rgb(	static_cast<unsigned char>(pointsDefaultCol.red()),
												static_cast<unsigned char>(pointsDefaultCol.green()),
												static_cast<unsigned char>(pointsDefaultCol.blue()));

	update();
}

void ccDisplayOptionsDlg::changeBBColor()
{
	QColor newCol = QColorDialog::getColor(bbDefaultCol, this);
	if (!newCol.isValid())
		return;

	bbDefaultCol = newCol;
	ccQtHelpers::SetButtonColor(bbColorButton,bbDefaultCol);

	parameters.bbDefaultCol = ccColor::Rgb(	static_cast<unsigned char>(bbDefaultCol.red()),
											static_cast<unsigned char>(bbDefaultCol.green()),
											static_cast<unsigned char>(bbDefaultCol.blue()));

	update();
}

void ccDisplayOptionsDlg::changeTextColor()
{
	QColor newCol = QColorDialog::getColor(textDefaultCol, this);
	if (!newCol.isValid())
		return;

	textDefaultCol = newCol;
	ccQtHelpers::SetButtonColor(textColorButton,textDefaultCol);

	parameters.textDefaultCol = ccColor::Rgb(	static_cast<unsigned char>(textDefaultCol.red()),
												static_cast<unsigned char>(textDefaultCol.green()),
												static_cast<unsigned char>(textDefaultCol.blue()));
	
	update();
}

void ccDisplayOptionsDlg::changeBackgroundColor()
{
	QColor newCol = QColorDialog::getColor(backgroundCol, this);
	if (!newCol.isValid())
		return;

	backgroundCol = newCol;
	ccQtHelpers::SetButtonColor(bkgColorButton,backgroundCol);

	parameters.backgroundCol = ccColor::Rgb(static_cast<unsigned char>(backgroundCol.red()),
											static_cast<unsigned char>(backgroundCol.green()),
											static_cast<unsigned char>(backgroundCol.blue()));

	update();
}

void ccDisplayOptionsDlg::changeLabelBackgroundColor()
{
	QColor newCol = QColorDialog::getColor(labelBackgroundCol, this);
	if (!newCol.isValid())
		return;

	labelBackgroundCol = newCol;
	ccQtHelpers::SetButtonColor(labelBkgColorButton,labelBackgroundCol);

	parameters.labelBackgroundCol = ccColor::Rgbub(	static_cast<unsigned char>(labelBackgroundCol.red()),
													static_cast<unsigned char>(labelBackgroundCol.green()),
													static_cast<unsigned char>(labelBackgroundCol.blue()));

	update();
}

void ccDisplayOptionsDlg::changeLabelMarkerColor()
{
	QColor newCol = QColorDialog::getColor(labelMarkerCol, this);
	if (!newCol.isValid())
		return;

	labelMarkerCol = newCol;
	ccQtHelpers::SetButtonColor(labelMarkerColorButton,labelMarkerCol);

	parameters.labelMarkerCol = ccColor::Rgbub(	static_cast<unsigned char>(labelMarkerCol.red()),
												static_cast<unsigned char>(labelMarkerCol.green()),
												static_cast<unsigned char>(labelMarkerCol.blue()));

	update();
}

void ccDisplayOptionsDlg::changeBackgroundGradient()
{
	parameters.drawBackgroundGradient = enableGradientCheckBox->isChecked();
}

void ccDisplayOptionsDlg::changeMeshDecimation()
{
	parameters.decimateMeshOnMove = decimateMeshBox->isChecked();
}

void ccDisplayOptionsDlg::changeMaxMeshSize(double val)
{
	parameters.minLoDMeshSize = static_cast<unsigned>(val * 1000000);
}

void ccDisplayOptionsDlg::changeCloudDecimation()
{
	parameters.decimateCloudOnMove = decimateCloudBox->isChecked();
}

void ccDisplayOptionsDlg::changeMaxCloudSize(double val)
{
	parameters.minLoDCloudSize = static_cast<unsigned>(val * 1000000);
}

void ccDisplayOptionsDlg::changeVBOUsage()
{
	parameters.useVBOs = useVBOCheckBox->isChecked();
	if (parameters.useVBOs && maxCloudSizeDoubleSpinBox->value() < s_defaultMaxVBOCloudSizeM)
		maxCloudSizeDoubleSpinBox->setValue(s_defaultMaxVBOCloudSizeM);
}

void ccDisplayOptionsDlg::changeCrossDisplayed()
{
	parameters.displayCross = showCrossCheckBox->isChecked();
}

void ccDisplayOptionsDlg::changeColorScaleShowHistogram()
{
	parameters.colorScaleShowHistogram = colorScaleShowHistogramCheckBox->isChecked();
}

void ccDisplayOptionsDlg::changeColorScaleUseShader()
{
	parameters.colorScaleUseShader = useColorScaleShaderCheckBox->isChecked();
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

	emit aspectHasChanged();

	reject();
}

void ccDisplayOptionsDlg::reset()
{
	parameters.reset();
	refresh();
}

void ccDisplayOptionsDlg::apply()
{
	ccGui::Set(parameters);

	emit aspectHasChanged();
}

void ccDisplayOptionsDlg::doAccept()
{
	apply();

	parameters.toPersistentSettings();

	accept();
}
