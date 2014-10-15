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

//Qt
#include <QColorDialog>

//System
#include <string.h>

ccDisplayOptionsDlg::ccDisplayOptionsDlg(QWidget* parent) : QDialog(parent), Ui::DisplayOptionsDlg()
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	connect(ambientColorButton, SIGNAL(clicked()), this, SLOT(changeLightAmbientColor()));
	connect(diffuseColorButton, SIGNAL(clicked()), this, SLOT(changeLightDiffuseColor()));
	connect(specularColorButton, SIGNAL(clicked()), this, SLOT(changeLightSpecularColor()));
	connect(meshBackColorButton, SIGNAL(clicked()), this, SLOT(changeMeshBackDiffuseColor()));
	connect(meshSpecularColorButton, SIGNAL(clicked()), this, SLOT(changeMeshSpecularColor()));
	connect(meshFrontColorButton, SIGNAL(clicked()), this, SLOT(changeMeshFrontDiffuseColor()));
	connect(bbColorButton, SIGNAL(clicked()), this, SLOT(changeBBColor()));
	connect(bkgColorButton, SIGNAL(clicked()), this, SLOT(changeBackgroundColor()));
	connect(histBkgColorButton, SIGNAL(clicked()), this, SLOT(changeHistBackgroundColor()));
	connect(labelsColorButton, SIGNAL(clicked()), this, SLOT(changeLabelColor()));
	connect(enableGradientCheckBox, SIGNAL(clicked()), this, SLOT(changeBackgroundGradient()));
	connect(pointsColorButton, SIGNAL(clicked()), this, SLOT(changePointsColor()));
	connect(textColorButton, SIGNAL(clicked()), this, SLOT(changeTextColor()));
	connect(decimateMeshBox, SIGNAL(clicked()), this, SLOT(changeMeshDecimation()));
	connect(decimateCloudBox, SIGNAL(clicked()), this, SLOT(changeCloudDecimation()));
	connect(useVBOCheckBox,	SIGNAL(clicked()), this, SLOT(changeVBOUsage()));
	connect(showCrossCheckBox, SIGNAL(clicked()), this, SLOT(changeCrossDisplayed()));

	connect(colorScaleShowHistogramCheckBox, SIGNAL(clicked()), this, SLOT(changeColorScaleShowHistogram()));
	connect(useColorScaleShaderCheckBox, SIGNAL(clicked()), this, SLOT(changeColorScaleUseShader()));
	connect(colorRampWidthSpinBox, SIGNAL(valueChanged(int)), this, SLOT(changeColorScaleRampWidth(int)));

	connect(defaultFontSizeSpinBox, SIGNAL(valueChanged(int)), this, SLOT(changeDefaultFontSize(int)));
	connect(numberPrecisionSpinBox, SIGNAL(valueChanged(int)), this, SLOT(changeNumberPrecision(int)));
	connect(labelsTransparencySpinBox, SIGNAL(valueChanged(int)), this, SLOT(changeLabelsTransparency(int)));
	connect(labelMarkerSizeSpinBox, SIGNAL(valueChanged(int)), this, SLOT(changeLabelsMarkerSize(int)));

	connect(okButton, SIGNAL(clicked()), this, SLOT(doAccept()));
	connect(applyButton, SIGNAL(clicked()), this, SLOT(apply()));
	connect(resetButton, SIGNAL(clicked()), this, SLOT(reset()));
	connect(cancelButton, SIGNAL(clicked()), this, SLOT(doReject()));

	oldParameters = parameters = ccGui::Parameters();

	refresh();

	setUpdatesEnabled(true);
}

void ccDisplayOptionsDlg::refresh()
{
	const float* ac = parameters.lightAmbientColor;
	lightAmbientColor.setRgbF(ac[0],ac[1],ac[2],ac[3]);
	SetButtonColor(ambientColorButton,lightAmbientColor);

	const float* dc = parameters.lightDiffuseColor;
	lightDiffuseColor.setRgbF(dc[0],dc[1],dc[2],dc[3]);
	SetButtonColor(diffuseColorButton,lightDiffuseColor);

	const float* sc = parameters.lightSpecularColor;
	lightSpecularColor.setRgbF(sc[0],sc[1],sc[2],sc[3]);
	SetButtonColor(specularColorButton,lightSpecularColor);

	const float* mbc = parameters.meshBackDiff;
	meshBackDiff.setRgbF(mbc[0],mbc[1],mbc[2],mbc[3]);
	SetButtonColor(meshBackColorButton,meshBackDiff);

	const float* mspec = parameters.meshSpecular;
	meshSpecularColor.setRgbF(mspec[0],mspec[1],mspec[2],mspec[3]);
	SetButtonColor(meshSpecularColorButton,meshSpecularColor);

	const float* mfc = parameters.meshFrontDiff;
	meshFrontDiff.setRgbF(mfc[0],mfc[1],mfc[2],mfc[3]);
	SetButtonColor(meshFrontColorButton,meshFrontDiff);

	const unsigned char* bbc = parameters.bbDefaultCol;
	bbDefaultCol.setRgb(bbc[0],bbc[1],bbc[2]);
	SetButtonColor(bbColorButton,bbDefaultCol);

	const unsigned char* bgc = parameters.backgroundCol;
	backgroundCol.setRgb(bgc[0],bgc[1],bgc[2]);
	SetButtonColor(bkgColorButton,backgroundCol);

	const unsigned char* hbgc = parameters.histBackgroundCol;
	histBackgroundCol.setRgb(hbgc[0],hbgc[1],hbgc[2]);
	SetButtonColor(histBkgColorButton,histBackgroundCol);

	const unsigned char* lblc = parameters.labelCol;
	labelCol.setRgb(lblc[0],lblc[1],lblc[2]);
	SetButtonColor(labelsColorButton,labelCol);

	const unsigned char* pdc = parameters.pointsDefaultCol;
	pointsDefaultCol.setRgb(pdc[0],pdc[1],pdc[2]);
	SetButtonColor(pointsColorButton,pointsDefaultCol);

	const unsigned char* tdc = parameters.textDefaultCol;
	textDefaultCol.setRgb(tdc[0],tdc[1],tdc[2]);
	SetButtonColor(textColorButton,textDefaultCol);

	enableGradientCheckBox->setChecked(parameters.drawBackgroundGradient);
	decimateMeshBox->setChecked(parameters.decimateMeshOnMove);
	decimateCloudBox->setChecked(parameters.decimateCloudOnMove);
	useVBOCheckBox->setChecked(parameters.useVBOs);
	showCrossCheckBox->setChecked(parameters.displayCross);

	colorScaleShowHistogramCheckBox->setChecked(parameters.colorScaleShowHistogram);
	useColorScaleShaderCheckBox->setChecked(parameters.colorScaleUseShader);
	useColorScaleShaderCheckBox->setEnabled(parameters.colorScaleShaderSupported);
	colorRampWidthSpinBox->setValue(parameters.colorScaleRampWidth);

	defaultFontSizeSpinBox->setValue(parameters.defaultFontSize);
	numberPrecisionSpinBox->setValue(parameters.displayedNumPrecision);
	labelsTransparencySpinBox->setValue(parameters.labelsTransparency);
	labelMarkerSizeSpinBox->setValue(parameters.pickedPointsSize);

	update();
}

static void QColorToFloat(const QColor& col, float rgba[4])
{
	rgba[0] = static_cast<float>(col.redF());
	rgba[1] = static_cast<float>(col.greenF());
	rgba[2] = static_cast<float>(col.blueF());
	rgba[3] = static_cast<float>(col.alphaF());
}

void ccDisplayOptionsDlg::changeLightDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(lightDiffuseColor, this);
	if (!newCol.isValid())
		return;

	lightDiffuseColor = newCol;
	SetButtonColor(diffuseColorButton,lightDiffuseColor);

	float rgba[4];
	QColorToFloat(lightDiffuseColor,rgba);

	memcpy(parameters.lightDiffuseColor,rgba,sizeof(float)*4);
}

void ccDisplayOptionsDlg::changeLightAmbientColor()
{
	QColor newCol = QColorDialog::getColor(lightAmbientColor, this);
	if (!newCol.isValid())
		return;

	lightAmbientColor = newCol;
	SetButtonColor(ambientColorButton,lightAmbientColor);

	float rgba[4];
	QColorToFloat(lightAmbientColor,rgba);
	memcpy(parameters.lightAmbientColor,rgba,sizeof(float)*4);

	update();
}

void ccDisplayOptionsDlg::changeLightSpecularColor()
{
	QColor newCol = QColorDialog::getColor(lightSpecularColor, this);
	if (!newCol.isValid())
		return;

	lightSpecularColor = newCol;
	SetButtonColor(specularColorButton,lightSpecularColor);

	float rgba[4];
	QColorToFloat(lightSpecularColor,rgba);
	memcpy(parameters.lightSpecularColor,rgba,sizeof(float)*4);

	update();
}

void ccDisplayOptionsDlg::changeMeshFrontDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(meshFrontDiff, this);
	if (!newCol.isValid())
		return;

	meshFrontDiff = newCol;
	SetButtonColor(meshFrontColorButton,meshFrontDiff);

	float rgba[4];
	QColorToFloat(meshFrontDiff,rgba);
	memcpy(parameters.meshFrontDiff,rgba,sizeof(float)*4);

	update();
}

void ccDisplayOptionsDlg::changeMeshBackDiffuseColor()
{
	QColor newCol = QColorDialog::getColor(meshBackDiff, this);
	if (!newCol.isValid())
		return;

	meshBackDiff = newCol;
	SetButtonColor(meshBackColorButton,meshBackDiff);

	float rgba[4];
	QColorToFloat(meshBackDiff,rgba);
	memcpy(parameters.meshBackDiff,rgba,sizeof(float)*4);

	update();
}

void ccDisplayOptionsDlg::changeMeshSpecularColor()
{
	QColor newCol = QColorDialog::getColor(meshSpecularColor, this);
	if (!newCol.isValid())
		return;

	meshSpecularColor = newCol;
	SetButtonColor(meshSpecularColorButton,meshSpecularColor);

	float rgba[4];
	QColorToFloat(meshSpecularColor,rgba);
	memcpy(parameters.meshSpecular,rgba,sizeof(float)*4);

	update();
}

void ccDisplayOptionsDlg::changePointsColor()
{
	QColor newCol = QColorDialog::getColor(pointsDefaultCol, this);
	if (!newCol.isValid())
		return;

	pointsDefaultCol = newCol;
	SetButtonColor(pointsColorButton,pointsDefaultCol);

	unsigned char rgb[3] = {static_cast<unsigned char>(pointsDefaultCol.red()),
							static_cast<unsigned char>(pointsDefaultCol.green()),
							static_cast<unsigned char>(pointsDefaultCol.blue())	};
	memcpy(parameters.pointsDefaultCol,rgb,sizeof(unsigned char)*3);

	update();
}

void ccDisplayOptionsDlg::changeBBColor()
{
	QColor newCol = QColorDialog::getColor(bbDefaultCol, this);
	if (!newCol.isValid())
		return;

	bbDefaultCol = newCol;
	SetButtonColor(bbColorButton,bbDefaultCol);

	unsigned char rgb[3] = {static_cast<unsigned char>(bbDefaultCol.red()),
							static_cast<unsigned char>(bbDefaultCol.green()),
							static_cast<unsigned char>(bbDefaultCol.blue())	};
	memcpy(parameters.bbDefaultCol,rgb,sizeof(unsigned char)*3);

	update();
}

void ccDisplayOptionsDlg::changeTextColor()
{
	QColor newCol = QColorDialog::getColor(textDefaultCol, this);
	if (!newCol.isValid())
		return;

	textDefaultCol = newCol;
	SetButtonColor(textColorButton,textDefaultCol);

	unsigned char rgb[3] = {static_cast<unsigned char>(textDefaultCol.red()),
							static_cast<unsigned char>(textDefaultCol.green()),
							static_cast<unsigned char>(textDefaultCol.blue())	};
	memcpy(parameters.textDefaultCol,rgb,sizeof(unsigned char)*3);
}

void ccDisplayOptionsDlg::changeBackgroundColor()
{
	QColor newCol = QColorDialog::getColor(backgroundCol, this);
	if (!newCol.isValid())
		return;

	backgroundCol = newCol;
	SetButtonColor(bkgColorButton,backgroundCol);

	unsigned char rgb[3] = {static_cast<unsigned char>(backgroundCol.red()),
							static_cast<unsigned char>(backgroundCol.green()),
							static_cast<unsigned char>(backgroundCol.blue())	};
	memcpy(parameters.backgroundCol,rgb,sizeof(unsigned char)*3);

	update();
}

void ccDisplayOptionsDlg::changeHistBackgroundColor()
{
	QColor newCol = QColorDialog::getColor(histBackgroundCol, this);
	if (!newCol.isValid())
		return;

	histBackgroundCol = newCol;
	SetButtonColor(histBkgColorButton,histBackgroundCol);

	unsigned char rgb[3] = {static_cast<unsigned char>(histBackgroundCol.red()),
							static_cast<unsigned char>(histBackgroundCol.green()),
							static_cast<unsigned char>(histBackgroundCol.blue())	};
	memcpy(parameters.histBackgroundCol,rgb,sizeof(unsigned char)*3);

	update();
}

void ccDisplayOptionsDlg::changeLabelColor()
{
	QColor newCol = QColorDialog::getColor(labelCol, this);
	if (!newCol.isValid())
		return;

	labelCol = newCol;
	SetButtonColor(labelsColorButton,labelCol);

	unsigned char rgb[3] = {static_cast<unsigned char>(labelCol.red()),
							static_cast<unsigned char>(labelCol.green()),
							static_cast<unsigned char>(labelCol.blue())	};
	memcpy(parameters.labelCol,rgb,sizeof(unsigned char)*3);

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

void ccDisplayOptionsDlg::changeCloudDecimation()
{
	parameters.decimateCloudOnMove = decimateCloudBox->isChecked();
}

void ccDisplayOptionsDlg::changeVBOUsage()
{
	parameters.useVBOs = useVBOCheckBox->isChecked();
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

void ccDisplayOptionsDlg::changeNumberPrecision(int val)
{
	if (val < 0)
		return;
	parameters.displayedNumPrecision = static_cast<unsigned>(val);
}

void ccDisplayOptionsDlg::changeLabelsTransparency(int val)
{
	if (val < 0 || val > 100)
		return;
	parameters.labelsTransparency = static_cast<unsigned>(val);
}

void ccDisplayOptionsDlg::changeLabelsMarkerSize(int val)
{
	if (val <= 0)
		return;

	parameters.pickedPointsSize = static_cast<unsigned>(val);
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

//#ifdef CC_WINDOWS
//#include <QWindowsStyle>
//static QWindowsStyle s_windowsStyle;
//#endif
void ccDisplayOptionsDlg::SetButtonTextColor(QAbstractButton* button, const QColor &col)
{
	if (!button)
		return;

	//QPalette pal = button->palette();
	//pal.setColor(QPalette::ButtonText, col);
	//button->setPalette(pal);
	button->setStyleSheet(QString("background-color: rgb(%1, %2, %3)").arg(col.red(),col.green(),col.blue()));
//#ifdef CC_WINDOWS
//	button->setStyle(&s_windowsStyle/*new QWindowsStyle()*/);
//	button->update();
//#endif
}
