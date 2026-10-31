// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                     COPYRIGHT: CloudCompare project                    #
// #                                                                        #
// ##########################################################################

#include "ccXRaySettingsEditor.h"

#include "ccDisplaySettingsDlg.h"

#include <QColorDialog>
#include <QSignalBlocker>
#include <ccQtHelpers.h>
#include <ui_displaySettingsDlg.h>

ccXRaySettingsEditor::ccXRaySettingsEditor(ccDisplaySettingsDlg& dialog, Ui::DisplaySettingsDlg& ui)
    : QObject(&dialog)
    , m_dialog(dialog)
    , m_ui(ui)
    , m_parameters(ccXRaySettings::Parameters())
    , m_oldParameters(m_parameters)
{
	connect(m_ui.xRayExposureSlider, &QSlider::valueChanged, this, [this](int value)
	        { m_ui.xRayExposureSpinBox->setValue(value / 10.0); });
	connect(m_ui.xRayExposureSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value)
	        {
		        const QSignalBlocker blocker(m_ui.xRayExposureSlider);
		        m_ui.xRayExposureSlider->setValue(qRound(value * 10));
		        m_parameters.exposure = value;
		        previewXRay(); });
	connect(m_ui.xRayReverseCheckBox, &QCheckBox::toggled, this, [this](bool checked)
	        { m_parameters.reverse = checked; previewXRay(); });
	auto bindBackground = [this](QCheckBox* gradient, QToolButton* primary, QToolButton* secondary, ccXRaySettings::Background* background)
	{
		connect(gradient, &QCheckBox::toggled, this, [this, background](bool checked)
		        { background->gradient = checked; refreshXRay(); previewXRay(); });
		auto chooseColor = [this](QToolButton* button, ccColor::Rgb* color)
		{
			connect(button, &QToolButton::clicked, this, [this, color]()
			        {
				        const QColor selected = QColorDialog::getColor(QColor(color->r, color->g, color->b), &m_dialog);
				        if (selected.isValid())
				        {
					        *color = ccColor::FromQColor(selected);
					        refreshXRay();
					        previewXRay();
				        } });
		};
		chooseColor(primary, &background->color);
		chooseColor(secondary, &background->secondColor);
	};
	bindBackground(m_ui.xRayGradientCheckBox, m_ui.xRayColorButton, m_ui.xRaySecondColorButton, &m_parameters.background);
	bindBackground(m_ui.xRayReverseGradientCheckBox, m_ui.xRayReverseColorButton, m_ui.xRayReverseSecondColorButton, &m_parameters.reverseBackground);

	// Every edit already updates the live preview. OK persists it; Cancel, Escape,
	// and the close button restore the opening snapshot, as the host dialog does.
	connect(&dialog, &QDialog::rejected, this, [this]
	        {
        m_parameters = m_oldParameters;
        previewXRay(); });
	connect(&dialog, &QDialog::accepted, this, [this]
	        { m_parameters.toPersistentSettings(); });
	connect(m_ui.resetButton, &QAbstractButton::clicked, this, [this]
	        {
        m_parameters = ccXRaySettings();
        refreshXRay();
        previewXRay(); });
	refreshXRay();
}

void ccXRaySettingsEditor::refreshXRay()
{
	const QSignalBlocker sliderBlocker(m_ui.xRayExposureSlider);
	const QSignalBlocker spinBlocker(m_ui.xRayExposureSpinBox);
	const QSignalBlocker reverseBlocker(m_ui.xRayReverseCheckBox);
	m_ui.xRayExposureSlider->setValue(qRound(m_parameters.exposure * 10));
	m_ui.xRayExposureSpinBox->setValue(m_parameters.exposure);
	m_ui.xRayReverseCheckBox->setChecked(m_parameters.reverse);
	auto refreshBackground = [](QCheckBox* gradient, QToolButton* primary, QToolButton* secondary, QLabel* secondaryLabel, const ccXRaySettings::Background& background)
	{
		const QSignalBlocker blocker(gradient);
		gradient->setChecked(background.gradient);
		ccQtHelpers::SetButtonColor(primary, QColor(background.color.r, background.color.g, background.color.b));
		ccQtHelpers::SetButtonColor(secondary, QColor(background.secondColor.r, background.secondColor.g, background.secondColor.b));
		secondary->setEnabled(background.gradient);
		secondaryLabel->setEnabled(background.gradient);
	};
	refreshBackground(m_ui.xRayGradientCheckBox, m_ui.xRayColorButton, m_ui.xRaySecondColorButton, m_ui.xRaySecondColorLabel, m_parameters.background);
	refreshBackground(m_ui.xRayReverseGradientCheckBox, m_ui.xRayReverseColorButton, m_ui.xRayReverseSecondColorButton, m_ui.xRayReverseSecondColorLabel, m_parameters.reverseBackground);
}

void ccXRaySettingsEditor::previewXRay()
{
	if (ccXRaySettings::Parameters() == m_parameters)
		return;
	ccXRaySettings::Set(m_parameters);
	Q_EMIT m_dialog.aspectHasChanged();
}
