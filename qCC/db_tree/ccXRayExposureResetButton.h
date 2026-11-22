// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <QDoubleSpinBox>
#include <QSignalBlocker>
#include <QSlider>
#include <QToolButton>
#include <ccPointCloud.h>
#include <ccXRaySettings.h>
#include <functional>
#include <limits>

// Reset the stored override, including when its value equals the default.
inline QToolButton* createXRayExposureResetButton(QDoubleSpinBox* spinBox, QSlider* slider, QWidget* parent, std::function<ccPointCloud*()> cloud, std::function<void()> changed)
{
	auto* button = new QToolButton(parent);
	button->setObjectName(QStringLiteral("xRayExposureReset"));
	button->setIcon(QIcon(QStringLiteral(":/CC/images/smallReset.png")));
	button->setToolTip(QObject::tr("Use default X-ray exposure"));
	button->setAccessibleName(QObject::tr("Use default X-ray exposure"));
	QObject::connect(button, &QToolButton::clicked, parent, [spinBox, slider, cloud, changed]
	                 {
		auto* object = cloud();
		if (!object) return;
		object->setXRayExposureOverride(std::numeric_limits<double>::quiet_NaN());
		const QSignalBlocker blockSpin(spinBox), blockSlider(slider);
		spinBox->setValue(spinBox->minimum());
		slider->setValue(qRound(ccXRaySettings::Parameters().exposure * 10));
		changed(); });
	return button;
}
