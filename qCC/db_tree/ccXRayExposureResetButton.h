// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <QDoubleSpinBox>
#include <QSignalBlocker>
#include <QSlider>
#include <QToolButton>
#include <ccPointCloud.h>
#include <ccXRaySettings.h>
#include <cmath>
#include <functional>
#include <limits>

inline void updateXRayExposureResetButton(QToolButton* button, const ccPointCloud* cloud)
{
	button->setEnabled(cloud && !std::isnan(cloud->xRayExposureOverride()));
}

// Reset the stored override, including when its value equals the default.
inline QToolButton* createXRayExposureResetButton(QDoubleSpinBox* spinBox, QSlider* slider, QWidget* parent, std::function<ccPointCloud*()> cloud, std::function<void()> changed)
{
	auto* button = new QToolButton(parent);
	button->setObjectName(QStringLiteral("xRayExposureReset"));
	button->setIcon(QIcon(QStringLiteral(":/CC/images/smallReset.png")));
	button->setToolTip(QObject::tr("Use default X-ray exposure"));
	button->setAccessibleName(QObject::tr("Use default X-ray exposure"));
	updateXRayExposureResetButton(button, cloud());
	QObject::connect(button, &QToolButton::clicked, parent, [button, spinBox, slider, cloud, changed]
	                 {
		auto* object = cloud();
		if (!object) return;
		object->setXRayExposureOverride(std::numeric_limits<double>::quiet_NaN());
		updateXRayExposureResetButton(button, object);
		const QSignalBlocker blockSpin(spinBox), blockSlider(slider);
		spinBox->setValue(spinBox->minimum());
		slider->setValue(qRound(ccXRaySettings::Parameters().exposure * 10));
		changed(); });
	return button;
}
