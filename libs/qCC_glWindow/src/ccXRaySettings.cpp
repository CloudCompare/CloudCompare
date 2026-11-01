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

#include "ccXRaySettings.h"

#include "ccGLWindowInterface.h"
#include "ccGLWindowSignalEmitter.h"

#include <QColor>
#include <QSettings>
#include <QVariant>
#include <algorithm>
#include <cmath>

Q_DECLARE_METATYPE(ccXRaySettings)

namespace
{
	constexpr char  OVERRIDE_PROPERTY[] = "CloudCompare.XRay.Settings";
	ccXRaySettings& GlobalSettings()
	{
		static ccXRaySettings s_settings = []
		{
			ccXRaySettings value;
			value.fromPersistentSettings();
			return value;
		}();
		return s_settings;
	}
} // namespace

bool ccXRaySettings::operator==(const ccXRaySettings& other) const
{
	return exposure == other.exposure && reverse == other.reverse
	       && background == other.background && reverseBackground == other.reverseBackground;
}

ccXRaySettings ccXRaySettings::Parameters(const ccGLWindowInterface* window)
{
	if (window)
	{
		const QVariant value = window->signalEmitter()->property(OVERRIDE_PROPERTY);
		if (value.isValid())
			return value.value<ccXRaySettings>();
	}
	return GlobalSettings();
}

void ccXRaySettings::Set(const ccXRaySettings& settings, ccGLWindowInterface* window)
{
	if (window)
		window->signalEmitter()->setProperty(OVERRIDE_PROPERTY, QVariant::fromValue(settings));
	else
		GlobalSettings() = settings;
}

void ccXRaySettings::ClearOverride(ccGLWindowInterface& window)
{
	window.signalEmitter()->setProperty(OVERRIDE_PROPERTY, QVariant());
}

void ccXRaySettings::fromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("OpenGL");
	bool         exposureValid  = false;
	const double storedExposure = settings.value("xRayExposure", -3.0).toDouble(&exposureValid);
	// An invalid saved value must not turn into maximum exposure (notably NaN).
	exposure            = exposureValid && std::isfinite(storedExposure) ? std::clamp(storedExposure, -8.0, 8.0) : -3.0;
	reverse             = settings.value("xRayReverse", false).toBool();
	auto loadBackground = [&](const QString& group, Background& background)
	{
		settings.beginGroup(group);
		auto loadColor = [&](const QString& key, ccColor::Rgb& color)
		{
			QColor value(settings.value(key, QColor(color.r, color.g, color.b).name()).toString());
			if (value.isValid())
				color = ccColor::FromQColor(value);
		};
		loadColor("color", background.color);
		loadColor("secondColor", background.secondColor);
		background.gradient = settings.value("gradient", false).toBool();
		settings.endGroup();
	};
	loadBackground("xRayBackground", background);
	loadBackground("xRayReverseBackground", reverseBackground);
	settings.endGroup();
}

void ccXRaySettings::toPersistentSettings() const
{
	QSettings settings;
	settings.beginGroup("OpenGL");
	settings.setValue("xRayExposure", exposure);
	settings.setValue("xRayReverse", reverse);
	auto saveBackground = [&](const QString& group, const Background& background)
	{
		settings.beginGroup(group);
		settings.setValue("color", QColor(background.color.r, background.color.g, background.color.b).name());
		settings.setValue("secondColor", QColor(background.secondColor.r, background.secondColor.g, background.secondColor.b).name());
		settings.setValue("gradient", background.gradient);
		settings.endGroup();
	};
	saveBackground("xRayBackground", background);
	saveBackground("xRayReverseBackground", reverseBackground);
	settings.endGroup();
}
