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
//#                 COPYRIGHT: Daniel Girardeau-Montaut                    #
//#                                                                        #
//##########################################################################

#include "ccOptions.h"

//Qt
#include <QSettings>

//qCC_db
#include <ccSingleton.h>

//! Unique instance of ccOptions
static ccSingleton<ccOptions> s_options;

ccOptions& ccOptions::InstanceNonConst()
{
	if (!s_options.instance)
	{
		s_options.instance = new ccOptions();
		s_options.instance->fromPersistentSettings();
	}

	return *s_options.instance;
}

void ccOptions::ReleaseInstance()
{
	s_options.release();
}

void ccOptions::Set(const ccOptions& params)
{
	InstanceNonConst() = params;
}

ccOptions::ccOptions()
{
	reset();
}

void ccOptions::reset()
{
	normalsDisplayedByDefault = false;
	useNativeDialogs = true;
}

void ccOptions::fromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("Options");
	{
		normalsDisplayedByDefault = settings.value("normalsDisplayedByDefault", false).toBool();
		useNativeDialogs = settings.value("useNativeDialogs", true).toBool();
	}
	settings.endGroup();
}

void ccOptions::toPersistentSettings() const
{
	QSettings settings;
	settings.beginGroup("Options");
	{
		settings.setValue("normalsDisplayedByDefault", normalsDisplayedByDefault);
		settings.setValue("useNativeDialogs", useNativeDialogs);
	}
	settings.endGroup();
}
