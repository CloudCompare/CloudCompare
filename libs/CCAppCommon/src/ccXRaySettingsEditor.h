#pragma once

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

// qCC_glWindow
#include <ccXRaySettings.h>

// Qt
#include <QObject>

class ccDisplaySettingsDlg;
namespace Ui
{
	class DisplaySettingsDlg;
}

//! Dialog-owned state for X-ray controls and reversible live previews.
class ccXRaySettingsEditor : public QObject
{
  public:
	//! Constructor
	ccXRaySettingsEditor(ccDisplaySettingsDlg& dialog, Ui::DisplaySettingsDlg& ui);

  protected: // methods
	void refreshXRay();
	void previewXRay();

  protected: // members
	ccDisplaySettingsDlg&   m_dialog;
	Ui::DisplaySettingsDlg& m_ui;
	ccXRaySettings          m_parameters;
	ccXRaySettings          m_oldParameters;
};
