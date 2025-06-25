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
// #                    COPYRIGHT: CloudCompare project                     #
// #                                                                        #
// ##########################################################################

#include "CCPluginAPI.h"

// Qt
#include <QDialog>

namespace Ui
{
	class InfoDialog;
}

//! Dialog to display some pieces of information in
class CCPLUGIN_LIB_API ccInfoDlg : public QDialog
{
  public:
	//! Default constructor
	ccInfoDlg(QWidget* parent);

	//! Destructor
	~ccInfoDlg() override;

	//! Shows a text in the info dialog
	void showText(const QString& text);

  private:
	Ui::InfoDialog* m_ui;
};
