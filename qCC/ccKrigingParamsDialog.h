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

// qCC_db
#include <ccRasterGrid.h>

// Qt
#include <QDialog>

namespace Ui
{
	class KrigingParamsDialog;
} // namespace Ui

//! Dialog to set the Kriging parameters
class ccKrigingParamsDialog : public QDialog
{
	Q_OBJECT

  public:
	//! Default constructor
	ccKrigingParamsDialog(QWidget* parent = nullptr);

	//! Destructor
	~ccKrigingParamsDialog() override;

	//! Sets the parameters
	void setParameters(const ccRasterGrid::KrigingParams& krigingParams);

	//! Gets the parameters
	void getParameters(ccRasterGrid::KrigingParams& krigingParams);

  protected:
	//! Associated ui
	std::unique_ptr<Ui::KrigingParamsDialog> m_ui;
};
