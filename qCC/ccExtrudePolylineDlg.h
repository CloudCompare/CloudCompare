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
// ##########################################################################

#include <ui_extrudePolylineDlg.h>

//! Dialog for extruding a polyline along Z into a zero-thickness mesh surface
class ccExtrudePolylineDlg : public QDialog
    , public Ui::ExtrudePolylineDialog
{
	Q_OBJECT

  public:
	explicit ccExtrudePolylineDlg(QWidget* parent = nullptr);

	double heightAbove() const;
	double depthBelow() const;

	void setHeightAbove(double value);
	void setDepthBelow(double value);
};
