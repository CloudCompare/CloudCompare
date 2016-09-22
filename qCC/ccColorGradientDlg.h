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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_COLOR_GRADIENT_DLG_HEADER
#define CC_COLOR_GRADIENT_DLG_HEADER

//Qt
#include <QColor>

#include <ui_colorGradientDlg.h>

//! Dialog to define a color gradient (default, with 2 colors, banding, etc.)
class ccColorGradientDlg : public QDialog, public Ui::ColorGradientDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccColorGradientDlg(QWidget* parent);

	//! Gradient types
	enum GradientType { Default, TwoColors, Banding };

	//! Returns selected gradient type
	GradientType getType() const;
	//! Sets the currently activated type
	void setType(GradientType type);

	//! Returns the two colors of the gradient ('TwoColors' mode)
	void getColors(QColor& first, QColor& second) const;

	//! Returns the frequency of the gradient ('Banding' mode)
	double getBandingFrequency() const;

	//! Returns the ramp dimension
	unsigned char getDimension() const;

protected slots:

	void changeFirstColor();
	void changeSecondColor();
};

#endif //CC_COLOR_GRADIENT_DLG_HEADER
