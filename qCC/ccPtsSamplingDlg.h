//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_POINTS_SAMPLING_DLG_HEADER
#define CC_POINTS_SAMPLING_DLG_HEADER

#include <ui_ptsSamplingDlg.h>

//! Dialog: points sampling on a mesh
class ccPtsSamplingDlg : public QDialog, public Ui::PointsSamplingDialog
{
public:

	//! Default constructor
	explicit ccPtsSamplingDlg(QWidget* parent = 0);

	bool generateNormals() const;
	bool interpolateRGB() const;
	bool interpolateTexture() const;

	bool useDensity() const;
	double getDensityValue() const;
	unsigned getPointsNumber() const;

	void setPointsNumber(int count);
	void setDensityValue(double density);
};

#endif //CC_POINTS_SAMPLING_DLG_HEADER
