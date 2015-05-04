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

#ifndef CC_DENSITY_DIALOG_HEADER
#define CC_DENSITY_DIALOG_HEADER

//Qt
#include <QDialog>

//CCLib
#include <GeometricalAnalysisTools.h>

#include <ui_densityDlg.h>

//! Dialog for computing the density of a point clouds
class ccDensityDlg: public QDialog, public Ui::DensityDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccDensityDlg(QWidget* parent = 0);

	//! Returns output type
	CCLib::GeometricalAnalysisTools::Density getDensityType() const;
	//! Returns	whether the computation should be 'precise' or not
	bool isPrecise() const;

	//! Sets the default kernel radius (for 'precise' mode only)
	void setRadius(double r);
	//! Returns	the kernel radius (for 'precise' mode only)
	double getRadius() const;

protected slots:
	void onPreciseToggled(bool);

};

#endif // CC_DENSITY_DIALOG_HEADER
