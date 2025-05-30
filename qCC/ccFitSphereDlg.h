#pragma once

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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include <ui_fitShereDlg.h>

//Qt
#include <QDialog>

//! Dialog to input the 'Fit Sphere' tool parameters
class ccFitSphereDlg : public QDialog, public Ui::FitSphereDialog
{
	Q_OBJECT

public:

	//! Default constructor
	ccFitSphereDlg(	double maxOutliersRatio,
					double confidence,
					bool autoDetectSphereRadius,
					double sphereRadius,
					QWidget* parent = nullptr);

	//! Returns the max outliers ratio
	double maxOutliersRatio() const;
	//! Returns the confidence
	double confidence() const;
	//! Returns whether the sphere radius should be automatically detected
	bool autoDetectSphereRadius() const;
	//! Returns the sphere radius (or 0.0 if it should be automatically detected)
	double sphereRadius() const;
};
