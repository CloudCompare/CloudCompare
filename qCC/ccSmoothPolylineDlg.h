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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

//qCC_db
#include <ccHObject.h>

//Qt
#include <QDialog>

class Ui_SmoothPolylineDialog;

//! Dialog to smooth a polyline (Chaikin algorithm)
class ccSmoothPolylineDialog : public QDialog
{
public:
	//! Default constructor
	ccSmoothPolylineDialog(QWidget* parent = nullptr);

	//! Destructor
	virtual ~ccSmoothPolylineDialog();

	//! Sets the number of iterations
	void setIerationCount(int count);

	//! Sets the smoothing ratio
	void setRatio(double ratio);

	//! Returns the number of iterations
	int getIerationCount() const;

	//! Returns the smoothing ratio
	double getRatio() const;

protected:

	//! Associated ui
	Ui_SmoothPolylineDialog* m_ui;
};
