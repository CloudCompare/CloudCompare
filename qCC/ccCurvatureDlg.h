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

#ifndef CC_CURVATURE_DLG_HEADER
#define CC_CURVATURE_DLG_HEADER

#include <ui_curvatureDlg.h>

//CCLib
#include <Neighbourhood.h>

//! Dialog to define curvature computation parameters
class ccCurvatureDlg : public QDialog, public Ui::CurvatureDialog
{
public:

	//! Default constructor
	explicit ccCurvatureDlg(QWidget* parent = 0);

	//! Returns kernel size
	double getKernelSize() const;

	//! Sets kernel size
	void setKernelSize(double size);

	//! Returns curvature type
	CCLib::Neighbourhood::CC_CURVATURE_TYPE getCurvatureType() const;
};

#endif //CC_CURVATURE_DLG_HEADER
