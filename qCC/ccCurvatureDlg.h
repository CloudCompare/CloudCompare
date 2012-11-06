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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1595                                                              $
//$LastChangedDate:: 2010-07-02 18:04:17 +0200 (ven., 02 juil. 2010)       $
//**************************************************************************
//

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
    ccCurvatureDlg(QWidget* parent=0);

    //! Returns kernel size
    double getKernelSize();

    //! Sets kernel size
    void setKernelSize(double size);

    //! Returns curvature type
    CCLib::Neighbourhood::CC_CURVATURE_TYPE getCurvatureType();
};

#endif
