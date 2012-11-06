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
//$Rev:: 1732                                                              $
//$LastChangedDate:: 2010-12-02 09:34:13 +0100 (jeu., 02 déc. 2010)       $
//**************************************************************************
//

#ifndef CC_UNROLL_DLG_HEADER
#define CC_UNROLL_DLG_HEADER

#include <ui_unrollDlg.h>

//CCLib
#include <CCGeom.h>

class ccUnrollDlg : public QDialog, public Ui::UnrollDialog
{
    Q_OBJECT

public:

    //! Default constructor
    ccUnrollDlg(QWidget* parent=0);

    int getType();
    int getAxisDimension();
    bool isAxisPositionAuto();
    CCVector3 getAxisPosition();
    double getRadius();
    double getAngle();

protected slots:
    void shapeTypeChanged(int index);
    void axisDimensionChanged(int index);
    void axisAutoStateChanged(int checkState);

protected:
    bool coneMode;

};

#endif
