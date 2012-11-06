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
//$Rev:: 1691                                                              $
//$LastChangedDate:: 2010-10-22 16:52:55 +0200 (ven., 22 oct. 2010)        $
//**************************************************************************
//

#ifndef CC_SENSOR_PROJECTION_DIALOG_HEADER
#define CC_SENSOR_PROJECTION_DIALOG_HEADER

#include <ui_sensorProjectDlg.h>

class ccGBLSensor;

class ccSensorProjectionDlg : public QDialog, public Ui::SensorProjectDialog
{
    Q_OBJECT

    public:

        ccSensorProjectionDlg(QWidget* parent=0);

        void initWithGBLSensor(ccGBLSensor* sensor);
        void updateGBLSensor(ccGBLSensor* sensor);

        bool isAxisMatIdentity();

protected slots:
    void rotationOrderChanged(int index);
    void axisMatrixStateChanged(int checkState);
};

#endif
