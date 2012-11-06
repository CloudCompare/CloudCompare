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
//$Rev:: 1945                                                              $
//$LastChangedDate:: 2011-11-23 19:17:28 +0100 (mer., 23 nov. 2011)        $
//**************************************************************************
//

#ifndef CC_SAMPLE_DLG_HEADER
#define CC_SAMPLE_DLG_HEADER

#include <QDialog>

#include <GenericProgressCallback.h>
#include <ReferenceCloud.h>

#include <ui_subsamplingDlg.h>

class ccGenericPointCloud;

//! Subsampling cloud dialog
class ccSubsamplingDlg : public QDialog, public Ui::SubsamplingDialog
{
    Q_OBJECT

public:

    typedef enum
    {
        RANDOM  = 0,
        SPACE   = 1,
        OCTREE  = 2,
    } CC_SUBSAMPLING_METHOD;

	//! Default constructor
    ccSubsamplingDlg(ccGenericPointCloud* cloud, QWidget* parent=0);

	//! Returns subsampled cloud (once dialog as been validated)
    CCLib::ReferenceCloud* getSampledCloud(CCLib::GenericProgressCallback *progressCb=NULL);

protected slots:
    void sliderReleased();
    void sliderMoved(int sliderPos);
    void samplingRateChanged(double value);
    void changeSamplingMethod(int index);

protected:

	//! Associated cloud
    ccGenericPointCloud* m_pointCloud;

    void updateLabels();
};

#endif
