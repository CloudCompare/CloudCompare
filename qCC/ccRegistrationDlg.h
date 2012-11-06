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
//$Rev:: 2183                                                              $
//$LastChangedDate:: 2012-07-02 18:38:13 +0200 (lun., 02 juil. 2012)       $
//**************************************************************************
//

#ifndef CC_REGISTRATION_DLG_HEADER
#define CC_REGISTRATION_DLG_HEADER

#include <QDialog>

//CCLib
#include <RegistrationTools.h>

#include <ui_registrationDlg.h>
#include <ReferenceCloud.h>

class ccHObject;

typedef CCLib::ICPRegistrationTools::CC_ICP_CONVERGENCE_TYPE ConvergenceMethod;

//! Point cloud or mesh registration dialog
class ccRegistrationDlg : public QDialog, public Ui::RegistrationDialog
{
    Q_OBJECT

public:

    //! Default constructor
    ccRegistrationDlg(ccHObject *data, ccHObject *model, QWidget* parent=0);

    //! Default destructor
    virtual ~ccRegistrationDlg();

    //! Returns convergence method
    ConvergenceMethod getConvergenceMethod() const;

    //! Returns max number of iterations
    /** Only valid ifregistration method is 'ITERATION_REG'.
    **/
    unsigned int getMaxIterationCount() const;

    //! Returns minimum error decrease between two consecutive iterations
    /** Only valid ifregistration method is 'MAX_ERROR_REG'.
    **/
    double getMinErrorDecrease() const;

    //! Returns whether farthest points should be ignored at each iteration
    /** This is a trick to improve registration for slightly different clouds.
    **/
    bool removeFarthestPoints() const;

    //! Returns the limit above which clouds should be randomly resampled
	unsigned randomSamplingLimit() const;

    //! Returns 'model' entity
    ccHObject *getModelEntity();

    //! Returns 'data' entity
    ccHObject *getDataEntity();

	//! Whether to use data displayed SF as weights
	bool useDataSFAsWeights() const;

	//! Whether to use model displayed SF as weights
	bool useModelSFAsWeights() const;

protected slots:
    void swapModelAndData();

protected:

    void setColorsAndLabels();

    //! 'Model' entity
    ccHObject* modelEntity;

    //! 'Data' entity
    ccHObject* dataEntity;
};

#endif
