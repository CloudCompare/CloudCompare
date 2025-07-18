// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#ifndef CC_REGISTRATION_DLG_HEADER
#define CC_REGISTRATION_DLG_HEADER

#include <QDialog>

// CCCoreLib
#include <ReferenceCloud.h>
#include <RegistrationTools.h>
#include <ui_registrationDlg.h>

class ccHObject;

//! Point cloud or mesh registration dialog
class ccRegistrationDlg : public QDialog
    , public Ui::RegistrationDialog
{
	Q_OBJECT

  public:
	//! Default constructor
	ccRegistrationDlg(ccHObject* data, ccHObject* model, QWidget* parent = nullptr);

	//! Default destructor
	virtual ~ccRegistrationDlg();

	// shortcuts
	typedef CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE ConvergenceMethod;

	//! Returns convergence method
	ConvergenceMethod getConvergenceMethod() const;

	//! Returns max number of iterations
	/** Only valid if registration method is 'ITERATION_REG'.
	 **/
	unsigned getMaxIterationCount() const;

	//! Returns the approximated final overlap
	unsigned getFinalOverlap() const;

	//! Returns minimum RMS decrease between two consecutive iterations
	/** Only valid if registration method is 'MAX_ERROR_REG'.
	 **/
	double getMinRMSDecrease() const;

	//! Returns the theoretical mininmum RMS decrease between two consecutive iterations
	static double GetAbsoluteMinRMSDecrease();

	//! Sets the minimum RMS decrease between two consecutive iterations
	/** Only valid if registration method is 'MAX_ERROR_REG'.
	 **/
	void setMinRMSDecrease(double value);

	//! Returns whether farthest points should be ignored at each iteration
	/** This is a trick to improve registration for slightly different clouds.
	 **/
	bool removeFarthestPoints() const;

	//! Returns the limit above which clouds should be randomly resampled
	unsigned randomSamplingLimit() const;

	//! Returns 'model' entity
	ccHObject* getModelEntity();

	//! Returns 'data' entity
	ccHObject* getDataEntity();

	//! Whether to use data displayed SF as weights
	bool useDataSFAsWeights() const;

	//! Whether to use model displayed SF as weights
	bool useModelSFAsWeights() const;

	//! Whether to use signed distances when the reference is a mesh
	bool useC2MSignedDistances(bool& robust) const;

	//! Method to take normals into account
	CCCoreLib::ICPRegistrationTools::NORMALS_MATCHING normalsMatchingOption() const;

	//! Returns whether to adjust the scale during optimization
	/** This is useful for co-registration of lidar and photogrammetric clouds
	for instance.
	**/
	bool adjustScale() const;

	//! Returns active transformation filters
	/** See CCCoreLib::RegistrationTools::TRANSFORMATION_FILTERS.
	 **/
	int getTransformationFilters() const;

	//! Returns the maximum number of threads
	int getMaxThreadCount() const;

	//! Saves parameters for next call
	void saveParameters() const;

  protected:
	void swapModelAndData();

  protected:
	//! Forces the update of the GUI
	void updateGUI();

	//! 'Model' entity
	ccHObject* modelEntity;

	//! 'Data' entity
	ccHObject* dataEntity;
};

#endif // CC_REGISTRATION_DLG_HEADER
