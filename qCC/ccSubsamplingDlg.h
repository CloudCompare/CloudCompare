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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_SAMPLE_DLG_HEADER
#define CC_SAMPLE_DLG_HEADER

//Qt
#include <QDialog>

//CCLib
#include <GenericProgressCallback.h>
#include <ReferenceCloud.h>

//GUI
#include <ui_subsamplingDlg.h>

class ccGenericPointCloud;

//! Subsampling cloud dialog
class ccSubsamplingDlg : public QDialog, public Ui::SubsamplingDialog
{
	Q_OBJECT

public:

	//! Sub-sampling method
	enum CC_SUBSAMPLING_METHOD
	{
		RANDOM  = 0,
		SPACE   = 1,
		OCTREE  = 2,
	};

	//! Default constructor
	ccSubsamplingDlg(unsigned maxPointCount, double maxCloudRadius, QWidget* parent = 0);

	//! Returns subsampled version of a cloud according to current parameters
	/** Should be called only once the dialog has been validated.
	**/
	CCLib::ReferenceCloud* getSampledCloud(ccGenericPointCloud* cloud, CCLib::GenericProgressCallback* progressCb = 0);

	//! Enables the SF modulation option (SPATIAL method)
	void enableSFModulation(ScalarType sfMin, ScalarType sfMax);

protected slots:

	void sliderMoved(int sliderPos);
	void samplingRateChanged(double value);
	void changeSamplingMethod(int index);

protected: //methods

	//! Updates the dialog lables depending on the active mode
	void updateLabels();

protected: //members

	//! Max point count (for RANDOM method)
	unsigned m_maxPointCount;

	//! Max radius (for SPACE method)
	double m_maxRadius;

	//! Scalar modulation
	bool m_sfModEnabled;
	//! Scalar modulation (min SF value)
	ScalarType m_sfMin;
	//! Scalar modulation (max SF value)
	ScalarType m_sfMax;

};

#endif
