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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//Qt
#include <QDialog>

//CCCoreLib
#include <CCTypes.h>

//System
#include <array>

class ccGenericPointCloud;

namespace CCCoreLib
{
	class GenericProgressCallback;
	class ReferenceCloud;
}

namespace Ui
{
	class SubsamplingDialog;
}

//! Subsampling cloud dialog
class ccSubsamplingDlg : public QDialog
{
	Q_OBJECT

public:

	//! Sub-sampling method
	enum CC_SUBSAMPLING_METHOD
	{
		RANDOM         = 0,
		RANDOM_PERCENT = 1,
		SPATIAL        = 2,
		OCTREE         = 3,
		COUNT          = 4 //Should always be the last one
	};

	//! Default constructor
	ccSubsamplingDlg(unsigned maxPointCount, double maxCloudRadius, QWidget* parent = nullptr);

	//! Destructor
	~ccSubsamplingDlg() override;

	//! Returns the subsampled version of a cloud according to the current parameters
	/** Should only be called after the dialog has been validated.
	**/
	CCCoreLib::ReferenceCloud* getSampledCloud(ccGenericPointCloud* cloud, CCCoreLib::GenericProgressCallback* progressCb = nullptr);

	//! Enables the SF modulation option (SPATIAL method)
	void enableSFModulation(ScalarType sfMin, ScalarType sfMax);

	//! Saves the current state to persistent settings
	void saveToPersistentSettings() const;
	//! Loads the state from persistent settings
	void loadFromPersistentSettings();

protected:

	void sliderMoved(int sliderPos);
	void valueChanged(double value);
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

	//! Last known sampling values (per method)
	std::array<double, CC_SUBSAMPLING_METHOD::COUNT> m_lastUsedValues;

	//! Associated UI
	Ui::SubsamplingDialog* m_ui;
};
