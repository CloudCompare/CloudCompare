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

#include <ui_FastGlobalRegistrationDlg.h>

class ccPointCloud;

//! Fast Global Registration dialog
class FastGlobalRegistrationDialog : public QDialog, public Ui::FastGlobalRegistrationDialog
{
	Q_OBJECT

public:

	//! Default constructor
	FastGlobalRegistrationDialog(	const std::vector<ccPointCloud*>& allClouds,
									QWidget* parent = nullptr);

	//! Default destructor
	~FastGlobalRegistrationDialog() override;

	//! Returns the feature descritor comptation radius
	double getFeatureRadius() const;

	//! Returns the 'reference' cloud
	ccPointCloud* getReferenceCloud();

	//! Saves parameters for next call
	void saveParameters() const;

protected:
	void autoEstimateRadius();
	void referenceEntityChanged(int index);

protected:

	void updateGUI();

	//! All clouds (input)
	std::vector<ccPointCloud*> clouds;

	//! Reference cloud unique ID
	unsigned referencesCloudUinqueID;
};

