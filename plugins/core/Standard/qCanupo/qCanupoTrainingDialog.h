//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCANUPO                       #
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
//#      COPYRIGHT: UEB (UNIVERSITE EUROPEENNE DE BRETAGNE) / CNRS         #
//#                                                                        #
//##########################################################################

#ifndef Q_CANUPO_TRAINING_DIALOG_HEADER
#define Q_CANUPO_TRAINING_DIALOG_HEADER

#include <ui_qCanupoTrainingDialog.h>

class ccMainAppInterface;
class ccPointCloud;

//! CANUPO plugin's training dialog
class qCanupoTrainingDialog : public QDialog, public Ui::CanupoTrainingDialog
{
	Q_OBJECT

public:

	//! Default constructor
	qCanupoTrainingDialog(ccMainAppInterface* app);

	//! Get origin point cloud
	ccPointCloud* getOriginPointCloud();
	//! Get class #1 point cloud
	ccPointCloud* getClass1Cloud();
	//! Get class #2 point cloud
	ccPointCloud* getClass2Cloud();
	//! Get evaluation point cloud
	ccPointCloud* getEvaluationCloud();

	//! Loads parameters from persistent settings
	void loadParamsFromPersistentSettings();
	//! Saves parameters to persistent settings
	void saveParamsToPersistentSettings();

	//! Returns input scales
	bool getScales(std::vector<float>& scales) const;
	//! Returns the max number of threads to use
	int getMaxThreadCount() const;

	//! Returns the selected descriptor ID
	unsigned getDescriptorID() const;

protected slots:

	void onClassChanged(int);
	void onCloudChanged(int);

protected:

	//! Gives access to the application (data-base, UI, etc.)
	ccMainAppInterface* m_app;

	//Returns whether the current parameters are valid or not
	bool validParameters() const;

};

#endif //Q_CANUPO_TRAINING_DIALOG_HEADER
