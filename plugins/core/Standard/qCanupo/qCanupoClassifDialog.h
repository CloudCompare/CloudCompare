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

#ifndef Q_CANUPO_CLASSIF_DIALOG_HEADER
#define Q_CANUPO_CLASSIF_DIALOG_HEADER

#include <ui_qCanupoClassifDialog.h>

class ccMainAppInterface;
class ccPointCloud;

//! CANUPO plugin's classification dialog
class qCanupoClassifDialog : public QDialog, public Ui::CanupoClassifDialog
{
	Q_OBJECT

public:

	//! Default constructor
	qCanupoClassifDialog(ccPointCloud* cloud, ccMainAppInterface* app);

	//! "Sources" of core points
	enum CORE_CLOUD_SOURCES
	{
		ORIGINAL,
		OTHER,
		SUBSAMPLED,
		MSC_FILE
	};
	
	//! Returns the selected source for core points
	CORE_CLOUD_SOURCES getCorePointsCloudSource() const;

	//! Get core points cloud (if any)
	/** Returns either the input cloud (ORIGINAL) or the other cloud
		specified in the dedicated combo box (OTHER).
		It can also return a null pointer if the user has requested
		sub-sampling (SUBSAMPLED) or MSC file (MSC_FILE).
	**/
	ccPointCloud* getCorePointsCloud();

	//! Returns MSC file name (if source == MSC_FILE)
	QString getMscFilename() const;

	//! Returns the confidence threshold
	double getConfidenceTrehshold() const;
	//! Returns whether a SF should be used to classify points with low confidence
	bool useSF() const;
	//! Returns the max number of threads to use
	int getMaxThreadCount() const;

	//! Loads parameters from persistent settings
	void loadParamsFromPersistentSettings();
	//! Saves parameters to persistent settings
	void saveParamsToPersistentSettings();

protected slots:

	void browseClassifierFile();
	void browseMscFile();

protected:

	//! Gives access to the application (data-base, UI, etc.)
	ccMainAppInterface* m_app;

	//! Associated cloud
	ccPointCloud* m_cloud;
};

#endif //Q_CANUPO_CLASSIF_DIALOG_HEADER
