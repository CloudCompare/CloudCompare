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

#ifndef CC_ALIGN_DLG_HEADER
#define CC_ALIGN_DLG_HEADER

#include <QDialog>

#include <ui_alignDlg.h>

#include <ReferenceCloud.h>

class ccGenericPointCloud;

//! Rough registration dialog
class ccAlignDlg : public QDialog, public Ui::AlignDialog
{
	Q_OBJECT

public:

	enum CC_SAMPLING_METHOD {	NONE = 0,
								RANDOM,
								SPACE,
								OCTREE
	};

	ccAlignDlg(ccGenericPointCloud *data, ccGenericPointCloud *model, QWidget* parent = 0);
	virtual ~ccAlignDlg();

	unsigned getNbTries();
	double getOverlap();
	double getDelta();
	ccGenericPointCloud *getModelObject();
	ccGenericPointCloud *getDataObject();
	CC_SAMPLING_METHOD getSamplingMethod();
	bool isNumberOfCandidatesLimited();
	unsigned getMaxNumberOfCandidates();
	CCLib::ReferenceCloud *getSampledModel();
	CCLib::ReferenceCloud *getSampledData();


protected slots:
	void swapModelAndData();
	void modelSliderReleased();
	void dataSliderReleased();
	void modelSamplingRateChanged(double value);
	void dataSamplingRateChanged(double value);
	void estimateDelta();
	void changeSamplingMethod(int index);
	void toggleNbMaxCandidates(bool activ);

protected:

	//! 'Model' cloud (static)
	ccGenericPointCloud* modelObject;

	//! 'Data' cloud (static)
	ccGenericPointCloud* dataObject;

	void setColorsAndLabels();

};

#endif //CC_ALIGN_DLG_HEADER
