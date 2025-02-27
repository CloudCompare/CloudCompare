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

#include <QDialog>

class ccGenericPointCloud;

namespace CCCoreLib {
	class ReferenceCloud;
}

namespace Ui {
	class AlignDialog;
}

//! Rough registration dialog
class ccAlignDlg : public QDialog
{
	Q_OBJECT

public:

	enum CC_SAMPLING_METHOD {	NONE = 0,
								RANDOM,
								SPACE,
								OCTREE
	};

	ccAlignDlg(ccGenericPointCloud* data, ccGenericPointCloud* model, QWidget* parent = nullptr);
	virtual ~ccAlignDlg();

	unsigned getNbTries();
	double getOverlap();
	double getDelta();
	ccGenericPointCloud* getModelObject();
	ccGenericPointCloud* getDataObject();
	CC_SAMPLING_METHOD getSamplingMethod();
	bool isNumberOfCandidatesLimited();
	unsigned getMaxNumberOfCandidates();
	CCCoreLib::ReferenceCloud* getSampledModel();
	CCCoreLib::ReferenceCloud* getSampledData();

protected:
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

	Ui::AlignDialog* m_ui;
};
