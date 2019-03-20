//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#ifndef Q_PCL_PLUGIN_EXTRACTSIFT_HEADER
#define Q_PCL_PLUGIN_EXTRACTSIFT_HEADER

#include "BaseFilter.h"

//Qt
#include <QString>

class SIFTExtractDlg;

//! SIFT keypoints extraction
class ExtractSIFT: public BaseFilter
{
public:
	ExtractSIFT();
	virtual ~ExtractSIFT();

	//inherited from BaseFilter
	virtual int compute();
protected:

	//inherited from BaseFilter
	virtual int checkSelected();
	virtual int openInputDialog();
	virtual void getParametersFromDialog();
	virtual int checkParameters();
	virtual QString getErrorMessage(int errorCode);

	SIFTExtractDlg* m_dialog;
	int m_nr_octaves;
	float m_min_scale;
	int m_nr_scales_per_octave;
	float m_min_contrast;
	bool m_use_min_contrast;
	QString m_field_to_use;
	std::string m_field_to_use_no_space;

	enum Modes {RGB, SCALAR_FIELD};
	Modes m_mode;
};

#endif
