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

#ifndef CC_COLOR_LEVELS_DLG_HEADER
#define CC_COLOR_LEVELS_DLG_HEADER

//Qt
#include <QColor>

#include <ui_colorLevelsDlg.h>

class ccHistogramWindow;
class ccGenericPointCloud;

//! Dialog to change the color levels
class ccColorLevelsDlg : public QDialog, public Ui::ColorLevelsDialog
{
	Q_OBJECT

public:

	//! Default constructor
	ccColorLevelsDlg(QWidget* parent, ccGenericPointCloud* pointCloud);

protected slots:

	void onChannelChanged(int);
	void onApply();

protected:

	//! Channels
	enum CHANNELS { RGB = 0, RED = 1, GREEN = 2, BLUE = 3 };

	//! Updates histogram
	void updateHistogram();

	//! Associated histogram view
	ccHistogramWindow* m_histogram;

	//! Associated point cloud (color source)
	ccGenericPointCloud* m_cloud;


};

#endif //CC_COLOR_LEVELS_DLG_HEADER
