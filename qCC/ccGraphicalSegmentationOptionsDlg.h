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
//#                   COPYRIGHT: CloudCompare project                      #
//#                                                                        #
//##########################################################################

#ifndef CC_GRAPHICAL_SEGMENTATION_OPTIONS_DLG_HEADER
#define CC_GRAPHICAL_SEGMENTATION_OPTIONS_DLG_HEADER

//Qt
#include <QString>

//GUI
#include <ui_graphicalSegmentationOptionsDlg.h>

class ccGraphicalSegmentationOptionsDlg : public QDialog, public Ui::GraphicalSegmentationOptionsDlg
{
	Q_OBJECT

public:

	//! Default constructor
	ccGraphicalSegmentationOptionsDlg(const QString windowTitle = QString(), QWidget* parent = nullptr);

	void accept();

	//! Returns the QSettings key to store the segmentation tool options
	static QString SegmentationToolOptionsKey() { return "SegmentationToolOptions"; }
	//! Returns the QSettings key to store the 'remaining entity' suffix
	static QString RemainingSuffixKey() { return "Remaining"; }
	//! Returns the QSettings key to store the 'segmented entity' suffix
	static QString SegmentedSuffixKey() { return "Segmented"; }
};

#endif // CC_GRAPHICAL_SEGMENTATION_OPTIONS_DLG_HEADER
