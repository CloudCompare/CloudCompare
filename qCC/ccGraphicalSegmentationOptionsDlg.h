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

#ifndef CCGRAPHICALSEGMENTATIONOPTIONSDLG_H
#define CCGRAPHICALSEGMENTATIONOPTIONSDLG_H

#include <ui_graphicalSegmentationOptionsDlg.h>

//Qt
#include <QString>

class ccGraphicalSegmentationOptionsDlg : public QDialog, public Ui::GraphicalSegmentationOptionsDlg
{
	Q_OBJECT

public:

	//! Default constructor
	ccGraphicalSegmentationOptionsDlg(const QString windowTitle = QString(),
		QWidget* parent = 0);

	void accept();
};

#endif // CCGRAPHICALSEGMENTATIONOPTIONSDLG_H
