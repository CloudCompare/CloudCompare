//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#               COPYRIGHT: Luca Penasa                                   #
//#                                                                        #
//##########################################################################
//
#ifndef CC_SIFT_DLG_HEADER
#define CC_SIFT_DLG_HEADER

#include <ui_SIFTExtractDlg.h>

//Qt
#include <QObject>

class SIFTExtractDlg : public QDialog, public Ui::SIFTExtractDlg
{
	Q_OBJECT

public:
	SIFTExtractDlg(QWidget* parent=0);

	void updateComboBox(std::vector<std::string> fields);

public slots:
	void disableCombo();

};

#endif
