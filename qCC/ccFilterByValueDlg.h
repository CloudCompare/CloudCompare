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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#ifndef CC_FILTER_BY_VALUE_DIALOG_HEADER
#define CC_FILTER_BY_VALUE_DIALOG_HEADER

#include <ui_filterByValueDlg.h>

//Qt
#include <QDialog>

//! Dialog to sepcify a range of SF values and how the corresponding points should be extracted
class ccFilterByValueDlg : public QDialog, public Ui::FilterByValueDialog
{
	Q_OBJECT

public:

	//! Default constructor
	ccFilterByValueDlg(	double minRange,
						double maxRange,
						double minVal = -1.0e9,
						double maxVal = 1.0e9,
						QWidget* parent = 0);

	//! Mode
	enum Mode { EXPORT, SPLIT, CANCEL };

	//! Returns the selected mode
	Mode mode() const { return m_mode; }
	
protected slots:

	void onExport() { m_mode = EXPORT; accept(); }
	void onSplit() { m_mode = SPLIT; accept(); }

protected:

	Mode m_mode;
};

#endif //CC_FILTER_BY_VALUE_DIALOG_HEADER
