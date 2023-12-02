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
//#                   COPYRIGHT: CloudCompare project                      #
//#                                                                        #
//##########################################################################

#include <ui_setSFAsNormalsDlg.h>

//Qt
#include <QDialog>

class ccPointCloud;

//! Let the user choose up to 3 scalar fields (to be used as Normal components)
class ccSetSFsAsNormalDialog : public QDialog, public Ui::SetSFsAsNormalDialog
{
	Q_OBJECT

public:
	
	//! Default constructor
	ccSetSFsAsNormalDialog(const ccPointCloud* cloud, QWidget* parent = nullptr);

	//! No SF index
	static const int SF_INDEX_NO   = -1;
	//! 'Zero' SF index
	static const int SF_INDEX_ZERO = -2;
	//! 'One' SF index
	static const int SF_INDEX_ONE  = -3;

	//! Sets the 3 SF indexes
	void setSFIndexes(int sf1Index, int sf2Index, int sf3Index);

	//! Returns the 3 SF indexes
	void getSFIndexes(int& sf1Index, int& sf2Index, int& sf3Index) const;
};

