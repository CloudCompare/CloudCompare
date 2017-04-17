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
//#                    COPYRIGHT: Daniel Girardeau-Montaut                 #
//#                                                                        #
//##########################################################################

#ifndef CC_SCALE_DLG_HEADER
#define CC_SCALE_DLG_HEADER

//CC_Lib
#include <CCGeom.h>

//Qt
#include <QDialog>

#include <ui_scaleDlg.h>

//! Scale / multiply dialog
class ccScaleDlg : public QDialog, public Ui::ScaleDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccScaleDlg(QWidget* parent = 0);

	//! Returns scales
	CCVector3d getScales() const;

	//! Whether the entity should be 'kept in place' or not
	bool keepInPlace() const;

	//! Whether the Global shift should be rescaled as well
	bool rescaleGlobalShift() const;

	//! Saves state
	void saveState();

protected slots:

	void allDimsAtOnceToggled(bool);
	void fxUpdated(double);

protected:

};

#endif //CC_SCALE_DLG_HEADER
