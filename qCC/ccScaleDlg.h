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

namespace Ui {
	class ScaleDialog;
}

//! Scale / multiply dialog
class ccScaleDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccScaleDlg(QWidget* parent = nullptr);
	
	~ccScaleDlg();

	//! Returns scales
	CCVector3d getScales() const;

	//! Whether the entity should be 'kept in place' or not
	bool keepInPlace() const;

	//! Whether the Global shift should be rescaled as well
	bool rescaleGlobalShift() const;

	//! Saves state
	void saveState();

private:

	void allDimsAtOnceToggled(bool);
	void fxUpdated(double);

	Ui::ScaleDialog* m_ui;
};

#endif //CC_SCALE_DLG_HEADER
