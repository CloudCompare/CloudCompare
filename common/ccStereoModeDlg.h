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

#ifndef CC_STEREO_MODE_DLG_HEADER
#define CC_STEREO_MODE_DLG_HEADER

//qCC_gl
#include <ccGLWindow.h>

//Qt
#include <QDialog>

namespace Ui
{
	class StereoModeDialog;
}

//! Dialog to define the parameters of the stereo mode (for 3D views)
class ccStereoModeDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccStereoModeDlg(QWidget* parent);
	~ccStereoModeDlg() override;

	//! Returns the current parameters
	ccGLWindow::StereoParams getParameters() const;

	//! Sets the current parameters
	void setParameters(const ccGLWindow::StereoParams& params);

	//! Returns whether the FOV should be updated or not
	bool updateFOV() const;

protected slots:

	//! Slot called when the glass type is modified
	void glassTypeChanged(int);
	
private:
	Ui::StereoModeDialog* m_ui;
};

#endif //CC_STEREO_MODE_DLG_HEADER
