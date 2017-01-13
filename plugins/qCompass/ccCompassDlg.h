//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_COMPASS_DIALOG_HEADER
#define CC_COMPASS_DIALOG_HEADER

//Qt
#include <QDialog>
#include <QList>
#include <QAction>

//CC
#include <ccGLWindow.h>
#include <ccOverlayDialog.h>

//Local
#include <ui_compassDlg.h>
#include "ccTrace.h"


class ccCompassDlg : public ccOverlayDialog, public Ui::compassDlg
{
	Q_OBJECT

public:
	//! Default constructor
	explicit ccCompassDlg(QWidget* parent = 0);

	/*
	Returns a flag describing the currently selected ccTrace::COST_MODE (used to build the cost function for optimisation)
	*/
	int getCostMode();
	/*
	Returns true if the m_plane_fit action is checked -> used to check if the user expects us to fit a plane to finished traces.
	*/
	bool planeFitMode();

	QMenu *m_cost_algorithm_menu;

private:
	QAction *m_dark;
	QAction *m_light;
	QAction *m_rgb;
	QAction *m_grad;
	QAction *m_curve;
	QAction *m_dist;
	QAction *m_scalar;
	QAction *m_scalar_inv;
	QAction *m_plane_fit;
};

#endif
