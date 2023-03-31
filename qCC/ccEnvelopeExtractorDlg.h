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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//Qt
#include <QDialog>
#include <QEventLoop>

//qCC_db
#include <ccBBox.h>

//GUI
#include <ui_envelopeExtractorDlg.h>

class ccGLWindowInterface;
class ccHObject;

//! Dialog for debugging envelope extraction
class ccEnvelopeExtractorDlg : public QDialog, public Ui::EnvelopeExtractorDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccEnvelopeExtractorDlg(QWidget* parent = nullptr);

	//! Initializes the display
	void init();

	//! Display a new message
	void displayMessage(QString message, bool waitForUserConfirmation = false);

	//! Waits for user action
	void waitForUser(unsigned defaultDelay_ms = 100);

	//! Returns associated GL window
	inline ccGLWindowInterface* win() { return m_glWindow; }

	//! Zooms on a given 2D region (3D bouding-box considered in 2D only)
	void zoomOn(const ccBBox& bbox);

	//! Forces refresh
	void refresh();

	//! Adds an entity to the (2D/3D) display
	void addToDisplay(ccHObject* obj, bool noDependency = true);
	//! Removes an entity from the (2D/3D) display
	void removFromDisplay(ccHObject* obj);

	//! Returns whether the dialog has been 'skipped' or not
	bool isSkipped() const;

protected:

	//! When the skip button is clicked
	void onSkipButtonClicked();

protected:

	//! Skip flag
	bool m_skipped;
	//! Local event loop
	QEventLoop m_loop;

	//! Associated 3D window
	ccGLWindowInterface* m_glWindow;
};
