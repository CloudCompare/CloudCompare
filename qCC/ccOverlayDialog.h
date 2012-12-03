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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2257                                                              $
//$LastChangedDate:: 2012-10-11 23:48:15 +0200 (jeu., 11 oct. 2012)        $
//**************************************************************************
//

#ifndef CC_OVERLAY_DIALOG_HEADER
#define CC_OVERLAY_DIALOG_HEADER

//Qt
#include <QDialog>

class ccGLWindow;

//! Generic overlay dialog interface
class ccOverlayDialog : public QDialog
{
    Q_OBJECT

public:

	//! Default constructor
	ccOverlayDialog(QWidget* parent=0);
	
	//! Destructor
	virtual ~ccOverlayDialog();

	//! Links the overlay dialog with a MDI window
	/** Warning: link can't be modified while dialog is displayed/process is running!
		\return success
	**/
	virtual bool linkWith(ccGLWindow* win);

    //! Starts process
    /** \return success
    **/
	virtual bool start();

    //! Stops process/dialog
    /** Automatically emits the 'processFinished' signal (with input state as argument).
        \param accepted process/dialog result
    **/
	virtual void stop(bool accepted);

	//reimplemented from QDialog
	virtual void reject();

signals:

    //! Signal emitted when process is finished
    /** \param accepted specifies how the process finished (accepted or not)
    **/
    void processFinished(bool accepted);

protected slots:

	//! Slot called when the linked window is deleted (calls 'onClose')
	virtual void onLinkedWindowDeletion(QObject* object=0);

protected:

	//! Associated (MDI) window
	ccGLWindow* m_associatedWin;

	//! Running/processing state
	bool m_processing;


};

#endif //CC_OVERLAY_DIALOG_HEADER
