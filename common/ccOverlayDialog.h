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

#ifndef CC_OVERLAY_DIALOG_HEADER
#define CC_OVERLAY_DIALOG_HEADER

//Qt
#include <QDialog>
#include <QList>

class ccGLWindow;

//! Generic overlay dialog interface
class ccOverlayDialog : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccOverlayDialog(QWidget* parent = nullptr, Qt::WindowFlags flags = Qt::FramelessWindowHint | Qt::Tool);

	//! Destructor
	~ccOverlayDialog() override;

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
	void reject() override;

	//! Adds a keyboard shortcut (single key) that will be overridden from the associated window
	/** When an overridden key is pressed, the shortcutTriggered(int) signal is emitted.
	**/
	void addOverridenShortcut(Qt::Key key);

	//! Returns whether the tool is currently started or not
	bool started() const { return m_processing; }

signals:

	//! Signal emitted when process is finished
	/** \param accepted specifies how the process finished (accepted or not)
	**/
	void processFinished(bool accepted);

	//! Signal emitted when an overridden key shortcut is pressed
	/** See ccOverlayDialog::addOverridenShortcut
	**/
	void shortcutTriggered(int key);

	//! Signal emitted when a 'show' event is detected
	void shown();

protected slots:

	//! Slot called when the linked window is deleted (calls 'onClose')
	virtual void onLinkedWindowDeletion(QObject* object = nullptr);

protected:

	//inherited from QObject
	bool eventFilter(QObject *obj, QEvent *e) override;

	//! Associated (MDI) window
	ccGLWindow* m_associatedWin;

	//! Running/processing state
	bool m_processing;

	//! Overridden keys
	QList<int> m_overriddenKeys;
};

#endif //CC_OVERLAY_DIALOG_HEADER
