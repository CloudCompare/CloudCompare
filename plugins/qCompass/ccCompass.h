//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: ccCompass                   #
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
//#                             COPYRIGHT: Sam Thiele  2017                #
//#                                                                        #
//##########################################################################

#ifndef CC_COMPASS_HEADER
#define CC_COMPASS_HEADER

//cc
#include <DgmOctree.h>
#include <DgmOctreeReferenceCloud.h>
#include <ccFacet.h>
#include <ccPlane.h>
#include <ccScalarField.h>

//qCC
#include "../ccStdPluginInterface.h"
#include <cc2DLabel.h>
#include <ccPointCloud.h>
#include <qmainwindow.h>
#include <ccPickingListener.h>
#include <ccPickingHub.h>
#include <ccGLWindow.h>

//qt
#include <qfiledialog.h>
#include <QFile>
#include <QTextStream>
#include <qinputdialog.h>

//this plugin
#include "ccMouseCircle.h"
#include "ccCompassDlg.h"
#include "ccTrace.h"
#include "ccLineation.h"
#include "ccCompassInfo.h"

//other
#include <math.h>
#include <qvariant.h>
#include <vector>

class ccCompass : public QObject, public ccStdPluginInterface, public ccPickingListener /*, public ccCompassDlg*/
{
	Q_OBJECT
		Q_INTERFACES(ccStdPluginInterface)
		//replace qDummy by the plugin name (IID should be unique - let's hope your plugin name is unique ;)
		Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.ccCompass")

public:

	//! Default constructor
	explicit ccCompass(QObject* parent = 0);

	//deconstructor
	~ccCompass();


	//inherited from ccPluginInterface
	virtual QString getName() const override { return "Compass"; }
	virtual QString getDescription() const override { return "A virtual 'compass' for measuring outcrop orientations"; }
	virtual QIcon getIcon() const override;

	//inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities) override;
	//sets the specified object to be the current trace - provided it is a ccTrace object. If not, ccTrace becomes null.
	void pickupTrace(ccHObject* o);

	virtual void getActions(QActionGroup& group) override;

	protected slots:

	/*** ADD YOUR CUSTOM ACTIONS' SLOTS HERE ***/
	void doAction();

	bool startMeasuring(); //start picking mode
	bool stopMeasuring(); //stop picking mode

	//picked point callbacks
	void pointPicked(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3& P);
	virtual void onItemPicked(const ccPickingListener::PickedItem& pi); //inherited from ccPickingListener

	//event to get mouse-move updates & trigger repaint of overlay circle
	bool eventFilter(QObject* obj, QEvent* event);

	//GUI actions
	void onClose();
	void onAccept();
	void onSave();
	void onUndo();
	void setLineationMode();
	void setPlaneMode();
	void setTraceMode();
	void toggleStipple(bool checked);
	void recurseStipple(ccHObject* object, bool checked);
	void toggleLabels(bool checked);
	void recurseLabels(ccHObject* object, bool checked);
	void toggleNormals(bool checked);
	void recurseNormals(ccHObject* object, bool checked);
	void changeType();
	void showHelp();
protected:
	//used while exporting plane data
	int writePlanes(ccHObject* object, QTextStream* out, QString parentName="");
	int writeTraces(ccHObject* object, QTextStream* out, QString parentName = "");
	int writeLineations(ccHObject* object, QTextStream* out, QString parentName = "");

	//checks if an object was made by this app (i.e. returns true if we are responsible for a given layer)
	bool madeByMe(ccHObject* object);
	//returns true if object is a lineation object created by ccCompass
	bool isLineation(ccHObject* object);
	//returns true if object is a plane created by ccCompass (has the associated data)
	bool isFitPlane(ccHObject* object);
	//returns true if object is a trace created by ccCompass (has the associated data)
	bool isTrace(ccHObject* object);

	//Action to start ccCompass
	QAction* m_action;

	//! Picking hub
	ccPickingHub* m_pickingHub;

	//link to application windows
	ccGLWindow* m_window;
	QMainWindow* m_main_window;

	//2D-circle for selection during plane-mode
	ccMouseCircle* m_mouseCircle;

	//ccCompass toolbar gui
    ccCompassDlg* m_dlg;
	
	enum MODE
	{
		PLANE_MODE,
		TRACE_MODE,
		LINEATION_MODE
	};
	MODE m_pickingMode = MODE::PLANE_MODE;

	//active trace for trace mode
	ccTrace* m_trace=0;
	int m_trace_id=-1; //used to check if m_trace has been deleted
	ccLineation* m_lineation = 0;

	//name of structure currently being digitized
	QString m_category = "Bedding";

	//drawing properties
	bool m_drawName = false;
	bool m_drawStippled = true;
	bool m_drawNormals = true;
};

#endif
