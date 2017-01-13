//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: ccCompass                      #
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
//#                     COPYRIGHT: Sam Thiele  2017                        #
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
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QInputDialog>
#include <QVariant>

//this plugin
#include "ccMouseCircle.h"
#include "ccCompassDlg.h"
#include "ccTrace.h"
#include "ccLineation.h"
#include "ccCompassInfo.h"

//other
#include <math.h>
#include <vector>

class ccCompass : public QObject, public ccStdPluginInterface, public ccPickingListener
{
	Q_OBJECT
		Q_INTERFACES(ccStdPluginInterface)
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
	virtual void stop() override { stopMeasuring(); m_dlg = nullptr; ccStdPluginInterface::stop(); }

	//inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual void getActions(QActionGroup& group) override;

	//sets the specified object to be the current trace - provided it is a ccTrace object. If not, ccTrace becomes null.
	void pickupTrace(ccHObject* o);

protected slots:

	void doAction();

	bool startMeasuring(); //start picking mode
	bool stopMeasuring(); //stop picking mode

	//picked point callbacks
	void pointPicked(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3& P);
	
	//inherited from ccPickingListener
	virtual void onItemPicked(const ccPickingListener::PickedItem& pi) override;

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

	//event to get mouse-move updates & trigger repaint of overlay circle
	virtual bool eventFilter(QObject* obj, QEvent* event) override;

	//used while exporting plane data
	int writePlanes(ccHObject* object, QTextStream* out, QString parentName = QString());
	int writeTraces(ccHObject* object, QTextStream* out, QString parentName = QString());
	int writeLineations(ccHObject* object, QTextStream* out, QString parentName = QString());

	//checks if an object was made by this app (i.e. returns true if we are responsible for a given layer)
	bool madeByMe(ccHObject* object);
	//returns true if object is a lineation object created by ccCompass
	bool isLineation(ccHObject* object);
	//returns true if object is a plane created by ccCompass (has the associated data)
	bool isFitPlane(ccHObject* object);
	//returns true if object is a trace created by ccCompass (has the associated data)
	bool isTrace(ccHObject* object);

	//cleans up pointers etc before changing tools
	void cleanupBeforeToolChange();

	//Action to start ccCompass
	QAction* m_action = nullptr;

	//link to application windows
	ccGLWindow* m_window = nullptr;
	QMainWindow* m_main_window = nullptr;

	//2D-circle for selection during plane-mode
	ccMouseCircle* m_mouseCircle = nullptr;

	//ccCompass toolbar gui
	ccCompassDlg* m_dlg = nullptr;
	
	enum MODE
	{
		PLANE_MODE,
		TRACE_MODE,
		LINEATION_MODE
	};
	MODE m_pickingMode = MODE::PLANE_MODE;

	//active trace for trace mode
	ccTrace* m_trace = nullptr;
	int m_trace_id=-1; //used to check if m_trace has been deleted
	ccLineation* m_lineation = nullptr;
	int m_lineation_id = -1; //used to check if m_lineation has been deleted

	//name of structure currently being digitized
	QString m_category = "Bedding";

	//drawing properties
	bool m_drawName = false;
	bool m_drawStippled = true;
	bool m_drawNormals = true;
};

#endif
