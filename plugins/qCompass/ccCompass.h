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
#include "ccMapDlg.h"
#include "ccTrace.h"
#include "ccLineation.h"
#include "ccCompassInfo.h"
#include "ccGeoObject.h"

//tools
#include "ccTool.h"
#include "ccFitPlaneTool.h"
#include "ccTraceTool.h"
#include "ccLineationTool.h"
#include "ccFloodTool.h"
#include "ccThicknessTool.h"

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
	virtual void stop() override { stopMeasuring(); m_dlg = nullptr; ccStdPluginInterface::stop(); } //called when the plugin is being stopped

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
	void setLineationMode(); //activates the lineation tool
	void setPlaneMode(); //activates the plane tool
	void setTraceMode(); //activates the trace tool
	void setPaintMode(); //activates the paint tool
	void setThicknessMode(); //activates the thickness tool
	void enableMapMode(); //turns on/off map mode
	void enableMeasureMode(); //turns on/off map mode
	void addGeoObject(); //creates a new GeoObject
	void pickGeoObject(); //uses a "picking tool" to select GeoObjects
	void clearGeoObject();  //clears the selected GeoObject
	void writeToInterior(); //new digitization will be added to the GeoObjects interior
	void writeToUpper(); //new digitization will be added to the GeoObjects upper boundary
	void writeToLower(); //new digitiziation will be added to the GeoObjects lower boundary
	void recalculateSelectedTraces();

	//updates drawing properites of fit planes etc.
	void toggleStipple(bool checked);
	void recurseStipple(ccHObject* object, bool checked);
	void toggleLabels(bool checked);
	void recurseLabels(ccHObject* object, bool checked);
	void toggleNormals(bool checked);
	void recurseNormals(ccHObject* object, bool checked);

	//change the m_category property
	void changeType();

	//display the help dialog
	void showHelp();

protected:

	//event to get mouse-move updates & trigger repaint of overlay circle
	virtual bool eventFilter(QObject* obj, QEvent* event) override;
	
	//used to get the place/object that new measurements or interpretation should be stored
	ccHObject* getInsertPoint();

	//cleans up pointers etc before changing tools
	void cleanupBeforeToolChange();

	//checks if the passed object, or any of it's children, represent unloaded ccCompass objects (e.g. traces, fitplanes etc).
	void tryLoading(ccHObject* obj, std::vector<int>* originals, std::vector<ccHObject*>* replacements);

	//Action to start ccCompass
	QAction* m_action = nullptr;

	//link to application windows
	ccGLWindow* m_window = nullptr;
	QMainWindow* m_main_window = nullptr;

	//ccCompass toolbar gui
	ccCompassDlg* m_dlg = nullptr;
	ccMapDlg* m_mapDlg = nullptr;

	//tools
	ccTool* m_activeTool = nullptr;
	ccFitPlaneTool* m_fitPlaneTool;
	ccTraceTool* m_traceTool;
	ccLineationTool* m_lineationTool;
	ccFloodTool* m_floodTool;
	ccThicknessTool* m_thicknessTool;

	//currently selected/active geoObject
	ccGeoObject* m_geoObject = nullptr; //the GeoObject currently being written to
	int m_geoObject_id = -1; //used to check if m_geoObject has been deleted
	
	//name/category of structure currently being digitized
	QString m_category = "Bedding";
	QString m_lastGeoObjectName = "GeoObject"; //used to 'guess' the name of new GeoObjects

	//used while exporting data
	int writePlanes(ccHObject* object, QTextStream* out, QString parentName = QString());
	int writeTraces(ccHObject* object, QTextStream* out, QString parentName = QString());
	int writeLineations(ccHObject* object, QTextStream* out, QString parentName = QString());

	//checks if an object was made by this app (i.e. returns true if we are responsible for a given layer)
	bool madeByMe(ccHObject* object);

//static flags used to define simple behaviours
public:
	//drawing properties
	static bool drawName;
	static bool drawStippled;
	static bool drawNormals;

	//calculation properties
	static bool fitPlanes;
	static int costMode;

	//digitization mode
	static bool mapMode; //true if map mode, false if measure mode
	static int mapTo; //see flags in ccGeoObject.h for definition of different mapping locations

	 

};

#endif
