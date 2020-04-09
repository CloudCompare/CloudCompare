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

//qCC
#include "ccStdPluginInterface.h"
#include <ccPickingListener.h>

class ccCompassDlg;
class ccFitPlaneTool;
class ccGeoObject;
class ccLineationTool;
class ccMapDlg;
class ccNoteTool;
class ccPinchNodeTool;
class ccThicknessTool;
class ccTool;
class ccTopologyTool;
class ccTraceTool;
class ccSNECloud;

class ccCompass : public QObject, public ccStdPluginInterface, public ccPickingListener
{
	Q_OBJECT
		Q_INTERFACES(ccStdPluginInterface)
		Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.ccCompass" FILE "info.json")

public:
	//! Default constructor
	explicit ccCompass(QObject* parent = nullptr);

	//deconstructor
	~ccCompass() override;

	//inherited from ccPluginInterface
	void stop() override;

	//inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities) override;
	QList<QAction *> getActions() override;

protected:

	//initialise the plugin
	void doAction();

	//start picking mode
	bool startMeasuring();

	//stop picking mode
	bool stopMeasuring(bool finalStop = false);
	
	//inherited from ccPickingListener
	void onItemPicked(const ccPickingListener::PickedItem& pi) override;

	//picked point callback (called by the above function)
	void pointPicked(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3& P);

	//**************
	//GUI actions:
	//**************
	//general
	void onClose();
	void onAccept();
	void onSave();
	void onUndo();

	//modes
	void enableMapMode(); //turns on/off map mode
	void enableMeasureMode(); //turns on/off map mode

	//tools
	void setPick(); //activates the picking tool
	void setLineation(); //activates the lineation tool
	void setPlane(); //activates the plane tool
	void setTrace(); //activates the trace tool

	//extra tools
	void addPinchNode(); //activates the pinch node tool
	void setThickness(); //activates the thickness tool
	void setThickness2(); //activates the thickness tool in two-point mode
	void setYoungerThan(); //activates topology tool in "younger-than" mode
	void setFollows(); //activates topology tool in "follows" mode
	void setEquivalent(); //activates topology mode in "equivalent" mode
	void setNote(); //activates the note tool
	void recalculateSelectedTraces(); //recalculate any selected traces (for updating with a different cost function)
	void mergeGeoObjects(); //merges the selected GeoObjects
	void fitPlaneToGeoObject(); //calculates best-fit plane for the upper and lower surfaces of the selected GeoObject
	void recalculateFitPlanes(); //recalcs fit planes for traces and GeoObjects
	void estimateStructureNormals(); //Estimate the normal vector to the structure this trace represents at each point in this trace.
	void estimateP21(); //Calculate the intensity of selected structures 
	void estimateStrain(); //Estimate strain from Mode-I dykes and veins
	void convertToPointCloud(); //converts selected traces or geoObjects to point clouds
	void distributeSelection(); //distributes selected objects into GeoObjects with the same name
	void exportToSVG(); //exports current view to SVG

	//map mode dialog
	void writeToInterior(); //new digitization will be added to the GeoObjects interior
	void writeToUpper(); //new digitization will be added to the GeoObjects upper boundary
	void writeToLower(); //new digitiziation will be added to the GeoObjects lower boundary
	void addGeoObject(bool singleSurface=false); //creates a new GeoObject
	void addGeoObjectSS(); //creates a new single surface GeoObject

	//drawing options
	void hideAllPointClouds(ccHObject* o); //hides all point clouds and adds them to the m_hiddenObjects list
	void toggleStipple(bool checked);
	void recurseStipple(ccHObject* object, bool checked);
	void toggleLabels(bool checked);
	void recurseLabels(ccHObject* object, bool checked);
	void toggleNormals(bool checked);
	void recurseNormals(ccHObject* object, bool checked);

	//display the help dialog
	void showHelp();

protected:

	//event to get mouse-move updates & trigger repaint of overlay circle
	bool eventFilter(QObject* obj, QEvent* event) override;
	
	//used to get the place/object that new measurements or interpretation should be stored
	ccHObject* getInsertPoint();

	//cleans up pointers etc before changing tools
	void cleanupBeforeToolChange(bool autoRestartPicking = true);

	//registers this plugin with the picking hub
	bool startPicking();

	//removes this plugin from the picking hub
	void stopPicking();

	//checks if the passed object, or any of it's children, represent unloaded ccCompass objects (e.g. traces, fitplanes etc).
	void tryLoading();
	void tryLoading(ccHObject* obj, std::vector<int>* originals, std::vector<ccHObject*>* replacements);

	//Action to start ccCompass
	QAction* m_action = nullptr;

	//link to application windows
	QMainWindow* m_main_window = nullptr;

	//picking or not?
	bool m_picking = false;
	bool m_active = false;

	//ccCompass toolbar gui
	ccCompassDlg* m_dlg = nullptr;
	ccMapDlg* m_mapDlg = nullptr;

	//tools
	ccTool* m_activeTool = nullptr;
	ccFitPlaneTool* m_fitPlaneTool;
	ccTraceTool* m_traceTool;
	ccLineationTool* m_lineationTool;
	ccThicknessTool* m_thicknessTool;
	ccTopologyTool* m_topologyTool;
	ccNoteTool* m_noteTool;
	ccPinchNodeTool* m_pinchNodeTool;

	//currently selected/active geoObject
	ccGeoObject* m_geoObject = nullptr; //the GeoObject currently being written to
	int m_geoObject_id = -1; //used to check if m_geoObject has been deleted
	std::vector<int> m_hiddenObjects; //used to hide objects (for picking)

	//used to 'guess' the name of new GeoObjects
	QString m_lastGeoObjectName = "GeoObject"; 

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
