//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qBroom                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#      COPYRIGHT: Wesley Grimes (Collision Engineering Associates)       #
//#                                                                        #
//##########################################################################

#ifndef CC_BROOM_DLG_HEADER
#define CC_BROOM_DLG_HEADER

#include "ui_broomDlg.h"

//CCLib
#include <CCGeom.h>

//qCC_db
#include <ccGLMatrix.h>

//system
#include <vector>
#include <stdint.h>

class ccHObject;
class ccGenericPointCloud;
class ccPointCloud;
class ccPolyline;
class ccGLWindow;
class ccBox;
class cc2DLabel;
class ccScalarField;
class ccProgressDialog;
class ccGenericGLDisplay;
class ccMainAppInterface;
class RGBAColorsTableType;

//! Dialog for the qBroom plugin
class qBroomDlg : public QDialog, public Ui::BroomDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit qBroomDlg(ccMainAppInterface* app = 0);

	//! Destructor
	virtual ~qBroomDlg();

	//! Sets associated point cloud
	/** \warning The cloud should already have an associated octree structure.
		If not, the octree will be computed and the method may return false (if
		an error occurs).
	**/
	bool setCloud(ccPointCloud* cloud, bool ownCloud = false, bool autoRedraw = true);

protected slots:

	//! Slot called when the 'reposition' button is clicked
	void onReposition();
	//! Slot called when the 'automate' button is clicked
	void onAutomate();

	//! Handles picked item (points)
	void handlePickedItem(ccHObject*, unsigned, int, int, const CCVector3&, const CCVector3d&);

	//! Slot called when the left mouse button is clicked (on the 3D view)
	void onLeftButtonClicked(int, int);
	//! Slot called when the right mouse button is clicked (on the 3D view)
	//void onRightButtonClicked(int, int);
	//! Slot called when the mouse is displaced over the 3D view
	void onMouseMoved(int, int, Qt::MouseButtons);
	//! Slot called when the mouse is released
	void onButtonReleased();

	//! Slot called when the height of the clean area is modified
	void onCleanHeightChanged(double);

	//! Slot called when one of the broom dimensions is modified
	void onDimensionChanged(double);

	//! Slot called when the selection mode changes
	void onSelectionModeChanged(int);

	//! Undoes the last selection
	void doUndo() { undo(1); }
	//! Undoes the last ten selections
	void doUndo10() { undo(10); }

	//! Cancels the segmentation process
	void cancel();
	//! Applies the segmentation
	void apply();
	//! Closes the tool
	void validate();

protected: //methods

	struct BroomDimensions
	{
		PointCoordinateType length, width, thick, height;
	};

	//! Returns the broom dimensions
	void getBroomDimensions(BroomDimensions& dimensions) const;

	//! Position the broom (between two points)
	bool positionBroom(const CCVector3& P0, const CCVector3& P1);

	//! Displace the broom (either with the mouse or with a know translation)
	/** \warning Doesn't modify the class state BUT THE UNDO/SELECTION mechanism!
		\param broomTrans starting position of the broom (updated with the new position on output)
		\param broomDelta desired displacement (updated with the actual displacement on output)
		\param stickToTheFloor whether to force the broom to stick to the 'floor' or not
		\return whether the broom could be displaced or not (e.g. it lost track)
	**/
	bool moveBroom(ccGLMatrix& broomTrans, CCVector3d& broomDelta, bool stickToTheFloor) const;

	//! Select the points inside or above/below the broom
	bool selectPoints(const ccGLMatrix& broomTrans, BroomDimensions* _broom = 0);

	//! Automate the process
	bool startAutomation();

	//! Frrezes most of the UI elements
	void freezeUI(bool state);

	//! Resets the GUI after the broom points picking process
	void stopBroomPicking();

	//! Resets the GUI after the 'automation' button as be clicked
	void stopAutomation();

	//! Updates the broom representation
	void updateBroomBox();
	//! Updates the cleaning are representation
	void updateSelectionBox();

	//! Selects a given point
	/** \return whether the point has been actually selected
	**/
	bool selectPoint(unsigned index);

	//! Undoes a given number of steps
	void undo(uint32_t count);

	//! Prepares a new 'undo' step
	uint32_t addUndoStep(const ccGLMatrix& broomPos);

	//! Displays an error message
	void displayError(QString message);

	//! Generates the segmented point cloud
	ccPointCloud* createSegmentedCloud(ccPointCloud* cloud, bool removeSelected, bool& error);

	//inherited from QWidget
	void closeEvent(QCloseEvent*);

	//! Updates the automation area preview polyline
	void updateAutomationAreaPolyline(int x, int y);

	//! Saves persistent settings
	void savePersistentSettings();

protected: //members

	//! Cloud original state backup structure
	struct CloudBackup
	{
		ccPointCloud* ref;
		RGBAColorsTableType* colors;
		bool hadColors;
		int displayedSFIndex;
		ccGenericGLDisplay* originDisplay;
		bool colorsWereDisplayed;
		bool sfWasDisplayed;
		bool wasVisible;
		bool wasEnabled;
		bool wasSelected;
		bool hadOctree;
		bool ownCloud;

		//! Default constructor
		CloudBackup()
			: ref(0)
			, colors(0)
			, hadColors(false)
			, displayedSFIndex(-1)
			, originDisplay(0)
			, colorsWereDisplayed(false)
			, sfWasDisplayed(false)
			, wasVisible(false)
			, wasEnabled(false)
			, wasSelected(false)
			, hadOctree(false)
			, ownCloud(false)
		{}

		//! Destructor
		~CloudBackup() { restore(); clear(); }

		//! Backups the given cloud
		void backup(ccPointCloud* cloud);

		//! Backups the colors (not done by default)
		bool backupColors();

		//! Restores the cloud
		void restore();

		//! Clears the structure
		void clear();
	};

	//! Associated cloud (descriptor)
	CloudBackup m_cloud;

	//! Dialog's own 3D view
	ccGLWindow* m_glWindow;

	//! Broom box
	ccBox* m_broomBox;
	//! Selection box
	ccBox* m_selectionBox;
	//! Boxes container
	ccHObject* m_boxes;

	//! Picking parameters
	struct Picking
	{
		Picking() : mode(NO_PICKING) {}
		~Picking() { clear(); }

		cc2DLabel* addLabel(ccGenericPointCloud* cloud, unsigned pointIndex);
		void clear();
		
		enum Mode { NO_PICKING, BROOM_PICKING, AUTO_AREA_PICKING };
		
		Mode mode;
		std::vector<cc2DLabel*> labels;
	};

	//! Automation area
	struct AutomationArea
	{
		AutomationArea() : polyline(0) {}
		~AutomationArea() { clear(); }

		void clear();

		ccPolyline* polyline;
		std::vector<CCVector3> clickedPoints;
	};

	//!  Current picking parameters
	Picking m_picking;

	//! Automation area
	AutomationArea m_autoArea;

	//! Last mouse cursor position
	QPoint m_lastMousePos;
	//! Last click position in 3D
	CCVector3 m_lastMousePos3D;
	//! Whether the last click position in 3D is valid or not
	bool m_hasLastMousePos3D;
	//! Whether the initial click occurred on the broom or not
	bool m_broomSelected;

	//! Selection modes
	enum SelectionModes {	INSIDE = 0,
							ABOVE = 1,
							BELOW = 2,
							ABOVE_AND_BELOW = 3
	};

	//! Current selection mode
	SelectionModes m_selectionMode;

	//! Selection table
	std::vector<uint32_t> m_selectionTable;

	//! Positions of the broom (for undo)
	std::vector<ccGLMatrix> m_undoPositions;

	//! Associated application
	ccMainAppInterface* m_app;

	//! First cloud
	ccPointCloud* m_initialCloud;
};

#endif
