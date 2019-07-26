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

#ifndef BDR_TRACE_FOOTPRINT_HEADER
#define BDR_TRACE_FOOTPRINT_HEADER

//Local
#include "ccContourExtractor.h"
#include "ccOverlayDialog.h"
#include "StFootPrint.h"

//qCC_db
//#include <ccHObject.h>

class ccGenericPointCloud;
class ccPointCloud;
class ccGLWindow;
class ccPlane;

namespace Ui
{
	class bdrSketcherDlg;
}

//! Section extraction tool
class bdrSketcher : public ccOverlayDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrSketcher(QWidget* parent);
	//! Destructor
	~bdrSketcher() override;

	//! Adds a cloud to the 'clouds' pool
	bool addCloud(ccGenericPointCloud* cloud, bool alreadyInDB = true);
	//! Adds a polyline to the 'sections' pool
	/** \warning: if this method returns true, the class takes the ownership of the cloud!
	**/
	bool addPolyline(ccPolyline* poly, bool alreadyInDB = true);
	
	//! Removes all registered entities (clouds & polylines)
	void removeAllEntities();
		
	//inherited from ccOverlayDialog
	bool linkWith(ccGLWindow* win) override;
	bool start() override;
	void stop(bool accepted) override;

	void setTraceViewMode(bool trace_image = true);
	void SetDestAndGround(ccHObject* dest, double ground);

	void setWorkingPlane();

	void importEntities(ccHObject::Container entities);

protected:

	void undo();
	bool reset(bool askForConfirmation = true);
	void apply();
	void cancel();
	void addPointToPolyline(int x, int y);
	void addCurrentPointToPolyline();
	void closeFootprint();
	void closePolyLine(int x=0, int y=0); //arguments for compatibility with ccGlWindow::rightButtonClicked signal
	void updatePolyLine(int x, int y, Qt::MouseButtons buttons);
	void enableSketcherEditingMode(bool);
	void doImportPolylinesFromDB();
	void setVertDimension(int);
	void entitySelected(ccHObject*);
	void exportFootprints();
	void exportSections();
	void exportFootprintInside();
	void exportFootprintOutside();

	//! To capture overridden shortcuts (pause button, etc.)
	void onShortcutTriggered(int);

protected:

	//! Projects a 2D (screen) point to 3D
	//CCVector3 project2Dto3D(int x, int y) const;

	//! Cancels currently edited polyline
	void cancelCurrentPolyline();

	//! Deletes currently selected polyline
	void deleteSelectedPolyline();

	//! Adds a 'step' on the undo stack
	void addUndoStep();

	//! Creates (if necessary) and returns a group to store entities in the main DB
	ccHObject* getExportGroup(unsigned& defaultGroupID, const QString& defaultName);

	enum POLYLINE_TYPE
	{
		FOOTPRINT_NORMAL,	// polygon counterclockwise
		FOOTPRINT_HOLE,		// polygon clockwise
		POLYLINE_OPEN,		// polyline
	};

	//! Imported entity
	template<class EntityType> struct ImportedEntity
	{
		//! Default constructor
		ImportedEntity()
			: entity(0)
			, originalDisplay(nullptr)
			, isInDB(false)
			, backupColorShown(false)
			, backupWidth(1)
		{}
		
		//! Copy constructor
		ImportedEntity(const ImportedEntity& section)
			: entity(section.entity)
			, originalDisplay(section.originalDisplay)
			, isInDB(section.isInDB)
			, backupColorShown(section.backupColorShown)
			, backupWidth(section.backupWidth)
		{
			backupColor = section.backupColor;
		}
		
		//! Constructor from an entity
		ImportedEntity(EntityType* e, bool alreadyInDB)
			: entity(e)
			, originalDisplay(e->getDisplay())
			, isInDB(alreadyInDB)
		{
			//specific case: polylines
			if (e->isA(CC_TYPES::POLY_LINE))
			{
				ccPolyline* poly = reinterpret_cast<ccPolyline*>(e);
				//backup color
				backupColor = poly->getColor();
				backupColorShown = poly->colorsShown();
				//backup thickness
				backupWidth = poly->getWidth();
			}
			if (e->isA(CC_TYPES::ST_FOOTPRINT))
			{
				StFootPrint* poly = reinterpret_cast<StFootPrint*>(e);
				//backup color
				backupColor = poly->getColor();
				backupColorShown = poly->colorsShown();
				//backup thickness
				backupWidth = poly->getWidth();

				if (!poly->isClosed()) {
					type = POLYLINE_OPEN;
				}
				else {
					//! hole state
					if (poly->isHole()) type = FOOTPRINT_HOLE;
					else type = FOOTPRINT_NORMAL;
				}
			}
		}

		bool operator ==(const ImportedEntity& ie) { return entity == ie.entity; }
		
		EntityType* entity;
		ccGenericGLDisplay* originalDisplay;
		bool isInDB;

		//backup info (for polylines only)
		ccColor::Rgb backupColor;
		bool backupColorShown;
		PointCoordinateType backupWidth;

		//! for footprint only
		POLYLINE_TYPE type;
	};

	//! Section
	using Section = ImportedEntity<ccPolyline>;

	//! Releases a polyline
	/** The polyline is removed from display. Then it is
		deleted if the polyline is not already in DB.
	**/
	void releasePolyline(Section* section);

	//! Cloud
	using Cloud = ImportedEntity<ccGenericPointCloud>;

	//! Type of the pool of active sections
	using SectionPool = QList<Section>;

	//! Type of the pool of clouds
	using CloudPool = QList<Cloud>;

	//! Process states
	enum ProcessStates
	{
		//...			= 1,
		//...			= 2,
		//...			= 4,
		//...			= 8,
		EDITING			= 16,
		PAUSED			= 32,
		STARTED			= 64,
		RUNNING			= 128,
	};

	//! Deselects the currently selected polyline
	void selectPolyline(Section* poly, bool autoRefreshDisplay = true);

	//! Updates the global clouds bounding-box
	void updateCloudsBox();

private: //members
	Ui::bdrSketcherDlg	*m_UI;
	
	//! Pool of active sections
	SectionPool m_sections;

	//! Selected polyline (if any)
	Section* m_selectedPoly;

	//! Pool of clouds
	CloudPool m_clouds;

	//! Current process state
	unsigned m_state;

	//! Last 'undo' count
	std::vector<size_t> m_undoCount;

	//! Currently edited polyline
	ccPolyline* m_editedPoly;
	//! Segmentation polyline vertices
	ccPointCloud* m_editedPolyVertices;

	//! Global clouds bounding-box
	ccBBox m_cloudsBox;

	double m_ground;
	ccHObject* m_dest_obj;

	bool m_trace_image;

	ccPlane* m_workingPlane;
};

#endif //BDR_TRACE_FOOTPRINT_HEADER
