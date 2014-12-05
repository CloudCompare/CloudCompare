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

#ifndef CC_SECTION_EXTRACTION_TOOL_HEADER
#define CC_SECTION_EXTRACTION_TOOL_HEADER

//Local
#include <ccOverlayDialog.h>

//qCC_db
#include <ccHObject.h>
#include <ccPolyline.h>

//Qt
#include <QList>

//GUI
#include <ui_sectionExtractionDlg.h>

class ccGenericPointCloud;
class ccPointCloud;
class ccGLWindow;

//! Section extraction tool
class ccSectionExtractionTool : public ccOverlayDialog, public Ui::SectionExtractionDlg
{
	Q_OBJECT

public:

	//! Default constructor
	ccSectionExtractionTool(QWidget* parent);
	//! Destructor
	virtual ~ccSectionExtractionTool();

	//! Adds a cloud to the 'clouds' pool
	bool addCloud(ccGenericPointCloud* cloud, bool alreadyInDB = true);
	//! Adds a polyline to the 'sections' pool
	/** \warning: if this method returns true, the class takes the ownership of the cloud!
	**/
	bool addPolyline(ccPolyline* poly, bool alreadyInDB = true);
	
	//! Removes all registered entities (clouds & polylines)
	void removeAllEntities();
	
	//inherited from ccOverlayDialog
	virtual bool linkWith(ccGLWindow* win);
	virtual bool start();
	virtual void stop(bool accepted);

protected slots:

	void undo();
	bool reset(bool askForConfirmation = true);
	void apply();
	void addPointToPolyline(int x, int y);
	void closePolyLine(int x=0, int y=0); //arguments for compatibility with ccGlWindow::rightButtonClicked signal
	void updatePolyLine(int x, int y, Qt::MouseButtons buttons);
	void enableSectionEditingMode(bool);
	void doImportPolylinesFromDB();
	void setVertDimension(int);
	void entitySelected(int);
	void generateOrthoSections();
	void extractPoints();
	void exportSections();

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

	//! Imported entity
	template<class EntityType> struct ImportedEntity
	{
		//! Default constructor
		ImportedEntity()
			: entity(0)
			, originalDisplay(0)
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
			backupColor[0] = section.backupColor[0];
			backupColor[1] = section.backupColor[1];
			backupColor[2] = section.backupColor[2];
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
				ccPolyline* poly = (ccPolyline*)e;
				//backup color
				backupColor[0] = poly->getColor()[0];
				backupColor[1] = poly->getColor()[1];
				backupColor[2] = poly->getColor()[2];
				backupColorShown = poly->colorsShown();
				//backup thickness
				backupWidth = poly->getWidth();
			}
		}

		bool operator ==(const ImportedEntity& ie) { return entity == ie.entity; }
		
		EntityType* entity;
		ccGenericGLDisplay* originalDisplay;
		bool isInDB;

		//backup info (for polylines only)
		colorType backupColor[3];
		bool backupColorShown;
		PointCoordinateType backupWidth;
	};

	//! Section
	typedef ImportedEntity<ccPolyline> Section;

	//! Releases a polyline
	/** The polyline is removed from display. Then it is
		deleted if the polyline is not already in DB.
	**/
	void releasePolyline(Section* section);

	//! Cloud
	typedef ImportedEntity<ccGenericPointCloud> Cloud;

	//! Type of the pool of active sections
	typedef QList<Section> SectionPool;

	//! Type of the pool of clouds
	typedef QList<Cloud> CloudPool;

	//! Process states
	enum ProcessStates
	{
		//...			= 1,
		//...			= 2,
		//...			= 4,
		//...			= 8,
		//...			= 16,
		PAUSED			= 32,
		STARTED			= 64,
		RUNNING			= 128,
	};

	//! Deselects the currently selected polyline
	void selectPolyline(Section* poly, bool autoRefreshDisplay = true);

protected: //members

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
};

#endif //CC_SECTION_EXTRACTION_TOOL_HEADER
