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

#ifndef CC_GRAPHICAL_SEGMENTATION_TOOLS_HEADER
#define CC_GRAPHICAL_SEGMENTATION_TOOLS_HEADER

//Local
#include <ccOverlayDialog.h>

//qCC_db
#include <ccHObject.h>

//system
#include <vector>

//GUI
#include <ui_graphicalSegmentationDlg.h>

class ccPolyline;
class ccPointCloud;
class ccGLWindow;

//! Graphical segmentation mechanism (with polyline)
class ccGraphicalSegmentationTool : public ccOverlayDialog, public Ui::GraphicalSegmentationDlg
{
    Q_OBJECT

public:

	//! Default constructor
	ccGraphicalSegmentationTool(QWidget* parent);
	//! Destructor
	virtual ~ccGraphicalSegmentationTool();

	//! Adds an entity (and/or its children) to the 'to be segmented' pool
	/** Warning: some entities may be rejected if they are
		locked, or can't be segmented this way.
		\return whether entity has been added to the pool or not
	**/
	bool addEntity(ccHObject* anObject);
	
	//! Returns the number of entites currently in the the 'to be segmented' pool
	unsigned getNumberOfValidEntities();

	//! Returns a given entity in the the 'to be segmented' pool
	ccHObject* getEntity(unsigned pos);

	//inherited from ccOverlayDialog
    virtual bool linkWith(ccGLWindow* win);
	virtual bool start();
	virtual void stop(bool accepted);

	//! Returns whether hidden points should be delete after segmentation
	bool deleteHiddenPoints() const { return m_deleteHiddenPoints; }

	//! Remove entities from the 'to be segmented' pool
	void removeAllEntities(bool unallocateVisibilityArrays);

protected slots:
    void segmentIn();
    void segmentOut();
    void segment(bool);
	void reset();
	void apply();
	void applyAndDelete();
	void cancel();
	void addPointToPolyline(int x, int y);
	void closePolyLine(int x=0, int y=0); //arguments for compatibility with ccGlWindow::rightButtonClicked signal
	void closeRectangle();
	void updatePolyLine(int x, int y, Qt::MouseButtons buttons);
	void pauseSegmentationMode(bool);
	void doSetPolylineSelection();
	void doSetRectangularSelection();

protected:

	//! To be segmented entities
	ccHObject::Container m_toSegment;

	//! Whether something has changed or not (for proper 'cancel')
	bool m_somethingHasChanged;

	//! Process states
	enum ProcessStates
	{
		POLYLINE		= 1,
		RECTANGLE		= 2,
		//...			= 4,
		//...			= 8,
		//...			= 16,
		PAUSED			= 32,
		STARTED			= 64,
		RUNNING			= 128,
	};

	//! Current process state
	unsigned m_state;

	//! Segmentation polyline
	ccPolyline* m_segmentationPoly;
	//! Segmentation polyline vertices
	ccPointCloud* m_polyVertices;

	//! Selection mode
	bool m_rectangularSelection;

	//! Whether to delete or not hidden points after segmentation
	bool m_deleteHiddenPoints;
};

#endif
