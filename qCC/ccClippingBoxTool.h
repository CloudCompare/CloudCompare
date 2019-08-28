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

#ifndef CC_CLIPPING_BOX_TOOL_HEADER
#define CC_CLIPPING_BOX_TOOL_HEADER

//common
#include <ccOverlayDialog.h>

//loncaol
#include "ccContourExtractor.h"

#include <ui_clippingBoxDlg.h>

//qCC_db
#include <ccGLUtils.h>

//system
#include <vector>

class ccGenericPointCloud;
class ccGenericMesh;
class ccProgressDialog;
class ccGLWindow;
class ccHObject;
class ccClipBox;
class ccPolyline;
class ccBBox;

//! Dialog for managing a clipping box
class ccClippingBoxTool : public ccOverlayDialog, public Ui::ClippingBoxDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccClippingBoxTool(QWidget* parent);
	//! Default destructor
	virtual ~ccClippingBoxTool();

	//inherited from ccOverlayDialog
	virtual bool linkWith(ccGLWindow* win) override;
	virtual bool start() override;
	virtual void stop(bool state) override;

	//! Returns box
	const ccClipBox* box() const { return m_clipBox; }

	//! Adds an entity
	/** \return success, if the entity is eligible for clipping
	**/
	bool addAssociatedEntity(ccHObject* anObject);

	//! Returns the current number of associated entities
	unsigned getNumberOfAssociatedEntity() const;

	//! Extract slices and optionally contours from various clouds and/or clouds
	/** \param clouds input clouds (may be empty if meshes are defined)
		\param meshes input meshes (may be empty if clouds are defined)
		\param clipBox clipping box
		\param singleContourMode if true, a single cut is made (the process is not repeated) and only the contour is extracted (not the slice)
		\param processDimensions If singleContourMode is true: the dimension normal to the slice should be true (and the others false). Otherwise: the dimensions along which to repeat the cuting process should be true.
		\param outputSlices output slices (if successful)
		\param extractContours whether to extract contours or not
		\param maxEdgeLength max contour edge length (the smaller, the tighter the contour will be)
		\param outputContours output contours (if successful)
		\param gap optional gap between each slice
		\param multiPass multi-pass contour extraction
		\param splitContours whether to split the contour when the segment can't be smaller than the specified 'maxEdgeLength'
		\param projectOnBestFitPlane to project the points on the slice best fitting plane (otherwise the plane normal to the 
		\param visualDebugMode displays a 'debugging' window during the contour extraction process
		\param generateRandomColors randomly colors the extracted slices
		\param progressDialog optional progress dialog
	**/
	static bool ExtractSlicesAndContours
		(
		const std::vector<ccGenericPointCloud*>& clouds,
		const std::vector<ccGenericMesh*>& meshes,
		ccClipBox& clipBox,
		bool singleContourMode,
		bool processDimensions[3],
		std::vector<ccHObject*>& outputSlices,
		bool extractContours,
		PointCoordinateType maxEdgeLength,
		std::vector<ccPolyline*>& outputContours,
		ccContourExtractor::ContourType contourType,
		PointCoordinateType gap = 0,
		bool multiPass = false,
		bool splitContours = false,
		bool projectOnBestFitPlane = false,
		bool visualDebugMode = false,
		bool generateRandomColors = false,
		ccProgressDialog* progressDialog = 0);

protected slots:

	void toggleInteractors(bool);
	void toggleBox(bool);

	void editBox();
	void restoreLastBox();
	void reset();
	void closeDialog();
	void extractContour();
	void removeLastContour();
	void exportSlice();
	void exportMultSlices();

	void onBoxModified(const ccBBox* box);

	void thicknessChanged(double);

	inline void shiftXMinus() { shiftBox(0, true);  }
	inline void shiftXPlus()  { shiftBox(0, false); }
	inline void shiftYMinus() { shiftBox(1, true);  }
	inline void shiftYPlus()  { shiftBox(1, false); }
	inline void shiftZMinus() { shiftBox(2, true);  }
	inline void shiftZPlus()  { shiftBox(2, false); }

	void setFrontView();
	void setBottomView();
	void setTopView();
	void setBackView();
	void setLeftView();
	void setRightView();

protected:

	//! Extracts slices and/or contours
	void extractSlicesAndContours(bool extractSlices, bool extractContours, bool singleContourMode);

	//! Shift box
	void shiftBox(unsigned char dim, bool minus);

	//! Sets predefined view
	void setView(CC_VIEW_ORIENTATION orientation);

	//! Clipping box
	ccClipBox* m_clipBox;

};

#endif //CC_CLIPPING_BOX_TOOL_HEADER
