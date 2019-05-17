#ifndef BDR_2_5D_EDITOR_HEADER
#define BDR_2_5D_EDITOR_HEADER

//qCC_db
#include <ccRasterGrid.h>

class ccBoundingBoxEditorDlg;
class ccGLWindow;
class QWidget;
class QFrame;
class QComboBox;
class ccImage;

//! 2.5D data editor (generic interface)
class bdr2Point5DimEditor
{
public:
	//! Default constructor
	bdr2Point5DimEditor();

	//! Destructor
	virtual ~bdr2Point5DimEditor();

protected: //standard methods

	//! Returns projection grid step
	virtual double getGridStep() const;

	//! Returns projection dimension
	/** \return dimension as int (0: X, 1: Y, 2:Z)
	**/
	virtual unsigned char getProjectionDimension() const;

	//! Returns type of projection
	virtual ccRasterGrid::ProjectionType getTypeOfProjection() const;

	//! Returns custom bbox
	virtual ccBBox getCustomBBox() const;

	//! Declares whether the grid is up-to-date or not
	virtual void gridIsUpToDate(bool state);

	//! Updates the 2D display zoom
	virtual void update2DDisplayZoom(ccBBox& box);

public: //raster grid related stuff

	//! Show grid box editor and update 
	/** \return whether the box was modified or not
	**/
	virtual bool showGridBoxEditor();

	//! Returns the grid size as a string
	virtual QString getGridSizeAsString() const;

	//! Returns the grid size
	virtual bool getGridSize(unsigned& width, unsigned& height) const;

	//! Creates the bounding-box editor
	void createBoundingBoxEditor(const ccBBox& gridBBox, QWidget* parent);

	//! Creates the 2D view
	void create2DView(QFrame* parentFrame);

	//! Returns the empty cell strategy (for a given combo-box)
	ccRasterGrid::EmptyCellFillOption getFillEmptyCellsStrategy(QComboBox* comboBox) const;

	//! Shortcut to ccRasterGrid::convertToCloud
	ccPointCloud* convertGridToCloud(	const std::vector<ccRasterGrid::ExportableFields>& exportedFields,
										bool interpolateSF,
										bool interpolateColors,
										bool resampleInputCloudXY,
										bool resampleInputCloudZ, //only considered if resampleInputCloudXY is true!
										ccGenericPointCloud* inputCloud,
										bool fillEmptyCells,
										double emptyCellsHeight,
										bool exportToOriginalCS) const;

protected: //members

	//! Raster grid
	ccRasterGrid m_grid;

	//! Associated dialog
	ccBoundingBoxEditorDlg* m_bbEditorDlg;

	//! 2D display
	ccGLWindow* m_glWindow;

	//! 'Raster' cloud
	ccPointCloud* m_rasterCloud;

	ccImage* m_image;
};

#endif //BDR_2_5D_EDITOR_HEADER
