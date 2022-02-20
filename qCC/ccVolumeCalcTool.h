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

#ifndef CC_VOLUME_CALC_TOOL_HEADER
#define CC_VOLUME_CALC_TOOL_HEADER

//Local
#include "cc2.5DimEditor.h"

//Qt
#include <QDialog>

class ccGenericPointCloud;
class ccPointCloud;
class ccPolyline;

namespace Ui {
	class VolumeCalcDialog;
}

//! Volume calculation tool (dialog)
class ccVolumeCalcTool : public QDialog, public cc2Point5DimEditor
{
	Q_OBJECT

public:
	//! Default constructor
	ccVolumeCalcTool(ccGenericPointCloud* cloud1, ccGenericPointCloud* cloud2, QWidget* parent = nullptr);

	//! Destructor
	~ccVolumeCalcTool();

	//Inherited from cc2Point5DimEditor
	virtual double getGridStep() const override;
	virtual unsigned char getProjectionDimension() const override;
	virtual ccRasterGrid::ProjectionType getTypeOfProjection() const override;

	//! Report info
	struct ReportInfo
	{
		ReportInfo()
			: volume(0)
			, addedVolume(0)
			, removedVolume(0)
			, surface(0)
			, matchingPrecent(0)
			, ceilNonMatchingPercent(0)
			, groundNonMatchingPercent(0)
			, averageNeighborsPerCell(0)
		{}

		QString toText(int precision = 6) const;

		double volume;
		double addedVolume;
		double removedVolume;
		double surface;
		float matchingPrecent;
		float ceilNonMatchingPercent;
		float groundNonMatchingPercent;
		double averageNeighborsPerCell;
	};

	//! Static accessor
	static bool ComputeVolume(	ccRasterGrid& grid,
								ccGenericPointCloud* ground,
								ccGenericPointCloud* ceil,
								const ccBBox& gridBox,
								unsigned char vertDim,
								double gridStep,
								unsigned gridWidth,
								unsigned gridHeight,
								ccRasterGrid::ProjectionType projectionType,
								ccRasterGrid::EmptyCellFillOption groundEmptyCellFillStrategy,
								double groundMaxEdgeLength,
								ccRasterGrid::EmptyCellFillOption ceilEmptyCellFillStrategy,
								double ceilMaxEdgeLength,
								ccVolumeCalcTool::ReportInfo& reportInfo,
								double groundHeight,
								double ceilHeight,
								QWidget* parentWidget = nullptr);

	//! Converts a (volume) grid to a point cloud
	static ccPointCloud* ConvertGridToCloud(	ccRasterGrid& grid,
												const ccBBox& gridBox,
												unsigned char vertDim,
												bool exportToOriginalCS);

	
protected:

	//! Accepts the dialog and save settings
	void saveSettingsAndAccept();

	//! Save persistent settings and 'accept' dialog
	void saveSettings();

	//! Called when the projection direction changes
	void projectionDirChanged(int);

	//! Called when the SF projection type changes
	void sfProjectionTypeChanged(int);

	//Inherited from cc2Point5DimEditor
	virtual bool showGridBoxEditor() override;

	//! Called when the (ground) empty cell filling strategy changes
	void groundFillEmptyCellStrategyChanged(int);
	//! Called when the (ceil) empty cell filling strategy changes
	void ceilFillEmptyCellStrategyChanged(int);

	//! Called when the an option of the grid generation has changed
	void gridOptionChanged();

	//! Updates the gid info
	void updateGridInfo();

	//! Update the grid and the 2D display
	void updateGridAndDisplay();

	//! Swap roles
	void swapRoles();

	//! Ground source changed
	void groundSourceChanged(int);
	//! Ceil source changed
	void ceilSourceChanged(int);

	//! Exports info to clipboard
	void exportToClipboard() const;

	//! Exports the grid as a point cloud
	void exportGridAsCloud() const;

	//! Sets the displayed number precision
	void setDisplayedNumberPrecision(int);

	//! Returns the ground cloud (or constant height)
	std::pair<ccGenericPointCloud*, double> getGroundCloud() const;

	//! Returns the ceil cloud (or constant height)
	std::pair<ccGenericPointCloud*, double> getCeilCloud() const;

protected: //standard methods

	//Inherited from cc2Point5DimEditor
	virtual void gridIsUpToDate(bool state) override;

	//! Load persistent settings
	void loadSettings();

	//! Updates the grid
	bool updateGrid();

	//! Converts the grid to a point cloud
	ccPointCloud* convertGridToCloud(bool exportToOriginalCS) const;

	//! Outputs the report
	void outputReport(const ReportInfo& info);

protected: //members

	//! First associated cloud
	ccGenericPointCloud* m_cloud1;
	//! Second associated cloud
	ccGenericPointCloud* m_cloud2;

	//! Last report
	/** Only valid if clipboardPushButton is enabled
	**/
	ReportInfo m_lastReport;
	
	Ui::VolumeCalcDialog* m_ui;
};

#endif //CC_VOLUME_CALC_TOOL_HEADER
