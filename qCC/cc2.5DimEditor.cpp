// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "cc2.5DimEditor.h"

// Local
#include "ccBoundingBoxEditorDlg.h"
#include "ccPersistentSettings.h"
#include "mainwindow.h"

// qCC_db
#include <ccColorTypes.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

// qCC_gl
#include <ccGLWindowInterface.h>

// Qt
#include <QCoreApplication>
#include <QFrame>
#include <QSettings>

// System
#include <assert.h>

cc2Point5DimEditor::cc2Point5DimEditor()
    : m_bbEditorDlg(nullptr)
    , m_glWindow(nullptr)
    , m_rasterCloud(nullptr)
{
}

cc2Point5DimEditor::~cc2Point5DimEditor()
{
	if (m_rasterCloud)
	{
		if (m_glWindow)
			m_glWindow->removeFromOwnDB(m_rasterCloud);
		delete m_rasterCloud;
		m_rasterCloud = nullptr;
	}
}

bool cc2Point5DimEditor::showGridBoxEditor()
{
	if (m_bbEditorDlg)
	{
		unsigned char projDim = getProjectionDimension();
		assert(projDim < 3);
		m_bbEditorDlg->set2DMode(true, projDim);

		if (m_bbEditorDlg->exec())
		{
			gridIsUpToDate(false); // will call updateGridInfo
			return true;
		}
	}

	return false;
}

void cc2Point5DimEditor::createBoundingBoxEditor(const ccBBox& gridBBox, QWidget* parent)
{
	if (!m_bbEditorDlg)
	{
		m_bbEditorDlg = new ccBoundingBoxEditorDlg(false, true, parent);
		m_bbEditorDlg->setBaseBBox(gridBBox, false);
	}
}

void cc2Point5DimEditor::create2DView(QFrame* parentFrame)
{
	if (!m_glWindow)
	{
		QWidget* glWidget = nullptr;
		ccGLWindowInterface::Create(m_glWindow, glWidget, false, true);
		assert(m_glWindow && glWidget);

		ccGui::ParamStruct params = m_glWindow->getDisplayParameters();
		// black (text) & white (background) display by default
		params.backgroundCol          = ccColor::white;
		params.textDefaultCol         = ccColor::black;
		params.drawBackgroundGradient = false;
		params.decimateMeshOnMove     = false;
		params.displayCross           = false;
		params.colorScaleUseShader    = false;
		m_glWindow->setDisplayParameters(params, true);
		m_glWindow->setPerspectiveState(false, true);
		m_glWindow->setInteractionMode(ccGLWindowInterface::INTERACT_PAN | ccGLWindowInterface::INTERACT_ZOOM_CAMERA | ccGLWindowInterface::INTERACT_CLICKABLE_ITEMS);
		m_glWindow->setPickingMode(ccGLWindowInterface::NO_PICKING);
		m_glWindow->displayOverlayEntities(true, false);
		m_glWindow->setSunLight(true);
		m_glWindow->setCustomLight(false);

		// add window to the input frame (if any)
		if (parentFrame)
		{
			auto layout = new QHBoxLayout;

			layout->setContentsMargins(0, 0, 0, 0);
			layout->addWidget(glWidget);

			parentFrame->setLayout(layout);
		}
	}
}

bool cc2Point5DimEditor::getGridSize(unsigned& gridWidth, unsigned& gridHeight) const
{
	// vertical dimension
	const unsigned char Z = getProjectionDimension();

	// cloud bounding-box --> grid size
	ccBBox box = getCustomBBox();

	// grid step
	double gridStep = getGridStep();

	return ccRasterGrid::ComputeGridSize(Z, box, gridStep, gridWidth, gridHeight);
}

QString cc2Point5DimEditor::getGridSizeAsString() const
{
	unsigned gridWidth  = 0;
	unsigned gridHeight = 0;
	if (!getGridSize(gridWidth, gridHeight))
	{
		return QObject::tr("invalid grid box");
	}

	return QString("%1 x %2 (%3 cells)").arg(gridWidth).arg(gridHeight).arg(QLocale::system().toString(gridWidth * gridHeight));
}

ccBBox cc2Point5DimEditor::getCustomBBox() const
{
	return (m_bbEditorDlg ? m_bbEditorDlg->getBox() : ccBBox());
}

void cc2Point5DimEditor::update2DDisplayZoom(ccBBox& box)
{
	if (!m_glWindow || !m_grid.isValid())
		return;

	// equivalent to 'ccGLWindow::updateConstellationCenterAndZoom' but we take aspect ratio into account

	// we set the pivot point on the box center
	CCVector3 P = box.getCenter();
	m_glWindow->setPivotPoint(P);
	m_glWindow->setCameraPos(P);

	// we compute the pixel size (in world coordinates)
	{
		double realGridWidth  = m_grid.width * m_grid.gridStep;
		double realGridHeight = m_grid.height * m_grid.gridStep;

		static const int screnMargin  = 20;
		int              screenWidth  = std::max(1, m_glWindow->glWidth() - 2 * screnMargin);
		int              screenHeight = std::max(1, m_glWindow->glHeight() - 2 * screnMargin);

		int pointSize = 1;
		if (static_cast<int>(m_grid.width) < screenWidth
		    && static_cast<int>(m_grid.height) < screenHeight)
		{
			int vPointSize = static_cast<int>(ceil(static_cast<float>(screenWidth) / m_grid.width));
			int hPointSize = static_cast<int>(ceil(static_cast<float>(screenHeight) / m_grid.height));
			pointSize      = std::min(vPointSize, hPointSize);

			// if the grid is too small (i.e. necessary point size > 10)
			if (pointSize > 10)
			{
				pointSize    = 10;
				screenWidth  = m_grid.width * pointSize;
				screenHeight = m_grid.height * pointSize;
			}
		}

		double targetDimension = realGridWidth;
		if (realGridHeight / screenHeight > realGridWidth / screenWidth)
		{
			targetDimension = (realGridHeight * screenWidth) / screenHeight;
		}

		m_glWindow->setCameraFocalToFitWidth(targetDimension);
		m_glWindow->setPointSize(pointSize);
	}

	m_glWindow->invalidateViewport();
	m_glWindow->invalidateVisualization();
	m_glWindow->deprecate3DLayer();
	m_glWindow->redraw();
}

ccPointCloud* cc2Point5DimEditor::convertGridToCloud(bool                                               exportHeightStats,
                                                     bool                                               exportSFStats,
                                                     const std::vector<ccRasterGrid::ExportableFields>& exportedStatistics,
                                                     bool                                               projectSFs,
                                                     bool                                               projectColors,
                                                     bool                                               resampleInputCloudXY,
                                                     bool                                               resampleInputCloudZ,
                                                     ccGenericPointCloud*                               inputCloud,
                                                     double                                             percentileValue,
                                                     bool                                               exportToOriginalCS,
                                                     bool                                               appendGridSizeToSFNames,
                                                     ccProgressDialog*                                  progressDialog /*=nullptr*/) const
{
	// projection dimension
	const unsigned char Z = getProjectionDimension();
	assert(Z <= 2);

	// cloud bounding-box
	ccBBox box = getCustomBBox();
	assert(box.isValid());

	return m_grid.convertToCloud(exportHeightStats,
	                             exportSFStats,
	                             exportedStatistics,
	                             projectSFs,
	                             projectColors,
	                             resampleInputCloudXY,
	                             resampleInputCloudZ,
	                             inputCloud,
	                             Z,
	                             box,
	                             percentileValue,
	                             exportToOriginalCS,
	                             appendGridSizeToSFNames,
	                             progressDialog);
}

ccRasterGrid::EmptyCellFillOption cc2Point5DimEditor::getFillEmptyCellsStrategy(QComboBox* comboBox) const
{
	if (!comboBox)
	{
		assert(false);
		return ccRasterGrid::LEAVE_EMPTY;
	}

	switch (comboBox->currentIndex())
	{
	case 0:
		return ccRasterGrid::LEAVE_EMPTY;
	case 1:
		return ccRasterGrid::FILL_MINIMUM_HEIGHT;
	case 2:
		return ccRasterGrid::FILL_AVERAGE_HEIGHT;
	case 3:
		return ccRasterGrid::FILL_MAXIMUM_HEIGHT;
	case 4:
		return ccRasterGrid::FILL_CUSTOM_HEIGHT;
	case 5:
		return ccRasterGrid::INTERPOLATE_DELAUNAY;
	case 6:
		return ccRasterGrid::KRIGING;
	default:
		// shouldn't be possible for this option!
		assert(false);
	}

	return ccRasterGrid::LEAVE_EMPTY;
}
