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

#include "bdr2.5DimEditor.h"

//Local
#include "ccBoundingBoxEditorDlg.h"
#include "ccPersistentSettings.h"
#include "mainwindow.h"

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccProgressDialog.h>
#include <ccColorTypes.h>
#include <ccImage.h>
#include <ccCameraSensor.h>

//qCC_gl
#include <ccGLWidget.h>

//Qt
#include <QFrame>
#include <QSettings>
#include <QCoreApplication>

//System
#include <assert.h>

bdr2Point5DimEditor::bdr2Point5DimEditor()
	: m_bbEditorDlg(0)
	, m_glWindow(0)
	, m_rasterCloud(0)
	, m_image(nullptr)
{
}

bdr2Point5DimEditor::~bdr2Point5DimEditor()
{
	if (m_rasterCloud)
	{
		if (m_glWindow)
			m_glWindow->removeFromOwnDB(m_rasterCloud);
		delete m_rasterCloud;
		m_rasterCloud = 0;
	}
	if (m_image) {
		if (m_glWindow)
			m_glWindow->removeFromOwnDB(m_image);
		delete m_image;
		m_image = nullptr;
	}
}

double bdr2Point5DimEditor::getGridStep() const
{
	return 0.0;
}

unsigned char bdr2Point5DimEditor::getProjectionDimension() const
{
	return 0;
}

ccRasterGrid::ProjectionType bdr2Point5DimEditor::getTypeOfProjection() const
{
	return ccRasterGrid::ProjectionType();
}

bool bdr2Point5DimEditor::showGridBoxEditor()
{
	if (m_bbEditorDlg)
	{
		unsigned char projDim = getProjectionDimension();
		assert(projDim < 3);
		m_bbEditorDlg->set2DMode(true, projDim);
		if (m_bbEditorDlg->exec())
		{
			gridIsUpToDate(false);
			return true;
		}
	}

	return false;
}

void bdr2Point5DimEditor::createBoundingBoxEditor(const ccBBox& gridBBox, QWidget* parent)
{
	if (!m_bbEditorDlg)
	{
		m_bbEditorDlg = new ccBoundingBoxEditorDlg(parent);
		m_bbEditorDlg->setBaseBBox(gridBBox, false);
	}
}

void bdr2Point5DimEditor::create2DView(QFrame* parentFrame)
{
	if (!m_glWindow)
	{
		QWidget* glWidget = 0;
		CreateGLWindow(m_glWindow, glWidget, false, true);
		assert(m_glWindow && glWidget);
		
		ccGui::ParamStruct params = m_glWindow->getDisplayParameters();
		//black (text) & white (background) display by default
		params.backgroundCol = ccColor::white;
		params.textDefaultCol = ccColor::black;
		params.drawBackgroundGradient = false;
		params.decimateMeshOnMove = false;
		params.displayCross = false;
		params.colorScaleUseShader = false;
		m_glWindow->setDisplayParameters(params,true);
		m_glWindow->setPerspectiveState(false,true);
		m_glWindow->setInteractionMode(ccGLWindow::INTERACT_PAN | ccGLWindow::INTERACT_ZOOM_CAMERA | ccGLWindow::INTERACT_CLICKABLE_ITEMS);
		m_glWindow->setPickingMode(ccGLWindow::NO_PICKING);
		m_glWindow->displayOverlayEntities(true);
		m_glWindow->showCursorCoordinates(true);
		
		//add window to the input frame (if any)
		if (parentFrame)
		{
			auto	layout = new QHBoxLayout;
			
			layout->setContentsMargins( 0, 0, 0, 0 );
			layout->addWidget( glWidget) ;

			parentFrame->setLayout( layout );
		}
	}
}

bool bdr2Point5DimEditor::getGridSize(unsigned& gridWidth, unsigned& gridHeight) const
{
	//vertical dimension
	const unsigned char Z = getProjectionDimension();

	//cloud bounding-box --> grid size
	ccBBox box = getCustomBBox();

	//grid step
	double gridStep = getGridStep();

	return ccRasterGrid::ComputeGridSize(Z, box, gridStep, gridWidth, gridHeight);
}

QString bdr2Point5DimEditor::getGridSizeAsString() const
{
	unsigned gridWidth = 0, gridHeight = 0;
	if (!getGridSize(gridWidth, gridHeight))
	{
		return QObject::tr("invalid grid box");
	}

	return QString("%1 x %2").arg(gridWidth).arg(gridHeight);
}

ccBBox bdr2Point5DimEditor::getCustomBBox() const
{
	return (m_bbEditorDlg ? m_bbEditorDlg->getBox() : ccBBox());
}

void bdr2Point5DimEditor::gridIsUpToDate(bool state)
{
}

void bdr2Point5DimEditor::update2DDisplayZoom(ccBBox& box)
{
	if (!m_glWindow /*|| !m_grid.isValid()*/)
		return;

	//equivalent to 'ccGLWindow::updateConstellationCenterAndZoom' but we take aspect ratio into account

	//we compute the pixel size (in world coordinates)
	if (m_grid.isValid())
	{
		ccViewportParameters params = m_glWindow->getViewportParameters();

		double realGridWidth  = m_grid.width  * m_grid.gridStep;
		double realGridHeight = m_grid.height * m_grid.gridStep;

		static const int screnMargin = 20;
		int screenWidth = std::max(1, m_glWindow->glWidth() - 2 * screnMargin);
		int screenHeight = std::max(1, m_glWindow->glHeight() - 2 * screnMargin);

		int pointSize = 1;
		if (	static_cast<int>(m_grid.width)  < screenWidth
			&&	static_cast<int>(m_grid.height) < screenHeight)
		{
			int vPointSize = static_cast<int>(ceil(static_cast<float>(screenWidth) / m_grid.width));
			int hPointSize = static_cast<int>(ceil(static_cast<float>(screenHeight) / m_grid.height));
			pointSize = std::min(vPointSize, hPointSize);

			//if the grid is too small (i.e. necessary point size > 10)
			if (pointSize > 10)
			{
				pointSize = 10;
				screenWidth  = m_grid.width  * pointSize;
				screenHeight = m_grid.height * pointSize;
			}
		}

		params.pixelSize = static_cast<float>(std::max(realGridWidth / screenWidth, realGridHeight / screenHeight));
		params.zoom = 1.0f;

		m_glWindow->setViewportParameters(params);
		m_glWindow->setPointSize(pointSize);
	}
	else {
		ccViewportParameters params = m_glWindow->getViewportParameters();
		static const int screnMargin = 5;
		int screenWidth = std::max(1, m_glWindow->glWidth() - 2 * screnMargin);
		int screenHeight = std::max(1, m_glWindow->glHeight() - 2 * screnMargin);

		params.pixelSize = 1.0f/*static_cast<float>(std::max(realGridWidth / screenWidth, realGridHeight / screenHeight))*/;
		params.zoom = static_cast<float>(std::min((double)screenWidth / (double)m_image->getW(), (double)screenHeight / (double)m_image->getH()));

		m_glWindow->setViewportParameters(params);
		m_glWindow->setPointSize(1.0f);
	}
	
	//we set the pivot point on the box center
	CCVector3 P = box.getCenter();
	m_glWindow->setPivotPoint(CCVector3d::fromArray(P.u));
	m_glWindow->setCameraPos(CCVector3d::fromArray(P.u));

	m_glWindow->invalidateViewport();
	m_glWindow->invalidateVisualization();
	m_glWindow->deprecate3DLayer();
	m_glWindow->redraw();
}

ccPointCloud* bdr2Point5DimEditor::convertGridToCloud(	const std::vector<ccRasterGrid::ExportableFields>& exportedFields,
														bool interpolateSF,
														bool interpolateColors,
														bool resampleInputCloudXY,
														bool resampleInputCloudZ,
														ccGenericPointCloud* inputCloud,
														bool fillEmptyCells,
														double emptyCellsHeight,
														bool exportToOriginalCS) const
{
	//projection dimension
	const unsigned char Z = getProjectionDimension();
	assert(Z <= 2);

	//cloud bounding-box
	ccBBox box = getCustomBBox();
	assert(box.isValid());

	return m_grid.convertToCloud(	exportedFields,
									interpolateSF,
									interpolateColors,
									resampleInputCloudXY,
									resampleInputCloudZ,
									inputCloud,
									Z,
									box,
									fillEmptyCells,
									emptyCellsHeight,
									exportToOriginalCS);
}

void bdr2Point5DimEditor::setImage(QString image_path)
{
	if (m_image) {
		if (m_glWindow)
			m_glWindow->removeFromOwnDB(m_image);
		delete m_image;
		m_image = nullptr;
	}
	m_image = new ccImage;
	m_image->setDisplayType(ccImage::IMAGE_DISPLAY_3D);
	m_image->setDisplay(m_glWindow);
	QString error;
	m_image->load(image_path, error);
	m_glWindow->addToOwnDB(m_image);
	ZoomFit();
}

void bdr2Point5DimEditor::setImageAndCamera(ccCameraSensor * cam)
{
	setImage(cam->imagePath());
	m_image->setAssociatedSensor(cam);
}

void bdr2Point5DimEditor::ZoomFit()
{
	if (!m_image) {
		return;
	}
	ccBBox box;
	box.add(CCVector3(0, 0, 0));
	box.add(CCVector3(m_image->getW(), m_image->getH(), 0));
	update2DDisplayZoom(box);
}

ccRasterGrid::EmptyCellFillOption bdr2Point5DimEditor::getFillEmptyCellsStrategy(QComboBox* comboBox) const
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
		return ccRasterGrid::INTERPOLATE;
	default:
		//shouldn't be possible for this option!
		assert(false);
	}

	return ccRasterGrid::LEAVE_EMPTY;
}
