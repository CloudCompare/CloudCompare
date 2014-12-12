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

#include "ccRasterizeTool.h"

//Local
#include "ccBoundingBoxEditorDlg.h"
#include "ccPersistentSettings.h"

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccProgressDialog.h>

//qCC_gl
#include <ccGLWindow.h>

//Qt
#include <QSettings>
#include <QPushButton>
#include <QMessageBox>

//System
#include <assert.h>
#include <string.h>

ccRasterizeTool::ccRasterizeTool(ccGenericPointCloud* cloud, QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::RasterizeToolDialog()
	, m_bbEditorDlg(0)
	, m_cloud(cloud)
	, m_window(0)
	, m_rasterCloud(0)
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

#ifndef CC_GDAL_SUPPORT
	generateRasterPushButton->setDisabled(true);
	generateRasterPushButton->setChecked(false);
#endif

	connect(buttonBox,					SIGNAL(accepted()),					this,	SLOT(saveSettings()));
	connect(gridStepDoubleSpinBox,		SIGNAL(valueChanged(double)),		this,	SLOT(updateGridInfo()));
	connect(dimensionComboBox,			SIGNAL(currentIndexChanged(int)),	this,	SLOT(projectionDirChanged(int)));
	connect(heightProjectionComboBox,	SIGNAL(currentIndexChanged(int)),	this,	SLOT(projectionTypeChanged(int)));
	connect(fillEmptyCells,				SIGNAL(currentIndexChanged(int)),	this,	SLOT(fillEmptyCellStrategyChanged(int)));
	connect(updateGridPushButton,		SIGNAL(clicked()),					this,	SLOT(updateGridAndDisplay()));
	connect(generateCloudPushButton,	SIGNAL(clicked()),					this,	SLOT(generateCloud(bool)));
	connect(generateImagePushButton,	SIGNAL(clicked()),					this,	SLOT(generateImage(bool)));
	connect(generateRasterPushButton,	SIGNAL(clicked()),					this,	SLOT(generateRaster(bool)));
	connect(generateASCIIPushButton,	SIGNAL(clicked()),					this,	SLOT(generateASCIIMatrix(bool)));

	//custom bbox editor
	ccBBox gridBBox = m_cloud ? m_cloud->getMyOwnBB() : ccBBox(); 
	if (gridBBox.isValid())
	{
		m_bbEditorDlg = new ccBoundingBoxEditorDlg(this);
		m_bbEditorDlg->setBaseBBox(gridBBox,false);
		connect(editGridToolButton, SIGNAL(clicked()), this, SLOT(showGridBoxEditor()));
	}
	else
	{
		editGridToolButton->setEnabled(false);
	}

	if (m_cloud)
	{
		cloudNameLabel->setText(m_cloud->getName());
		pointCountLabel->setText(QString::number(m_cloud->size()));
		interpolateSFFrame->setEnabled(cloud->hasScalarFields());

		//add window
		{
			m_window = new ccGLWindow(this);
			ccGui::ParamStruct params = m_window->getDisplayParameters();
			//black (text) & white (background) display by default
			memcpy(params.backgroundCol,ccColor::white,3*sizeof(unsigned char));
			memcpy(params.textDefaultCol,ccColor::black,3*sizeof(unsigned char));
			params.drawBackgroundGradient = false;
			params.decimateMeshOnMove = false;
			params.displayCross = false;
			params.colorScaleUseShader = false;
			m_window->setDisplayParameters(params,true);
			m_window->setPerspectiveState(false,true);
			m_window->setInteractionMode(ccGLWindow::PAN_ONLY);
			m_window->displayOverlayEntities(false);
			//add window to the right side layout
			mapFrame->setLayout(new QHBoxLayout());
			mapFrame->layout()->addWidget(m_window);
		}
	}

	loadSettings();

	updateGridInfo();

	gridIsUpToDate(false);
}

ccRasterizeTool::~ccRasterizeTool()
{
	if (m_rasterCloud)
	{
		if (m_window)
			m_window->removeFromOwnDB(m_rasterCloud);
		delete m_rasterCloud;
		m_rasterCloud = 0;
	}
}

void ccRasterizeTool::showGridBoxEditor()
{
	if (m_bbEditorDlg)
	{
		unsigned char projDim = getProjectionDimension();
		assert(projDim < 3);
		m_bbEditorDlg->set2DMode(true,projDim);
		if (m_bbEditorDlg->exec())
			updateGridInfo();
	}
}

void ccRasterizeTool::updateGridInfo()
{
	//Vertical dimension
	const unsigned char Z = getProjectionDimension();
	assert(Z >= 0 && Z <= 2);
	const unsigned char X = Z == 2 ? 0 : Z +1;
	const unsigned char Y = X == 2 ? 0 : X +1;

	//cloud bounding-box --> grid size
	ccBBox box = getCustomBBox();
	if (box.isValid())
	{
		double gridStep = getGridStep();
		assert(gridStep != 0);

		CCVector3d boxDiag(	static_cast<double>(box.maxCorner().x) - static_cast<double>(box.minCorner().x),
							static_cast<double>(box.maxCorner().y) - static_cast<double>(box.minCorner().y),
							static_cast<double>(box.maxCorner().z) - static_cast<double>(box.minCorner().z) );

		unsigned gridWidth  = static_cast<unsigned>(ceil(boxDiag.u[X] / gridStep));
		unsigned gridHeight = static_cast<unsigned>(ceil(boxDiag.u[Y] / gridStep));

		gridWidthLabel->setText(QString("%1 x %2").arg(gridWidth).arg(gridHeight));
		gridHeightRangeLabel->setText(QString("%1 (%2 - %3)").arg(boxDiag.u[Z]).arg(box.minCorner().u[Z]).arg(box.maxCorner().u[Z]));
	}
	else
	{
		gridWidthLabel->setText("invalid grid box");
	}
}

ccBBox ccRasterizeTool::getCustomBBox() const
{
	return (m_bbEditorDlg ? m_bbEditorDlg->getBox() : ccBBox());
}

double ccRasterizeTool::getGridStep() const
{
	return gridStepDoubleSpinBox->value();
}

bool ccRasterizeTool::generateCountSF() const
{
	return generateCountSFcheckBox->isChecked();
}

bool ccRasterizeTool::resampleOriginalCloud() const
{
	return resampleOriginalCloudCheckBox->isEnabled() && resampleOriginalCloudCheckBox->isChecked();
}

void ccRasterizeTool::generateCloud() const
{
	//return generateCloudGroupBox->isChecked();
}

void ccRasterizeTool::generateImage() const
{
	//return generateImageCheckBox->isChecked();
}

void ccRasterizeTool::generateRaster() const
{
	//return generateRasterCheckBox->isChecked();
}

void ccRasterizeTool::generateASCIIMatrix() const
{
	//return generateASCIICheckBox->isChecked();
}

unsigned char ccRasterizeTool::getProjectionDimension() const
{
	int dim = dimensionComboBox->currentIndex();
	assert(dim >= 0 && dim < 3);

	return static_cast<unsigned char>(dim);
}

void ccRasterizeTool::projectionTypeChanged(int index)
{
	//we can't use the 'resample origin cloud' option with 'average height' projection
	resampleOriginalCloudCheckBox->setEnabled(index != PROJ_AVERAGE_VALUE);
}

void ccRasterizeTool::projectionDirChanged(int dir)
{
	updateGridInfo();
}

void ccRasterizeTool::fillEmptyCellStrategyChanged(int)
{
	emptyValueDoubleSpinBox->setEnabled(getFillEmptyCellsStrategy() == FILL_CUSTOM_HEIGHT);
}

double ccRasterizeTool::getCustomHeightForEmptyCells() const
{
	return emptyValueDoubleSpinBox->value();
}

ccRasterizeTool::ProjectionType ccRasterizeTool::getTypeOfProjection() const
{
	switch (heightProjectionComboBox->currentIndex())
	{
	case 0:
		return PROJ_MINIMUM_VALUE;
	case 1:
		return PROJ_AVERAGE_VALUE;
	case 2:
		return PROJ_MAXIMUM_VALUE;
	default:
		//shouldn't be possible for this option!
		assert(false);
	}

	return INVALID_PROJECTION_TYPE;
}

ccRasterizeTool::ProjectionType ccRasterizeTool::getTypeOfSFInterpolation() const
{
	if (!interpolateSFFrame->isEnabled() || !interpolateSFCheckBox->isChecked())
		return INVALID_PROJECTION_TYPE; //means that we don't want to keep SF values

	switch (scalarFieldProjection->currentIndex())
	{
	case 0:
		return PROJ_MINIMUM_VALUE;
	case 1:
		return PROJ_AVERAGE_VALUE;
	case 2:
		return PROJ_MAXIMUM_VALUE;
	default:
		//shouldn't be possible for this option!
		assert(false);
	}

	return INVALID_PROJECTION_TYPE;
}

ccRasterizeTool::EmptyCellFillOption ccRasterizeTool::getFillEmptyCellsStrategy() const
{
	switch (fillEmptyCells->currentIndex())
	{
	case 0:
		return LEAVE_EMPTY;
	case 1:
		return FILL_MINIMUM_HEIGHT;
	case 2:
		return FILL_AVERAGE_HEIGHT;
	case 3:
		return FILL_MAXIMUM_HEIGHT;
	case 4:
		return FILL_CUSTOM_HEIGHT;
	case 5:
		return INTERPOLATE;
	default:
		//shouldn't be possible for this option!
		assert(false);
	}

	return LEAVE_EMPTY;
}

void ccRasterizeTool::loadSettings()
{
	QSettings settings;
	settings.beginGroup(ccPS::HeightGridGeneration());
	int projType		= settings.value("ProjectionType",heightProjectionComboBox->currentIndex()).toInt();
	int projDim			= settings.value("ProjectionDim",dimensionComboBox->currentIndex()).toInt();
	bool sfProj			= settings.value("SfProjEnabled",interpolateSFCheckBox->isChecked()).toBool();
	int sfProjStrategy	= settings.value("SfProjStrategy",scalarFieldProjection->currentIndex()).toInt();
	int fillStrategy	= settings.value("FillStrategy",fillEmptyCells->currentIndex()).toInt();
	double step			= settings.value("GridStep",gridStepDoubleSpinBox->value()).toDouble();
	double emptyHeight	= settings.value("EmptyCellsHeight",emptyValueDoubleSpinBox->value()).toDouble();
	bool genCountSF		= settings.value("GenerateCountSF",generateCountSFcheckBox->isChecked()).toBool();
	bool resampleCloud	= settings.value("ResampleOrigCloud",resampleOriginalCloudCheckBox->isChecked()).toBool();
	settings.endGroup();

	gridStepDoubleSpinBox->setValue(step);
	heightProjectionComboBox->setCurrentIndex(projType);
	fillEmptyCells->setCurrentIndex(fillStrategy);
	emptyValueDoubleSpinBox->setValue(emptyHeight);
	dimensionComboBox->setCurrentIndex(projDim);
	interpolateSFCheckBox->setChecked(sfProj);
	scalarFieldProjection->setCurrentIndex(sfProjStrategy);
	generateCountSFcheckBox->setChecked(genCountSF);
	resampleOriginalCloudCheckBox->setChecked(resampleCloud);
}

void ccRasterizeTool::saveSettings()
{
	QSettings settings;
	settings.beginGroup(ccPS::HeightGridGeneration());
	settings.setValue("ProjectionType",heightProjectionComboBox->currentIndex());
	settings.setValue("ProjectionDim",dimensionComboBox->currentIndex());
	settings.setValue("SfProjEnabled",interpolateSFCheckBox->isChecked());
	settings.setValue("SfProjStrategy",scalarFieldProjection->currentIndex());
	settings.setValue("FillStrategy",fillEmptyCells->currentIndex());
	settings.setValue("GridStep",gridStepDoubleSpinBox->value());
	settings.setValue("EmptyCellsHeight",emptyValueDoubleSpinBox->value());
	settings.setValue("GenerateCountSF",generateCountSFcheckBox->isChecked());
	settings.setValue("ResampleOrigCloud",resampleOriginalCloudCheckBox->isChecked());
	settings.endGroup();

	accept();
}

void ccRasterizeTool::update2DDisplayZoom(ccBBox& box)
{
	if (!m_window || !box.isValid())
		return;

	//equivalent to 'ccGLWindow::updateConstellationCenterAndZoom' but we take aspect ratio into account

	//we get the bounding-box diagonal length
	PointCoordinateType bbDiag = box.getDiagNorm();
	if (bbDiag > ZERO_TOLERANCE)
	{
		//we compute the pixel size (in world coordinates)
		{
			ccViewportParameters params = m_window->getViewportParameters();

			int screenWidth = m_window->width();
			float screenHeight = static_cast<float>(m_window->height()) * params.orthoAspectRatio;
			params.pixelSize = static_cast<float>(std::max(	box.getDiagVec().x/static_cast<PointCoordinateType>(screenWidth),
															box.getDiagVec().y/static_cast<PointCoordinateType>(screenHeight) ) );
			params.zoom = 1.0f;

			m_window->setViewportParameters(params);
		}

		//we set the pivot point on the box center
		CCVector3 P = box.getCenter();
		m_window->setPivotPoint(CCVector3d::fromArray(P.u));
		m_window->setCameraPos(CCVector3d::fromArray(P.u));

		m_window->invalidateViewport();
		m_window->invalidateVisualization();
	}

	m_window->redraw();
}

void ccRasterizeTool::gridIsUpToDate(bool state)
{
	if (state)
	{
		//standard button
		updateGridPushButton->setStyleSheet(QString());
	}
	else
	{
		//red button
		updateGridPushButton->setStyleSheet("color: white; background-color:red;");
	}
	updateGridPushButton->setDisabled(state);
}

void ccRasterizeTool::RasterGrid::clear()
{
	//reset
	width = height = 0;

	//properly clean memory
	for (size_t i=0; i<data.size(); ++i)
	{
		if (data[i])
			delete[] data[i];
	}
	data.clear();

	for (size_t j=0; j<scalarFields.size(); ++j)
	{
		if (scalarFields[j])
			delete[] scalarFields[j];
	}
	scalarFields.clear();
}

void ccRasterizeTool::RasterGrid::reset()
{
	//reset values
	for (size_t j=0; j<data.size(); ++j)
	{
		RasterCell* cell = data[j];
		for (unsigned i=0; i<width; ++i, ++cell)
		{
			*cell = RasterCell();
		}
	}

	//not necessary
	//for (size_t j=0; j<scalarFields.size(); ++j)
	//{
	//	if (scalarFields[j])
	//		memset(scalarFields[j],0,sizeof(double)*width*height);
	//}

	minHeight = maxHeight = meanHeight = 0;
	nonEmptyCells = 0;
}

bool ccRasterizeTool::RasterGrid::init(unsigned w, unsigned h)
{
	if (w == width && h == height)
	{
		//simply reset values
		reset();
		return true;
	}

	try
	{
		data.resize(h,0);
		for (unsigned i=0; i<h; ++i)
		{
			data[i] = new RasterCell[w];
			if (!data[i])
			{
				//not enough memory
				clear();
				return false;
			}

		}
	}
	catch(std::bad_alloc)
	{
		//not enough memory
		return false;
	}

	width = w;
	height = h;

	return true;
}

void ccRasterizeTool::updateGridAndDisplay()
{
	bool success = updateGrid();

	gridIsUpToDate(success);

	if (success)
	{
	}

	if (m_rasterCloud)
		update2DDisplayZoom(m_rasterCloud->getMyOwnBB());

}

bool ccRasterizeTool::updateGrid(bool interpolateSF/*=false*/)
{
	if (!m_cloud)
	{
		assert(false);
		return false;
	}

	//main parameters
	ProjectionType projectionType = getTypeOfProjection();
	ProjectionType sfInterpolation = getTypeOfSFInterpolation();
	
	//Vertical dimension
	const unsigned char Z = getProjectionDimension();
	assert(Z >= 0 && Z <= 2);
	const unsigned char X = Z == 2 ? 0 : Z +1;
	const unsigned char Y = X == 2 ? 0 : X +1;

	//cloud bounding-box --> grid size
	ccBBox box = getCustomBBox();
	if (!box.isValid())
		return false;

	double gridStep = getGridStep();
	assert(gridStep != 0);

	CCVector3d boxDiag(	static_cast<double>(box.maxCorner().x) - static_cast<double>(box.minCorner().x),
						static_cast<double>(box.maxCorner().y) - static_cast<double>(box.minCorner().y),
						static_cast<double>(box.maxCorner().z) - static_cast<double>(box.minCorner().z) );

	if (boxDiag.u[X] <= 0 || boxDiag.u[Y] <= 0)
	{
		ccLog::Error("Invalid cloud bounding box!");
		return false;
	}

	unsigned gridWidth  = static_cast<unsigned>(ceil(boxDiag.u[X] / gridStep));
	unsigned gridHeight = static_cast<unsigned>(ceil(boxDiag.u[Y] / gridStep));

	//grid size
	unsigned gridTotalSize = gridWidth * gridHeight;
	if (gridTotalSize == 1)
	{
		if (QMessageBox::question(0,"Unexpected grid size","The generated grid will only have 1 cell! Do you want to proceed anyway?",QMessageBox::Yes,QMessageBox::No) == QMessageBox::No)
			return false;
	}

	//memory allocation
	if (!m_grid.init(gridWidth,gridHeight))
	{
		//not enough memory
		ccLog::Error("Not enough memory");
		return false;
	}

	//do we need to interpolate scalar fields?
	ccPointCloud* pc = (m_cloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(m_cloud) : 0);
	interpolateSF &= (sfInterpolation != INVALID_PROJECTION_TYPE) && (pc && pc->hasScalarFields());
	if (interpolateSF)
	{
		unsigned sfCount = pc->getNumberOfScalarFields();
		try
		{
			m_grid.scalarFields.resize(sfCount,0);
		}
		catch(std::bad_alloc)
		{
			//not enough memory
			m_grid.clear();
			return false;
		}
		for (unsigned i=0; i<sfCount; ++i)
		{
			m_grid.scalarFields[i] = new double[gridTotalSize];
			if (!m_grid.scalarFields[i])
			{
				ccLog::Warning(QString("Failed to allocate memory for SF '%1' (and potentially the next ones)!").arg(pc->getScalarField(i)->getName()));
				break;
			}
		}
	}

	//filling the grid
	unsigned pointCount = m_cloud->size();

	CCVector3d minGridCorner = CCVector3d::fromArray(box.minCorner().u);
	double gridMaxX = gridStep * m_grid.width;
	double gridMaxY = gridStep * m_grid.height;

	ccProgressDialog pDlg(true,this);
	pDlg.setMethodTitle("Grid generation");
	pDlg.setInfo(qPrintable(QString("Points: %1\nCells: %2 x %3").arg(pointCount).arg(m_grid.width).arg(m_grid.height)));
	pDlg.start();
	pDlg.show();
	QApplication::processEvents();
	CCLib::NormalizedProgress nProgress(&pDlg,pointCount);

	for (unsigned n=0; n<pointCount; ++n)
	{
		const CCVector3* P = m_cloud->getPoint(n);

		CCVector3d relativePos = CCVector3d::fromArray(P->u) - minGridCorner;
		int i = static_cast<int>(relativePos.u[X]/gridStep);
		int j = static_cast<int>(relativePos.u[Y]/gridStep);

		//specific case: if we fall exactly on the max corner of the grid box
		if (i == static_cast<int>(m_grid.width) && relativePos.u[X] == gridMaxX)
			--i;
		if (j == static_cast<int>(m_grid.height) && relativePos.u[Y] == gridMaxY)
			--j;

		//we skip points outside the box!
		if (	i < 0 || i >= static_cast<int>(m_grid.width)
			||	j < 0 || j >= static_cast<int>(m_grid.height) )
			continue;

		assert(i >= 0 && j >= 0);

		RasterCell* aCell = m_grid.data[j]+i;
		unsigned& pointsInCell = aCell->nbPoints;
		if (pointsInCell)
		{
			switch (projectionType)
			{
			case PROJ_MINIMUM_VALUE:
				// Set the minimum height
				if (P->u[Z] < aCell->height)
				{
					aCell->height = P->u[Z];
					aCell->pointIndex = n;
				}
				break;
			case PROJ_MAXIMUM_VALUE:
				// Set the maximum height
				if (P->u[Z] > aCell->height)
				{
					aCell->height = P->u[Z];
					aCell->pointIndex = n;
				}
				break;
			case PROJ_AVERAGE_VALUE:
				// Sum the points heights
				aCell->height += P->u[Z];
				break;
			default:
				assert(false);
				break;
			}
		}
		else
		{
			//for the first point, we simply have to store its height (in any case)
			aCell->height = P->u[Z];
			aCell->pointIndex = n;
		}

		//scalar fields
		if (interpolateSF)
		{
			assert(sfInterpolation != INVALID_PROJECTION_TYPE);
			int pos = j*static_cast<int>(m_grid.width)+i; //pos in 2D SF grid(s)
			assert(pos < static_cast<int>(gridTotalSize));
			for (size_t k=0; k<m_grid.scalarFields.size(); ++k)
			{
				if (m_grid.scalarFields[k])
				{
					CCLib::ScalarField* sf = pc->getScalarField(static_cast<unsigned>(k));
					assert(sf);
					ScalarType sfValue = sf->getValue(n);
					ScalarType formerValue = static_cast<ScalarType>(m_grid.scalarFields[k][pos]);

					if (pointsInCell && ccScalarField::ValidValue(formerValue))
					{
						if (ccScalarField::ValidValue(sfValue))
						{
							switch (sfInterpolation)
							{
							case PROJ_MINIMUM_VALUE:
								// keep the minimum value
								m_grid.scalarFields[k][pos] = std::min<double>(formerValue,sfValue);
								break;
							case PROJ_AVERAGE_VALUE:
								//we sum all values (we will divide them later)
								m_grid.scalarFields[k][pos] += sfValue;
								break;
							case PROJ_MAXIMUM_VALUE:
								// keep the maximum value
								m_grid.scalarFields[k][pos] = std::max<double>(formerValue,sfValue);
								break;
							default:
								break;
							}
						}
					}
					else
					{
						//for the first (vaild) point, we simply have to store its SF value (in any case)
						m_grid.scalarFields[k][pos] = sfValue;
					}
				}
			}
		}

		pointsInCell++;

		if (!nProgress.oneStep())
		{
			//process cancelled by user
			m_grid.clear();
			return false;
		}
	}

	//update grids for 'average' cases
	if (sfInterpolation == PROJ_AVERAGE_VALUE)
	{
		for (size_t k=0; k<m_grid.scalarFields.size(); ++k)
		{
			if (m_grid.scalarFields[k])
			{
				double* _gridSF = m_grid.scalarFields[k];
				for (unsigned j=0;j<m_grid.height;++j)
				{
					RasterCell* cell = m_grid.data[j];
					for (unsigned i=0;i<m_grid.width;++i,++cell,++_gridSF)
					{
						if (cell->nbPoints)
						{
							ScalarType s = static_cast<ScalarType>(*_gridSF);
							if (ccScalarField::ValidValue(s)) //valid SF value
								*_gridSF /= static_cast<double>(cell->nbPoints);
						}
					}
				}
			}
		}
	}

	//we need to finish the average height computation
	if (projectionType == PROJ_AVERAGE_VALUE)
	{
		for (unsigned j=0; j<m_grid.height; ++j)
		{
			RasterCell* cell = m_grid.data[j];
			for (unsigned i=0; i<m_grid.width; ++i,++cell)
				if (cell->nbPoints > 1)
					cell->height /= static_cast<PointCoordinateType>(cell->nbPoints);
		}
	}

	//Computation of the average and extreme height values in the grid
	m_grid.minHeight = 0;
	m_grid.maxHeight = 0;
	m_grid.meanHeight = 0;
	m_grid.nonEmptyCells = 0; //non empty cells count
	{
		for (unsigned i=0; i<m_grid.height; ++i)
		{
			for (unsigned j=0; j<m_grid.width; ++j)
			{
				if (m_grid.data[i][j].nbPoints) //non empty cell
				{
					double h = m_grid.data[i][j].height;

					if (m_grid.nonEmptyCells++)
					{
						if (h < m_grid.minHeight)
							m_grid.minHeight = h;
						else if (h > m_grid.maxHeight)
							m_grid.maxHeight = h;
						m_grid.meanHeight += h;
					}
					else
					{
						m_grid.meanHeight = m_grid.minHeight = m_grid.maxHeight = h;
					}
				}
			}
		}
	}

	return true;
}

