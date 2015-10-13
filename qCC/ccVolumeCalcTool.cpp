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

#include "ccVolumeCalcTool.h"

//Local
#include "ccBoundingBoxEditorDlg.h"
#include "ccPersistentSettings.h"
#include "ccCommon.h"
#include "mainwindow.h"
#include "ccIsolines.h"

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccProgressDialog.h>
#include <ccPolyline.h>
#include <ccMesh.h>

//qCC_gl
#include <ccGLWindow.h>

//CCLib
#include <Delaunay2dMesh.h>
#include <PointProjectionTools.h>

//Qt
#include <QSettings>
#include <QPushButton>
#include <QMessageBox>
#include <QImageWriter>
#include <QFileDialog>
#include <QMap>
#include <QComboBox>

//System
#include <assert.h>

ccVolumeCalcTool::ccVolumeCalcTool(ccGenericPointCloud* cloud1, ccGenericPointCloud* cloud2, QWidget* parent/*=0*/)
	: QDialog(parent)
	, cc2Point5DimEditor()
	, Ui::VolumeCalcDialog()
	, m_cloud1(cloud1)
	, m_cloud2(cloud2)
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	connect(buttonBox,						SIGNAL(accepted()),					this,	SLOT(saveSettingsAndAccept()));
	connect(buttonBox,						SIGNAL(rejected()),					this,	SLOT(reject()));
	connect(gridStepDoubleSpinBox,			SIGNAL(valueChanged(double)),		this,	SLOT(updateGridInfo()));
	connect(gridStepDoubleSpinBox,			SIGNAL(valueChanged(double)),		this,	SLOT(gridOptionChanged()));
	connect(groundEmptyValueDoubleSpinBox,	SIGNAL(valueChanged(double)),		this,	SLOT(gridOptionChanged()));
	connect(ceilEmptyValueDoubleSpinBox,	SIGNAL(valueChanged(double)),		this,	SLOT(gridOptionChanged()));
	connect(projDimComboBox,				SIGNAL(currentIndexChanged(int)),	this,	SLOT(projectionDirChanged(int)));
	connect(updatePushButton,				SIGNAL(clicked()),					this,	SLOT(updateGridAndDisplay()));
	connect(heightProjectionComboBox,		SIGNAL(currentIndexChanged(int)),	this,	SLOT(gridOptionChanged()));
	connect(fillGroundEmptyCellsComboBox,	SIGNAL(currentIndexChanged(int)),	this,	SLOT(groundFillEmptyCellStrategyChanged(int)));
	connect(fillCeilEmptyCellsComboBox,		SIGNAL(currentIndexChanged(int)),	this,	SLOT(ceilFillEmptyCellStrategyChanged(int)));
	connect(swapToolButton,					SIGNAL(clicked()),					this,	SLOT(swapRoles()));
	connect(groundComboBox,					SIGNAL(currentIndexChanged(int)),	this,	SLOT(groundSourceChanged(int)));
	connect(ceilComboBox,					SIGNAL(currentIndexChanged(int)),	this,	SLOT(ceilSourceChanged(int)));

	if (m_cloud1 && !m_cloud2)
	{
		//the existing cloud is always the second by default
		std::swap(m_cloud1, m_cloud2);
	}
	assert(m_cloud2);


	//custom bbox editor
	ccBBox gridBBox = m_cloud1 ? m_cloud1->getOwnBB() : ccBBox(); 
	if (m_cloud2)
	{
		gridBBox += m_cloud2->getOwnBB();
	}
	if (gridBBox.isValid())
	{
		createBoundingBoxEditor(gridBBox, this);
		connect(editGridToolButton, SIGNAL(clicked()), this, SLOT(showGridBoxEditor()));
	}
	else
	{
		editGridToolButton->setEnabled(false);
	}

	groundComboBox->addItem("Constant");
	ceilComboBox->addItem("Constant");
	if (m_cloud1)
	{
		groundComboBox->addItem(m_cloud1->getName());
		ceilComboBox->addItem(m_cloud1->getName());
	}
	if (m_cloud2)
	{
		groundComboBox->addItem(m_cloud2->getName());
		ceilComboBox->addItem(m_cloud2->getName());
	}
	assert(groundComboBox->count() >= 2);
	groundComboBox->setCurrentIndex(groundComboBox->count()-2);
	ceilComboBox->setCurrentIndex(ceilComboBox->count()-1);

	//add window
	create2DView(mapFrame);

	loadSettings();

	updateGridInfo();

	gridIsUpToDate(false);
}

ccVolumeCalcTool::~ccVolumeCalcTool()
{
}

void ccVolumeCalcTool::groundSourceChanged(int)
{
	fillGroundEmptyCellsComboBox->setEnabled(groundComboBox->currentIndex() > 0);
	groundFillEmptyCellStrategyChanged(-1);
}

void ccVolumeCalcTool::ceilSourceChanged(int)
{
	fillCeilEmptyCellsComboBox->setEnabled(ceilComboBox->currentIndex() > 0);
	ceilFillEmptyCellStrategyChanged(-1);
}

void ccVolumeCalcTool::swapRoles()
{
	int sourceIndex = ceilComboBox->currentIndex();
	int emptyCellStrat = fillCeilEmptyCellsComboBox->currentIndex();
	double emptyCellValue = ceilEmptyValueDoubleSpinBox->value();

	ceilComboBox->setCurrentIndex(groundComboBox->currentIndex());
	fillCeilEmptyCellsComboBox->setCurrentIndex(fillGroundEmptyCellsComboBox->currentIndex());
	ceilEmptyValueDoubleSpinBox->setValue(groundEmptyValueDoubleSpinBox->value());
	
	groundComboBox->setCurrentIndex(sourceIndex);
	fillGroundEmptyCellsComboBox->setCurrentIndex(emptyCellStrat);
	groundEmptyValueDoubleSpinBox->setValue(emptyCellValue);
	
	gridIsUpToDate(false);
}

bool ccVolumeCalcTool::showGridBoxEditor()
{
	if (cc2Point5DimEditor::showGridBoxEditor())
	{
		updateGridInfo();
		return true;
	}

	return false;
}

void ccVolumeCalcTool::updateGridInfo()
{
	gridWidthLabel->setText(getGridSizeAsString());
}

double ccVolumeCalcTool::getGridStep() const
{
	return gridStepDoubleSpinBox->value();
}

unsigned char ccVolumeCalcTool::getProjectionDimension() const
{
	int dim = projDimComboBox->currentIndex();
	assert(dim >= 0 && dim < 3);

	return static_cast<unsigned char>(dim);
}

void ccVolumeCalcTool::sfProjectionTypeChanged(int index)
{
	gridIsUpToDate(false);
}

void ccVolumeCalcTool::projectionDirChanged(int dir)
{
	updateGridInfo();
	gridIsUpToDate(false);
}

void ccVolumeCalcTool::groundFillEmptyCellStrategyChanged(int)
{
	EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(fillGroundEmptyCellsComboBox);

	groundEmptyValueDoubleSpinBox->setEnabled(	groundComboBox->currentIndex() == 0
											||	fillEmptyCellsStrategy == FILL_CUSTOM_HEIGHT
											||	fillEmptyCellsStrategy == INTERPOLATE );
	gridIsUpToDate(false);
}

void ccVolumeCalcTool::ceilFillEmptyCellStrategyChanged(int)
{
	EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(fillCeilEmptyCellsComboBox);

	ceilEmptyValueDoubleSpinBox->setEnabled(	ceilComboBox->currentIndex() == 0
											||	fillEmptyCellsStrategy == FILL_CUSTOM_HEIGHT
											||	fillEmptyCellsStrategy == INTERPOLATE );
	gridIsUpToDate(false);
}

void ccVolumeCalcTool::gridOptionChanged()
{
	gridIsUpToDate(false);
}

ccVolumeCalcTool::ProjectionType ccVolumeCalcTool::getTypeOfProjection() const
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

void ccVolumeCalcTool::loadSettings()
{
	QSettings settings;
	settings.beginGroup(ccPS::VolumeCalculation());
	int projType				= settings.value("ProjectionType",heightProjectionComboBox->currentIndex()).toInt();
	int projDim					= settings.value("ProjectionDim",projDimComboBox->currentIndex()).toInt();
	int groundFillStrategy		= settings.value("gFillStrategy",fillGroundEmptyCellsComboBox->currentIndex()).toInt();
	int ceilFillStrategy		= settings.value("cFillStrategy",fillCeilEmptyCellsComboBox->currentIndex()).toInt();
	double step					= settings.value("GridStep",gridStepDoubleSpinBox->value()).toDouble();
	double groundEmptyHeight	= settings.value("gEmptyCellsHeight",groundEmptyValueDoubleSpinBox->value()).toDouble();
	double ceilEmptyHeight		= settings.value("cEmptyCellsHeight",ceilEmptyValueDoubleSpinBox->value()).toDouble();
	settings.endGroup();

	gridStepDoubleSpinBox->setValue(step);
	heightProjectionComboBox->setCurrentIndex(projType);
	fillGroundEmptyCellsComboBox->setCurrentIndex(groundFillStrategy);
	fillCeilEmptyCellsComboBox->setCurrentIndex(ceilFillStrategy);
	groundEmptyValueDoubleSpinBox->setValue(groundEmptyHeight);
	ceilEmptyValueDoubleSpinBox->setValue(ceilEmptyHeight);
	projDimComboBox->setCurrentIndex(projDim);
}

void ccVolumeCalcTool::saveSettingsAndAccept()
{
	saveSettings();
	accept();
}

void ccVolumeCalcTool::saveSettings()
{
	QSettings settings;
	settings.beginGroup(ccPS::VolumeCalculation());
	settings.setValue("ProjectionType",heightProjectionComboBox->currentIndex());
	settings.setValue("ProjectionDim",projDimComboBox->currentIndex());
	settings.setValue("gFillStrategy",fillGroundEmptyCellsComboBox->currentIndex());
	settings.setValue("cFillStrategy",fillCeilEmptyCellsComboBox->currentIndex());
	settings.setValue("GridStep",gridStepDoubleSpinBox->value());
	settings.setValue("gEmptyCellsHeight",groundEmptyValueDoubleSpinBox->value());
	settings.setValue("cEmptyCellsHeight",ceilEmptyValueDoubleSpinBox->value());
	settings.endGroup();
}

void ccVolumeCalcTool::gridIsUpToDate(bool state)
{
	if (state)
	{
		//standard button
		updatePushButton->setStyleSheet(QString());
	}
	else
	{
		//red button
		updatePushButton->setStyleSheet("color: white; background-color:red;");
	}
	updatePushButton->setDisabled(state);

	tabWidget->setEnabled(state);
}

void ccVolumeCalcTool::updateGridAndDisplay()
{
	bool success = updateGrid();
	if (success && m_window)
	{
		//convert grid to point cloud
		if (m_rasterCloud)
		{
			m_window->removeFromOwnDB(m_rasterCloud);
			delete m_rasterCloud;
			m_rasterCloud = 0;
		}

		std::vector<ExportableFields> exportedFields;
		try
		{
			//we only compute the default 'height' layer
			exportedFields.push_back(PER_CELL_HEIGHT);
			m_rasterCloud = cc2Point5DimEditor::convertGridToCloud(	exportedFields,
																	false,
																	false,
																	0,
																	false,
																	std::numeric_limits<double>::quiet_NaN());

			if (m_rasterCloud && m_rasterCloud->hasScalarFields())
			{
				m_rasterCloud->showSF(true);
				m_rasterCloud->setCurrentDisplayedScalarField(0);
			}
		}
		catch (const std::bad_alloc&)
		{
			//see below
		}

		if (m_rasterCloud)
		{
			m_window->addToOwnDB(m_rasterCloud);
			ccBBox box = m_rasterCloud->getDisplayBB_recursive(false,m_window);
			update2DDisplayZoom(box);
		}
		else
		{
			ccLog::Error("Not enough memory!");
			m_window->redraw();
		}
	}

	gridIsUpToDate(success);
}

void ccVolumeCalcTool::outputReport(float coveragePercentage, double volume)
{
	QStringList reportText;
	reportText << QString("Volume: %1").arg(volume);
	reportText << QString("Coverage: %1%").arg(coveragePercentage);
	reportPlainTextEdit->setPlainText(reportText.join("\n"));
}

bool ccVolumeCalcTool::updateGrid()
{
	if (!m_cloud2)
	{
		assert(false);
		return false;
	}

	//per-cell Z computation
	ProjectionType projectionType = getTypeOfProjection();

	//vertical dimension
	const unsigned char Z = getProjectionDimension();
	assert(Z >= 0 && Z <= 2);
	const unsigned char X = Z == 2 ? 0 : Z +1;
	const unsigned char Y = X == 2 ? 0 : X +1;

	//cloud bounding-box --> grid size
	ccBBox box = getCustomBBox();
	if (!box.isValid())
	{
		return false;
	}

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
	else if (gridTotalSize > 10000000)
	{
		if (QMessageBox::question(0,"Big grid size","The generated grid will have more than 10.000.000 cells! Do you want to proceed anyway?",QMessageBox::Yes,QMessageBox::No) == QMessageBox::No)
			return false;
	}

	//memory allocation
	if (!m_grid.init(gridWidth,gridHeight))
	{
		//not enough memory
		ccLog::Error("Not enough memory");
		return false;
	}
	m_grid.gridStep = gridStep;
	m_grid.minCorner = CCVector3d::fromArray(box.minCorner().u);

	//ground
	ccGenericPointCloud* groundCloud = 0;
	double groundHeight = 0;
	switch (groundComboBox->currentIndex())
	{
	case 0:
		groundHeight =	groundEmptyValueDoubleSpinBox->value();
		break;
	case 1:
		groundCloud = m_cloud1 ? m_cloud1 : m_cloud2;
		break;
	case 2:
		groundCloud = m_cloud2;
		break;
	default:
		assert(false);
		return false;
	}

	ccProgressDialog pDlg(true,this);

	RasterGrid groundRaster;
	if (groundCloud)
	{
		if (!groundRaster.init(gridWidth,gridHeight))
		{
			//not enough memory
			ccLog::Error("Not enough memory");
			return false;
		}
		groundRaster.gridStep = m_grid.gridStep;
		groundRaster.minCorner = m_grid.minCorner;

		if (groundRaster.fillWith(	groundCloud,
									Z,
									projectionType,
									getFillEmptyCellsStrategy(fillGroundEmptyCellsComboBox) == INTERPOLATE,
									INVALID_PROJECTION_TYPE,
									&pDlg))
		{
			groundRaster.fillEmptyGridCells(getFillEmptyCellsStrategy(fillGroundEmptyCellsComboBox), groundEmptyValueDoubleSpinBox->value());
			ccLog::Print(QString("[Volume] Ground raster grid: size: %1 x %2 / heights: [%3 ; %4]").arg(m_grid.width).arg(m_grid.height).arg(m_grid.minHeight).arg(m_grid.maxHeight));
		}
		else
		{
			return false;
		}
	}

	//ceil
	ccGenericPointCloud* ceilCloud = 0;
	double ceilHeight = 0;
	switch (ceilComboBox->currentIndex())
	{
	case 0:
		ceilHeight = ceilEmptyValueDoubleSpinBox->value();
		break;
	case 1:
		ceilCloud = m_cloud1 ? m_cloud1 : m_cloud2;
		break;
	case 2:
		ceilCloud = m_cloud2;
		break;
	default:
		assert(false);
		return false;
	}

	RasterGrid ceilRaster;
	if (ceilCloud)
	{
		if (!ceilRaster.init(gridWidth,gridHeight))
		{
			//not enough memory
			ccLog::Error("Not enough memory");
			return false;
		}
		ceilRaster.gridStep = m_grid.gridStep;
		ceilRaster.minCorner = m_grid.minCorner;

		if (ceilRaster.fillWith(ceilCloud,
								Z,
								projectionType,
								getFillEmptyCellsStrategy(fillCeilEmptyCellsComboBox) == INTERPOLATE,
								INVALID_PROJECTION_TYPE,
								&pDlg))
		{
			ceilRaster.fillEmptyGridCells(getFillEmptyCellsStrategy(fillCeilEmptyCellsComboBox), ceilEmptyValueDoubleSpinBox->value());
			ccLog::Print(QString("[Volume] Ceil raster grid: size: %1 x %2 / heights: [%3 ; %4]").arg(m_grid.width).arg(m_grid.height).arg(m_grid.minHeight).arg(m_grid.maxHeight));
		}
		else
		{
			return false;
		}
	}

	//update grid and compute volume
	{
		pDlg.setMethodTitle("Volume computation");
		pDlg.setInfo(qPrintable(QString("Cells: %1 x %2").arg(m_grid.width).arg(m_grid.height)));
		pDlg.start();
		pDlg.show();
		QApplication::processEvents();
		CCLib::NormalizedProgress nProgress(&pDlg, m_grid.width*m_grid.height);
		
		double volume = 0;

		//at least one of the grid is based on a cloud
		m_grid.nonEmptyCellCount = 0;
		for (unsigned i=0; i<m_grid.height; ++i)
		{
			for (unsigned j=0; j<m_grid.width; ++j)
			{
				RasterCell& cell = m_grid.data[i][j];

				bool validGround = true;
				cell.minHeight = groundHeight;
				if (groundCloud)
				{
					cell.minHeight = groundRaster.data[i][j].h;
					validGround = (cell.minHeight == cell.minHeight);
				}

				bool validCeil = true;
				cell.maxHeight = ceilHeight;
				if (ceilCloud)
				{
					cell.maxHeight = ceilRaster.data[i][j].h;
					validCeil = (cell.maxHeight == cell.maxHeight);
				}

				if (validGround && validCeil)
				{
					cell.h = cell.maxHeight - cell.minHeight;
					cell.nbPoints = 1;

					volume += cell.h;
					++m_grid.nonEmptyCellCount;
				}
				else
				{
					cell.h = std::numeric_limits<double>::quiet_NaN();
					cell.nbPoints = 0;
				}

				cell.avgHeight = (groundHeight + ceilHeight)/2;
				cell.stdDevHeight = 0;

				if (!nProgress.oneStep())
				{
					ccLog::Warning("[Volume] Process cancelled by the user");
					return false;
				}
			}
		}
		m_grid.validCellCount = m_grid.nonEmptyCellCount;

		float coverage = static_cast<float>(m_grid.validCellCount * 100) / (m_grid.height * m_grid.width);
		volume *= (m_grid.gridStep * m_grid.gridStep);

		outputReport(coverage, volume);
	}

	m_grid.setValid(true);

	return true;
}
