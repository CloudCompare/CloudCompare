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

//Qt
#include <QSettings>
#include <QPushButton>
#include <QMessageBox>
#include <QImageWriter>
#include <QFileDialog>
#include <QMap>

//System
#include <assert.h>

ccRasterizeTool::ccRasterizeTool(ccGenericPointCloud* cloud, QWidget* parent/*=0*/)
	: QDialog(parent)
	, cc2Point5DimEditor()
	, Ui::RasterizeToolDialog()
	, m_cloud(cloud)
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

#ifndef CC_GDAL_SUPPORT
	generateRasterPushButton->setDisabled(true);
	generateRasterPushButton->setChecked(false);
#endif

	connect(buttonBox,					SIGNAL(accepted()),					this,	SLOT(testAndAccept()));
	connect(buttonBox,					SIGNAL(rejected()),					this,	SLOT(testAndReject()));
	connect(gridStepDoubleSpinBox,		SIGNAL(valueChanged(double)),		this,	SLOT(updateGridInfo()));
	connect(gridStepDoubleSpinBox,		SIGNAL(valueChanged(double)),		this,	SLOT(gridOptionChanged()));
	connect(emptyValueDoubleSpinBox,	SIGNAL(valueChanged(double)),		this,	SLOT(gridOptionChanged()));
	connect(resampleCloudCheckBox,		SIGNAL(toggled(bool)),				this,	SLOT(gridOptionChanged()));
	connect(dimensionComboBox,			SIGNAL(currentIndexChanged(int)),	this,	SLOT(projectionDirChanged(int)));
	connect(heightProjectionComboBox,	SIGNAL(currentIndexChanged(int)),	this,	SLOT(projectionTypeChanged(int)));
	connect(scalarFieldProjection,		SIGNAL(currentIndexChanged(int)),	this,	SLOT(sfProjectionTypeChanged(int)));
	connect(fillEmptyCellsComboBox,		SIGNAL(currentIndexChanged(int)),	this,	SLOT(fillEmptyCellStrategyChanged(int)));
	connect(updateGridPushButton,		SIGNAL(clicked()),					this,	SLOT(updateGridAndDisplay()));
	connect(generateCloudPushButton,	SIGNAL(clicked()),					this,	SLOT(generateCloud()));
	connect(generateImagePushButton,	SIGNAL(clicked()),					this,	SLOT(generateImage()));
	connect(generateRasterPushButton,	SIGNAL(clicked()),					this,	SLOT(generateRaster()));
	connect(generateASCIIPushButton,	SIGNAL(clicked()),					this,	SLOT(generateASCIIMatrix()));
	connect(generateMeshPushButton,		SIGNAL(clicked()),					this,	SLOT(generateMesh()));
	connect(generateContoursPushButton,	SIGNAL(clicked()),					this,	SLOT(generateContours()));
	connect(exportContoursPushButton,	SIGNAL(clicked()),					this,	SLOT(exportContourLines()));
	connect(clearContoursPushButton,	SIGNAL(clicked()),					this,	SLOT(removeContourLines()));
	connect(activeLayerComboBox,		SIGNAL(currentIndexChanged(int)),	this,	SLOT(activeLayerChanged(int)));

	//custom bbox editor
	ccBBox gridBBox = m_cloud ? m_cloud->getOwnBB() : ccBBox(); 
	if (gridBBox.isValid())
	{
		createBoundingBoxEditor(gridBBox, this);
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

		//populate layer box
		activeLayerComboBox->addItem(GetDefaultFieldName(PER_CELL_HEIGHT));
		if (cloud->isA(CC_TYPES::POINT_CLOUD) && cloud->hasScalarFields())
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
			for (unsigned i=0; i<pc->getNumberOfScalarFields(); ++i)
				activeLayerComboBox->addItem(pc->getScalarField(i)->getName());
		}
		else
		{
			activeLayerComboBox->setEnabled(false);
		}

		//add window
		create2DView(mapFrame);
	}

	loadSettings();

	updateGridInfo();

	gridIsUpToDate(false);
}

ccRasterizeTool::~ccRasterizeTool()
{
	removeContourLines();
}

void ccRasterizeTool::removeContourLines()
{
	while (!m_contourLines.empty())
	{
		ccPolyline* poly = m_contourLines.back();
		if (m_window)
			m_window->removeFromOwnDB(poly);
		delete poly;
		m_contourLines.pop_back();
	}

	exportContoursPushButton->setEnabled(false);
	clearContoursPushButton->setEnabled(false);

	if (m_window)
		m_window->redraw();
}

bool ccRasterizeTool::showGridBoxEditor()
{
	if (cc2Point5DimEditor::showGridBoxEditor())
	{
		updateGridInfo();
		return true;
	}

	return false;
}

void ccRasterizeTool::updateGridInfo()
{
	gridWidthLabel->setText(getGridSizeAsString());
}

double ccRasterizeTool::getGridStep() const
{
	return gridStepDoubleSpinBox->value();
}

bool ccRasterizeTool::exportAsSF(ExportableFields field) const
{
	switch (field)
	{
	case PER_CELL_COUNT:
		return generateCountSFcheckBox->isChecked();
	case PER_CELL_MIN_HEIGHT:
		return generateMinHeightSFcheckBox->isChecked();
	case PER_CELL_MAX_HEIGHT:
		return generateMaxHeightSFcheckBox->isChecked();
	case PER_CELL_AVG_HEIGHT:
		return generateAvgHeightSFcheckBox->isChecked();
	case PER_CELL_HEIGHT_STD_DEV:
		return generateStdDevHeightSFcheckBox->isChecked();
	case PER_CELL_HEIGHT_RANGE:
		return generateHeightRangeSFcheckBox->isChecked();
	default:
		assert(false);
	};
	
	return false;
}

bool ccRasterizeTool::resampleOriginalCloud() const
{
	return resampleCloudCheckBox->isEnabled() && resampleCloudCheckBox->isChecked();
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
	resampleCloudCheckBox->setEnabled(index != PROJ_AVERAGE_VALUE);
	gridIsUpToDate(false);
}

void ccRasterizeTool::sfProjectionTypeChanged(int index)
{
	gridIsUpToDate(false);
}

void ccRasterizeTool::projectionDirChanged(int dir)
{
	updateGridInfo();
	gridIsUpToDate(false);
}

void ccRasterizeTool::activeLayerChanged(int layerIndex, bool autoRedraw/*=true*/)
{
	if (layerIndex != 0)
	{
		interpolateSFCheckBox->setChecked(true);
		interpolateSFCheckBox->setEnabled(false);
	}
	else
	{
		interpolateSFCheckBox->setEnabled(true);
	}

	if (m_rasterCloud)
	{
		//does the selected 'layer' exist?
		int sfIndex = m_rasterCloud->getScalarFieldIndexByName(qPrintable(activeLayerComboBox->itemText(layerIndex)));
		m_rasterCloud->setCurrentDisplayedScalarField(sfIndex);

		if (sfIndex >= 0)
		{
			ccScalarField* activeLayer = m_rasterCloud->getCurrentDisplayedScalarField();
			if (activeLayer)
			{
				const ccScalarField::Range& layerValues = activeLayer->displayRange();
				gridLayerRangeLabel->setText(QString("%1 (%2 - %3)").arg(layerValues.range()).arg(layerValues.min()).arg(layerValues.max()));
				contourStartDoubleSpinBox->setValue(layerValues.min());
				contourStepDoubleSpinBox->setValue(layerValues.range() / 10.0);
			}
			else
			{
				assert(false);
				gridLayerRangeLabel->setText("no active layer?!");
			}
		}
		else
		{
			gridIsUpToDate(false);
		}

		if (m_window && autoRedraw)
			m_window->redraw();
	}
}

void ccRasterizeTool::fillEmptyCellStrategyChanged(int)
{
	EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(fillEmptyCellsComboBox);

	emptyValueDoubleSpinBox->setEnabled(	fillEmptyCellsStrategy == FILL_CUSTOM_HEIGHT
										||	fillEmptyCellsStrategy == INTERPOLATE );
	gridIsUpToDate(false);
}

void ccRasterizeTool::gridOptionChanged()
{
	gridIsUpToDate(false);
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

void ccRasterizeTool::loadSettings()
{
	QSettings settings;
	settings.beginGroup(ccPS::HeightGridGeneration());
	int projType				= settings.value("ProjectionType",heightProjectionComboBox->currentIndex()).toInt();
	int projDim					= settings.value("ProjectionDim",dimensionComboBox->currentIndex()).toInt();
	bool sfProj					= settings.value("SfProjEnabled",interpolateSFCheckBox->isChecked()).toBool();
	int sfProjStrategy			= settings.value("SfProjStrategy",scalarFieldProjection->currentIndex()).toInt();
	int fillStrategy			= settings.value("FillStrategy",fillEmptyCellsComboBox->currentIndex()).toInt();
	double step					= settings.value("GridStep",gridStepDoubleSpinBox->value()).toDouble();
	double emptyHeight			= settings.value("EmptyCellsHeight",emptyValueDoubleSpinBox->value()).toDouble();
	bool genCountSF				= settings.value("GenerateCountSF",generateCountSFcheckBox->isChecked()).toBool();
	bool resampleCloud			= settings.value("ResampleOrigCloud",resampleCloudCheckBox->isChecked()).toBool();
	int minVertexCount			= settings.value("MinVertexCount",minVertexCountSpinBox->value()).toInt();
	bool ignoreBorders			= settings.value("IgnoreBorders",ignoreContourBordersCheckBox->isChecked()).toBool();
	bool generateCountSF		= settings.value("generateCountSF",generateCountSFcheckBox->isChecked()).toBool();
	bool generateMinHeightSF	= settings.value("generateMinHeightSF",generateMinHeightSFcheckBox->isChecked()).toBool();
	bool generateMaxHeightSF	= settings.value("generateMaxHeightSF",generateMinHeightSFcheckBox->isChecked()).toBool();
	bool generateAbgHeightSF	= settings.value("generateAvgHeightSF",generateAvgHeightSFcheckBox->isChecked()).toBool();
	bool generateStdDevHeightSF	= settings.value("generateStdDevHeightSF",generateStdDevHeightSFcheckBox->isChecked()).toBool();
	bool generateHeightRangeSF	= settings.value("generateHeightRangeSF",generateHeightRangeSFcheckBox->isChecked()).toBool();
	
	settings.endGroup();

	gridStepDoubleSpinBox->setValue(step);
	heightProjectionComboBox->setCurrentIndex(projType);
	fillEmptyCellsComboBox->setCurrentIndex(fillStrategy);
	emptyValueDoubleSpinBox->setValue(emptyHeight);
	dimensionComboBox->setCurrentIndex(projDim);
	interpolateSFCheckBox->setChecked(sfProj);
	scalarFieldProjection->setCurrentIndex(sfProjStrategy);
	generateCountSFcheckBox->setChecked(genCountSF);
	resampleCloudCheckBox->setChecked(resampleCloud);
	minVertexCountSpinBox->setValue(minVertexCount);
	ignoreContourBordersCheckBox->setChecked(ignoreBorders);
	generateCountSFcheckBox->setChecked(generateCountSF);
	generateMinHeightSFcheckBox->setChecked(generateMinHeightSF);
	generateMinHeightSFcheckBox->setChecked(generateMaxHeightSF);
	generateAvgHeightSFcheckBox->setChecked(generateAbgHeightSF);
	generateStdDevHeightSFcheckBox->setChecked(generateStdDevHeightSF);
	generateHeightRangeSFcheckBox->setChecked(generateHeightRangeSF);
}

bool ccRasterizeTool::canClose()
{
	if (!m_contourLines.empty())
	{
		//ask the user to confirm before it's tool late!
		if (QMessageBox::question(this,"Unsaved contour lines","Contour lines have not been exported! Do you really want to close the tool?",QMessageBox::Yes,QMessageBox::No) == QMessageBox::No)
			return false;
	}
	return true;
}

void ccRasterizeTool::testAndAccept()
{
	if (!canClose())
		return;
	
	saveSettings();
	accept();
}

void ccRasterizeTool::testAndReject()
{
	if (!canClose())
		return;
	
	reject();
}

void ccRasterizeTool::saveSettings()
{
	QSettings settings;
	settings.beginGroup(ccPS::HeightGridGeneration());
	settings.setValue("ProjectionType",heightProjectionComboBox->currentIndex());
	settings.setValue("ProjectionDim",dimensionComboBox->currentIndex());
	settings.setValue("SfProjEnabled",interpolateSFCheckBox->isChecked());
	settings.setValue("SfProjStrategy",scalarFieldProjection->currentIndex());
	settings.setValue("FillStrategy",fillEmptyCellsComboBox->currentIndex());
	settings.setValue("GridStep",gridStepDoubleSpinBox->value());
	settings.setValue("EmptyCellsHeight",emptyValueDoubleSpinBox->value());
	settings.setValue("GenerateCountSF",generateCountSFcheckBox->isChecked());
	settings.setValue("ResampleOrigCloud",resampleCloudCheckBox->isChecked());
	settings.setValue("MinVertexCount",minVertexCountSpinBox->value());
	settings.setValue("IgnoreBorders",ignoreContourBordersCheckBox->isChecked());
	settings.setValue("generateCountSF",generateCountSFcheckBox->isChecked());
	settings.setValue("generateMinHeightSF",generateMinHeightSFcheckBox->isChecked());
	settings.setValue("generateMaxHeightSF",generateMinHeightSFcheckBox->isChecked());
	settings.setValue("generateAvgHeightSF",generateAvgHeightSFcheckBox->isChecked());
	settings.setValue("generateStdDevHeightSF",generateStdDevHeightSFcheckBox->isChecked());
	settings.setValue("generateHeightRangeSF",generateHeightRangeSFcheckBox->isChecked());
	settings.endGroup();
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

	tabWidget->setEnabled(state);
}

ccPointCloud* ccRasterizeTool::convertGridToCloud(	const std::vector<ExportableFields>& exportedFields,
													bool interpolateSF,
													QString activeSFName) const
{
	if (!m_cloud || !m_grid.isValid())
		return 0;

	//default values
	double emptyCellsHeight = 0;
	double minHeight = m_grid.minHeight;
	double maxHeight = m_grid.maxHeight;
	//get real values
	EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategyExt(	emptyCellsHeight,
																				minHeight,
																				maxHeight);

	//call parent method
	ccPointCloud* cloudGrid = cc2Point5DimEditor::convertGridToCloud(	exportedFields,
																		interpolateSF,
																		resampleOriginalCloud(),
																		m_cloud,
																		fillEmptyCellsStrategy != LEAVE_EMPTY,
																		emptyCellsHeight);

	//success?
	if (cloudGrid)
	{
		//currently displayed SF
		int activeSFIndex = cloudGrid->getScalarFieldIndexByName(qPrintable(activeSFName));
		cloudGrid->setCurrentDisplayedScalarField(activeSFIndex);
		cloudGrid->showSF(true);

		//don't forget original shift
		cloudGrid->setGlobalShift(m_cloud->getGlobalShift());
		cloudGrid->setGlobalScale(m_cloud->getGlobalScale());
	}

	return cloudGrid;
}

void ccRasterizeTool::updateGridAndDisplay()
{
	bool activeLayerIsSF = activeLayerComboBox->currentIndex() != 0;
	bool interpolateSF = activeLayerIsSF || (getTypeOfSFInterpolation() != INVALID_PROJECTION_TYPE);
	bool success = updateGrid(interpolateSF);

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
			//we always compute the default 'height' layer
			exportedFields.push_back(PER_CELL_HEIGHT);
			//but we may also have to compute the 'original SF(s)' layer(s)
			QString activeLayerName = activeLayerComboBox->currentText();
			m_rasterCloud = convertGridToCloud(exportedFields,activeLayerIsSF,activeLayerName);
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

			//update 
			activeLayerChanged(activeLayerComboBox->currentIndex(),false);
		}
		else
		{
			ccLog::Error("Not enough memory!");
			m_window->redraw();
		}
	}

	gridIsUpToDate(success);
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
	ProjectionType sfInterpolation = interpolateSF ? getTypeOfSFInterpolation() : INVALID_PROJECTION_TYPE;

	//vertical dimension
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
	else if (gridTotalSize > 10000000)
	{
		if (QMessageBox::question(0,"Big grid size","The generated grid will have more than 10.000.000 cells! Do you want to proceed anyway?",QMessageBox::Yes,QMessageBox::No) == QMessageBox::No)
			return false;
	}

	removeContourLines();

	//memory allocation
	if (!m_grid.init(gridWidth,gridHeight))
	{
		//not enough memory
		ccLog::Error("Not enough memory");
		return false;
	}
	
	m_grid.gridStep = gridStep;
	m_grid.minCorner = CCVector3d::fromArray(box.minCorner().u);

	ccProgressDialog pDlg(true,this);
	if (m_grid.fillWith(m_cloud,
						Z,
						projectionType,
						getFillEmptyCellsStrategy(fillEmptyCellsComboBox) == INTERPOLATE,
						sfInterpolation,
						&pDlg))
	{
		ccLog::Print(QString("[Rasterize] Current raster grid: size: %1 x %2 / heights: [%3 ; %4]").arg(m_grid.width).arg(m_grid.height).arg(m_grid.minHeight).arg(m_grid.maxHeight));
	}

	return true;
}

ccPointCloud* ccRasterizeTool::generateCloud(bool autoExport/*=true*/) const
{
	if (!m_cloud || !m_grid.isValid())
		return 0;

	//look for fields to be exported
	std::vector<ExportableFields> exportedFields;
	try
	{
		exportedFields.push_back(PER_CELL_HEIGHT);
		if (exportAsSF(PER_CELL_COUNT))
			exportedFields.push_back(PER_CELL_COUNT);
		if (exportAsSF(PER_CELL_MIN_HEIGHT))
			exportedFields.push_back(PER_CELL_MIN_HEIGHT);
		if (exportAsSF(PER_CELL_MAX_HEIGHT))
			exportedFields.push_back(PER_CELL_MAX_HEIGHT);
		if (exportAsSF(PER_CELL_AVG_HEIGHT))
			exportedFields.push_back(PER_CELL_AVG_HEIGHT);
		if (exportAsSF(PER_CELL_HEIGHT_STD_DEV))
			exportedFields.push_back(PER_CELL_HEIGHT_STD_DEV);
		if (exportAsSF(PER_CELL_HEIGHT_RANGE))
			exportedFields.push_back(PER_CELL_HEIGHT_RANGE);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory!");
		return 0;
	}
	QString activeLayerName = activeLayerComboBox->currentText();
	bool activeLayerIsSF = activeLayerComboBox->currentIndex() != 0;
	ccPointCloud* rasterCloud = convertGridToCloud(	exportedFields,
													getTypeOfSFInterpolation() != INVALID_PROJECTION_TYPE || activeLayerIsSF,
													activeLayerName);

	if (rasterCloud && autoExport)
	{
		if (m_cloud->getParent())
			m_cloud->getParent()->addChild(rasterCloud);
		rasterCloud->setDisplay(m_cloud->getDisplay());
		
		if (m_cloud->isEnabled())
		{
			m_cloud->setEnabled(false);
			ccLog::Warning("[Rasterize] Previously selected entity (source cloud) has been hidden!");
		}

		MainWindow* mainWindow = MainWindow::TheInstance();
		if (mainWindow)
			MainWindow::TheInstance()->addToDB(rasterCloud);
		ccLog::Print(QString("[Rasterize] Cloud '%1' successfully exported").arg(rasterCloud->getName()));
	}

	return rasterCloud;
}

void ccRasterizeTool::generateMesh() const
{
	ccPointCloud* rasterCloud = generateCloud(false);
	if (rasterCloud)
	{
		char errorStr[1024];
		CCLib::GenericIndexedMesh* baseMesh = CCLib::PointProjectionTools::computeTriangulation(rasterCloud,
																								DELAUNAY_2D_AXIS_ALIGNED,
																								0,
																								2,
																								errorStr);
		ccMesh* rasterMesh = 0;
		if (baseMesh)
		{
			rasterMesh = new ccMesh(baseMesh,rasterCloud);
			delete baseMesh;
			baseMesh = 0;
		}
		if (rasterMesh)
		{
			if (m_cloud->getParent())
				m_cloud->getParent()->addChild(rasterMesh);
			rasterCloud->setEnabled(false);
			rasterCloud->setVisible(true);
			rasterCloud->setName("vertices");
			rasterMesh->addChild(rasterCloud);
			rasterMesh->setDisplay_recursive(m_cloud->getDisplay());
			rasterMesh->setName(m_cloud->getName() + ".mesh");

			MainWindow* mainWindow = MainWindow::TheInstance();
			if (mainWindow)
				MainWindow::TheInstance()->addToDB(rasterMesh);
			ccLog::Print(QString("[Rasterize] Mesh '%1' successfully exported").arg(rasterMesh->getName()));
		}
		else
		{
			ccLog::Error(QString("Failed to create mesh (Triangle lib error: '%1')").arg(errorStr));
		}
	}
}

#ifdef CC_GDAL_SUPPORT
//GDAL
#include <gdal.h>
#include <gdal_priv.h>
#include <cpl_string.h>

//local
#include "ui_rasterExportOptionsDlg.h"

//system
#include <assert.h>

class RasterExportOptionsDlg : public QDialog, public Ui::RasterExportOptionsDialog
{
public:

	explicit RasterExportOptionsDlg(QWidget* parent = 0)
		: QDialog(parent)
		, Ui::RasterExportOptionsDialog()
	{
		setupUi(this);
		setWindowFlags(Qt::Tool);
	}
};

#endif

void ccRasterizeTool::generateRaster() const
{
#ifdef CC_GDAL_SUPPORT

	if (!m_cloud || !m_grid.isValid())
		return;

	GDALAllRegister();
	ccLog::PrintDebug("(GDAL drivers: %i)", GetGDALDriverManager()->GetDriverCount());

	const char *pszFormat = "GTiff";
	GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
	if (!poDriver)
	{
		ccLog::Error("[GDAL] Driver %s is not supported", pszFormat);
		return;
	}

	char** papszMetadata = poDriver->GetMetadata();
	if( !CSLFetchBoolean( papszMetadata, GDAL_DCAP_CREATE, FALSE ) )
	{
		ccLog::Error("[GDAL] Driver %s doesn't support Create() method", pszFormat);
		return;
	}

	//which (and how many) bands shall we create?
	bool heightBand = true; //height by default
	bool densityBand = false;
	bool allSFBands = false;
	int sfBandIndex = -1; //scalar field index
	int totalBands = 0;

	bool interpolateSF = (getTypeOfSFInterpolation() != INVALID_PROJECTION_TYPE);
	ccPointCloud* pc = m_cloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(m_cloud) : 0;

	bool hasSF =  interpolateSF && pc && !m_grid.scalarFields.empty();
	
	RasterExportOptionsDlg reoDlg;
	reoDlg.dimensionsLabel->setText(QString("%1 x %2").arg(m_grid.width).arg(m_grid.height));
	reoDlg.exportHeightsCheckBox->setChecked(heightBand);
	reoDlg.exportDensityCheckBox->setChecked(densityBand);
	reoDlg.exportDisplayedSFCheckBox->setEnabled(hasSF);
	reoDlg.exportAllSFCheckBox->setEnabled(hasSF);
	reoDlg.exportAllSFCheckBox->setChecked(allSFBands);

	if (!reoDlg.exec())
		return;

	//we ask the output filename AFTER displaying the export parameters ;)
	QString outputFilename;
	{
		QSettings settings;
		settings.beginGroup(ccPS::HeightGridGeneration());
		QString imageSavePath = settings.value("savePathImage",QApplication::applicationDirPath()).toString();
		outputFilename = QFileDialog::getSaveFileName(0,"Save height grid raster",imageSavePath+QString("/raster.tif"),"geotiff (*.tif)");

		if (outputFilename.isNull())
			return;

		//save current export path to persistent settings
		settings.setValue("savePathImage",QFileInfo(outputFilename).absolutePath());
	}

	heightBand = reoDlg.exportHeightsCheckBox->isChecked();
	densityBand = reoDlg.exportDensityCheckBox->isChecked();
	if (hasSF)
	{
		assert(pc);
		allSFBands = reoDlg.exportAllSFCheckBox->isChecked() && hasSF;
		if (!allSFBands && reoDlg.exportDisplayedSFCheckBox->isChecked())
		{
			sfBandIndex = pc->getCurrentDisplayedScalarFieldIndex();
			if (sfBandIndex < 0)
				ccLog::Warning("[Rasterize] Cloud has no active (displayed) SF!");
		}
	}

	totalBands = heightBand ? 1 : 0;
	if (densityBand)
	{
		++totalBands;
	}
	if (allSFBands)
	{
		assert(hasSF);
		for (size_t i=0; i<m_grid.scalarFields.size(); ++i)
			if (m_grid.scalarFields[i])
				++totalBands;
	}
	else if (sfBandIndex >= 0)
	{
		++totalBands;
	}
	
	if (totalBands == 0)
	{
		ccLog::Warning("[Rasterize] Warning, can't output a raster with no band! (check export parameters)");
		return;
	}

	//data type
	GDALDataType dataType = (std::max(sizeof(PointCoordinateType),sizeof(ScalarType)) > 4 ? GDT_Float64 : GDT_Float32);

	char **papszOptions = NULL;
	GDALDataset* poDstDS = poDriver->Create(qPrintable(outputFilename),
											static_cast<int>(m_grid.width),
											static_cast<int>(m_grid.height),
											totalBands,
											dataType, 
											papszOptions);

	if (!poDstDS)
	{
		ccLog::Error("[GDAL] Failed to create output raster (not enough memory?)");
		return;
	}

	ccBBox box = getCustomBBox();
	assert(box.isValid());

	//vertical dimension
	const unsigned char Z = getProjectionDimension();
	assert(Z >= 0 && Z <= 2);
	const unsigned char X = Z == 2 ? 0 : Z +1;
	const unsigned char Y = X == 2 ? 0 : X +1;

	double shiftX = box.minCorner().u[X];
	double shiftY = box.minCorner().u[Y];

	double stepX = m_grid.gridStep;
	double stepY = m_grid.gridStep;
	if (pc)
	{
		const CCVector3d& shift = pc->getGlobalShift();
		shiftX -= shift.u[X];
		shiftY -= shift.u[Y];

		double scale = pc->getGlobalScale();
		assert(scale != 0);
		stepX /= scale;
		stepY /= scale;
	}

	double adfGeoTransform[6] = {	shiftX,		//top left x
									stepX,		//w-e pixel resolution (can be negative)
									0,			//0
									shiftY,		//top left y
									0,			//0
									stepY		//n-s pixel resolution (can be negative)
	};

	poDstDS->SetGeoTransform( adfGeoTransform );

	//OGRSpatialReference oSRS;
	//oSRS.SetUTM( 11, TRUE );
	//oSRS.SetWellKnownGeogCS( "NAD27" );
	//char *pszSRS_WKT = NULL;
	//oSRS.exportToWkt( &pszSRS_WKT );
	//poDstDS->SetProjection( pszSRS_WKT );
	//CPLFree( pszSRS_WKT );

	double* scanline = (double*) CPLMalloc(sizeof(double)*m_grid.width);
	int currentBand = 0;

	//exort height band?
	if (heightBand)
	{
		GDALRasterBand* poBand = poDstDS->GetRasterBand(++currentBand);
		assert(poBand);
		poBand->SetColorInterpretation(GCI_Undefined);

		EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(fillEmptyCellsComboBox);

		double emptyCellHeight = 0;
		switch (fillEmptyCellsStrategy)
		{
		case LEAVE_EMPTY:
			emptyCellHeight = m_grid.minHeight-1.0;
			poBand->SetNoDataValue(emptyCellHeight); //should be transparent!
			break;
		case FILL_MINIMUM_HEIGHT:
			emptyCellHeight = m_grid.minHeight;
			break;
		case FILL_MAXIMUM_HEIGHT:
			emptyCellHeight = m_grid.maxHeight;
			break;
		case FILL_CUSTOM_HEIGHT:
			emptyCellHeight = getCustomHeightForEmptyCells();
			break;
		case FILL_AVERAGE_HEIGHT:
			emptyCellHeight = m_grid.meanHeight;
			break;
		default:
			assert(false);
		}

		for (unsigned j=0; j<m_grid.height; ++j)
		{
			const RasterCell* aCell = m_grid.data[j];
			for (unsigned i=0; i<m_grid.width; ++i,++aCell)
			{
				scanline[i] = aCell->h == aCell->h ? aCell->h : emptyCellHeight;
			}

			if (poBand->RasterIO( GF_Write, 0, static_cast<int>(j), static_cast<int>(m_grid.width), 1, scanline, static_cast<int>(m_grid.width), 1, GDT_Float64, 0, 0 ) != CE_None)
			{
				ccLog::Error("[GDAL] An error occurred while writing the height band!");
				if (scanline)
					CPLFree(scanline);
				GDALClose( (GDALDatasetH) poDstDS );
				return;
			}
		}
	}

	//export density band
	if (densityBand)
	{
		GDALRasterBand* poBand = poDstDS->GetRasterBand(++currentBand);
		assert(poBand);
		poBand->SetColorInterpretation(GCI_Undefined);
		for (unsigned j=0; j<m_grid.height; ++j)
		{
			const RasterCell* aCell = m_grid.data[j];
			for (unsigned i=0; i<m_grid.width; ++i,++aCell)
			{
				scanline[i] = aCell->nbPoints;
			}

			if (poBand->RasterIO( GF_Write, 0, static_cast<int>(j), static_cast<int>(m_grid.width), 1, scanline, static_cast<int>(m_grid.width), 1, GDT_Float64, 0, 0 ) != CE_None)
			{
				ccLog::Error("[GDAL] An error occurred while writing the height band!");
				if (scanline)
					CPLFree(scanline);
				GDALClose( (GDALDatasetH) poDstDS );
				return;
			}
		}
	}

	//export SF bands
	if (allSFBands || sfBandIndex >= 0)
	{
		for (size_t k=0; k<m_grid.scalarFields.size(); ++k)
		{
			double* _sfGrid = m_grid.scalarFields[k];
			if (_sfGrid && (allSFBands || sfBandIndex == static_cast<int>(k))) //valid SF grid
			{
				GDALRasterBand* poBand = poDstDS->GetRasterBand(++currentBand);

				double sfNanValue = static_cast<double>(CCLib::ScalarField::NaN());
				poBand->SetNoDataValue(sfNanValue); //should be transparent!
				assert(poBand);
				poBand->SetColorInterpretation(GCI_Undefined);

				for (unsigned j=0; j<m_grid.height; ++j)
				{
					const RasterCell* aCell = m_grid.data[j];
					for (unsigned i=0; i<m_grid.width; ++i,++_sfGrid,++aCell)
					{
						scanline[i] = aCell->nbPoints ? *_sfGrid : sfNanValue;
					}

					if (poBand->RasterIO( GF_Write, 0, static_cast<int>(j), static_cast<int>(m_grid.width), 1, scanline, static_cast<int>(m_grid.width), 1, GDT_Float64, 0, 0 ) != CE_None)
					{
						//the corresponding SF should exist on the input cloud
						CCLib::ScalarField* formerSf = pc->getScalarField(static_cast<int>(k));
						assert(formerSf);
						ccLog::Error(QString("[GDAL] An error occurred while writing the '%1' scalar field band!").arg(formerSf->getName()));
						k = m_grid.scalarFields.size(); //quick stop
						break;
					}
				}
			}
		}
	}

	if (scanline)
		CPLFree(scanline);
	scanline = 0;

	/* Once we're done, close properly the dataset */
	GDALClose( (GDALDatasetH) poDstDS );

	ccLog::Print(QString("[Rasterize] Raster '%1' succesfully saved").arg(outputFilename));

#else
	assert(false);
	ccLog::Error("[Rasterize] GDAL not supported by this version! Can't generate a raster...");
#endif
}

void ccRasterizeTool::generateContours()
{
	if (!m_grid.isValid() || !m_rasterCloud)
	{
		ccLog::Error("Need a valid raster/cloud to compute contours!");
		return;
	}

	ccScalarField* activeLayer = m_rasterCloud->getCurrentDisplayedScalarField();
	if (!activeLayer)
	{
		ccLog::Error("No valid/active layer!");
		return;
	}

	double startValue = contourStartDoubleSpinBox->value();
	if (startValue > activeLayer->getMax())
	{
		ccLog::Error("Start value is above the layer maximum value!");
		return;
	}
	double step = contourStepDoubleSpinBox->value();
	assert(step > 0);
	unsigned levelCount = 1 + static_cast<unsigned>(floor((activeLayer->getMax()-startValue)/step));

	removeContourLines();
	bool ignoreBorders = ignoreContourBordersCheckBox->isChecked();

	unsigned xDim = m_grid.width;
	unsigned yDim = m_grid.height;
	int margin = 0;
	if (!ignoreBorders)
	{
		margin = 1;
		xDim += 2;
		yDim += 2;
	}
	std::vector<double> grid;
	try
	{
		grid.resize(xDim * yDim, 0);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory!");
		if (m_window)
			m_window->redraw();
		return;
	}

	//fill grid
	{
		bool sparseLayer = (activeLayer->currentSize() != m_grid.height * m_grid.width);
		double emptyCellsValue = activeLayer->getMin()-1.0;

		unsigned layerIndex = 0;
		for (unsigned j=0; j<m_grid.height; ++j)
		{
			RasterCell* cell = m_grid.data[j];
			double* row = &(grid[(j+margin)*xDim + margin]);
			for (unsigned i=0; i<m_grid.width; ++i)
			{
				if (cell[i].nbPoints || !sparseLayer)
				{
					ScalarType value = activeLayer->getValue(layerIndex++);
					row[i] = ccScalarField::ValidValue(value) ? value : emptyCellsValue;
				}
				else
				{
					row[i] = emptyCellsValue;
				}
			}
		}
	}

	bool memoryError = false;

	try
	{
		Isolines<double> iso(static_cast<int>(xDim),static_cast<int>(yDim));
		if (!ignoreBorders)
			iso.createOnePixelBorder(&(grid.front()),activeLayer->getMin()-1.0);
		//bounding box
		ccBBox box = getCustomBBox();
		assert(box.isValid());

		//vertical dimension
		const unsigned char Z = getProjectionDimension();
		assert(Z >= 0 && Z <= 2);
		const unsigned char X = Z == 2 ? 0 : Z +1;
		const unsigned char Y = X == 2 ? 0 : X +1;

		int minVertexCount = minVertexCountSpinBox->value();
		assert(minVertexCount >= 3);

		ccProgressDialog pDlg(true,this);
		pDlg.setMethodTitle("Contour plot");
		pDlg.setInfo(qPrintable(QString("Levels: %1\nCells: %2 x %3").arg(levelCount).arg(m_grid.width).arg(m_grid.height)));
		pDlg.start();
		pDlg.show();
		QApplication::processEvents();
		CCLib::NormalizedProgress nProgress(&pDlg,levelCount);

		int lineWidth = contourWidthSpinBox->value();
		bool colorize = colorizeContoursCheckBox->isChecked();

		double v = startValue;
		while (v <= activeLayer->getMax() && !memoryError)
		{
			//extract contour lines for the current level
			iso.setThreshold(v);
			int lineCount = iso.find(&(grid.front()));

			ccLog::PrintDebug(QString("[Rasterize][Isolines] value=%1 : %2 lines").arg(v).arg(lineCount));

			//convert them to poylines
			int realCount = 0;
			for (int i=0; i<lineCount; ++i)
			{
				int vertCount = iso.getContourLength(i);
				if (vertCount >= minVertexCount)
				{
					ccPointCloud* vertices = new ccPointCloud("vertices");
					ccPolyline* poly = new ccPolyline(vertices);
					poly->addChild(vertices);
					bool isClosed = iso.isContourClosed(i);
					if (poly->reserve(vertCount) && vertices->reserve(vertCount))
					{
						unsigned localIndex = 0;
						for (int vi=0; vi<vertCount; ++vi)
						{
							double x = iso.getContourX(i,vi) - margin + 0.5;
							double y = iso.getContourY(i,vi) - margin + 0.5;

							CCVector3 P;
							P.u[X] = static_cast<PointCoordinateType>(x * m_grid.gridStep + box.minCorner().u[X]);
							P.u[Y] = static_cast<PointCoordinateType>(y * m_grid.gridStep + box.minCorner().u[Y]);
							P.u[Z] = static_cast<PointCoordinateType>(v);

							vertices->addPoint(P);
							assert(localIndex < vertices->size());
							poly->addPointIndex(localIndex++);
						}

						assert(poly);
						if (poly->size() > 1)
						{
							poly->setName(QString("Contour line value=%1 (#%2)").arg(v).arg(++realCount));
							poly->setGlobalScale(m_cloud->getGlobalScale());
							poly->setGlobalShift(m_cloud->getGlobalShift());
							poly->setWidth(lineWidth);
							poly->setClosed(isClosed); //if we have less vertices, it means we have 'chopped' the original contour
							poly->setColor(ccColor::darkGrey);
							if (colorize)
							{
								const ColorCompType* col = activeLayer->getColor(v);
								if (col)
									poly->setColor(ccColor::Rgb(col));
							}
							poly->showColors(true);
							vertices->setEnabled(false);
							//add the 'const altitude' meta-data as well
							poly->setMetaData(ccPolyline::MetaKeyConstAltitude(),QVariant(v));
						
							if (m_window)
								m_window->addToOwnDB(poly);

							m_contourLines.push_back(poly);
						}
						else
						{
							delete poly;
							poly = 0;
						}
					}
					else
					{
						delete poly;
						poly = 0;
						ccLog::Error("Not enough memory!");
						memoryError = true; //early stop
						break;
					}
				}
			}
			v += step;

			if (!nProgress.oneStep())
			{
				//process cancelled by user
				break;
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory!");
	}

	ccLog::Print(QString("[Rasterize] %1 iso-lines generated (%2 levels)").arg(m_contourLines.size()).arg(levelCount));

	if (!m_contourLines.empty())
	{
		if (memoryError)
		{
			removeContourLines();
		}
		else
		{
			exportContoursPushButton->setEnabled(true);
			clearContoursPushButton->setEnabled(true);
		}
	}

	if (m_window)
		m_window->redraw();
}

void ccRasterizeTool::exportContourLines()
{
	MainWindow* mainWindow = MainWindow::TheInstance();
	if (!mainWindow || !m_cloud)
	{
		assert(false);
		return;
	}

	bool colorize = colorizeContoursCheckBox->isChecked();

	ccHObject* group = new ccHObject(QString("Contour plot(%1) [step=%2]").arg(m_cloud->getName()).arg(contourStepDoubleSpinBox->value()));
	for (size_t i=0; i<m_contourLines.size(); ++i)
	{
		ccPolyline* poly = m_contourLines[i];
		if (!colorize)
			poly->showColors(false);
		group->addChild(poly);
		if (m_window)
			m_window->removeFromOwnDB(poly);
	}
	m_contourLines.clear();

	group->setDisplay_recursive(m_cloud->getDisplay());
	mainWindow->addToDB(group);

	ccLog::Print(QString("Contour lines have been succesfully exported to DB (group name: %1)").arg(group->getName()));
}

cc2Point5DimEditor::EmptyCellFillOption ccRasterizeTool::getFillEmptyCellsStrategyExt(	double& emptyCellsHeight,
																						double& minHeight,
																						double& maxHeight) const
{
	EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(fillEmptyCellsComboBox);

	emptyCellsHeight = 0.0;
	minHeight = m_grid.minHeight;
	maxHeight = m_grid.maxHeight;
	
	switch (fillEmptyCellsStrategy)
	{
	case LEAVE_EMPTY:
		//nothing to do
		break;
	case FILL_MINIMUM_HEIGHT:
		emptyCellsHeight = m_grid.minHeight;
		break;
	case FILL_MAXIMUM_HEIGHT:
		emptyCellsHeight = m_grid.maxHeight;
		break;
	case FILL_CUSTOM_HEIGHT:
	case INTERPOLATE:
		{
			double customEmptyCellsHeight = getCustomHeightForEmptyCells();
			//update min and max height by the way (only if there are invalid cells ;)
			if (m_grid.validCellCount != m_grid.width * m_grid.height)
			{
				if (customEmptyCellsHeight <= m_grid.minHeight)
					minHeight = customEmptyCellsHeight;
				else if (customEmptyCellsHeight >= m_grid.maxHeight)
					maxHeight = customEmptyCellsHeight;
				emptyCellsHeight = customEmptyCellsHeight;
			}
		}
		break;
	case FILL_AVERAGE_HEIGHT:
		//'average height' is a kind of 'custom height' so we can fall back to this mode!
		fillEmptyCellsStrategy = FILL_CUSTOM_HEIGHT;
		emptyCellsHeight = m_grid.meanHeight;
		break;
	default:
		assert(false);
	}

	return fillEmptyCellsStrategy;
}

void ccRasterizeTool::generateImage() const
{
	if (!m_grid.isValid())
		return;

	//default values
	double emptyCellsHeight = 0;
	double minHeight = m_grid.minHeight;
	double maxHeight = m_grid.maxHeight;
	//get real values
	EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategyExt(	emptyCellsHeight,
																				minHeight,
																				maxHeight);

	QImage bitmap8(m_grid.width,m_grid.height,QImage::Format_Indexed8);
	if (!bitmap8.isNull())
	{
		// Build a custom palette (gray scale)
		QVector<QRgb> palette(256);
		{
			for (unsigned i = 0; i < 256; i++)
				palette[i] = qRgba(i,i,i,255);
		}
		double maxColorComp = 255.99; //.99 --> to avoid round-off issues later!

		if (fillEmptyCellsStrategy == LEAVE_EMPTY)
		{
			palette[255] = qRgba(255,0,255,0); //magenta/transparent color for empty cells (in place of pure white)
			maxColorComp = 254.99;
		}

		bitmap8.setColorTable(palette);
		//bitmap8.fill(255);

		unsigned emptyCellColorIndex = 0;
		switch (fillEmptyCellsStrategy)
		{
		case LEAVE_EMPTY:
			emptyCellColorIndex = 255; //should be transparent!
			break;
		case FILL_MINIMUM_HEIGHT:
			emptyCellColorIndex = 0;
			break;
		case FILL_MAXIMUM_HEIGHT:
			emptyCellColorIndex = 255;
			break;
		case FILL_CUSTOM_HEIGHT:
			{
				double normalizedHeight = (emptyCellsHeight-minHeight)/(maxHeight-minHeight);
				//min and max should have already been updated with custom empty cell height!
				assert(normalizedHeight >= 0.0 && normalizedHeight <= 1.0);
				emptyCellColorIndex = static_cast<unsigned>(floor(normalizedHeight*maxColorComp));
			}
			break;
		case FILL_AVERAGE_HEIGHT:
		default:
			assert(false);
		}

		double range = maxHeight - minHeight;
		if (range < ZERO_TOLERANCE)
			range = 1.0;

		// Filling the image with grid values
		for (unsigned j=0; j<m_grid.height; ++j)
		{
			const RasterCell* aCell = m_grid.data[j];
			for (unsigned i=0; i<m_grid.width; ++i,++aCell)
			{
				if (aCell->h == aCell->h)
				{
					double normalizedHeight = (aCell->h - minHeight)/range;
					assert(normalizedHeight >= 0.0 && normalizedHeight <= 1.0);
					unsigned char val = static_cast<unsigned char>(floor(normalizedHeight*maxColorComp));
					bitmap8.setPixel(i,m_grid.height-1-j,val);
				}
				else //NaN
				{
					bitmap8.setPixel(i,m_grid.height-1-j,emptyCellColorIndex);
				}
			}
		}

		//open file saving dialog
		{
			//add images output file filters
			QString filters;

			//we grab the list of supported image file formats (writing)
			QList<QByteArray> formats = QImageWriter::supportedImageFormats();
			if (formats.empty())
			{
				ccLog::Error("No image format supported by your system?!\n(check that the 'imageformats' directory is alongside CC executable)");
			}
			else
			{
				//we convert this list into a proper "filters" string
				for (int i=0; i<formats.size(); ++i)
					filters.append(QString("%1 image (*.%2)\n").arg(QString(formats[i].data()).toUpper()).arg(formats[i].data()));

				QSettings settings;
				settings.beginGroup(ccPS::HeightGridGeneration());
				QString imageSavePath = settings.value("savePathImage",QApplication::applicationDirPath()).toString();
				QString outputFilename = QFileDialog::getSaveFileName(0,"Save raster image",imageSavePath+QString("/raster_image.%1").arg(formats[0].data()),filters);

				if (!outputFilename.isNull())
				{
					if (bitmap8.save(outputFilename))
					{
						ccLog::Print(QString("[Rasterize] Image '%1' succesfully saved").arg(outputFilename));
						//save current export path to persistent settings
						settings.setValue("savePathImage",QFileInfo(outputFilename).absolutePath());
					}
					else
					{
						ccLog::Error("Failed to save image file!");
					}
				}
			}
		}
	}
	else
	{
		ccLog::Error("Failed to create output image! (not enough memory?)");
	}
}

void ccRasterizeTool::generateASCIIMatrix() const
{
	if (!m_grid.isValid())
		return;

	QSettings settings;
	settings.beginGroup(ccPS::HeightGridGeneration());
	QString asciiGridSavePath = settings.value("savePathASCIIGrid",QApplication::applicationDirPath()).toString();

	//open file saving dialog
	QString filter("ASCII file (*.txt)");
	QString outputFilename = QFileDialog::getSaveFileName(0,"Save grid as ASCII file",asciiGridSavePath+QString("/raster_matrix.txt"),filter);
	if (outputFilename.isNull())
		return;

	FILE* pFile = fopen(qPrintable(outputFilename),"wt");
	if (!pFile)
	{
		ccLog::Warning(QString("[ccHeightGridGeneration] Failed to write '%1' file!").arg(outputFilename));
	}

	//default values
	double emptyCellsHeight = 0;
	double minHeight = m_grid.minHeight;
	double maxHeight = m_grid.maxHeight;
	//get real values
	getFillEmptyCellsStrategyExt(emptyCellsHeight, minHeight, maxHeight);
	for (unsigned j=0; j<m_grid.height; ++j)
	{
		const RasterCell* aCell = m_grid.data[j];
		for (unsigned i=0; i<m_grid.width; ++i,++aCell)
			fprintf(pFile,"%.8f ", aCell->h == aCell->h ? aCell->h : emptyCellsHeight);

		fprintf(pFile,"\n");
	}

	fclose(pFile);
	pFile = 0;

	//save current export path to persistent settings
	settings.setValue("savePathASCIIGrid",QFileInfo(outputFilename).absolutePath());

	ccLog::Print(QString("[Rasterize] Raster matrix '%1' succesfully saved").arg(outputFilename));
}

