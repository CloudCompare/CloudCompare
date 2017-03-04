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

#include "ccRasterizeTool.h"

//Local
#include "ccPersistentSettings.h"
#include "ccCommon.h"
#include "mainwindow.h"
#include "ccIsolines.h"

//qCC_db
#include <ccFileUtils.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccProgressDialog.h>
#include <ccPolyline.h>
#include <ccMesh.h>
#include <ccColorScalesManager.h>

//qCC_gl
#include <ccGLWindow.h>

//qCC_io
#include <ImageFileFilter.h>

//Qt
#include <QSettings>
#include <QPushButton>
#include <QMessageBox>
#include <QFileDialog>
#include <QMap>

//System
#include <assert.h>

static const char HILLSHADE_FIELD_NAME[] = "Hillshade";

ccRasterizeTool::ccRasterizeTool(ccGenericPointCloud* cloud, QWidget* parent/*=0*/)
	: QDialog(parent, Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint)
	, cc2Point5DimEditor()
	, Ui::RasterizeToolDialog()
	, m_cloud(cloud)
{
	setupUi(this);

#ifndef CC_GDAL_SUPPORT
	generateRasterPushButton->setDisabled(true);
	generateRasterPushButton->setChecked(false);
#endif

	//force update
	resampleOptionToggled(resampleCloudCheckBox->isChecked());

	connect(buttonBox,					SIGNAL(accepted()),					this,	SLOT(testAndAccept()));
	connect(buttonBox,					SIGNAL(rejected()),					this,	SLOT(testAndReject()));
	connect(gridStepDoubleSpinBox,		SIGNAL(valueChanged(double)),		this,	SLOT(updateGridInfo()));
	connect(gridStepDoubleSpinBox,		SIGNAL(valueChanged(double)),		this,	SLOT(gridOptionChanged()));
	connect(emptyValueDoubleSpinBox,	SIGNAL(valueChanged(double)),		this,	SLOT(gridOptionChanged()));
	connect(resampleCloudCheckBox,		SIGNAL(toggled(bool)),				this,	SLOT(resampleOptionToggled(bool)));
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
	connect(generateHillshadePushButton,SIGNAL(clicked()),					this,	SLOT(generateHillshade()));

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
		activeLayerComboBox->addItem(ccRasterGrid::GetDefaultFieldName(ccRasterGrid::PER_CELL_HEIGHT), QVariant(LAYER_HEIGHT));
		if (m_cloud->hasColors())
		{
			activeLayerComboBox->addItem("RGB", QVariant(LAYER_RGB));
		}
		if (cloud->isA(CC_TYPES::POINT_CLOUD) && cloud->hasScalarFields())
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
			for (unsigned i = 0; i < pc->getNumberOfScalarFields(); ++i)
			{
				activeLayerComboBox->addItem(pc->getScalarField(i)->getName(), QVariant(LAYER_SF));
			}
		}

		activeLayerComboBox->setEnabled(activeLayerComboBox->count() > 1);

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
		if (m_glWindow)
			m_glWindow->removeFromOwnDB(poly);
		delete poly;
		m_contourLines.pop_back();
	}

	exportContoursPushButton->setEnabled(false);
	clearContoursPushButton->setEnabled(false);

	if (m_glWindow)
		m_glWindow->redraw();
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

bool ccRasterizeTool::exportAsSF(ccRasterGrid::ExportableFields field) const
{
	switch (field)
	{
	case ccRasterGrid::PER_CELL_COUNT:
		return generateCountSFcheckBox->isChecked();
	case ccRasterGrid::PER_CELL_MIN_HEIGHT:
		return generateMinHeightSFcheckBox->isChecked();
	case ccRasterGrid::PER_CELL_MAX_HEIGHT:
		return generateMaxHeightSFcheckBox->isChecked();
	case ccRasterGrid::PER_CELL_AVG_HEIGHT:
		return generateAvgHeightSFcheckBox->isChecked();
	case ccRasterGrid::PER_CELL_HEIGHT_STD_DEV:
		return generateStdDevHeightSFcheckBox->isChecked();
	case ccRasterGrid::PER_CELL_HEIGHT_RANGE:
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

void ccRasterizeTool::resampleOptionToggled(bool state)
{
	warningResampleWithAverageLabel->setVisible(resampleCloudCheckBox->isChecked() && getTypeOfProjection() == ccRasterGrid::PROJ_AVERAGE_VALUE);
	gridOptionChanged();
}

void ccRasterizeTool::projectionTypeChanged(int index)
{
	//we can't use the 'resample origin cloud' option with 'average height' projection
	//resampleCloudCheckBox->setEnabled(index != PROJ_AVERAGE_VALUE);
	//DGM: now we can! We simply display a warning message
	warningResampleWithAverageLabel->setVisible(resampleCloudCheckBox->isChecked() && index == ccRasterGrid::PROJ_AVERAGE_VALUE);
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
	if (layerIndex != 0) //0 is always the cell height
	{
		interpolateSFCheckBox->setChecked(true);
		interpolateSFCheckBox->setEnabled(false);
		generateImagePushButton->setEnabled(false);
		generateASCIIPushButton->setEnabled(false);
		projectContoursOnAltCheckBox->setEnabled(true);
	}
	else
	{
		generateImagePushButton->setEnabled(true);
		interpolateSFCheckBox->setEnabled(true);
		generateASCIIPushButton->setEnabled(true);
		projectContoursOnAltCheckBox->setEnabled(false);
	}

	if (m_rasterCloud)
	{
		//active layer = RGB colors
		if (activeLayerComboBox->currentData().toInt() == LAYER_RGB)
		{
			if (!m_rasterCloud->hasColors())
			{
				gridIsUpToDate(false);
			}
			gridLayerRangeLabel->setText("[0 ; 255]");

			m_rasterCloud->showColors(true);
			m_rasterCloud->showSF(false);
		}
		else
		{
			//does the selected 'layer' exist?
			int sfIndex = m_rasterCloud->getScalarFieldIndexByName(qPrintable(activeLayerComboBox->itemText(layerIndex)));
			m_rasterCloud->setCurrentDisplayedScalarField(sfIndex);
			m_rasterCloud->showSF(true);
			m_rasterCloud->showColors(false);

			if (sfIndex >= 0)
			{
				ccScalarField* activeLayer = m_rasterCloud->getCurrentDisplayedScalarField();
				if (activeLayer)
				{
					const ccScalarField::Range& layerValues = activeLayer->displayRange();
					gridLayerRangeLabel->setText(QString("%1 [%2 ; %3]").arg(layerValues.range()).arg(layerValues.min()).arg(layerValues.max()));
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
				gridLayerRangeLabel->setText("invalid layer?!");
				gridIsUpToDate(false);
			}
		}

		if (m_glWindow && autoRedraw)
		{
			m_glWindow->redraw();
		}
	}
}

void ccRasterizeTool::fillEmptyCellStrategyChanged(int)
{
	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(fillEmptyCellsComboBox);

	emptyValueDoubleSpinBox->setEnabled(	fillEmptyCellsStrategy == ccRasterGrid::FILL_CUSTOM_HEIGHT
										||	fillEmptyCellsStrategy == ccRasterGrid::INTERPOLATE );
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

ccRasterGrid::ProjectionType ccRasterizeTool::getTypeOfProjection() const
{
	switch (heightProjectionComboBox->currentIndex())
	{
	case 0:
		return ccRasterGrid::PROJ_MINIMUM_VALUE;
	case 1:
		return ccRasterGrid::PROJ_AVERAGE_VALUE;
	case 2:
		return ccRasterGrid::PROJ_MAXIMUM_VALUE;
	default:
		//shouldn't be possible for this option!
		assert(false);
	}

	return ccRasterGrid::INVALID_PROJECTION_TYPE;
}

ccRasterGrid::ProjectionType ccRasterizeTool::getTypeOfSFInterpolation() const
{
	if (!interpolateSFFrame->isEnabled() || !interpolateSFCheckBox->isChecked())
		return ccRasterGrid::INVALID_PROJECTION_TYPE; //means that we don't want to keep SF values

	switch (scalarFieldProjection->currentIndex())
	{
	case 0:
		return ccRasterGrid::PROJ_MINIMUM_VALUE;
	case 1:
		return ccRasterGrid::PROJ_AVERAGE_VALUE;
	case 2:
		return ccRasterGrid::PROJ_MAXIMUM_VALUE;
	default:
		//shouldn't be possible for this option!
		assert(false);
	}

	return ccRasterGrid::INVALID_PROJECTION_TYPE;
}

void ccRasterizeTool::loadSettings()
{
	QSettings settings;
	settings.beginGroup(ccPS::HeightGridGeneration());
	int projType				= settings.value("ProjectionType",        heightProjectionComboBox->currentIndex()).toInt();
	int projDim					= settings.value("ProjectionDim",         dimensionComboBox->currentIndex()).toInt();
	bool sfProj					= settings.value("SfProjEnabled",         interpolateSFCheckBox->isChecked()).toBool();
	int sfProjStrategy			= settings.value("SfProjStrategy",        scalarFieldProjection->currentIndex()).toInt();
	int fillStrategy			= settings.value("FillStrategy",          fillEmptyCellsComboBox->currentIndex()).toInt();
	double step					= settings.value("GridStep",              gridStepDoubleSpinBox->value()).toDouble();
	double emptyHeight			= settings.value("EmptyCellsHeight",      emptyValueDoubleSpinBox->value()).toDouble();
	bool genCountSF				= settings.value("GenerateCountSF",       generateCountSFcheckBox->isChecked()).toBool();
	bool resampleCloud			= settings.value("ResampleOrigCloud",     resampleCloudCheckBox->isChecked()).toBool();
	int minVertexCount			= settings.value("MinVertexCount",        minVertexCountSpinBox->value()).toInt();
	bool ignoreBorders			= settings.value("IgnoreBorders",         ignoreContourBordersCheckBox->isChecked()).toBool();
	bool generateCountSF		= settings.value("generateCountSF",       generateCountSFcheckBox->isChecked()).toBool();
	bool generateMinHeightSF	= settings.value("generateMinHeightSF",   generateMinHeightSFcheckBox->isChecked()).toBool();
	bool generateMaxHeightSF	= settings.value("generateMaxHeightSF",   generateMinHeightSFcheckBox->isChecked()).toBool();
	bool generateAbgHeightSF	= settings.value("generateAvgHeightSF",   generateAvgHeightSFcheckBox->isChecked()).toBool();
	bool generateStdDevHeightSF	= settings.value("generateStdDevHeightSF",generateStdDevHeightSFcheckBox->isChecked()).toBool();
	bool generateHeightRangeSF	= settings.value("generateHeightRangeSF", generateHeightRangeSFcheckBox->isChecked()).toBool();
	bool projectContoursOnAlt	= settings.value("projectContoursOnAlt",  projectContoursOnAltCheckBox->isChecked()).toBool();
	
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
	projectContoursOnAltCheckBox->setChecked(projectContoursOnAlt);
}

bool ccRasterizeTool::canClose()
{
	if (!m_contourLines.empty())
	{
		//ask the user to confirm before it's tool late!
		if (QMessageBox::question(	this,
									"Unsaved contour lines",
									"Contour lines have not been exported! Do you really want to close the tool?",
									QMessageBox::Yes,
									QMessageBox::No) == QMessageBox::No)
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
	settings.setValue("ProjectionType", heightProjectionComboBox->currentIndex());
	settings.setValue("ProjectionDim", dimensionComboBox->currentIndex());
	settings.setValue("SfProjEnabled", interpolateSFCheckBox->isChecked());
	settings.setValue("SfProjStrategy", scalarFieldProjection->currentIndex());
	settings.setValue("FillStrategy", fillEmptyCellsComboBox->currentIndex());
	settings.setValue("GridStep", gridStepDoubleSpinBox->value());
	settings.setValue("EmptyCellsHeight", emptyValueDoubleSpinBox->value());
	settings.setValue("GenerateCountSF", generateCountSFcheckBox->isChecked());
	settings.setValue("ResampleOrigCloud", resampleCloudCheckBox->isChecked());
	settings.setValue("MinVertexCount", minVertexCountSpinBox->value());
	settings.setValue("IgnoreBorders", ignoreContourBordersCheckBox->isChecked());
	settings.setValue("generateCountSF", generateCountSFcheckBox->isChecked());
	settings.setValue("generateMinHeightSF", generateMinHeightSFcheckBox->isChecked());
	settings.setValue("generateMaxHeightSF", generateMinHeightSFcheckBox->isChecked());
	settings.setValue("generateAvgHeightSF", generateAvgHeightSFcheckBox->isChecked());
	settings.setValue("generateStdDevHeightSF", generateStdDevHeightSFcheckBox->isChecked());
	settings.setValue("generateHeightRangeSF", generateHeightRangeSFcheckBox->isChecked());
	settings.setValue("projectContoursOnAlt", projectContoursOnAltCheckBox->isChecked());
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

ccPointCloud* ccRasterizeTool::convertGridToCloud(	const std::vector<ccRasterGrid::ExportableFields>& exportedFields,
													bool interpolateSF,
													bool interpolateColors,
													bool copyHillshadeSF,
													QString activeSFName,
													bool exportToOriginalCS) const
{
	if (!m_cloud || !m_grid.isValid())
		return 0;

	//default values
	double emptyCellsHeight = 0;
	double minHeight = m_grid.minHeight;
	double maxHeight = m_grid.maxHeight;
	//get real values
	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategyExt(emptyCellsHeight,
																							minHeight,
																							maxHeight);

	//call parent method
	ccPointCloud* cloudGrid = cc2Point5DimEditor::convertGridToCloud(	exportedFields,
																		interpolateSF,
																		interpolateColors,
																		/*resampleInputCloudXY=*/resampleOriginalCloud(),
																		/*resampleInputCloudZ=*/getTypeOfProjection() != ccRasterGrid::PROJ_AVERAGE_VALUE,
																		/*inputCloud=*/m_cloud,
																		/*fillEmptyCells=*/fillEmptyCellsStrategy != ccRasterGrid::LEAVE_EMPTY,
																		emptyCellsHeight,
																		exportToOriginalCS);

	//success?
	if (cloudGrid)
	{
		//add the hillshade SF
		if (copyHillshadeSF)
		{
			int hillshadeSFIdx = m_rasterCloud->getScalarFieldIndexByName(HILLSHADE_FIELD_NAME);
			if (hillshadeSFIdx >= 0)
			{
				CCLib::ScalarField* hillshadeField = m_rasterCloud->getScalarField(hillshadeSFIdx);
				if (hillshadeField->currentSize() == cloudGrid->size())
				{
					try
					{
						ccScalarField* hillshadeClone = new ccScalarField(*static_cast<ccScalarField*>(hillshadeField));
						cloudGrid->addScalarField(hillshadeClone);
					}
					catch (const std::bad_alloc&)
					{
						ccLog::Warning("[Rasterize] Not enough memory to export the hillshade field");
					}
				}
			}
		}

		//currently displayed SF
		int activeSFIndex = cloudGrid->getScalarFieldIndexByName(qPrintable(activeSFName));
		cloudGrid->showSF(activeSFIndex >= 0);
		if (activeSFIndex < 0 && cloudGrid->getNumberOfScalarFields() != 0)
		{
			//if no SF is displayed, we should at least set a valid one (for later)
			activeSFIndex = static_cast<int>(cloudGrid->getNumberOfScalarFields()) - 1;
		}
		cloudGrid->setCurrentDisplayedScalarField(activeSFIndex);

		cloudGrid->showColors(interpolateColors);

		//don't forget the original shift
		cloudGrid->setGlobalShift(m_cloud->getGlobalShift());
		cloudGrid->setGlobalScale(m_cloud->getGlobalScale());
	}

	return cloudGrid;
}

void ccRasterizeTool::updateGridAndDisplay()
{
	//special case: remove the (temporary) hillshade field entry
	int hillshadeIndex = activeLayerComboBox->findText(HILLSHADE_FIELD_NAME);
	if (hillshadeIndex >= 0)
	{
		if (activeLayerComboBox->currentIndex() == hillshadeIndex && activeLayerComboBox->count() > 1)
		{
			activeLayerComboBox->setCurrentIndex(0);
		}
		activeLayerComboBox->removeItem(hillshadeIndex);
	}

	//remove the previous cloud
	if (m_rasterCloud)
	{
		if (m_glWindow)
		{
			m_glWindow->removeFromOwnDB(m_rasterCloud);
			m_glWindow->redraw();
		}
		
		delete m_rasterCloud;
		m_rasterCloud = 0;
	}

	bool activeLayerIsSF = (activeLayerComboBox->currentData().toInt() == LAYER_SF);
	bool activeLayerIsRGB = (activeLayerComboBox->currentData().toInt() == LAYER_RGB);
	bool interpolateSF = activeLayerIsSF || (getTypeOfSFInterpolation() != ccRasterGrid::INVALID_PROJECTION_TYPE);
	bool success = updateGrid(interpolateSF);

	if (success && m_glWindow)
	{
		//convert grid to point cloud
		std::vector<ccRasterGrid::ExportableFields> exportedFields;
		try
		{
			//we always compute the default 'height' layer
			exportedFields.push_back(ccRasterGrid::PER_CELL_HEIGHT);
			//but we may also have to compute the 'original SF(s)' layer(s)
			QString activeLayerName = activeLayerComboBox->currentText();
			m_rasterCloud = convertGridToCloud(	exportedFields,
												/*interpolateSF=*/activeLayerIsSF,
												/*interpolateColors=*/activeLayerIsRGB,
												/*copyHillshadeSF=*/false,
												activeLayerName,
												false);
		}
		catch (const std::bad_alloc&)
		{
			//see below
		}

		if (m_rasterCloud)
		{
			m_glWindow->addToOwnDB(m_rasterCloud);
			ccBBox box = m_rasterCloud->getDisplayBB_recursive(false, m_glWindow);
			update2DDisplayZoom(box);

			//update 
			activeLayerChanged(activeLayerComboBox->currentIndex(), false);
		}
		else
		{
			ccLog::Error("Not enough memory!");
			m_glWindow->redraw();
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
	ccRasterGrid::ProjectionType projectionType = getTypeOfProjection();
	ccRasterGrid::ProjectionType interpolateSFs = interpolateSF ? getTypeOfSFInterpolation() : ccRasterGrid::INVALID_PROJECTION_TYPE;
	bool fillEmptyCells = (getFillEmptyCellsStrategy(fillEmptyCellsComboBox) == ccRasterGrid::INTERPOLATE);

	//cloud bounding-box --> grid size
	ccBBox box = getCustomBBox();
	if (!box.isValid())
	{
		return false;
	}

	//clear volume info
	{
		volumeLabel->setText("0");
		filledCellsPercentageLabel->setText("0 %");
	}

	unsigned gridWidth = 0, gridHeight = 0;
	if (!getGridSize(gridWidth, gridHeight))
	{
		return false;
	}

	//grid size
	unsigned gridTotalSize = gridWidth * gridHeight;
	if (gridTotalSize == 1)
	{
		if (QMessageBox::question(	this,
									"Unexpected grid size",
									"The generated grid will only have 1 cell! Do you want to proceed anyway?",
									QMessageBox::Yes,
									QMessageBox::No) == QMessageBox::No)
			return false;
	}
	else if (gridTotalSize > 10000000)
	{
		if (QMessageBox::question(	this,
									"Big grid size",
									"The generated grid will have more than 10.000.000 cells! Do you want to proceed anyway?",
									QMessageBox::Yes,
									QMessageBox::No) == QMessageBox::No)
			return false;
	}

	removeContourLines();

	//grid step
	double gridStep = getGridStep();
	assert(gridStep != 0);

	//memory allocation
	CCVector3d minCorner = CCVector3d::fromArray(box.minCorner().u);
	if (!m_grid.init(gridWidth, gridHeight, gridStep, minCorner))
	{
		//not enough memory
		ccLog::Error("Not enough memory");
		return false;
	}
	
	//vertical dimension
	const unsigned char Z = getProjectionDimension();
	assert(Z >= 0 && Z <= 2);

	ccProgressDialog pDlg(true, this);
	if (!m_grid.fillWith(	m_cloud,
							Z,
							projectionType,
							fillEmptyCells,
							interpolateSFs,
							&pDlg))
	{
		return false;
	}

	//update volume estimate
	{
		double hSum = 0;
		unsigned filledCellCount = 0;
		for (unsigned j = 0; j < m_grid.height; ++j)
		{
			const ccRasterGrid::Row& row = m_grid.rows[j];
			for (unsigned i = 0; i < m_grid.width; ++i)
			{
				if (std::isfinite(row[i].h))
				{
					hSum += row[i].h;
					++filledCellCount;
				}
			}
		}

		if (filledCellCount)
		{
			double cellArea = m_grid.gridStep * m_grid.gridStep;
			volumeLabel->setText(QString::number(hSum * cellArea));
			filledCellsPercentageLabel->setText(QString::number(static_cast<double>(100 * filledCellCount) / (m_grid.width * m_grid.height), 'f', 2) + " %");
		}

	}

	ccLog::Print(QString("[Rasterize] Current raster grid:\n\tSize: %1 x %2\n\tHeight values: [%3 ; %4]").arg(m_grid.width).arg(m_grid.height).arg(m_grid.minHeight).arg(m_grid.maxHeight));
	
	return true;
}

ccPointCloud* ccRasterizeTool::generateCloud(bool autoExport/*=true*/) const
{
	if (!m_cloud || !m_grid.isValid())
	{
		return 0;
	}

	//look for fields to be exported
	std::vector<ccRasterGrid::ExportableFields> exportedFields;
	try
	{
		exportedFields.push_back(ccRasterGrid::PER_CELL_HEIGHT);
		if (exportAsSF(ccRasterGrid::PER_CELL_COUNT))
			exportedFields.push_back(ccRasterGrid::PER_CELL_COUNT);
		if (exportAsSF(ccRasterGrid::PER_CELL_MIN_HEIGHT))
			exportedFields.push_back(ccRasterGrid::PER_CELL_MIN_HEIGHT);
		if (exportAsSF(ccRasterGrid::PER_CELL_MAX_HEIGHT))
			exportedFields.push_back(ccRasterGrid::PER_CELL_MAX_HEIGHT);
		if (exportAsSF(ccRasterGrid::PER_CELL_AVG_HEIGHT))
			exportedFields.push_back(ccRasterGrid::PER_CELL_AVG_HEIGHT);
		if (exportAsSF(ccRasterGrid::PER_CELL_HEIGHT_STD_DEV))
			exportedFields.push_back(ccRasterGrid::PER_CELL_HEIGHT_STD_DEV);
		if (exportAsSF(ccRasterGrid::PER_CELL_HEIGHT_RANGE))
			exportedFields.push_back(ccRasterGrid::PER_CELL_HEIGHT_RANGE);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory!");
		return 0;
	}
	QString activeLayerName = activeLayerComboBox->currentText();
	bool activeLayerIsSF = (activeLayerComboBox->currentData().toInt() == LAYER_SF);
	//bool activeLayerIsRGB = (activeLayerComboBox->currentData().toInt() == LAYER_RGB);
	ccPointCloud* rasterCloud = convertGridToCloud(	exportedFields,
													/*interpolateSF=*/(getTypeOfSFInterpolation() != ccRasterGrid::INVALID_PROJECTION_TYPE) || activeLayerIsSF,
													/*interpolateColors=*/true,
													/*copyHillshadeSF=*/true,
													activeLayerName,
													true);

	if (rasterCloud && autoExport)
	{
		if (m_cloud->getParent())
		{
			m_cloud->getParent()->addChild(rasterCloud);
		}
		rasterCloud->setDisplay(m_cloud->getDisplay());
		
		if (m_cloud->isEnabled())
		{
			m_cloud->setEnabled(false);
			ccLog::Warning("[Rasterize] Previously selected entity (source cloud) has been hidden!");
		}

		MainWindow* mainWindow = MainWindow::TheInstance();
		if (mainWindow)
		{
			mainWindow->addToDB(rasterCloud);
			ccLog::Print(QString("[Rasterize] Cloud '%1' successfully exported").arg(rasterCloud->getName()));
		}
		else
		{
			assert(false);
			delete rasterCloud;
		}
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
																								getProjectionDimension(),
																								errorStr);
		ccMesh* rasterMesh = 0;
		if (baseMesh)
		{
			rasterMesh = new ccMesh(baseMesh, rasterCloud);
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
			rasterMesh->showSF(rasterCloud->sfShown());
			rasterMesh->showColors(rasterCloud->colorsShown());

			MainWindow* mainWindow = MainWindow::TheInstance();
			if (mainWindow)
				MainWindow::TheInstance()->addToDB(rasterMesh);
			ccLog::Print(QString("[Rasterize] Mesh '%1' successfully exported").arg(rasterMesh->getName()));
		}
		else
		{
			ccLog::Error(QString("Failed to create mesh ('%1')").arg(errorStr));
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

class RasterExportOptionsDlg
	: public QDialog
	, public Ui::RasterExportOptionsDialog
{
public:

	explicit RasterExportOptionsDlg(QWidget* parent = 0)
		: QDialog(parent, Qt::Tool)
		, Ui::RasterExportOptionsDialog()
	{
		setupUi(this);
	}
};

#endif

void ccRasterizeTool::generateRaster() const
{
#ifdef CC_GDAL_SUPPORT

	if (!m_cloud || !m_grid.isValid())
	{
		return;
	}

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

	bool hasScalarFields = !m_grid.scalarFields.empty();
	int visibleSfIndex = -1;
	if (activeLayerComboBox->currentData().toInt() == LAYER_SF && m_cloud->isA(CC_TYPES::POINT_CLOUD))
	{
		//the indexes in the 'm_grid.scalarFields' are the same as in the cloud
		visibleSfIndex = static_cast<ccPointCloud*>(m_cloud)->getScalarFieldIndexByName(qPrintable(activeLayerComboBox->currentText()));
	}

	//which (and how many) bands shall we create?
	static bool heightBand = true; //height by default
	static bool rgbBand = false; //not a good idea to mix RGB and height values!
	static bool densityBand = false;
	static bool visibleSFBand = false;
	static bool allSFBands = false;

	RasterExportOptionsDlg reoDlg;
	reoDlg.dimensionsLabel->setText(QString("%1 x %2").arg(m_grid.width).arg(m_grid.height));
	reoDlg.exportRGBCheckBox->setEnabled(m_grid.hasColors);
	reoDlg.exportRGBCheckBox->setChecked(rgbBand);
	reoDlg.exportHeightsCheckBox->setChecked(heightBand);
	reoDlg.exportDensityCheckBox->setChecked(densityBand);
	reoDlg.exportActiveLayerCheckBox->setChecked(visibleSFBand);
	reoDlg.exportActiveLayerCheckBox->setEnabled(visibleSfIndex >= 0);
	reoDlg.exportAllSFCheckBox->setEnabled(hasScalarFields);
	reoDlg.exportAllSFCheckBox->setChecked(allSFBands);

	while (true)
	{
		if (!reoDlg.exec())
		{
			//cancelled by user
			return;
		}

		//check the selection
		if (	reoDlg.exportRGBCheckBox->isEnabled()
			&&	reoDlg.exportRGBCheckBox->isChecked()
			&& (	reoDlg.exportHeightsCheckBox->isChecked()
				||	reoDlg.exportDensityCheckBox->isChecked()
				||	reoDlg.exportActiveLayerCheckBox->isChecked()
				||	reoDlg.exportAllSFCheckBox->isChecked())
			)
		{
			if (QMessageBox::warning(	0,
										"Mixed raster",
										"Mixing colors and other layers will result in\na strange raster file with 64 bits color bands\n(some applications won't handle them properly)",
										QMessageBox::Ignore,
										QMessageBox::Retry) == QMessageBox::Ignore)
			{
				//the user ignored the warning
				break;
			}
		}
		else
		{
			//nothing to worry about :D
			break;
		}
	}

	//we ask the output filename AFTER displaying the export parameters ;)
	QString outputFilename;
	{
		QSettings settings;
		settings.beginGroup(ccPS::HeightGridGeneration());
		QString imageSavePath = settings.value("savePathImage", ccFileUtils::defaultDocPath()).toString();
		outputFilename = QFileDialog::getSaveFileName(	const_cast<ccRasterizeTool*>(this),
														"Save height grid raster",
														imageSavePath + QString("/raster.tif"),
														"geotiff (*.tif)");

		if (outputFilename.isNull())
		{
			return;
		}

		//save current export path to persistent settings
		settings.setValue("savePathImage", QFileInfo(outputFilename).absolutePath());
	}

	heightBand    = reoDlg.exportHeightsCheckBox->isChecked();
	rgbBand       = reoDlg.exportRGBCheckBox->isChecked();
	densityBand   = reoDlg.exportDensityCheckBox->isChecked();
	allSFBands    = reoDlg.exportAllSFCheckBox->isChecked();
	visibleSFBand = reoDlg.exportActiveLayerCheckBox->isChecked();

	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(fillEmptyCellsComboBox);

	int totalBands = 0;
	bool onlyRGBA = true;

	if (heightBand)
	{
		++totalBands;
		onlyRGBA = false;
	}
	
	bool rgbaMode = false;
	if (rgbBand)
	{
		totalBands += 3; //one per component
		if (fillEmptyCellsStrategy == ccRasterGrid::LEAVE_EMPTY && m_grid.validCellCount < m_grid.height * m_grid.width)
		{
			rgbaMode = true;
			++totalBands; //alpha
		}
	}
	else
	{
		onlyRGBA = false;
	}
	
	if (densityBand)
	{
		++totalBands;
		onlyRGBA = false;
	}
	
	if (allSFBands)
	{
		for (size_t i = 0; i < m_grid.scalarFields.size(); ++i)
		{
			if (!m_grid.scalarFields[i].empty())
			{
				++totalBands;
				onlyRGBA = false;
			}
		}
	}
	else if (visibleSFBand && visibleSfIndex >= 0)
	{
		++totalBands;
		onlyRGBA = false;
	}
	
	if (totalBands == 0)
	{
		ccLog::Error("Can't output a raster with no band! (check export parameters)");
		return;
	}

	char **papszOptions = NULL;
	GDALDataset* poDstDS = poDriver->Create(qPrintable(outputFilename),
											static_cast<int>(m_grid.width),
											static_cast<int>(m_grid.height),
											totalBands,
											onlyRGBA ? GDT_Byte : GDT_Float64,
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
	double shiftY = box.maxCorner().u[Y];

	double stepX = m_grid.gridStep;
	double stepY = m_grid.gridStep;
	if (m_cloud)
	{
		const CCVector3d& shift = m_cloud->getGlobalShift();
		shiftX -= shift.u[X];
		shiftY -= shift.u[Y];

		double scale = m_cloud->getGlobalScale();
		assert(scale != 0);
		stepX /= scale;
		stepY /= scale;
	}

	poDstDS->SetMetadataItem("AREA_OR_POINT", "AREA");

	double adfGeoTransform[6] = {	shiftX,		//top left x
									stepX,		//w-e pixel resolution (can be negative)
									0,			//0
									shiftY,		//top left y
									0,			//0
									-stepY		//n-s pixel resolution (can be negative)
	};

	poDstDS->SetGeoTransform( adfGeoTransform );

	//OGRSpatialReference oSRS;
	//oSRS.SetUTM( 11, TRUE );
	//oSRS.SetWellKnownGeogCS( "NAD27" );
	//char *pszSRS_WKT = NULL;
	//oSRS.exportToWkt( &pszSRS_WKT );
	//poDstDS->SetProjection( pszSRS_WKT );
	//CPLFree( pszSRS_WKT );

	int currentBand = 0;

	//exort RGB band?
	if (rgbBand)
	{
		GDALRasterBand* rgbBands[3] = { poDstDS->GetRasterBand(++currentBand),
										poDstDS->GetRasterBand(++currentBand),
										poDstDS->GetRasterBand(++currentBand) };
		rgbBands[0]->SetColorInterpretation(GCI_RedBand);
		rgbBands[1]->SetColorInterpretation(GCI_GreenBand);
		rgbBands[2]->SetColorInterpretation(GCI_BlueBand);

		unsigned char* cLine = (unsigned char*)CPLMalloc(sizeof(unsigned char)*m_grid.width);
		if (!cLine)
		{
			ccLog::Error("[GDAL] Not enough memory");
			GDALClose((GDALDatasetH)poDstDS);
			return;
		}

		bool error = false;
		
		//export the R, G and B components
		for (unsigned k = 0; k < 3; ++k)
		{
			rgbBands[k]->SetStatistics(0, 255, 128, 0); //warning: arbitrary average and std. dev. values

			for (unsigned j = 0; j<m_grid.height; ++j)
			{
				const ccRasterGrid::Row& row = m_grid.rows[m_grid.height - 1 - j]; //the first row is the northest one (i.e. Ymax)
				for (unsigned i = 0; i<m_grid.width; ++i)
				{
					cLine[i] = (std::isfinite(row[i].h) ? static_cast<unsigned char>(std::max(0.0, std::min(255.0, row[i].color.u[k]))) : 0);
				}

				if (rgbBands[k]->RasterIO(GF_Write, 0, static_cast<int>(j), static_cast<int>(m_grid.width), 1, cLine, static_cast<int>(m_grid.width), 1, GDT_Byte, 0, 0) != CE_None)
				{
					error = true;
					k = 3; //early stop
					break;
				}
			}
		}

		//export the alpha band (if necessary)
		if (!error && rgbaMode)
		{
			GDALRasterBand* aBand = poDstDS->GetRasterBand(++currentBand);
			aBand->SetColorInterpretation(GCI_AlphaBand);
			aBand->SetStatistics(0, 255, 255, 0); //warning: arbitrary average and std. dev. values

			for (unsigned j = 0; j<m_grid.height; ++j)
			{
				const ccRasterGrid::Row& row = m_grid.rows[m_grid.height - 1 - j];
				for (unsigned i = 0; i<m_grid.width; ++i)
				{
					cLine[i] = (std::isfinite(row[i].h) ? 255 : 0);
				}

				if (aBand->RasterIO(GF_Write, 0, static_cast<int>(j), static_cast<int>(m_grid.width), 1, cLine, static_cast<int>(m_grid.width), 1, GDT_Byte, 0, 0) != CE_None)
				{
					error = true;
					break;
				}
			}
		}

		CPLFree(cLine);

		if (error)
		{
			ccLog::Error("[GDAL] An error occurred while writing the color bands!");
			GDALClose((GDALDatasetH)poDstDS);
			return;
		}
	}

	double* scanline = (double*)CPLMalloc(sizeof(double)*m_grid.width);
	if (!scanline)
	{
		ccLog::Error("[GDAL] Not enough memory");
		GDALClose((GDALDatasetH)poDstDS);
		return;
	}

	//exort height band?
	if (heightBand)
	{
		GDALRasterBand* poBand = poDstDS->GetRasterBand(++currentBand);
		assert(poBand);
		poBand->SetColorInterpretation(GCI_Undefined);

		double emptyCellHeight = 0;
		switch (fillEmptyCellsStrategy)
		{
		case ccRasterGrid::LEAVE_EMPTY:
			emptyCellHeight = m_grid.minHeight - 1.0;
			poBand->SetNoDataValue(emptyCellHeight); //should be transparent!
			break;
		case ccRasterGrid::FILL_MINIMUM_HEIGHT:
			emptyCellHeight = m_grid.minHeight;
			break;
		case ccRasterGrid::FILL_MAXIMUM_HEIGHT:
			emptyCellHeight = m_grid.maxHeight;
			break;
		case ccRasterGrid::FILL_CUSTOM_HEIGHT:
		case ccRasterGrid::INTERPOLATE:
			emptyCellHeight = getCustomHeightForEmptyCells();
			break;
		case ccRasterGrid::FILL_AVERAGE_HEIGHT:
			emptyCellHeight = m_grid.meanHeight;
			break;
		default:
			assert(false);
		}

		for (unsigned j = 0; j < m_grid.height; ++j)
		{
			const ccRasterGrid::Row& row = m_grid.rows[m_grid.height - 1 - j];
			for (unsigned i = 0; i<m_grid.width; ++i)
			{
				scanline[i] = std::isfinite(row[i].h) ? row[i].h : emptyCellHeight;
			}

			if (poBand->RasterIO(GF_Write, 0, static_cast<int>(j), static_cast<int>(m_grid.width), 1, scanline, static_cast<int>(m_grid.width), 1, GDT_Float64, 0, 0) != CE_None)
			{
				ccLog::Error("[GDAL] An error occurred while writing the height band!");
				if (scanline)
					CPLFree(scanline);
				GDALClose((GDALDatasetH)poDstDS);
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
		for (unsigned j = 0; j < m_grid.height; ++j)
		{
			const ccRasterGrid::Row& row = m_grid.rows[m_grid.height - 1 - j];
			for (unsigned i = 0; i < m_grid.width; ++i)
			{
				scanline[i] = row[i].nbPoints;
			}

			if (poBand->RasterIO(GF_Write, 0, static_cast<int>(j), static_cast<int>(m_grid.width), 1, scanline, static_cast<int>(m_grid.width), 1, GDT_Float64, 0, 0) != CE_None)
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
	if (allSFBands || (visibleSFBand && visibleSfIndex >= 0))
	{
		for (size_t k = 0; k < m_grid.scalarFields.size(); ++k)
		{
			assert(!m_grid.scalarFields[k].empty());
			if (allSFBands || (visibleSFBand && visibleSfIndex == static_cast<int>(k)))
			{
				const double* sfGrid = &(m_grid.scalarFields[k].front());
				GDALRasterBand* poBand = poDstDS->GetRasterBand(++currentBand);

				double sfNanValue = std::numeric_limits<ccRasterGrid::SF::value_type>::quiet_NaN();
				poBand->SetNoDataValue(sfNanValue); //should be transparent!
				assert(poBand);
				poBand->SetColorInterpretation(GCI_Undefined);

				for (unsigned j = 0; j < m_grid.height; ++j)
				{
					const ccRasterGrid::Row& row = m_grid.rows[m_grid.height - 1 - j];
					const double* sfRow = sfGrid + (m_grid.height - 1 - j) * m_grid.width;
					for (unsigned i = 0; i < m_grid.width; ++i)
					{
						scanline[i] = row[i].nbPoints ? sfRow[i] : sfNanValue;
					}

					if (poBand->RasterIO(	GF_Write,
											0,
											static_cast<int>(j),
											static_cast<int>(m_grid.width),
											1,
											scanline,
											static_cast<int>(m_grid.width),
											1,
											GDT_Float64, 0, 0 ) != CE_None)
					{
						//the corresponding SF should exist on the input cloud
						ccLog::Error(QString("[GDAL] An error occurred while writing a scalar field band!"));
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

	ccLog::Print(QString("[Rasterize] Raster '%1' successfully saved").arg(outputFilename));

#else
	assert(false);
	ccLog::Error("[Rasterize] GDAL not supported by this version! Can't generate a raster...");
#endif
}

//See http://edndoc.esri.com/arcobjects/9.2/net/shared/geoprocessing/spatial_analyst_tools/how_hillshade_works.htm
void ccRasterizeTool::generateHillshade()
{
	if (!m_grid.isValid() || !m_rasterCloud)
	{
		ccLog::Error("Need a valid raster/cloud to compute contours!");
		return;
	}
	if (m_grid.height < 3 || m_grid.width < 3)
	{
		ccLog::Error("Grid is too small");
		return;
	}

	//get/create layer
	ccScalarField* hillshadeLayer = 0;
	int sfIdx = m_rasterCloud->getScalarFieldIndexByName(HILLSHADE_FIELD_NAME);
	if (sfIdx >= 0)
	{
		hillshadeLayer = static_cast<ccScalarField*>(m_rasterCloud->getScalarField(sfIdx));
	}
	else
	{
		hillshadeLayer = new ccScalarField(HILLSHADE_FIELD_NAME);
		if (!hillshadeLayer->reserve(m_rasterCloud->size()))
		{
			ccLog::Error("Not enough memory!");
			hillshadeLayer->release();
			return;
		}

		sfIdx = m_rasterCloud->addScalarField(hillshadeLayer);
		activeLayerComboBox->addItem(HILLSHADE_FIELD_NAME, QVariant(LAYER_SF));
		activeLayerComboBox->setEnabled(true);
	}
	assert(hillshadeLayer && hillshadeLayer->currentSize() == m_rasterCloud->size());
	hillshadeLayer->fill(NAN_VALUE);

	bool sparseSF = (hillshadeLayer->currentSize() != m_grid.height * m_grid.width);

	//now we can compute the hillshade
	int zenith_deg = sunZenithSpinBox->value();
	double zenith_rad = zenith_deg * CC_DEG_TO_RAD;

	double cos_zenith_rad = cos(zenith_rad);
	double sin_zenith_rad = sin(zenith_rad);

	int azimuth_deg = sunAzimuthSpinBox->value();
	int azimuth_math = 360 - azimuth_deg + 90;
	double azimuth_rad = azimuth_math * CC_DEG_TO_RAD;

	//for all cells
	unsigned validCellIndex = 0;
	for (unsigned j = (sparseSF ? 0 : 1); j < m_grid.height - 1; ++j)
	{
		const ccRasterGrid::Row& row = m_grid.rows[j];
		
		for (unsigned i=sparseSF ? 0 : 1; i<m_grid.width; ++i)
		{
			//valid height value
			if (std::isfinite(row[i].h))
			{
				if (i != 0 && i + 1 != m_grid.width && j != 0)
				{
					double dz_dx = 0.0;
					int dz_dx_count = 0;
					double dz_dy = 0.0;
					int dz_dy_count = 0;
				
					for (int di=-1; di<=1; ++di)
					{
						for (int dj=-1; dj<=1; ++dj)
						{
							const ccRasterCell& n = m_grid.rows[j - dj][i + di]; //-dj (instead of + dj) because we scan the grid in the reverse orientation! (from bottom to top)
							if (n.h == n.h)
							{
								if (di != 0)
								{
									int dx_weight = (dj == 0 ? 2 : 1);
									dz_dx += (di < 0 ? -1.0 : 1.0) * dx_weight * n.h;
									dz_dx_count += dx_weight;
								}

								if (dj != 0)
								{
									int dy_weight = (di == 0 ? 2 : 1);
									dz_dy += (dj < 0 ? -1.0 : 1.0) * dy_weight * n.h;
									dz_dy_count += dy_weight;
								}
							}
						}
					}

					//for now we only handle the cell that have 8 valid neighbors!
					if (	dz_dx_count == 8
						&&	dz_dy_count == 8)
					{
						dz_dx /= (8.0 * m_grid.gridStep);
						dz_dy /= (8.0 * m_grid.gridStep);

						 double slope_rad = atan( /*z_factor **/sqrt(dz_dx*dz_dx + dz_dy*dz_dy) );

						 double aspect_rad = 0;
						 static const double s_zero = 1.0e-8;
						 if (fabs(dz_dx) > s_zero)
						 {
							 aspect_rad = atan2(dz_dy, -dz_dx);
							 if (aspect_rad < 0)
							 {
								 aspect_rad += 2.0 * M_PI;
							 }
						 }
						 else // dz_dx == 0
						 {
							 if (dz_dy > s_zero)
							 {
								 aspect_rad = 0.5 * M_PI;
							 }
							 else if (dz_dy < s_zero)
							 {
								 aspect_rad = 1.5 * M_PI;
							 }
						 }

						 ScalarType hillshade = static_cast<ScalarType>(std::max(0.0, cos_zenith_rad * cos(slope_rad) + sin_zenith_rad * sin(slope_rad) * cos(azimuth_rad - aspect_rad)));
						 hillshadeLayer->setValue(sparseSF ? validCellIndex : i + j * m_grid.width, hillshade);
					}
				}
				++validCellIndex;
			}
		}
	}

	hillshadeLayer->computeMinAndMax();
	hillshadeLayer->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::GREY));
	m_rasterCloud->setCurrentDisplayedScalarField(sfIdx);
	m_rasterCloud->showSF(true);
	activeLayerComboBox->setCurrentIndex(activeLayerComboBox->findText(HILLSHADE_FIELD_NAME));

	if (m_glWindow)
	{
		m_glWindow->redraw();
	}
}

void ccRasterizeTool::generateContours()
{
	if (!m_grid.isValid() || !m_rasterCloud)
	{
		ccLog::Error("Need a valid raster/cloud to compute contours!");
		return;
	}

	bool projectContourOnAltitudes = false;
	switch (activeLayerComboBox->currentData().toInt())
	{
	case LAYER_HEIGHT:
	{
		//nothing to do
		break;
	}
	case LAYER_RGB:
	{
		ccLog::Error("Can't generate contours from RGB colors");
		return;
	}
	default:
	{
		projectContourOnAltitudes = projectContoursOnAltCheckBox->isChecked();
		break;
	}
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
	unsigned levelCount = 1 + static_cast<unsigned>(floor((activeLayer->getMax() - startValue) / step));

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
		if (m_glWindow)
			m_glWindow->redraw();
		return;
	}

	//fill grid
	{
		bool sparseLayer = (activeLayer->currentSize() != m_grid.height * m_grid.width);
		double emptyCellsValue = activeLayer->getMin()-1.0;

		unsigned layerIndex = 0;
		for (unsigned j = 0; j < m_grid.height; ++j)
		{
			ccRasterGrid::Row& cellRow = m_grid.rows[j];
			double* row = &(grid[(j + margin)*xDim + margin]);
			for (unsigned i = 0; i < m_grid.width; ++i)
			{
				if (cellRow[i].nbPoints || !sparseLayer)
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
		Isolines<double> iso(static_cast<int>(xDim), static_cast<int>(yDim));
		if (!ignoreBorders)
		{
			iso.createOnePixelBorder(&(grid.front()), activeLayer->getMin() - 1.0);
		}
		//bounding box
		ccBBox box = getCustomBBox();
		assert(box.isValid());

		//vertical dimension
		const unsigned char Z = getProjectionDimension();
		assert(Z >= 0 && Z <= 2);
		const unsigned char X = (Z == 2 ? 0 : Z + 1);
		const unsigned char Y = (X == 2 ? 0 : X + 1);

		int minVertexCount = minVertexCountSpinBox->value();
		assert(minVertexCount >= 3);

		ccProgressDialog pDlg(true,this);
		pDlg.setMethodTitle(tr("Contour plot"));
		pDlg.setInfo(tr("Levels: %1\nCells: %2 x %3").arg(levelCount).arg(m_grid.width).arg(m_grid.height));
		pDlg.start();
		pDlg.show();
		QApplication::processEvents();
		CCLib::NormalizedProgress nProgress(&pDlg, levelCount);

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
			for (int i = 0; i < lineCount; ++i)
			{
				int vertCount = iso.getContourLength(i);
				if (vertCount >= minVertexCount)
				{
					int startVi = 0; //we may have to split the polyline in multiple chunks
					while (startVi < vertCount)
					{
						ccPointCloud* vertices = new ccPointCloud("vertices");
						ccPolyline* poly = new ccPolyline(vertices);
						poly->addChild(vertices);
						bool isClosed = (startVi == 0 ? iso.isContourClosed(i) : false);
						if (poly->reserve(vertCount - startVi) && vertices->reserve(vertCount - startVi))
						{
							unsigned localIndex = 0;
							for (int vi = startVi; vi < vertCount; ++vi)
							{
								++startVi;
								
								double x = iso.getContourX(i, vi) - margin;
								double y = iso.getContourY(i, vi) - margin;

								CCVector3 P;
								//DGM: we will only do the dimension mapping at export time
								//(otherwise the contour lines appear in the wrong orientation compared to the grid/raster which
								// is in the XY plane by default!)
								/*P.u[X] = */P.x = static_cast<PointCoordinateType>((x + 0.5) * m_grid.gridStep + box.minCorner().u[X]);
								/*P.u[Y] = */P.y = static_cast<PointCoordinateType>((y + 0.5) * m_grid.gridStep + box.minCorner().u[Y]);
								if (projectContourOnAltitudes)
								{
									int xi = std::min(std::max(static_cast<int>(x), 0), static_cast<int>(m_grid.width) - 1);
									int yi = std::min(std::max(static_cast<int>(y), 0), static_cast<int>(m_grid.height) - 1);
									double h = m_grid.rows[y][x].h;
									if (std::isfinite(h))
									{
										/*P.u[Z] = */P.z = static_cast<PointCoordinateType>(h);
									}
									else
									{
										//DGM: we stop the current polyline
										isClosed = false;
										break;
									}
								}
								else
								{
									/*P.u[Z] = */P.z = static_cast<PointCoordinateType>(v);
								}

								vertices->addPoint(P);
								assert(localIndex < vertices->size());
								poly->addPointIndex(localIndex++);
							}

							assert(poly);
							if (poly->size() > 1)
							{
								poly->setName(QString("Contour line value = %1 (#%2)").arg(v).arg(++realCount));
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
								poly->setMetaData(ccPolyline::MetaKeyConstAltitude(), QVariant(v));

								if (m_glWindow)
									m_glWindow->addToOwnDB(poly);

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

	if (m_glWindow)
	{
		m_glWindow->redraw();
	}
}

void ccRasterizeTool::exportContourLines()
{
	MainWindow* mainWindow = MainWindow::TheInstance();
	if (!mainWindow || !m_cloud || m_contourLines.empty())
	{
		assert(false);
		return;
	}

	bool colorize = colorizeContoursCheckBox->isChecked();

	//vertical dimension
	const unsigned char Z = getProjectionDimension();
	assert(Z >= 0 && Z <= 2);
	const unsigned char X = (Z == 2 ? 0 : Z + 1);
	const unsigned char Y = (X == 2 ? 0 : X + 1);

	ccHObject* group = new ccHObject(QString("Contour plot(%1) [step=%2]").arg(m_cloud->getName()).arg(contourStepDoubleSpinBox->value()));
	for (size_t i = 0; i < m_contourLines.size(); ++i)
	{
		ccPolyline* poly = m_contourLines[i];

		//now is the time to map the polyline coordinates to the right dimensions!
		ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(poly->getAssociatedCloud());
		assert(vertices);
		if (vertices && Z != 2)
		{
			for (unsigned j = 0; j < vertices->size(); ++j)
			{
				CCVector3* P = const_cast<CCVector3*>(vertices->getPoint(j));
				CCVector3 Q = *P;
				P->u[X] = Q.x;
				P->u[Y] = Q.y;
				P->u[Z] = Q.z;
			}
			vertices->invalidateBoundingBox();
			poly->invalidateBoundingBox();
		}

		if (!colorize)
			poly->showColors(false);
		group->addChild(poly);
		if (m_glWindow)
			m_glWindow->removeFromOwnDB(poly);
	}
	m_contourLines.clear();
	exportContoursPushButton->setEnabled(false);

	group->setDisplay_recursive(m_cloud->getDisplay());
	mainWindow->addToDB(group);

	ccLog::Print(QString("Contour lines have been successfully exported to DB (group name: %1)").arg(group->getName()));
}

ccRasterGrid::EmptyCellFillOption ccRasterizeTool::getFillEmptyCellsStrategyExt(double& emptyCellsHeight,
																				double& minHeight,
																				double& maxHeight) const
{
	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(fillEmptyCellsComboBox);

	emptyCellsHeight = 0.0;
	minHeight = m_grid.minHeight;
	maxHeight = m_grid.maxHeight;
	
	switch (fillEmptyCellsStrategy)
	{
	case ccRasterGrid::LEAVE_EMPTY:
		//nothing to do
		break;
	case ccRasterGrid::FILL_MINIMUM_HEIGHT:
		emptyCellsHeight = m_grid.minHeight;
		break;
	case ccRasterGrid::FILL_MAXIMUM_HEIGHT:
		emptyCellsHeight = m_grid.maxHeight;
		break;
	case ccRasterGrid::FILL_CUSTOM_HEIGHT:
	case ccRasterGrid::INTERPOLATE:
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
	case ccRasterGrid::FILL_AVERAGE_HEIGHT:
		//'average height' is a kind of 'custom height' so we can fall back to this mode!
		fillEmptyCellsStrategy = ccRasterGrid::FILL_CUSTOM_HEIGHT;
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
	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategyExt(	emptyCellsHeight,
																				minHeight,
																				maxHeight);

	QImage bitmap8(m_grid.width, m_grid.height, QImage::Format_Indexed8);
	if (!bitmap8.isNull())
	{
		bool addTransparentColor = (fillEmptyCellsStrategy == ccRasterGrid::LEAVE_EMPTY);

		//build a custom palette
		QVector<QRgb> palette(256);
		if (	m_rasterCloud
			&&	m_rasterCloud->getCurrentDisplayedScalarField()
			&&	m_rasterCloud->getCurrentDisplayedScalarField()->getColorScale())
		{
			const ccColorScale::Shared& colorScale = m_rasterCloud->getCurrentDisplayedScalarField()->getColorScale();
			unsigned steps = (addTransparentColor ? 255 : 256);
			for (unsigned i = 0; i < steps; i++)
			{
				const ColorCompType* col = colorScale->getColorByRelativePos(i / static_cast<double>(steps - 1), steps, ccColor::lightGrey.rgba);
				palette[i] = qRgba(col[0], col[1], col[2], 255);
			}
		}
		else
		{
			for (unsigned i = 0; i < 256; i++)
			{
				palette[i] = qRgba(i, i, i, 255);
			}
		}
		
		double maxColorComp = 255.99; //.99 --> to avoid round-off issues later!
		if (addTransparentColor)
		{
			palette[255] = qRgba(255, 0, 255, 0); //magenta/transparent color for empty cells (in place of pure white)
			maxColorComp = 254.99;
		}

		bitmap8.setColorTable(palette);
		//bitmap8.fill(255);

		unsigned emptyCellColorIndex = 0;
		switch (fillEmptyCellsStrategy)
		{
		case ccRasterGrid::LEAVE_EMPTY:
			emptyCellColorIndex = 255; //should be transparent!
			break;
		case ccRasterGrid::FILL_MINIMUM_HEIGHT:
			emptyCellColorIndex = 0;
			break;
		case ccRasterGrid::FILL_MAXIMUM_HEIGHT:
			emptyCellColorIndex = 255;
			break;
		case ccRasterGrid::FILL_CUSTOM_HEIGHT:
			{
				double normalizedHeight = (emptyCellsHeight - minHeight) / (maxHeight - minHeight);
				//min and max should have already been updated with custom empty cell height!
				assert(normalizedHeight >= 0.0 && normalizedHeight <= 1.0);
				emptyCellColorIndex = static_cast<unsigned>(floor(normalizedHeight*maxColorComp));
			}
			break;
		case ccRasterGrid::FILL_AVERAGE_HEIGHT:
		default:
			assert(false);
		}

		double range = maxHeight - minHeight;
		if (range < ZERO_TOLERANCE)
		{
			range = 1.0;
		}

		// Filling the image with grid values
		for (unsigned j = 0; j < m_grid.height; ++j)
		{
			const ccRasterGrid::Row& row = m_grid.rows[j];
			for (unsigned i = 0; i < m_grid.width; ++i)
			{
				if (std::isfinite(row[i].h))
				{
					double normalizedHeight = (row[i].h - minHeight) / range;
					assert(normalizedHeight >= 0.0 && normalizedHeight <= 1.0);
					unsigned char val = static_cast<unsigned char>(floor(normalizedHeight*maxColorComp));
					bitmap8.setPixel(i, m_grid.height - 1 - j, val);
				}
				else //NaN
				{
					bitmap8.setPixel(i, m_grid.height - 1 - j, emptyCellColorIndex);
				}
			}
		}

		//open file saving dialog
		{
			QSettings settings;
			settings.beginGroup(ccPS::HeightGridGeneration());
			QString imageSavePath = settings.value("savePathImage", ccFileUtils::defaultDocPath()).toString();

			QString outputFilename = ImageFileFilter::GetSaveFilename(	"Save raster image",
																		"raster_image",
																		imageSavePath,
																		const_cast<ccRasterizeTool*>(this));

			if (!outputFilename.isNull())
			{
				//save current export path to persistent settings
				settings.setValue("savePathImage", QFileInfo(outputFilename).absolutePath());
				settings.endGroup();

				if (bitmap8.save(outputFilename))
				{
					ccLog::Print(QString("[Rasterize] Image '%1' successfully saved").arg(outputFilename));
				}
				else
				{
					ccLog::Error("Failed to save image file!");
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
	QString asciiGridSavePath = settings.value("savePathASCIIGrid", ccFileUtils::defaultDocPath()).toString();

	//open file saving dialog
	QString filter("ASCII file (*.txt)");
	QString outputFilename = QFileDialog::getSaveFileName(0, "Save grid as ASCII file", asciiGridSavePath + QString("/raster_matrix.txt"), filter);
	if (outputFilename.isNull())
		return;

	FILE* pFile = fopen(qPrintable(outputFilename), "wt");
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
	for (unsigned j = 0; j < m_grid.height; ++j)
	{
		const ccRasterGrid::Row& row = m_grid.rows[m_grid.height - 1 - j];
		for (unsigned i = 0; i < m_grid.width; ++i)
		{
			fprintf(pFile, "%.8f ", std::isfinite(row[i].h) ? row[i].h : emptyCellsHeight);
		}

		fprintf(pFile, "\n");
	}

	fclose(pFile);
	pFile = 0;

	//save current export path to persistent settings
	settings.setValue("savePathASCIIGrid", QFileInfo(outputFilename).absolutePath());

	ccLog::Print(QString("[Rasterize] Raster matrix '%1' successfully saved").arg(outputFilename));
}

