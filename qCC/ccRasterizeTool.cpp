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
#include "ui_rasterizeDlg.h"

//Local
#include "ccCommon.h"
#include "ccPersistentSettings.h"
#include "ccContourLinesGenerator.h"
#include "mainwindow.h"
#include "ccKrigingParamsDialog.h"

//qCC_db
#include <ccColorScalesManager.h>
#include <ccFileUtils.h>
#include <ccGenericPointCloud.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

//qCC_gl
#include <ccGLWindowInterface.h>

//qCC_io
#include <ImageFileFilter.h>

//Qt
#include <QFileDialog>
#include <QMap>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QStandardItemModel>
#include <QInputDialog>

#ifdef CC_GDAL_SUPPORT
//GDAL
#include <cpl_string.h>
#include <gdal.h>
#include <gdal_priv.h>
#include <ogr_api.h>
//local
#include "ui_rasterExportOptionsDlg.h"
#endif

//System
#include <cassert>

constexpr char HILLSHADE_FIELD_NAME[] = "Hillshade";

static void MakeComboBoxOptionInaccessible(QComboBox* comboBox, int index)
{
	if (!comboBox)
	{
		assert(false);
		return;
	}
	
	const QStandardItemModel* model = qobject_cast<const QStandardItemModel*>(comboBox->model());
	QStandardItem* item = model ? model->item(index) : 0;
	if (item)
	{
		item->setFlags(item->flags() & ~(Qt::ItemIsSelectable | Qt::ItemIsEnabled));
		// visually disable by greying out - works only if combobox has been painted already and palette returns the wanted color
		item->setData(comboBox->palette().color(QPalette::Disabled, QPalette::Text), Qt::TextColorRole); // clear item data in order to use default color
	}
}

ccRasterizeTool::ccRasterizeTool(ccGenericPointCloud* cloud, QWidget* parent)
	: QDialog(parent, Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint)
	, cc2Point5DimEditor()
	, m_UI( new Ui::RasterizeToolDialog )
	, m_cloud(cloud)
	, m_cloudHasScalarFields(false)
{
	m_UI->setupUi(this);

#ifdef CC_GDAL_SUPPORT
	m_UI->ignoreContourBordersCheckBox->setVisible(false);
#else
	m_UI->generateRasterPushButton->setDisabled(true);
	m_UI->generateRasterPushButton->setChecked(false);
#endif

	//custom bbox editor (needs to be setup first)
	ccBBox gridBBox = m_cloud ? m_cloud->getOwnBB() : ccBBox();
	if (gridBBox.isValid())
	{
		createBoundingBoxEditor(gridBBox, this);
		connect(m_UI->editGridToolButton, &QAbstractButton::clicked, this, &ccRasterizeTool::showGridBoxEditor);
	}
	else
	{
		m_UI->editGridToolButton->setEnabled(false);
	}

	//force update
	resampleOptionToggled(m_UI->resampleCloudCheckBox->isChecked());
	fillEmptyCellStrategyChanged(0);

	connect(m_UI->buttonBox,					&QDialogButtonBox::accepted,						this,	&ccRasterizeTool::testAndAccept);
	connect(m_UI->buttonBox,					&QDialogButtonBox::rejected,						this,	&ccRasterizeTool::testAndReject);
	
	connect(m_UI->gridStepDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccRasterizeTool::updateGridInfo);
	connect(m_UI->gridStepDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccRasterizeTool::gridOptionChanged);
	connect(m_UI->emptyValueDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this,	&ccRasterizeTool::gridOptionChanged);

	connect(m_UI->dimensionComboBox,			qOverload<int>(&QComboBox::currentIndexChanged),	this,	&ccRasterizeTool::projectionDirChanged);
	connect(m_UI->heightProjectionComboBox,		qOverload<int>(&QComboBox::currentIndexChanged),	this,	&ccRasterizeTool::projectionTypeChanged);
	connect(m_UI->scalarFieldProjection,		qOverload<int>(&QComboBox::currentIndexChanged),	this,	&ccRasterizeTool::sfProjectionTypeChanged);
	connect(m_UI->fillEmptyCellsComboBox,		qOverload<int>(&QComboBox::currentIndexChanged),	this,	&ccRasterizeTool::fillEmptyCellStrategyChanged);
	connect(m_UI->stdDevLayerComboBox,			qOverload<int>(&QComboBox::currentIndexChanged),	this,	&ccRasterizeTool::stdDevLayerChanged);
	connect(m_UI->activeLayerComboBox,			qOverload<int>(&QComboBox::currentIndexChanged),	this,	[this] (int index) { activeLayerChanged( index ); } );

	connect(m_UI->resampleCloudCheckBox,		&QAbstractButton::toggled,							this,	&ccRasterizeTool::resampleOptionToggled);
	connect(m_UI->updateGridPushButton,			&QAbstractButton::clicked,							this,	&ccRasterizeTool::updateGridAndDisplay);
	connect(m_UI->generateCloudPushButton,		&QAbstractButton::clicked,							this,	[this] () { generateCloud(true); } );
	connect(m_UI->generateImagePushButton,		&QAbstractButton::clicked,							this,	&ccRasterizeTool::generateImage);
	connect(m_UI->generateRasterPushButton,		&QAbstractButton::clicked,							this,	&ccRasterizeTool::generateRaster);
	connect(m_UI->generateASCIIPushButton,		&QAbstractButton::clicked,							this,	&ccRasterizeTool::generateASCIIMatrix);
	connect(m_UI->generateMeshPushButton,		&QAbstractButton::clicked,							this,	&ccRasterizeTool::generateMesh);
	connect(m_UI->generateContoursPushButton,	&QAbstractButton::clicked,							this,	&ccRasterizeTool::generateContours);
	connect(m_UI->exportContoursPushButton,		&QAbstractButton::clicked,							this,	&ccRasterizeTool::exportContourLines);
	connect(m_UI->clearContoursPushButton,		&QAbstractButton::clicked,							this,	&ccRasterizeTool::removeContourLines);
	connect(m_UI->generateHillshadePushButton,	&QAbstractButton::clicked,							this,	&ccRasterizeTool::generateHillshade);
	connect(m_UI->interpParamsToolButton,		&QAbstractButton::clicked,							this,	&ccRasterizeTool::showInterpolationParamsDialog);

	connect(m_UI->exportHeightStatsCheckBox,	&QCheckBox::toggled,								this,	&ccRasterizeTool::onStatExportTargetChanged);
	connect(m_UI->exportSFStatsCheckBox,		&QCheckBox::toggled,								this,	&ccRasterizeTool::onStatExportTargetChanged);

	if (m_cloud)
	{
		//populate layer box
		m_UI->activeLayerComboBox->addItem(ccRasterGrid::GetDefaultFieldName(ccRasterGrid::PER_CELL_VALUE), QVariant(LAYER_HEIGHT));
		if (m_cloud->hasColors())
		{
			m_UI->activeLayerComboBox->addItem("RGB", QVariant(LAYER_RGB));
		}

		if (cloud->isA(CC_TYPES::POINT_CLOUD) && cloud->hasScalarFields())
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
			for (unsigned i = 0; i < pc->getNumberOfScalarFields(); ++i)
			{
				m_UI->activeLayerComboBox->addItem(QString::fromStdString(pc->getScalarField(i)->getName()), QVariant(LAYER_SF));
				//populate std. dev. layer box as well
				m_UI->stdDevLayerComboBox->addItem(QString::fromStdString(pc->getScalarField(i)->getName()));
			}
			m_cloudHasScalarFields = true;
		}
		
		//add window
		create2DView(m_UI->mapFrame);
	}

	if (!m_cloudHasScalarFields)
	{
		m_UI->projectSFCheckBox->setChecked(false);
		MakeComboBoxOptionInaccessible(m_UI->heightProjectionComboBox, ccRasterGrid::PROJ_INVERSE_VAR_VALUE);
		MakeComboBoxOptionInaccessible(m_UI->scalarFieldProjection, ccRasterGrid::PROJ_INVERSE_VAR_VALUE);
	}

	m_UI->projectSFCheckBox->setEnabled(m_cloudHasScalarFields);
	m_UI->scalarFieldProjection->setEnabled(m_cloudHasScalarFields);
	m_UI->stdDevLayerComboBox->setEnabled(m_cloudHasScalarFields); // real state will be set later (--> updateStdDevLayerComboBox)
	m_UI->exportSFStatsCheckBox->setEnabled(m_cloudHasScalarFields);

	loadSettings();

	updateStdDevLayerComboBox();

	gridIsUpToDate(false); // will call updateGridInfo

	resize(minimumSize());
}

ccRasterizeTool::~ccRasterizeTool()
{
	removeContourLines();
	
	delete m_UI;
	m_UI = nullptr;
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

	m_UI->exportContoursPushButton->setEnabled(false);
	m_UI->clearContoursPushButton->setEnabled(false);

	if (m_glWindow)
		m_glWindow->redraw();
}

bool ccRasterizeTool::showGridBoxEditor()
{
	return cc2Point5DimEditor::showGridBoxEditor();
}

void ccRasterizeTool::updateCloudName(bool withNonEmptyCells)
{
	QString str;
	if (m_cloud)
	{
		str = QString("<b>%1</b> (%2 points").arg(m_cloud->getName(), QLocale::system().toString(m_cloud->size()));
		if (withNonEmptyCells)
			str += QString(" - %1 non-empty cells)").arg(QLocale::system().toString(m_grid.validCellCount));
		else
			str += ')';
	}
	else
	{
		str = "No cloud loaded";
	}

	m_UI->cloudNameLabel->setText(str);
}

void ccRasterizeTool::updateGridInfo(bool withNonEmptyCells/*=false*/)
{
	m_UI->gridWidthLabel->setText(getGridSizeAsString());
	
	updateCloudName(withNonEmptyCells);
}

double ccRasterizeTool::getGridStep() const
{
	return m_UI->gridStepDoubleSpinBox->value();
}

void ccRasterizeTool::getExportedStats(std::vector<ccRasterGrid::ExportableFields>& stats) const
{
	stats.clear();

	stats.push_back(ccRasterGrid::PER_CELL_VALUE);
	if (m_UI->generateStatisticsPopulationCheckBox->isChecked())
		stats.push_back(ccRasterGrid::PER_CELL_COUNT);
	if (m_UI->generateStatisticsMinCheckBox->isChecked())
		stats.push_back(ccRasterGrid::PER_CELL_MIN_VALUE);
	if (m_UI->generateStatisticsMaxCheckBox->isChecked())
		stats.push_back(ccRasterGrid::PER_CELL_MAX_VALUE);
	if (m_UI->generateStatisticsAverageCheckBox->isChecked())
		stats.push_back(ccRasterGrid::PER_CELL_AVG_VALUE);
	if (m_UI->generateStatisticsStdDevCheckBox->isChecked())
		stats.push_back(ccRasterGrid::PER_CELL_VALUE_STD_DEV);
	if (m_UI->generateStatisticsRangeCheckBox->isChecked())
		stats.push_back(ccRasterGrid::PER_CELL_VALUE_RANGE);
	if (m_UI->generateStatisticsMedianCheckBox->isChecked())
		stats.push_back(ccRasterGrid::PER_CELL_MEDIAN_VALUE);
	if (m_UI->generateStatisticsUniqueCheckBox->isChecked())
		stats.push_back(ccRasterGrid::PER_CELL_UNIQUE_COUNT_VALUE);
	if (m_UI->generateStatisticsPercentileCheckBox->isChecked())
		stats.push_back(ccRasterGrid::PER_CELL_PERCENTILE_VALUE);
}

bool ccRasterizeTool::resampleOriginalCloud() const
{
	return m_UI->resampleCloudCheckBox->isEnabled() && m_UI->resampleCloudCheckBox->isChecked();
}

unsigned char ccRasterizeTool::getProjectionDimension() const
{
	int dim = m_UI->dimensionComboBox->currentIndex();
	assert(dim >= 0 && dim < 3);

	return static_cast<unsigned char>(dim);
}

int ccRasterizeTool::getStdDevLayerIndex() const
{
	return  m_UI->stdDevLayerComboBox->currentIndex();
}

void ccRasterizeTool::resampleOptionToggled(bool state)
{
	m_UI->warningResampleWithAverageLabel->setVisible(m_UI->resampleCloudCheckBox->isChecked()
		&& (	getTypeOfProjection() == ccRasterGrid::PROJ_AVERAGE_VALUE
			||	getTypeOfProjection() == ccRasterGrid::PROJ_INVERSE_VAR_VALUE)
	);
	gridOptionChanged();
}

void ccRasterizeTool::updateStdDevLayerComboBox()
{
	m_UI->stdDevLayerComboBox->setEnabled(	m_UI->stdDevLayerComboBox->count() != 0
										&& (m_UI->heightProjectionComboBox->currentIndex() == ccRasterGrid::PROJ_INVERSE_VAR_VALUE
										||	m_UI->scalarFieldProjection->currentIndex() == ccRasterGrid::PROJ_INVERSE_VAR_VALUE) );
}

void ccRasterizeTool::projectionTypeChanged(int index)
{
	//we can't use the 'resample origin cloud' option with 'average height' projection
	//resampleCloudCheckBox->setEnabled(index != PROJ_AVERAGE_VALUE && index != PROJ_INVERSE_VAR_VALUE);
	//DGM: now we can! We simply display a warning message
	m_UI->warningResampleWithAverageLabel->setVisible
	(
		m_UI->resampleCloudCheckBox->isChecked()
		&& (	index == ccRasterGrid::PROJ_AVERAGE_VALUE
			||	index == ccRasterGrid::PROJ_INVERSE_VAR_VALUE)
	);

	updateStdDevLayerComboBox();

	gridIsUpToDate(false);
}

void ccRasterizeTool::sfProjectionTypeChanged(int index)
{
	updateStdDevLayerComboBox();

	gridIsUpToDate(false);
}

void ccRasterizeTool::projectionDirChanged(int dir)
{
	gridIsUpToDate(false); // will call updateGridInfo
}

void ccRasterizeTool::stdDevLayerChanged(int index)
{
	gridIsUpToDate(false);
}

void ccRasterizeTool::activeLayerChanged(int layerIndex, bool autoRedraw/*=true*/)
{
	if (m_UI->activeLayerComboBox->itemData(layerIndex).toInt() == LAYER_SF)
	{
		if (m_UI->activeLayerComboBox->itemText(layerIndex) != HILLSHADE_FIELD_NAME)
		{
			m_UI->projectSFCheckBox->setChecked(true); //force the choice of a SF projection strategy
			m_UI->projectSFCheckBox->setEnabled(false);
			m_UI->generateImagePushButton->setEnabled(true);
			m_UI->generateASCIIPushButton->setEnabled(false);
			m_UI->projectContoursOnAltCheckBox->setEnabled(true);
		}
		else
		{
			//m_UI->projectSFCheckBox->setChecked(false); //we shouldn't change that as it only impacts the other fields
			//m_UI->projectSFCheckBox->setEnabled(false);
			m_UI->generateImagePushButton->setEnabled(false);
			m_UI->generateASCIIPushButton->setEnabled(false);
			m_UI->projectContoursOnAltCheckBox->setEnabled(false);
		}
	}
	else
	{
		//m_UI->projectSFCheckBox->setChecked(false); //DGM: we can't force that, just let the user decide
		m_UI->projectSFCheckBox->setEnabled(m_cloudHasScalarFields); //we need SF fields!
		m_UI->generateImagePushButton->setEnabled(true);
		m_UI->generateASCIIPushButton->setEnabled(true);
		m_UI->projectContoursOnAltCheckBox->setEnabled(false);
	}

	if (m_rasterCloud)
	{
		//active layer = RGB colors
		if (m_UI->activeLayerComboBox->currentData().toInt() == LAYER_RGB)
		{
			if (!m_rasterCloud->hasColors())
			{
				gridIsUpToDate(false);
			}
			m_UI->gridLayerRangeLabel->setText("[0 ; 255]");

			m_rasterCloud->showColors(true);
			m_rasterCloud->showSF(false);
		}
		else
		{
			//does the selected 'layer' exist?
			int sfIndex = m_rasterCloud->getScalarFieldIndexByName(m_UI->activeLayerComboBox->itemText(layerIndex).toStdString());
			m_rasterCloud->setCurrentDisplayedScalarField(sfIndex);
			m_rasterCloud->showSF(true);
			m_rasterCloud->showColors(false);

			if (sfIndex >= 0)
			{
				ccScalarField* activeLayer = m_rasterCloud->getCurrentDisplayedScalarField();
				if (activeLayer)
				{
					const ccScalarField::Range& layerValues = activeLayer->displayRange();
					m_UI->gridLayerRangeLabel->setText(QString("%1 [%2 ; %3]").arg(layerValues.range()).arg(layerValues.min()).arg(layerValues.max()));
					m_UI->contourStartDoubleSpinBox->setValue(layerValues.min());
					m_UI->contourStepDoubleSpinBox->setValue(layerValues.range() / 10.0);
				}
				else
				{
					assert(false);
					m_UI->gridLayerRangeLabel->setText("no active layer?!");
				}
			}
			else
			{
				m_UI->gridLayerRangeLabel->setText("Layer not computed");
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
	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(m_UI->fillEmptyCellsComboBox);

	// empty cell value
	{
		bool active = (fillEmptyCellsStrategy == ccRasterGrid::FILL_CUSTOM_HEIGHT) || (fillEmptyCellsStrategy == ccRasterGrid::INTERPOLATE_DELAUNAY);
		m_UI->emptyValueDoubleSpinBox->setEnabled(active);
		m_UI->emptyValueDoubleSpinBox->setVisible(active);
	}

	// interpolation parameters button
	{
		m_UI->interpParamsToolButton->setEnabled(fillEmptyCellsStrategy == ccRasterGrid::INTERPOLATE_DELAUNAY || fillEmptyCellsStrategy == ccRasterGrid::KRIGING);
	}

	gridIsUpToDate(false);
}

void ccRasterizeTool::gridOptionChanged()
{
	gridIsUpToDate(false);
}

double ccRasterizeTool::getCustomHeightForEmptyCells() const
{
	return m_UI->emptyValueDoubleSpinBox->value();
}

double ccRasterizeTool::getStatisticsPercentileValue() const
{
	return m_UI->generateStatisticsPercentileDoubleSpinBox->value();
}

ccRasterGrid::ProjectionType ccRasterizeTool::getTypeOfProjection() const
{
	switch (m_UI->heightProjectionComboBox->currentIndex())
	{
	case 0:
		return ccRasterGrid::PROJ_MINIMUM_VALUE;
	case 1:
		return ccRasterGrid::PROJ_AVERAGE_VALUE;
	case 2:
		return ccRasterGrid::PROJ_MAXIMUM_VALUE;
	case 3:
		return ccRasterGrid::PROJ_MEDIAN_VALUE;
	case 4:
		return ccRasterGrid::PROJ_INVERSE_VAR_VALUE;
	default:
		//shouldn't be possible for this option!
		assert(false);
	}

	return ccRasterGrid::INVALID_PROJECTION_TYPE;
}

ccRasterGrid::ProjectionType ccRasterizeTool::getTypeOfSFProjection() const
{
	if (/*!m_UI->projectSFCheckBox->isEnabled() || */!m_UI->projectSFCheckBox->isChecked()) //DGM: the check-box might be disabled to actually 'force' the user to choose a projection type
	{
		return ccRasterGrid::INVALID_PROJECTION_TYPE; //means that we don't want to project SF values
	}

	switch (m_UI->scalarFieldProjection->currentIndex())
	{
	case 0:
		return ccRasterGrid::PROJ_MINIMUM_VALUE;
	case 1:
		return ccRasterGrid::PROJ_AVERAGE_VALUE;
	case 2:
		return ccRasterGrid::PROJ_MAXIMUM_VALUE;
	case 3:
		return ccRasterGrid::PROJ_MEDIAN_VALUE;
	case 4:
		return ccRasterGrid::PROJ_INVERSE_VAR_VALUE;
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
	int projType							= settings.value("ProjectionType",        m_UI->heightProjectionComboBox->currentIndex()).toInt();
	int projDim								= settings.value("ProjectionDim",         m_UI->dimensionComboBox->currentIndex()).toInt();
	bool sfProj								= settings.value("SfProjEnabled",         m_UI->projectSFCheckBox->isChecked()).toBool();
	int sfProjStrategy						= settings.value("SfProjStrategy",        m_UI->scalarFieldProjection->currentIndex()).toInt();
	int fillStrategy						= settings.value("FillStrategy",          m_UI->fillEmptyCellsComboBox->currentIndex()).toInt();
	m_delaunayInterpParams.maxEdgeLength	= settings.value("MaxEdgeLength",         m_delaunayInterpParams.maxEdgeLength).toDouble();
	m_krigingParams.kNN						= settings.value("KrigingKNN",            m_krigingParams.kNN).toDouble();
	double step								= settings.value("GridStep",              m_UI->gridStepDoubleSpinBox->value()).toDouble();
	double emptyHeight						= settings.value("EmptyCellsHeight",      m_UI->emptyValueDoubleSpinBox->value()).toDouble();
	bool resampleCloud						= settings.value("ResampleOrigCloud",     m_UI->resampleCloudCheckBox->isChecked()).toBool();
	int minVertexCount						= settings.value("MinVertexCount",        m_UI->minVertexCountSpinBox->value()).toInt();
	bool ignoreBorders						= settings.value("IgnoreBorders",         m_UI->ignoreContourBordersCheckBox->isChecked()).toBool();
	bool projectContoursOnAlt				= settings.value("projectContoursOnAlt",  m_UI->projectContoursOnAltCheckBox->isChecked()).toBool();
	
	//Statistics checkboxes
	bool generateHeightStatistics				= settings.value("GenerateHeightStatistics",			m_UI->exportHeightStatsCheckBox->isChecked()).toBool();
	bool generateSFStatistics					= settings.value("GenerateSFStatistics",				m_UI->exportSFStatsCheckBox->isChecked()).toBool();
	bool generateStatisticsPopulation			= settings.value("GenerateStatisticsPopulation",		m_UI->generateStatisticsPopulationCheckBox->isChecked()).toBool();
	bool generateStatisticsMin					= settings.value("GenerateStatisticsMin",	   			m_UI->generateStatisticsMinCheckBox->isChecked()).toBool();
	bool generateStatisticsMax					= settings.value("GenerateStatisticsMax",	   			m_UI->generateStatisticsMaxCheckBox->isChecked()).toBool();
	bool generateStatisticsAverage				= settings.value("GenerateStatisticsAverage",	   		m_UI->generateStatisticsAverageCheckBox->isChecked()).toBool();
	bool generateStatisticsStdDev				= settings.value("GenerateStatisticsStdDev",	   		m_UI->generateStatisticsStdDevCheckBox->isChecked()).toBool();
	bool generateStatisticsRange				= settings.value("GenerateStatisticsRange",	   			m_UI->generateStatisticsRangeCheckBox->isChecked()).toBool();
	bool generateStatisticsMedian				= settings.value("GenerateStatisticsMedian",	   		m_UI->generateStatisticsMedianCheckBox->isChecked()).toBool();
	bool generateStatisticsUnique				= settings.value("GenerateStatisticsUnique",	   		m_UI->generateStatisticsUniqueCheckBox->isChecked()).toBool();
	bool generateStatisticsPercentile			= settings.value("GenerateStatisticsPercentile",		m_UI->generateStatisticsPercentileCheckBox->isChecked()).toBool();
	double generateStatisticsPercentileValue	= settings.value("GenerateStatisticsPercentileValue",	m_UI->generateStatisticsPercentileDoubleSpinBox->value()).toDouble();
	
	settings.endGroup();

	m_UI->gridStepDoubleSpinBox->setValue(step);
	m_UI->heightProjectionComboBox->setCurrentIndex(m_cloudHasScalarFields || projType != ccRasterGrid::PROJ_INVERSE_VAR_VALUE ? projType : 0);
	m_UI->fillEmptyCellsComboBox->setCurrentIndex(fillStrategy);
	m_UI->emptyValueDoubleSpinBox->setValue(emptyHeight);
	m_UI->dimensionComboBox->setCurrentIndex(projDim);
	m_UI->projectSFCheckBox->setChecked(m_cloudHasScalarFields && sfProj);
	m_UI->scalarFieldProjection->setCurrentIndex(m_cloudHasScalarFields || sfProjStrategy != ccRasterGrid::PROJ_INVERSE_VAR_VALUE ? sfProjStrategy : 0);
	m_UI->resampleCloudCheckBox->setChecked(resampleCloud);
	m_UI->minVertexCountSpinBox->setValue(minVertexCount);
	m_UI->ignoreContourBordersCheckBox->setChecked(ignoreBorders);
	m_UI->projectContoursOnAltCheckBox->setChecked(projectContoursOnAlt);

	//SF Statistics checkboxes
	m_UI->exportHeightStatsCheckBox->setChecked(				generateHeightStatistics						);
	m_UI->exportSFStatsCheckBox->setChecked(					generateSFStatistics && m_cloudHasScalarFields	);
	m_UI->generateStatisticsPopulationCheckBox->setChecked(		generateStatisticsPopulation					);
    m_UI->generateStatisticsMinCheckBox->setChecked(			generateStatisticsMin							);
    m_UI->generateStatisticsMaxCheckBox->setChecked(			generateStatisticsMax							);
    m_UI->generateStatisticsAverageCheckBox->setChecked(		generateStatisticsAverage						);
    m_UI->generateStatisticsStdDevCheckBox->setChecked(			generateStatisticsStdDev						);
    m_UI->generateStatisticsRangeCheckBox->setChecked(			generateStatisticsRange							);
    m_UI->generateStatisticsMedianCheckBox->setChecked(			generateStatisticsMedian						);
    m_UI->generateStatisticsUniqueCheckBox->setChecked(			generateStatisticsUnique						);
    m_UI->generateStatisticsPercentileCheckBox->setChecked(		generateStatisticsPercentile					);
	m_UI->generateStatisticsPercentileDoubleSpinBox->setValue(	generateStatisticsPercentileValue				);

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
	settings.setValue("ProjectionType", m_UI->heightProjectionComboBox->currentIndex());
	settings.setValue("ProjectionDim", m_UI->dimensionComboBox->currentIndex());
	settings.setValue("SfProjEnabled", m_UI->projectSFCheckBox->isChecked());
	settings.setValue("SfProjStrategy", m_UI->scalarFieldProjection->currentIndex());
	settings.setValue("FillStrategy", m_UI->fillEmptyCellsComboBox->currentIndex());
	settings.setValue("MaxEdgeLength", m_delaunayInterpParams.maxEdgeLength);
	settings.setValue("KrigingKNN", m_krigingParams.kNN);
	settings.setValue("GridStep", m_UI->gridStepDoubleSpinBox->value());
	settings.setValue("EmptyCellsHeight", m_UI->emptyValueDoubleSpinBox->value());
	settings.setValue("ResampleOrigCloud", m_UI->resampleCloudCheckBox->isChecked());
	settings.setValue("MinVertexCount", m_UI->minVertexCountSpinBox->value());
	settings.setValue("IgnoreBorders", m_UI->ignoreContourBordersCheckBox->isChecked());
	settings.setValue("projectContoursOnAlt", m_UI->projectContoursOnAltCheckBox->isChecked());

	//SF Statistics checkboxes
	settings.setValue("generateStatisticsPopulation",	   	m_UI->generateStatisticsPopulationCheckBox->isChecked());
	settings.setValue("generateStatisticsMin",	   			m_UI->generateStatisticsMinCheckBox->isChecked());
	settings.setValue("generateStatisticsMax",	   			m_UI->generateStatisticsMaxCheckBox->isChecked());
	settings.setValue("generateStatisticsAverage",	   		m_UI->generateStatisticsAverageCheckBox->isChecked());
	settings.setValue("generateStatisticsStdDev",	   		m_UI->generateStatisticsStdDevCheckBox->isChecked());
	settings.setValue("generateStatisticsRange",	   		m_UI->generateStatisticsRangeCheckBox->isChecked());
	settings.setValue("generateStatisticsMedian",	   		m_UI->generateStatisticsMedianCheckBox->isChecked());
	settings.setValue("generateStatisticsUnique",	   		m_UI->generateStatisticsUniqueCheckBox->isChecked());
	settings.setValue("generateStatisticsPercentile",	   	m_UI->generateStatisticsPercentileCheckBox->isChecked());
	settings.setValue("generateStatisticsPercentileValue",	m_UI->generateStatisticsPercentileDoubleSpinBox->value());

	settings.endGroup();
}

void ccRasterizeTool::gridIsUpToDate(bool state)
{
	if (state)
	{
		//standard button
		m_UI->updateGridPushButton->setStyleSheet(QString());
	}
	else
	{
		//red button
		m_UI->updateGridPushButton->setStyleSheet("color: white; background-color:red;");
	}
	
	m_UI->updateGridPushButton->setDisabled(state);

	m_UI->tabWidget->setEnabled(state);

	updateGridInfo(state);
}

ccPointCloud* ccRasterizeTool::convertGridToCloud(	bool exportHeightStats,
													bool exportSFStats,
													const std::vector<ccRasterGrid::ExportableFields>& exportedStatistics,
													bool projectSFs,
													bool projectColors,
													bool copyHillshadeSF,
													const QString& activeSFName,
													double percentileValue,
													bool exportToOriginalCS,
													bool appendGridSizeToSFNames,
													ccProgressDialog* progressDialog/*=nullptr*/ ) const
{
	if (!m_cloud || !m_grid.isValid())
		return nullptr;

	//call parent method
	ccPointCloud* cloudGrid = cc2Point5DimEditor::convertGridToCloud(	exportHeightStats,
																		exportSFStats,
																		exportedStatistics,
																		projectSFs,
																		projectColors,
																		/*resampleInputCloudXY=*/resampleOriginalCloud(),
																		/*resampleInputCloudZ=*/getTypeOfProjection() != ccRasterGrid::PROJ_AVERAGE_VALUE,
																		/*inputCloud=*/m_cloud,
                                            							percentileValue,
																		exportToOriginalCS,
																		appendGridSizeToSFNames,
																		progressDialog );

	//success?
	if (cloudGrid)
	{
		//add the hillshade SF
		if (copyHillshadeSF)
		{
			int hillshadeSFIdx = m_rasterCloud->getScalarFieldIndexByName(HILLSHADE_FIELD_NAME);
			if (hillshadeSFIdx >= 0)
			{
				CCCoreLib::ScalarField* hillshadeField = m_rasterCloud->getScalarField(hillshadeSFIdx);
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
		int activeSFIndex = cloudGrid->getScalarFieldIndexByName(activeSFName.toStdString());
		if (activeSFIndex < 0 && cloudGrid->getNumberOfScalarFields() != 0)
		{
			//if no SF is displayed, we should at least set a valid one (for later)
			activeSFIndex = 0;
		}
		cloudGrid->setCurrentDisplayedScalarField(activeSFIndex);

		cloudGrid->showColors(projectColors && cloudGrid->hasColors());
		cloudGrid->showSF(activeSFIndex >= 0);

		//don't forget the original shift
		cloudGrid->copyGlobalShiftAndScale(*m_cloud);
	}

	return cloudGrid;
}

void ccRasterizeTool::updateGridAndDisplay()
{
	//special case: remove the (temporary) hillshade field entry
	int hillshadeIndex = m_UI->activeLayerComboBox->findText(HILLSHADE_FIELD_NAME);
	if (hillshadeIndex >= 0)
	{
		if (m_UI->activeLayerComboBox->currentIndex() == hillshadeIndex && m_UI->activeLayerComboBox->count() > 1)
		{
			m_UI->activeLayerComboBox->setCurrentIndex(0);
		}
		m_UI->activeLayerComboBox->removeItem(hillshadeIndex);
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
		m_rasterCloud = nullptr;
	}

	bool activeLayerIsSF = (m_UI->activeLayerComboBox->currentData().toInt() == LAYER_SF);
	bool projectSFs = (getTypeOfSFProjection() != ccRasterGrid::INVALID_PROJECTION_TYPE) || activeLayerIsSF;
	bool projectColors = m_cloud && m_cloud->hasColors();
	bool success = updateGrid(projectSFs);

	if (success && m_glWindow)
	{
		//convert grid to point cloud
		try
		{
			//we always compute the default 'height' layer
			std::vector<ccRasterGrid::ExportableFields> exportedStatistics(1);
			exportedStatistics.front() = ccRasterGrid::PER_CELL_VALUE;
			//but we may also have to compute the 'original SF(s)' layer(s)
			QString activeLayerName = m_UI->activeLayerComboBox->currentText();
			m_rasterCloud = convertGridToCloud(	true,
												false,
												exportedStatistics,
												projectSFs,
												projectColors,
												/*copyHillshadeSF=*/false,
												activeLayerName,
												getStatisticsPercentileValue(),
												false,
												false,
												nullptr );

			// Special case: the 'LAYER_HEIGHT' field has now a dynamic name
			if (m_UI->activeLayerComboBox->currentIndex() == 0
				&& m_rasterCloud
				&& m_rasterCloud->getNumberOfScalarFields() != 0 )
			{
				assert(m_UI->activeLayerComboBox->itemData(0).toInt() == LAYER_HEIGHT);
				m_UI->activeLayerComboBox->setItemText(0, QString::fromStdString(m_rasterCloud->getScalarField(0)->getName()));
			}
		}
		catch (const std::bad_alloc&)
		{
			//see below
		}

		if (m_rasterCloud)
		{
			//just in case
			m_rasterCloud->setVisible(true);
			m_rasterCloud->setEnabled(true);

			m_glWindow->addToOwnDB(m_rasterCloud);
			ccBBox box = m_rasterCloud->getDisplayBB_recursive(false, m_glWindow);
			update2DDisplayZoom(box);

			//update 
			activeLayerChanged(m_UI->activeLayerComboBox->currentIndex(), false);
		}
		else
		{
			ccLog::Error("Not enough memory!");
			m_glWindow->redraw();
		}
	}

	gridIsUpToDate(success);
}

bool ccRasterizeTool::updateGrid(bool projectSFs/*=false*/)
{
	if (!m_cloud)
	{
		assert(false);
		return false;
	}

	//main parameters
	ccRasterGrid::ProjectionType projectionType = getTypeOfProjection();
	ccRasterGrid::ProjectionType sfProjectionType = projectSFs ? getTypeOfSFProjection() : ccRasterGrid::INVALID_PROJECTION_TYPE;

	ccRasterGrid::InterpolationType interpolationType = ccRasterGrid::InterpolationTypeFromEmptyCellFillOption(getFillEmptyCellsStrategy(m_UI->fillEmptyCellsComboBox));
	void* interpolationParams = nullptr;
	switch (interpolationType)
	{
	case ccRasterGrid::InterpolationType::DELAUNAY:
		interpolationParams = (void*)&m_delaunayInterpParams;
		break;
	case ccRasterGrid::InterpolationType::KRIGING:
		interpolationParams = (void*)&m_krigingParams;
		break;
	default:
		// do nothing
		break;
	}

	//cloud bounding-box --> grid size
	ccBBox box = getCustomBBox();
	if (!box.isValid())
	{
		return false;
	}

	//clear volume info
	{
		m_UI->volumeLabel->setText("0");
		m_UI->filledCellsPercentageLabel->setText("0 %");
	}

	unsigned gridWidth = 0;
	unsigned gridHeight = 0;
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
	CCVector3d minCorner = box.minCorner();
	if (!m_grid.init(gridWidth, gridHeight, gridStep, minCorner))
	{
		//not enough memory
		ccLog::Error("Not enough memory");
		return false;
	}
	
	//vertical dimension
	const unsigned char Z = getProjectionDimension();
	assert(Z <= 2);

    int zStdDevSfIndex = getStdDevLayerIndex();

	ccProgressDialog pDlg(true, this);
	if (!m_grid.fillWith(	m_cloud,
							Z,
							projectionType,
							interpolationType,
							interpolationParams,
							sfProjectionType,
							&pDlg,
                            zStdDevSfIndex))
	{
		return false;
	}

	//fill empty cells (if necessary)
	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(m_UI->fillEmptyCellsComboBox);
	double customEmptyCellsHeight = getCustomHeightForEmptyCells();
	m_grid.fillEmptyCells(fillEmptyCellsStrategy, customEmptyCellsHeight);

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
			m_UI->volumeLabel->setText(QString::number(hSum * cellArea));
			m_UI->filledCellsPercentageLabel->setText(QString::number(static_cast<double>(100 * filledCellCount) / (m_grid.width * m_grid.height), 'f', 2) + " %");
		}
	}

	ccLog::Print(QString("[Rasterize] Current raster grid:\n\tSize: %1 x %2\n\tHeight values: [%3 ; %4]").arg(m_grid.width).arg(m_grid.height).arg(m_grid.minHeight).arg(m_grid.maxHeight));
	
	return true;
}

ccPointCloud* ccRasterizeTool::generateCloud(bool autoExport/*=true*/)
{
	if (!m_cloud || !m_grid.isValid())
	{
		return nullptr;
	}

	//look for statistics fields (min,max,median,etc) fields to be exported
	std::vector<ccRasterGrid::ExportableFields> exportedStatistics;
	bool exportHeightStats = m_UI->exportHeightStatsCheckBox->isChecked();
	bool exportSFStats = m_UI->exportSFStatsCheckBox->isEnabled() && m_UI->exportSFStatsCheckBox->isChecked();
	if (exportHeightStats || exportSFStats)
	{
		try
		{
			getExportedStats(exportedStatistics);
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Error("Not enough memory!");
			return nullptr;
		}
	}


	QString activeLayerName = m_UI->activeLayerComboBox->currentText();
	bool activeLayerIsSF = (m_UI->activeLayerComboBox->currentData().toInt() == LAYER_SF);
	bool projectSFs = (getTypeOfSFProjection() != ccRasterGrid::INVALID_PROJECTION_TYPE) || activeLayerIsSF;
	//bool activeLayerIsRGB = (activeLayerComboBox->currentData().toInt() == LAYER_RGB);
	bool projectColors = m_cloud->hasColors();

	ccProgressDialog pDlg(true, this);
	ccPointCloud* rasterCloud = convertGridToCloud(	exportHeightStats,
													exportSFStats,
													exportedStatistics,
													projectSFs,
													projectColors,
													/*copyHillshadeSF=*/true,
													activeLayerName,
													getStatisticsPercentileValue(),
													true,
													true, // we want nicer SF names
                                                    &pDlg );

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

void ccRasterizeTool::generateMesh()
{
	if (!m_cloud)
	{
		assert(false);
		return;
	}

	ccPointCloud* rasterCloud = generateCloud(false);
	if (rasterCloud)
	{
		std::string errorStr;
		CCCoreLib::GenericIndexedMesh* baseMesh = CCCoreLib::PointProjectionTools::computeTriangulation(rasterCloud,
																								CCCoreLib::DELAUNAY_2D_AXIS_ALIGNED,
																								CCCoreLib::PointProjectionTools::IGNORE_MAX_EDGE_LENGTH,
																								getProjectionDimension(),
																								errorStr);
		ccMesh* rasterMesh = nullptr;
		if (baseMesh)
		{
			rasterMesh = new ccMesh(baseMesh, rasterCloud);
			delete baseMesh;
			baseMesh = nullptr;
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
			ccLog::Error( QStringLiteral("Failed to create mesh ('%1')")
						  .arg( QString::fromStdString( errorStr ) ) );
		}
	}
}

#ifdef CC_GDAL_SUPPORT
class RasterExportOptionsDlg
	: public QDialog
	, public Ui::RasterExportOptionsDialog
{
public:

	explicit RasterExportOptionsDlg(QWidget* parent = nullptr)
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

	bool hasScalarFields = !m_grid.scalarFields.empty();
	int visibleSfIndex = -1;
	if (m_UI->activeLayerComboBox->currentData().toInt() == LAYER_SF && m_cloud->isA(CC_TYPES::POINT_CLOUD))
	{
		//the indexes in the 'm_grid.scalarFields' are the same as in the cloud
		visibleSfIndex = static_cast<ccPointCloud*>(m_cloud)->getScalarFieldIndexByName(m_UI->activeLayerComboBox->currentText().toStdString());
	}

	//which (and how many) bands shall we create?
	ExportBands exportBands;
	exportBands.height = true; //height by default
	exportBands.rgb = false; //not a good idea to mix RGB and height values!

	RasterExportOptionsDlg reoDlg;
	reoDlg.dimensionsLabel->setText(QString("%1 x %2").arg(m_grid.width).arg(m_grid.height));
	reoDlg.exportRGBCheckBox->setEnabled(m_grid.hasColors);
	reoDlg.exportRGBCheckBox->setChecked(exportBands.rgb);
	reoDlg.exportHeightsCheckBox->setChecked(exportBands.height);
	reoDlg.exportDensityCheckBox->setChecked(exportBands.density);
	reoDlg.exportActiveLayerCheckBox->setChecked(exportBands.visibleSF);
	reoDlg.exportActiveLayerCheckBox->setEnabled(visibleSfIndex >= 0);
	reoDlg.exportAllSFCheckBox->setEnabled(hasScalarFields);
	reoDlg.exportAllSFCheckBox->setChecked(exportBands.allSFs);

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
			if (QMessageBox::warning(	nullptr,
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

	exportBands.height    = reoDlg.exportHeightsCheckBox->isChecked();
	exportBands.rgb       = reoDlg.exportRGBCheckBox->isChecked();
	exportBands.density   = reoDlg.exportDensityCheckBox->isChecked();
	exportBands.allSFs    = reoDlg.exportAllSFCheckBox->isChecked();
	exportBands.visibleSF = reoDlg.exportActiveLayerCheckBox->isChecked();

	ExportGeoTiff
	(
		outputFilename,
		exportBands,
		getFillEmptyCellsStrategy(m_UI->fillEmptyCellsComboBox),
		m_grid,
		getCustomBBox(),
		getProjectionDimension(),
		getCustomHeightForEmptyCells(),
		m_cloud,
		visibleSfIndex
	);

#else

	assert(false);
	ccLog::Error("[Rasterize] GDAL not supported by this version! Can't generate a raster...");

#endif
}
		
bool ccRasterizeTool::ExportGeoTiff(const QString& outputFilename,
									const ExportBands& exportBands,
									ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy,
									const ccRasterGrid& grid,
									const ccBBox& gridBBox,
									unsigned char Z,
									double customHeightForEmptyCells/*=std::numeric_limits<double>::quiet_NaN()*/,
									ccGenericPointCloud* originCloud/*=nullptr*/,
									int visibleSfIndex/*=-1*/)
{
#ifdef CC_GDAL_SUPPORT

	if (exportBands.visibleSF && visibleSfIndex < 0)
	{
		assert(false);
		return false;
	}

	//vertical dimension
	assert(Z <= 2);
	const unsigned char X = (Z == 2 ? 0 : Z + 1);
	const unsigned char Y = (X == 2 ? 0 : X + 1);

	double stepX = grid.gridStep;
	double stepY = grid.gridStep;

	//global shift
	assert(gridBBox.isValid());
	double shiftX = gridBBox.minCorner().u[X] - stepX / 2; //we will declare the raster grid as 'Pixel-is-area'!
	double shiftY = gridBBox.maxCorner().u[Y] + stepY / 2; //we will declare the raster grid as 'Pixel-is-area'!
	double shiftZ = 0.0;

	if (originCloud)
	{
		const CCVector3d& shift = originCloud->getGlobalShift();
		shiftX -= shift.u[X];
		shiftY -= shift.u[Y];
		shiftZ -= shift.u[Z];

		double scale = originCloud->getGlobalScale();
		assert(scale != 0);
		stepX /= scale;
		stepY /= scale;
	}

	int totalBands = 0;
	bool onlyRGBA = true;

	if (exportBands.height)
	{
		++totalBands;
		onlyRGBA = false;
	}
	
	bool rgbaMode = false;
	if (exportBands.rgb)
	{
		totalBands += 3; //one per component
		if (fillEmptyCellsStrategy == ccRasterGrid::LEAVE_EMPTY && grid.validCellCount < grid.height * grid.width)
		{
			rgbaMode = true;
			++totalBands; //alpha
		}
	}
	else
	{
		onlyRGBA = false;
	}
	
	if (exportBands.density)
	{
		++totalBands;
		onlyRGBA = false;
	}
	
	if (exportBands.allSFs)
	{
		for (const auto & scalarField : grid.scalarFields)
		{
			if (!scalarField.empty())
			{
				++totalBands;
				onlyRGBA = false;
			}
		}
	}
	else if (exportBands.visibleSF && visibleSfIndex >= 0)
	{
		++totalBands;
		onlyRGBA = false;
	}
	
	if (totalBands == 0)
	{
		ccLog::Error("Can't output a raster with no band! (check export parameters)");
		return false;
	}

	GDALAllRegister();
	ccLog::PrintDebug("(GDAL drivers: %i)", GetGDALDriverManager()->GetDriverCount());

	const char pszFormat[] = "GTiff";
	GDALDriver* poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
	if (!poDriver)
	{
		ccLog::Error("[GDAL] Driver %s is not supported", pszFormat);
		return false;
	}

	char** papszMetadata = poDriver->GetMetadata();
	if (!CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE))
	{
		ccLog::Error("[GDAL] Driver %s doesn't support Create() method", pszFormat);
		return false;
	}

	char **papszOptions = nullptr;
	GDALDataset* poDstDS = poDriver->Create(qUtf8Printable(outputFilename),
											static_cast<int>(grid.width),
											static_cast<int>(grid.height),
											totalBands,
											onlyRGBA ? GDT_Byte : GDT_Float64,
											papszOptions);

	if (!poDstDS)
	{
		ccLog::Error("[GDAL] Failed to create output raster (not enough memory?)");
		return false;
	}

	poDstDS->SetMetadataItem("AREA_OR_POINT", "AREA");

	double adfGeoTransform[6] {	shiftX,		//top left x
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
	//char *pszSRS_WKT = nullptr;
	//oSRS.exportToWkt( &pszSRS_WKT );
	//poDstDS->SetProjection( pszSRS_WKT );
	//CPLFree( pszSRS_WKT );

	int currentBand = 0;

	//exort RGB band?
	if (exportBands.rgb)
	{
		GDALRasterBand* rgbBands[3] {	poDstDS->GetRasterBand(++currentBand),
										poDstDS->GetRasterBand(++currentBand),
										poDstDS->GetRasterBand(++currentBand) };
		rgbBands[0]->SetColorInterpretation(GCI_RedBand);
		rgbBands[1]->SetColorInterpretation(GCI_GreenBand);
		rgbBands[2]->SetColorInterpretation(GCI_BlueBand);

		unsigned char* cLine = static_cast<unsigned char*>(CPLMalloc(sizeof(unsigned char)*grid.width));
		if (!cLine)
		{
			ccLog::Error("[GDAL] Not enough memory");
			GDALClose(poDstDS);
			return false;
		}

		bool error = false;
		
		//export the R, G and B components
		for (unsigned k = 0; k < 3; ++k)
		{
			rgbBands[k]->SetStatistics(0, 255, 128, 0); //warning: arbitrary average and std. dev. values

			for (unsigned j = 0; j < grid.height; ++j)
			{
				const ccRasterGrid::Row& row = grid.rows[grid.height - 1 - j]; //the first row is the northest one (i.e. Ymax)
				for (unsigned i = 0; i < grid.width; ++i)
				{
					cLine[i] = (std::isfinite(row[i].h) ? static_cast<unsigned char>(std::max(0.0, std::min(255.0, row[i].color.u[k]))) : 0);
				}

				if (rgbBands[k]->RasterIO(GF_Write, 0, static_cast<int>(j), static_cast<int>(grid.width), 1, cLine, static_cast<int>(grid.width), 1, GDT_Byte, 0, 0) != CE_None)
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

			for (unsigned j = 0; j < grid.height; ++j)
			{
				const ccRasterGrid::Row& row = grid.rows[grid.height - 1 - j];
				for (unsigned i = 0; i < grid.width; ++i)
				{
					cLine[i] = (std::isfinite(row[i].h) ? 255 : 0);
				}

				if (aBand->RasterIO(GF_Write, 0, static_cast<int>(j), static_cast<int>(grid.width), 1, cLine, static_cast<int>(grid.width), 1, GDT_Byte, 0, 0) != CE_None)
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
			GDALClose(poDstDS);
			return false;
		}
	}

	double* scanline = static_cast<double*>(CPLMalloc(sizeof(double)*grid.width));
	if (!scanline)
	{
		ccLog::Error("[GDAL] Not enough memory");
		GDALClose(poDstDS);
		return false;
	}

	//exort height band?
	if (exportBands.height)
	{
		GDALRasterBand* poBand = poDstDS->GetRasterBand(++currentBand);
		assert(poBand);
		poBand->SetColorInterpretation(GCI_Undefined);

		double emptyCellHeight = 0.0;
		switch (fillEmptyCellsStrategy)
		{
		case ccRasterGrid::LEAVE_EMPTY:
		{
			emptyCellHeight = std::numeric_limits<double>::quiet_NaN();
			if (CE_None != poBand->SetNoDataValue(emptyCellHeight))
			{
				ccLog::Warning("[GDAL] Failed to set the No Data value");
			}
			break;
		}
		case ccRasterGrid::FILL_MINIMUM_HEIGHT:
			emptyCellHeight = grid.minHeight;
			break;
		case ccRasterGrid::FILL_MAXIMUM_HEIGHT:
			emptyCellHeight = grid.maxHeight;
			break;
		case ccRasterGrid::FILL_CUSTOM_HEIGHT:
		case ccRasterGrid::INTERPOLATE_DELAUNAY:
			emptyCellHeight = customHeightForEmptyCells;
			break;
		case ccRasterGrid::FILL_AVERAGE_HEIGHT:
			emptyCellHeight = grid.meanHeight;
			break;
		default:
			assert(false);
		}

		if (fillEmptyCellsStrategy != ccRasterGrid::LEAVE_EMPTY)
		{
			emptyCellHeight += shiftZ;
		}

		for (unsigned j = 0; j < grid.height; ++j)
		{
			const ccRasterGrid::Row& row = grid.rows[grid.height - 1 - j];
			for (unsigned i = 0; i < grid.width; ++i)
			{
				scanline[i] = std::isfinite(row[i].h) ? row[i].h + shiftZ : emptyCellHeight;
			}

			if (poBand->RasterIO(	GF_Write,
									0,
									static_cast<int>(j),
									static_cast<int>(grid.width),
									1,
									scanline,
									static_cast<int>(grid.width),
									1,
									GDT_Float64,
									0,
									0) != CE_None)
			{
				ccLog::Error("[GDAL] An error occurred while writing the height band!");
				if (scanline)
				{
					CPLFree(scanline);
				}
				GDALClose(poDstDS);
				return false;
			}
		}
	}

	//export density band
	if (exportBands.density)
	{
		GDALRasterBand* poBand = poDstDS->GetRasterBand(++currentBand);
		assert(poBand);
		poBand->SetColorInterpretation(GCI_Undefined);
		for (unsigned j = 0; j < grid.height; ++j)
		{
			const ccRasterGrid::Row& row = grid.rows[grid.height - 1 - j];
			for (unsigned i = 0; i < grid.width; ++i)
			{
				scanline[i] = row[i].nbPoints;
			}

			if (poBand->RasterIO(	GF_Write,
									0,
									static_cast<int>(j),
									static_cast<int>(grid.width),
									1,
									scanline,
									static_cast<int>(grid.width),
									1,
									GDT_Float64,
									0,
									0) != CE_None)
			{
				ccLog::Error("[GDAL] An error occurred while writing the density band!");
				if (scanline)
				{
					CPLFree(scanline);
				}
				GDALClose(poDstDS);
				return false;
			}
		}
	}

	//export SF bands
	if (exportBands.allSFs || (exportBands.visibleSF && visibleSfIndex >= 0))
	{
		for (size_t k = 0; k < grid.scalarFields.size(); ++k)
		{
			assert(!grid.scalarFields[k].empty());
			if (exportBands.allSFs || (exportBands.visibleSF && visibleSfIndex == static_cast<int>(k)))
			{
				const double* sfGrid = grid.scalarFields[k].data();
				GDALRasterBand* poBand = poDstDS->GetRasterBand(++currentBand);

				double sfNanValue = std::numeric_limits<ccRasterGrid::SF::value_type>::quiet_NaN();
				poBand->SetNoDataValue(sfNanValue); //should be transparent!
				assert(poBand);
				poBand->SetColorInterpretation(GCI_Undefined);

				for (unsigned j = 0; j < grid.height; ++j)
				{
					const ccRasterGrid::Row& row = grid.rows[grid.height - 1 - j];
					const double* sfRow = sfGrid + (grid.height - 1 - j) * grid.width;
					for (unsigned i = 0; i < grid.width; ++i)
					{
						scanline[i] = std::isfinite(sfRow[i]) ? sfRow[i] : sfNanValue;
					}

					if (poBand->RasterIO(	GF_Write,
											0,
											static_cast<int>(j),
											static_cast<int>(grid.width),
											1,
											scanline,
											static_cast<int>(grid.width),
											1,
											GDT_Float64,
											0,
											0 ) != CE_None)
					{
						//the corresponding SF should exist on the input cloud
						ccLog::Error(QString("[GDAL] An error occurred while writing a scalar field band!"));
						k = grid.scalarFields.size(); //quick stop
						break;
					}
				}
			}
		}
	}

	if (scanline)
	{
		CPLFree(scanline);
	}
	scanline = nullptr;

	/* Once we're done, close properly the dataset */
	GDALClose(poDstDS);

	ccLog::Print(QString("[Rasterize] Raster '%1' successfully saved").arg(outputFilename));
	return true;

#else
	assert(false);
	ccLog::Error("[Rasterize] GDAL not supported by this version! Can't generate a raster...");
	return false;
#endif
}

//See http://edndoc.esri.com/arcobjects/9.2/net/shared/geoprocessing/spatial_analyst_tools/how_hillshade_works.htm
void ccRasterizeTool::generateHillshade()
{
	if (!m_grid.isValid() || !m_rasterCloud)
	{
		ccLog::Error("Need a valid raster/cloud to generate hillshade!");
		return;
	}
	if (m_grid.height < 3 || m_grid.width < 3)
	{
		ccLog::Error("Grid is too small");
		return;
	}

	//get/create layer
	ccScalarField* hillshadeLayer = nullptr;
	int sfIdx = m_rasterCloud->getScalarFieldIndexByName(HILLSHADE_FIELD_NAME);
	if (sfIdx >= 0)
	{
		hillshadeLayer = static_cast<ccScalarField*>(m_rasterCloud->getScalarField(sfIdx));
	}
	else
	{
		hillshadeLayer = new ccScalarField(HILLSHADE_FIELD_NAME);
		if (!hillshadeLayer->reserveSafe(m_rasterCloud->size()))
		{
			ccLog::Error("Not enough memory!");
			hillshadeLayer->release();
			hillshadeLayer = nullptr;
			return;
		}

		sfIdx = m_rasterCloud->addScalarField(hillshadeLayer);
		m_UI->activeLayerComboBox->addItem(HILLSHADE_FIELD_NAME, QVariant(LAYER_SF));
		m_UI->activeLayerComboBox->setEnabled(true);
	}
	assert(hillshadeLayer && hillshadeLayer->currentSize() == m_rasterCloud->size());
	hillshadeLayer->fill(CCCoreLib::NAN_VALUE);

	bool sparseSF = (hillshadeLayer->currentSize() != m_grid.height * m_grid.width);
	bool resampleInputCloudXY = resampleOriginalCloud();

	//now we can compute the hillshade
	int zenith_deg = m_UI->sunZenithSpinBox->value();
	double zenith_rad = CCCoreLib::DegreesToRadians(static_cast<double>(zenith_deg));

	double cos_zenith_rad = cos(zenith_rad);
	double sin_zenith_rad = sin(zenith_rad);

	int azimuth_deg = m_UI->sunAzimuthSpinBox->value();
	int azimuth_math = 360 - azimuth_deg + 90;
	double azimuth_rad = CCCoreLib::DegreesToRadians(static_cast<double>(azimuth_math));

	//for all cells
	unsigned nonEmptyCellIndex = 0;
	unsigned validButEmptyCellIndex = 0;
	for (unsigned j = 0; j < m_grid.height - 1; ++j)
	{
		const ccRasterGrid::Row& row = m_grid.rows[j];

		for (unsigned i = 0; i < m_grid.width; ++i)
		{
			//valid height value
			const ccRasterCell& cell = row[i];
			if (std::isfinite(cell.h))
			{
				if (j != 0 && i != 0 && i + 1 != m_grid.width)
				{
					double dz_dx = 0.0;
					int dz_dx_count = 0;
					double dz_dy = 0.0;
					int dz_dy_count = 0;

					for (int di = -1; di <= 1; ++di)
					{
						for (int dj = -1; dj <= 1; ++dj)
						{
							const ccRasterCell& n = m_grid.rows[j - dj][i + di]; //-dj (instead of + dj) because we scan the grid in the reverse orientation! (from bottom to top)
							if (std::isfinite(n.h))
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
					if (dz_dx_count == 8 && dz_dy_count == 8)
					{
						dz_dx /= (8 * m_grid.gridStep);
						dz_dy /= (8 * m_grid.gridStep);

						double slope_rad = atan( /*z_factor **/sqrt(dz_dx*dz_dx + dz_dy * dz_dy));

						double aspect_rad = 0;
						static const double s_Zero = 1.0e-8;
						if (std::abs(dz_dx) > s_Zero)
						{
							aspect_rad = atan2(dz_dy, -dz_dx);
							if (aspect_rad < 0)
							{
								aspect_rad += 2.0 * M_PI;
							}
						}
						else // dz_dx == 0
						{
							if (dz_dy > s_Zero)
							{
								aspect_rad = 0.5 * M_PI;
							}
							else if (dz_dy < s_Zero)
							{
								aspect_rad = 1.5 * M_PI;
							}
						}

						ScalarType hillshade = static_cast<ScalarType>(std::max(0.0, cos_zenith_rad * cos(slope_rad) + sin_zenith_rad * sin(slope_rad) * cos(azimuth_rad - aspect_rad)));
						if (!resampleInputCloudXY)
						{
							hillshadeLayer->setValue(sparseSF ? nonEmptyCellIndex : i + j * m_grid.width, hillshade);
						}
						else // resampling mode
						{
							if (cell.nbPoints != 0)
							{
								// non-empty cells are at the beginning
								hillshadeLayer->setValue(nonEmptyCellIndex, hillshade);
							}
							else
							{
								// filled or interpolated cells are appended at the end
								hillshadeLayer->setValue(m_grid.nonEmptyCellCount + validButEmptyCellIndex, hillshade);
							}
						}
					}
				}

				if (!resampleInputCloudXY || cell.nbPoints != 0)
				{
					++nonEmptyCellIndex;
				}
				else
				{
					++validButEmptyCellIndex;
				}
			}
			else
			{
				if (cell.nbPoints)
				{
					// with inv. var. projection mode, it's possible to have a non empty cell with NaN height!
					++nonEmptyCellIndex;
				}
			}
		}
	}

	hillshadeLayer->computeMinAndMax();
	hillshadeLayer->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::GREY));
	m_rasterCloud->setCurrentDisplayedScalarField(sfIdx);
	m_rasterCloud->showSF(true);
	m_UI->activeLayerComboBox->setCurrentIndex(m_UI->activeLayerComboBox->findText(HILLSHADE_FIELD_NAME));

	if (m_glWindow)
	{
		m_glWindow->redraw();
	}
}

void ccRasterizeTool::addNewContour(ccPolyline* poly, double height)
{
	if (!m_cloud || !poly)
	{
		assert(false);
		return;
	}

	if (poly->size() > 1)
	{
		poly->setGlobalScale(m_cloud->getGlobalScale());
		poly->setGlobalShift(m_cloud->getGlobalShift());
		poly->setWidth(m_UI->contourWidthSpinBox->value() < 2 ? 0 : m_UI->contourWidthSpinBox->value()); //size 1 is equivalent to the default size
		poly->setColor(ccColor::darkGrey);
		//poly->setClosed(isClosed);
		if (m_UI->colorizeContoursCheckBox->isChecked())
		{
			ccScalarField* activeLayer = m_rasterCloud->getCurrentDisplayedScalarField();
			if (activeLayer)
			{
				const ccColor::Rgb* col = activeLayer->getColor(height);
				if (col)
				{
					poly->setColor(*col);
				}
			}
		}
		poly->showColors(true);
		//vertices->setEnabled(false);

		if (m_glWindow)
			m_glWindow->addToOwnDB(poly);

		m_contourLines.push_back(poly);
	}
}

void ccRasterizeTool::generateContours()
{
	if (!m_grid.isValid() || !m_rasterCloud)
	{
		ccLog::Error("Need a valid raster/cloud to compute contours!");
		return;
	}

	//initialize parameters
	ccContourLinesGenerator::Parameters params;
	{
		params.projectContourOnAltitudes = false;
		{
			switch (m_UI->activeLayerComboBox->currentData().toInt())
			{
			case LAYER_HEIGHT:
				//nothing to do
				break;
			case LAYER_RGB:
				ccLog::Error("Can't generate contours from RGB colors");
				return;
			default:
				params.projectContourOnAltitudes = m_UI->projectContoursOnAltCheckBox->isChecked();
				break;
			}
		}

		//use current layer for 'altitudes'
		params.altitudes = m_rasterCloud->getCurrentDisplayedScalarField();
		if (!params.altitudes)
		{
			ccLog::Error("No valid/active layer!");
			return;
		}
		params.emptyCellsValue = params.altitudes->getMin() - 1.0;

		//min and max 'altitudes'
		params.startAltitude = m_UI->contourStartDoubleSpinBox->value();
		params.maxAltitude = params.altitudes->getMax();
		assert(params.startAltitude <= params.maxAltitude);

		//gap between levels
		params.step = m_UI->contourStepDoubleSpinBox->value();
		assert(params.step > 0);

		//minimum number of vertices per contour line
		params.minVertexCount = m_UI->minVertexCountSpinBox->value();
		assert(params.minVertexCount >= 3);

		//the parameters below are only required if GDAL is not supported (but we can set them anyway)
		params.ignoreBorders = m_UI->ignoreContourBordersCheckBox->isChecked();
		params.parentWidget = this;
	}

	removeContourLines();

	//compute the grid min corner (2D)
	CCVector2d gridMinCorner;
	{
		const unsigned char Z = getProjectionDimension();
		assert(Z <= 2);
		const unsigned char X = Z == 2 ? 0 : Z + 1;
		const unsigned char Y = X == 2 ? 0 : X + 1;
		gridMinCorner = CCVector2d(m_grid.minCorner.u[X], m_grid.minCorner.u[Y]);
	}

	//generate the contour lines
	std::vector<ccPolyline*> contourLines;
	if (!ccContourLinesGenerator::GenerateContourLines(	&m_grid,
														gridMinCorner,
														params,
														contourLines))
	{
		ccLog::Error("Process failed (see console)");
		return;
	}

	for (ccPolyline* poly : contourLines)
	{
		addNewContour(poly, poly->getMetaData(ccPolyline::MetaKeyConstAltitude()).toUInt());
	}

	if (!m_contourLines.empty())
	{
		m_UI->exportContoursPushButton->setEnabled(true);
		m_UI->clearContoursPushButton->setEnabled(true);
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

	bool colorize = m_UI->colorizeContoursCheckBox->isChecked();

	//vertical dimension
	const unsigned char Z = getProjectionDimension();
	assert(Z <= 2);
	const unsigned char X = (Z == 2 ? 0 : Z + 1);
	const unsigned char Y = (X == 2 ? 0 : X + 1);

	ccHObject* group = new ccHObject(QString("Contour plot(%1) [step=%2]").arg(m_cloud->getName()).arg(m_UI->contourStepDoubleSpinBox->value()));
	for (auto poly : m_contourLines)
	{
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
	m_contourLines.resize(0);
	m_UI->exportContoursPushButton->setEnabled(false);

	group->setDisplay_recursive(m_cloud->getDisplay());
	mainWindow->addToDB(group);

	ccLog::Print(QString("Contour lines have been successfully exported to DB (group name: %1)").arg(group->getName()));
}

ccRasterGrid::EmptyCellFillOption ccRasterizeTool::getFillEmptyCellsStrategyExt(double& emptyCellsHeight,
																				double& minHeight,
																				double& maxHeight) const
{
	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = getFillEmptyCellsStrategy(m_UI->fillEmptyCellsComboBox);

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
	case ccRasterGrid::INTERPOLATE_DELAUNAY:
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
	if (!m_cloud || !m_grid.isValid())
	{
		assert(false);
		return;
	}

	bool exportRGB = (m_UI->activeLayerComboBox->currentData().toInt() == LAYER_RGB);
	const ccRasterGrid::SF* gridSF = nullptr;
	const CCCoreLib::ScalarField* cloudSF = nullptr;
	if (!exportRGB && m_UI->activeLayerComboBox->currentData().toInt() == LAYER_SF && m_cloud->isA(CC_TYPES::POINT_CLOUD))
	{
		//the indexes in the 'm_grid.scalarFields' are the same as in the cloud
		ccPointCloud* pc = static_cast<ccPointCloud*>(m_cloud);
		int visibleSfIndex = pc->getScalarFieldIndexByName(m_UI->activeLayerComboBox->currentText().toStdString());
		if (visibleSfIndex >= 0 && static_cast<size_t>(visibleSfIndex) < m_grid.scalarFields.size())
		{
			cloudSF = pc->getScalarField(visibleSfIndex);
			gridSF = &(m_grid.scalarFields[visibleSfIndex]);
		}
		else
		{
			ccLog::Error("Internal error: can't find the selected field");
			return;
		}
	}

	ccRasterGrid::EmptyCellFillOption fillEmptyCellsStrategy = ccRasterGrid::LEAVE_EMPTY;
		
	// exported field extreme values
	double emptyCellsValue = std::numeric_limits<double>::quiet_NaN();
	double minValue = 0.0;
	double maxValue = 0.0;
	if (cloudSF)
	{
		// override the default min and max values (based on the height values)
		minValue = cloudSF->getMin();
		maxValue = cloudSF->getMax();
	}
	else if (!exportRGB)
	{
		// retrieve height values
		fillEmptyCellsStrategy = getFillEmptyCellsStrategyExt(	emptyCellsValue,
																minValue,
																maxValue);
	}

	double valueRange = maxValue - minValue;
	if (!exportRGB && CCCoreLib::LessThanEpsilon(valueRange))
	{
		ccLog::Warning("[Rasterize::generateImage] Exported field has a flat range");
		valueRange = 1.0; // to simplify tests below
	}

	QImage outputImage(m_grid.width, m_grid.height, exportRGB ? QImage::Format_ARGB32 : QImage::Format_Indexed8);

	if (!outputImage.isNull())
	{
		unsigned emptyCellColorIndex = 0;
		double maxColorComp = 255.99; //.99 --> to avoid round-off issues later!

		if (!exportRGB)
		{
			bool addTransparentColor = (cloudSF || fillEmptyCellsStrategy == ccRasterGrid::LEAVE_EMPTY);

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
					const ccColor::Rgb* col = colorScale->getColorByRelativePos(i / static_cast<double>(steps - 1), steps, &ccColor::lightGreyRGB);
					palette[i] = qRgba(col->r, col->g, col->b, 255);
				}
			}
			else
			{
				for (unsigned i = 0; i < 256; i++)
				{
					palette[i] = qRgba(i, i, i, 255);
				}
			}

			if (addTransparentColor)
			{
				palette[255] = qRgba(255, 0, 255, 0); //magenta/transparent color for empty cells (in place of pure white)
				maxColorComp = 254.99;
			}

			outputImage.setColorTable(palette);

			if (!cloudSF) //we are using height values
			{
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
						double normalizedHeight = (emptyCellsValue - minValue) / valueRange;
						assert(normalizedHeight >= 0.0 && normalizedHeight <= 1.0);
						emptyCellColorIndex = static_cast<unsigned>(normalizedHeight*maxColorComp); //static_cast is equivalent to floor if value >= 0
					}
					break;
				case ccRasterGrid::FILL_AVERAGE_HEIGHT:
				default:
					assert(false);
				}
			}
			else
			{
				emptyCellColorIndex = 255;
			}
			//outputImage.fill(emptyCellColorIndex);
		}

		// Filling the image with grid values
		for (unsigned j = 0; j < m_grid.height; ++j)
		{
			const ccRasterGrid::Row& row = m_grid.rows[j];
			const double* sfRow = (gridSF ? gridSF->data() + j * m_grid.width : nullptr);
			for (unsigned i = 0; i < m_grid.width; ++i)
			{
				if (std::isfinite(row[i].h))
				{
					if (exportRGB)
					{
						int r = static_cast<int>(std::max(0.0, std::min(255.0, row[i].color.u[0])));
						int g = static_cast<int>(std::max(0.0, std::min(255.0, row[i].color.u[1])));
						int b = static_cast<int>(std::max(0.0, std::min(255.0, row[i].color.u[2])));
						outputImage.setPixel(i, m_grid.height - 1 - j, qRgba(r, g, b, 255));
					}
					else
					{
						double value = sfRow ? sfRow[i] : row[i].h;
						double normalizedHeight = (value - minValue) / valueRange;
						assert(normalizedHeight >= 0.0 && normalizedHeight <= 1.0);
						unsigned char val = static_cast<unsigned char>(normalizedHeight*maxColorComp); //static_cast is equivalent to floor if value >= 0
						outputImage.setPixel(i, m_grid.height - 1 - j, val);
					}
				}
				else //NaN
				{
					outputImage.setPixel(i, m_grid.height - 1 - j, emptyCellColorIndex); //in RGBA mode, it should be 0
				}
			}
		}

		//open file saving dialog
		{
			QSettings settings;
			settings.beginGroup(ccPS::HeightGridGeneration());
			QString imageSavePath = settings.value("savePathImage", ccFileUtils::defaultDocPath()).toString();

			QString outputFilename = ImageFileFilter::GetSaveFilename(	"Save raster as image",
																		"image",
																		imageSavePath,
																		const_cast<ccRasterizeTool*>(this));

			if (!outputFilename.isNull())
			{
				//save current export path to persistent settings
				settings.setValue("savePathImage", QFileInfo(outputFilename).absolutePath());
				settings.endGroup();

				if (outputImage.save(outputFilename))
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
	QString outputFilename = QFileDialog::getSaveFileName(nullptr, "Save grid as ASCII file", asciiGridSavePath + QString("/raster_matrix.txt"), filter);
	if (outputFilename.isNull())
		return;

	QFile fp(outputFilename);
	if (!fp.open(QFile::WriteOnly))
	{
		ccLog::Warning(QString("[ccHeightGridGeneration] Failed to write '%1' file!").arg(outputFilename));
	}

	//default values
	double emptyCellsHeight = 0;
	double minHeight = m_grid.minHeight;
	double maxHeight = m_grid.maxHeight;
	//get real values
	getFillEmptyCellsStrategyExt(emptyCellsHeight, minHeight, maxHeight);
	QTextStream stream(&fp);
	stream.setRealNumberPrecision(8);
	for (unsigned j = 0; j < m_grid.height; ++j)
	{
		const ccRasterGrid::Row& row = m_grid.rows[m_grid.height - 1 - j];
		for (unsigned i = 0; i < m_grid.width; ++i)
		{
			stream << (std::isfinite(row[i].h) ? row[i].h : emptyCellsHeight) << ' ';
		}
		stream << endl;
	}

	//save current export path to persistent settings
	settings.setValue("savePathASCIIGrid", QFileInfo(outputFilename).absolutePath());

	ccLog::Print(QString("[Rasterize] Raster matrix '%1' successfully saved").arg(outputFilename));
}

void ccRasterizeTool::onStatExportTargetChanged(bool)
{
	m_UI->exportStatisticsFrame->setEnabled(	m_UI->exportHeightStatsCheckBox->isChecked()
											||	(m_UI->exportSFStatsCheckBox->isEnabled() && m_UI->exportSFStatsCheckBox->isChecked()));
}

void ccRasterizeTool::showInterpolationParamsDialog()
{
	switch (getFillEmptyCellsStrategy(m_UI->fillEmptyCellsComboBox))
	{
	case ccRasterGrid::EmptyCellFillOption::INTERPOLATE_DELAUNAY:
	{
		bool ok = false;
		double value = QInputDialog::getDouble(this, tr("Delaunay triangulation"), tr("Triangles max edge length (0 = no limit)"), m_delaunayInterpParams.maxEdgeLength , 0, 1.0e6, 6, &ok);
		if (ok)
		{
			m_delaunayInterpParams.maxEdgeLength = value;
			gridIsUpToDate(false);
		}
	}
	break;

	case ccRasterGrid::EmptyCellFillOption::KRIGING:
	{
		ccKrigingParamsDialog dlg(this);

		dlg.setParameters(m_krigingParams);

		if (dlg.exec())
		{
			dlg.getParameters(m_krigingParams);

			gridIsUpToDate(false);
		}
	}
	break;

	default:
		assert(false);
		break;
	}

}
