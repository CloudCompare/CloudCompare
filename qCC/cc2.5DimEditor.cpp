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

#include "cc2.5DimEditor.h"

//Local
#include "ccBoundingBoxEditorDlg.h"
#include "ccPersistentSettings.h"
#include "ccCommon.h"
#include "mainwindow.h"

//CCLib
#include <Delaunay2dMesh.h>
#include <PointProjectionTools.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccProgressDialog.h>

//qCC_gl
#include <ccGLWindow.h>

//Qt
#include <QFrame>
#include <QSettings>
#include <QImageWriter>
#include <QFileDialog>
#include <QCoreApplication>
#include <QMap>

//System
#include <assert.h>

//default field names
struct DefaultFieldNames : public QMap<cc2Point5DimEditor::ExportableFields, QString>
{
	DefaultFieldNames()
	{
		insert(cc2Point5DimEditor::PER_CELL_HEIGHT,         CC_HEIGHT_GRID_FIELD_NAME);
		insert(cc2Point5DimEditor::PER_CELL_COUNT,          "Per-cell population");
		insert(cc2Point5DimEditor::PER_CELL_MIN_HEIGHT,     "Min height");
		insert(cc2Point5DimEditor::PER_CELL_MAX_HEIGHT,     "Max height");
		insert(cc2Point5DimEditor::PER_CELL_AVG_HEIGHT,     "Average height");
		insert(cc2Point5DimEditor::PER_CELL_HEIGHT_STD_DEV, "Std. dev. height");
		insert(cc2Point5DimEditor::PER_CELL_HEIGHT_RANGE,   "Height range");
	}
};
static DefaultFieldNames s_defaultFieldNames;

QString cc2Point5DimEditor::GetDefaultFieldName(ExportableFields field)
{
	return s_defaultFieldNames[field];
}

cc2Point5DimEditor::cc2Point5DimEditor()
	: m_bbEditorDlg(0)
	, m_window(0)
	, m_rasterCloud(0)
{
}

cc2Point5DimEditor::~cc2Point5DimEditor()
{
	if (m_rasterCloud)
	{
		if (m_window)
			m_window->removeFromOwnDB(m_rasterCloud);
		delete m_rasterCloud;
		m_rasterCloud = 0;
	}

	m_grid.clear();
}

bool cc2Point5DimEditor::showGridBoxEditor()
{
	if (m_bbEditorDlg)
	{
		unsigned char projDim = getProjectionDimension();
		assert(projDim < 3);
		m_bbEditorDlg->set2DMode(true,projDim);
		if (m_bbEditorDlg->exec())
		{
			gridIsUpToDate(false);
			return true;
		}
	}

	return false;
}

void cc2Point5DimEditor::createBoundingBoxEditor(const ccBBox& gridBBox, QWidget* parent)
{
	if (!m_bbEditorDlg)
	{
		m_bbEditorDlg = new ccBoundingBoxEditorDlg(parent);
		m_bbEditorDlg->setBaseBBox(gridBBox,false);
	}
}

void cc2Point5DimEditor::create2DView(QFrame* parentFrame)
{
	if (!m_window)
	{
		m_window = new ccGLWindow(parentFrame);
		
		ccGui::ParamStruct params = m_window->getDisplayParameters();
		//black (text) & white (background) display by default
		params.backgroundCol = ccColor::white;
		params.textDefaultCol = ccColor::black;
		params.drawBackgroundGradient = false;
		params.decimateMeshOnMove = false;
		params.displayCross = false;
		params.colorScaleUseShader = false;
		m_window->setDisplayParameters(params,true);
		m_window->setPerspectiveState(false,true);
		m_window->setInteractionMode(ccGLWindow::PAN_ONLY);
		m_window->displayOverlayEntities(true);
		m_window->setPickingMode(ccGLWindow::NO_PICKING);
		
		//add window to the input frame (if any)
		if (parentFrame)
		{
			parentFrame->setLayout(new QHBoxLayout());
			parentFrame->layout()->addWidget(m_window);
		}
	}
}

QString cc2Point5DimEditor::getGridSizeAsString() const
{
	//vertical dimension
	const unsigned char Z = getProjectionDimension();
	assert(Z >= 0 && Z <= 2);
	const unsigned char X = Z == 2 ? 0 : Z +1;
	const unsigned char Y = X == 2 ? 0 : X +1;

	//cloud bounding-box --> grid size
	ccBBox box = getCustomBBox();
	if (!box.isValid())
	{
		return "invalid grid box";
	}

	double gridStep = getGridStep();
	assert(gridStep != 0);

	CCVector3d boxDiag(	static_cast<double>(box.maxCorner().x) - static_cast<double>(box.minCorner().x),
		static_cast<double>(box.maxCorner().y) - static_cast<double>(box.minCorner().y),
		static_cast<double>(box.maxCorner().z) - static_cast<double>(box.minCorner().z) );

	unsigned gridWidth  = static_cast<unsigned>(ceil(boxDiag.u[X] / gridStep));
	unsigned gridHeight = static_cast<unsigned>(ceil(boxDiag.u[Y] / gridStep));

	return QString("%1 x %2").arg(gridWidth).arg(gridHeight);
}

ccBBox cc2Point5DimEditor::getCustomBBox() const
{
	return (m_bbEditorDlg ? m_bbEditorDlg->getBox() : ccBBox());
}

void cc2Point5DimEditor::update2DDisplayZoom(ccBBox& box)
{
	if (!m_window || !m_grid.isValid())
		return;

	//equivalent to 'ccGLWindow::updateConstellationCenterAndZoom' but we take aspect ratio into account

	//we compute the pixel size (in world coordinates)
	{
		ccViewportParameters params = m_window->getViewportParameters();

		double realGridWidth  = m_grid.width  * m_grid.gridStep;
		double realGridHeight = m_grid.height * m_grid.gridStep;

		static const int screnMargin = 20;
		int screenWidth  = std::max(1,m_window->width()  - 2*screnMargin);
		int screenHeight = std::max(1,m_window->height() - 2*screnMargin);

		int pointSize = 1;
		if (	static_cast<int>(m_grid.width)  < screenWidth
			&&	static_cast<int>(m_grid.height) < screenHeight)
		{
			int vPointSize = static_cast<int>(ceil(static_cast<float>(screenWidth) /m_grid.width));
			int hPointSize = static_cast<int>(ceil(static_cast<float>(screenHeight)/m_grid.height));
			pointSize = std::min(vPointSize, hPointSize);

			//if the grid is too small (i.e. necessary point size > 10)
			if (pointSize > 10)
			{
				pointSize = 10;
				screenWidth  = m_grid.width  * pointSize;
				screenHeight = m_grid.height * pointSize;
			}
		}

		params.pixelSize = static_cast<float>( std::max( realGridWidth/screenWidth, realGridHeight/screenHeight ) );
		params.zoom = 1.0f;

		m_window->setViewportParameters(params);
		m_window->setPointSize(pointSize);
	}
	
	//we set the pivot point on the box center
	CCVector3 P = box.getCenter();
	m_window->setPivotPoint(CCVector3d::fromArray(P.u));
	m_window->setCameraPos(CCVector3d::fromArray(P.u));

	m_window->invalidateViewport();
	m_window->invalidateVisualization();
	m_window->redraw();
}

void cc2Point5DimEditor::RasterGrid::clear()
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

void cc2Point5DimEditor::RasterGrid::reset()
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
	nonEmptyCellCount = 0;
	validCellCount = 0;
}

bool cc2Point5DimEditor::RasterGrid::init(unsigned w, unsigned h)
{
	setValid(false);

	if (w == width && h == height)
	{
		//simply reset values
		reset();
		return true;
	}

	clear();

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
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	width = w;
	height = h;

	return true;
}

bool cc2Point5DimEditor::RasterGrid::fillWith(	ccGenericPointCloud* cloud,
												unsigned char projectionDimension,
												cc2Point5DimEditor::ProjectionType projectionType,
												bool interpolateEmptyCells,
												cc2Point5DimEditor::ProjectionType sfInterpolation/*=INVALID_PROJECTION_TYPE*/,
												ccProgressDialog* progressDialog/*=0*/)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}

	//current parameters
	unsigned gridTotalSize = width * height;
	
	//vertical dimension
	const unsigned char Z = projectionDimension;
	assert(Z >= 0 && Z <= 2);
	const unsigned char X = Z == 2 ? 0 : Z +1;
	const unsigned char Y = X == 2 ? 0 : X +1;

	//do we need to interpolate scalar fields?
	ccPointCloud* pc = cloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(cloud) : 0;
	bool interpolateSF = (sfInterpolation != INVALID_PROJECTION_TYPE);
	interpolateSF &= (pc && pc->hasScalarFields());
	if (interpolateSF)
	{
		unsigned sfCount = pc->getNumberOfScalarFields();

		bool memoryError = false;
		size_t previousCount = scalarFields.size();
		if (sfCount > previousCount)
		{
			try
			{
				scalarFields.resize(sfCount,0);
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				memoryError = true;
			}
		}

		for (size_t i=previousCount; i<sfCount; ++i)
		{
			assert(scalarFields[i] == 0);
			scalarFields[i] = new double[gridTotalSize];
			if (!scalarFields[i])
			{
				//not enough memory
				memoryError = true;
				break;
			}
		}

		if (memoryError)
		{
			ccLog::Warning(QString("[Rasterize] Failed to allocate memory for scalar fields!"));
		}
	}

	//filling the grid
	unsigned pointCount = cloud->size();

	double gridMaxX = gridStep * width;
	double gridMaxY = gridStep * height;

	if (progressDialog)
	{
		progressDialog->setMethodTitle("Grid generation");
		progressDialog->setInfo(qPrintable(QString("Points: %1\nCells: %2 x %3").arg(pointCount).arg(width).arg(height)));
		progressDialog->start();
		progressDialog->show();
		QCoreApplication::processEvents();
	}
	CCLib::NormalizedProgress nProgress(progressDialog,pointCount);

	for (unsigned n=0; n<pointCount; ++n)
	{
		const CCVector3* P = cloud->getPoint(n);

		CCVector3d relativePos = CCVector3d::fromArray(P->u) - minCorner;
		int i = static_cast<int>(relativePos.u[X]/gridStep);
		int j = static_cast<int>(relativePos.u[Y]/gridStep);

		//specific case: if we fall exactly on the max corner of the grid box
		if (i == static_cast<int>(width) && relativePos.u[X] == gridMaxX)
			--i;
		if (j == static_cast<int>(height) && relativePos.u[Y] == gridMaxY)
			--j;

		//we skip points outside the box!
		if (	i < 0 || i >= static_cast<int>(width)
			||	j < 0 || j >= static_cast<int>(height) )
			continue;

		assert(i >= 0 && j >= 0);

		RasterCell* aCell = data[j]+i;
		unsigned& pointsInCell = aCell->nbPoints;
		if (pointsInCell)
		{
			if (P->u[Z] < aCell->minHeight)
			{
				aCell->minHeight = P->u[Z];
				if (projectionType == PROJ_MINIMUM_VALUE)
					aCell->pointIndex = n;
			}
			else if (P->u[Z] > aCell->maxHeight)
			{
				aCell->maxHeight = P->u[Z];
				if (projectionType == PROJ_MAXIMUM_VALUE)
					aCell->pointIndex = n;
			}
		}
		else
		{
			aCell->minHeight = aCell->maxHeight = P->u[Z];
			aCell->pointIndex = n;
		}
		// Sum the points heights
		aCell->avgHeight += P->u[Z];
		aCell->stdDevHeight += static_cast<double>(P->u[Z])*P->u[Z];

		//scalar fields
		if (interpolateSF)
		{
			int pos = j*static_cast<int>(width)+i; //pos in 2D SF grid(s)
			assert(pos < static_cast<int>(gridTotalSize));
			for (size_t k=0; k<scalarFields.size(); ++k)
			{
				if (scalarFields[k])
				{
					CCLib::ScalarField* sf = pc->getScalarField(static_cast<unsigned>(k));
					assert(sf);
					ScalarType sfValue = sf->getValue(n);
					ScalarType formerValue = static_cast<ScalarType>(scalarFields[k][pos]);

					if (pointsInCell && ccScalarField::ValidValue(formerValue))
					{
						if (ccScalarField::ValidValue(sfValue))
						{
							switch (sfInterpolation)
							{
							case PROJ_MINIMUM_VALUE:
								// keep the minimum value
								scalarFields[k][pos] = std::min<double>(formerValue,sfValue);
								break;
							case PROJ_AVERAGE_VALUE:
								//we sum all values (we will divide them later)
								scalarFields[k][pos] += sfValue;
								break;
							case PROJ_MAXIMUM_VALUE:
								// keep the maximum value
								scalarFields[k][pos] = std::max<double>(formerValue,sfValue);
								break;
							default:
								assert(false);
								break;
							}
						}
					}
					else
					{
						//for the first (vaild) point, we simply have to store its SF value (in any case)
						scalarFields[k][pos] = sfValue;
					}
				}
			}
		}

		pointsInCell++;

		if (!nProgress.oneStep())
		{
			//process cancelled by user
			return false;
		}
	}

	//update SF grids for 'average' cases
	if (sfInterpolation == PROJ_AVERAGE_VALUE)
	{
		for (size_t k=0; k<scalarFields.size(); ++k)
		{
			if (scalarFields[k])
			{
				double* _gridSF = scalarFields[k];
				for (unsigned j=0;j<height;++j)
				{
					RasterCell* cell = data[j];
					for (unsigned i=0; i<width; ++i,++cell,++_gridSF)
					{
						if (cell->nbPoints > 1)
						{
							ScalarType s = static_cast<ScalarType>(*_gridSF);
							if (ccScalarField::ValidValue(s)) //valid SF value
							{
								*_gridSF /= cell->nbPoints;
							}
						}
					}
				}
			}
		}
	}

	//update the main grid (average height and std.dev. computation + current 'height' value)
	{
		for (unsigned j=0; j<height; ++j)
		{
			RasterCell* cell = data[j];
			for (unsigned i=0; i<width; ++i,++cell)
			{
				if (cell->nbPoints > 1)
				{
					cell->avgHeight /= cell->nbPoints;
					cell->stdDevHeight = sqrt(fabs(cell->stdDevHeight/cell->nbPoints - cell->avgHeight*cell->avgHeight));
				}
				else
				{
					cell->stdDevHeight = 0;
				}

				if (cell->nbPoints != 0)
				{
					//set the right 'height' value
					switch (projectionType)
					{
					case PROJ_MINIMUM_VALUE:
						cell->h = cell->minHeight;
						break;
					case PROJ_AVERAGE_VALUE:
						cell->h = cell->avgHeight;
						break;
					case PROJ_MAXIMUM_VALUE:
						cell->h = cell->maxHeight;
						break;
					default:
						assert(false);
						break;
					}
				}
			}
		}
	}

	//compute the number of non empty cells
	nonEmptyCellCount = 0;
	{
		for (unsigned i=0; i<height; ++i)
			for (unsigned j=0; j<width; ++j)
				if (data[i][j].nbPoints)
					++nonEmptyCellCount;
	}

	//specific case: interpolate the empty cells
	if (interpolateEmptyCells)
	{
		std::vector<CCVector2> the2DPoints;
		if (nonEmptyCellCount < 3)
		{
			ccLog::Warning("[Rasterize] Not enough non-empty cells to interpolate!");
		}
		else if (nonEmptyCellCount < width * height) //otherwise it's useless!
		{
			try
			{
				the2DPoints.resize(nonEmptyCellCount);
			}
			catch (const std::bad_alloc&)
			{
				//out of memory
				ccLog::Warning("[Rasterize] Not enough memory to interpolate empty cells!");
			}
		}

		//fill 2D vector with non-empty cell indexes
		if (!the2DPoints.empty())
		{
			unsigned index = 0;
			for (unsigned j=0; j<height; ++j)
			{
				const RasterCell* cell = data[j];
				for (unsigned i=0; i<width; ++i, ++cell)
				{
					if (cell->nbPoints)
					{
						//we only use the non-empty cells to interpolate
						the2DPoints[index++] = CCVector2(static_cast<PointCoordinateType>(i),static_cast<PointCoordinateType>(j));
					}
				}
			}
			assert(index == nonEmptyCellCount);

			//mesh the '2D' points
			CCLib::Delaunay2dMesh delaunayMesh;
			char errorStr[1024];
			if (delaunayMesh.buildMesh(the2DPoints,0,errorStr))
			{
				unsigned triNum = delaunayMesh.size();
				//now we are going to 'project' all triangles on the grid
				delaunayMesh.placeIteratorAtBegining();
				for (unsigned k=0; k<triNum; ++k)
				{
					const CCLib::VerticesIndexes* tsi = delaunayMesh.getNextTriangleVertIndexes();
					//get the triangle bounding box (in grid coordinates)
					int P[3][2];
					int xMin = 0, yMin = 0, xMax = 0, yMax = 0;
					{
						for (unsigned j=0; j<3; ++j)
						{
							const CCVector2& P2D = the2DPoints[tsi->i[j]];
							P[j][0] = static_cast<int>(P2D.x);
							P[j][1] = static_cast<int>(P2D.y);
						}
						xMin = std::min(std::min(P[0][0],P[1][0]),P[2][0]);
						yMin = std::min(std::min(P[0][1],P[1][1]),P[2][1]);
						xMax = std::max(std::max(P[0][0],P[1][0]),P[2][0]);
						yMax = std::max(std::max(P[0][1],P[1][1]),P[2][1]);
					}
					//now scan the cells
					{
						//pre-computation for barycentric coordinates
						const double& valA = data[ P[0][1] ][ P[0][0] ].h;
						const double& valB = data[ P[1][1] ][ P[1][0] ].h;
						const double& valC = data[ P[2][1] ][ P[2][0] ].h;

						int det = (P[1][1]-P[2][1])*(P[0][0]-P[2][0]) + (P[2][0]-P[1][0])*(P[0][1]-P[2][1]);

						for (int j=yMin; j<=yMax; ++j)
						{
							RasterCell* cell = data[static_cast<unsigned>(j)];

							for (int i=xMin; i<=xMax; ++i)
							{
								//if the cell is empty
								if (!cell[i].nbPoints)
								{
									//we test if it's included or not in the current triangle
									//Point Inclusion in Polygon Test (inspired from W. Randolph Franklin - WRF)
									bool inside = false;
									for (int ti=0; ti<3; ++ti)
									{
										const int* P1 = P[ti];
										const int* P2 = P[(ti+1)%3];
										if ((P2[1] <= j &&j < P1[1]) || (P1[1] <= j && j < P2[1]))
										{
											int t = (i-P2[0])*(P1[1]-P2[1])-(P1[0]-P2[0])*(j-P2[1]);
											if (P1[1] < P2[1])
												t = -t;
											if (t < 0)
												inside = !inside;
										}
									}
									//can we interpolate?
									if (inside)
									{
										double l1 = static_cast<double>((P[1][1]-P[2][1])*(i-P[2][0])+(P[2][0]-P[1][0])*(j-P[2][1]))/det;
										double l2 = static_cast<double>((P[2][1]-P[0][1])*(i-P[2][0])+(P[0][0]-P[2][0])*(j-P[2][1]))/det;
										double l3 = 1.0-l1-l2;

										cell[i].h = l1 * valA + l2 * valB + l3 * valC;
										assert(cell[i].h == cell[i].h);

										//interpolate SFs as well!
										for (size_t sfIndex=0; sfIndex<scalarFields.size(); ++sfIndex)
										{
											if (scalarFields[sfIndex])
											{
												double* gridSF = scalarFields[sfIndex];
												const double& sfValA = gridSF[ P[0][0] + P[0][1]*width ];
												const double& sfValB = gridSF[ P[1][0] + P[1][1]*width ];
												const double& sfValC = gridSF[ P[2][0] + P[2][1]*width ];
												gridSF[i + j*width] = l1 * sfValA + l2 * sfValB + l3 * sfValC;
											}
										}
									}
								}
							}
						}
					}
				}
			}
			else
			{
				ccLog::Warning(QString("[Rasterize] Empty cells interpolation failed: Triangle lib. said '%1'").arg(errorStr));
			}
		}
	}

	//computation of the average and extreme height values in the grid
	{
		minHeight = 0;
		maxHeight = 0;
		meanHeight = 0;
		validCellCount = 0;

		for (unsigned i=0; i<height; ++i)
		{
			for (unsigned j=0; j<width; ++j)
			{
				if (data[i][j].h == data[i][j].h) //valid height
				{
					double h = data[i][j].h;

					if (validCellCount)
					{
						if (h < minHeight)
							minHeight = h;
						else if (h > maxHeight)
							maxHeight = h;
						
						meanHeight += h;
					}
					else
					{
						//first valid cell
						meanHeight = minHeight = maxHeight = h;
					}
					++validCellCount;
				}
			}
		}
		meanHeight /= validCellCount;
	}

	setValid(true);

	return true;
}

void cc2Point5DimEditor::RasterGrid::fillEmptyGridCells(cc2Point5DimEditor::EmptyCellFillOption fillEmptyCellsStrategy,
														double customCellHeight/*=0*/)
{
	//fill empty cells (all but the 'INTERPOLATE' case)
	if (	fillEmptyCellsStrategy != LEAVE_EMPTY
		&&	fillEmptyCellsStrategy != INTERPOLATE)
	{
		double defaultHeight = std::numeric_limits<double>::quiet_NaN();
		switch (fillEmptyCellsStrategy)
		{
		case FILL_MINIMUM_HEIGHT:
			defaultHeight = minHeight;
			break;
		case FILL_MAXIMUM_HEIGHT:
			defaultHeight = maxHeight;
			break;
		case FILL_CUSTOM_HEIGHT:
			defaultHeight = customCellHeight;
			break;
		case FILL_AVERAGE_HEIGHT:
			defaultHeight = meanHeight;
			break;
		default:
			assert(false);
			break;
		}
		assert(defaultHeight != 0);

		for (unsigned i=0; i<height; ++i)
		{
			for (unsigned j=0; j<width; ++j)
			{
				if (!(data[i][j].h == data[i][j].h)) //empty cell (NaN)
				{
					data[i][j].h = defaultHeight;
				}
			}
		}
	}
}

ccPointCloud* cc2Point5DimEditor::convertGridToCloud(	const std::vector<ExportableFields>& exportedFields,
														bool interpolateSF,
														bool resampleInputCloud,
														ccGenericPointCloud* inputCloud,
														bool fillEmptyCells,
														double emptyCellsHeight) const
{
	if (!m_grid.isValid())
		return 0;

	unsigned pointsCount = (fillEmptyCells ? m_grid.width * m_grid.height : m_grid.validCellCount);
	if (pointsCount == 0)
	{
		ccLog::Warning("[Rasterize] Empty grid!");
		return 0;
	}

	ccPointCloud* cloudGrid = 0;
	if (resampleInputCloud)
	{
		CCLib::ReferenceCloud refCloud(inputCloud);
		if (refCloud.reserve(m_grid.nonEmptyCellCount))
		{
			for (unsigned j=0; j<m_grid.height; ++j)
			{
				for (unsigned i=0; i<m_grid.width; ++i)
				{
					const RasterCell& cell = m_grid.data[j][i];
					if (cell.nbPoints) //non empty cell
					{
						refCloud.addPointIndex(cell.pointIndex);
					}
				}
			}

			assert(refCloud.size() != 0);
			cloudGrid = inputCloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(inputCloud)->partialClone(&refCloud) : ccPointCloud::From(&refCloud,inputCloud);
			cloudGrid->setPointSize(0); //to avoid display issues

			//even if we have already resampled the original cloud we may have to create new points and/or scalar fields
			//if (!interpolateSF && !fillEmptyCells)
			//	return cloudGrid;
		}
		else
		{
			ccLog::Warning("[Rasterize] Not enough memory!");
			return 0;
		}
	}
	else
	{
		cloudGrid = new ccPointCloud("grid");
	}
	assert(cloudGrid);
	
	//shall we generate per-cell fields as well?
	std::vector<CCLib::ScalarField*> exportedSFs;
	if (!exportedFields.empty())
	{
		exportedSFs.resize(exportedFields.size(),0);
		for (size_t i=0; i<exportedFields.size(); ++i)
		{
			int sfIndex = -1;
			switch (exportedFields[i])
			{
			case PER_CELL_HEIGHT:
			case PER_CELL_COUNT:
			case PER_CELL_MIN_HEIGHT:
			case PER_CELL_MAX_HEIGHT:
			case PER_CELL_AVG_HEIGHT:
			case PER_CELL_HEIGHT_STD_DEV:
			case PER_CELL_HEIGHT_RANGE:
				sfIndex = cloudGrid->addScalarField(qPrintable(GetDefaultFieldName(exportedFields[i])));
				break;
			default:
				assert(false);
				break;
			}
			if (sfIndex < 0)
			{
				ccLog::Warning("[Rasterize] Couldn't allocate scalar field(s)! Try to free some memory ...");
				break;
			}

			exportedSFs[i] = cloudGrid->getScalarField(sfIndex);
			assert(exportedSFs[i]);
		}
	}

	//the resampled cloud already contains the points corresponding to 'filled' cells so we will only
	//need to add the empty ones (if requested)
	if ((!resampleInputCloud || fillEmptyCells) && !cloudGrid->reserve(pointsCount))
	{
		ccLog::Warning("[Rasterize] Not enough memory!");
		delete cloudGrid;
		return 0;
	}

	//vertical dimension
	const unsigned char Z = getProjectionDimension();
	assert(Z >= 0 && Z <= 2);
	const unsigned char X = Z == 2 ? 0 : Z +1;
	const unsigned char Y = X == 2 ? 0 : X +1;

	//cloud bounding-box
	ccBBox box = getCustomBBox();
	assert(box.isValid());

	//we work with doubles as grid step can be much smaller than the cloud coordinates!
	double Py = box.minCorner().u[Y] + m_grid.gridStep / 2;

	//as the 'non empty cells points' are already in the cloud
	//we must take care of where we put the scalar fields values!
	unsigned nonEmptyCellIndex = 0;

	for (unsigned j=0; j<m_grid.height; ++j)
	{
		const RasterCell* aCell = m_grid.data[j];
		double Px = box.minCorner().u[X] + m_grid.gridStep / 2;
		for (unsigned i=0; i<m_grid.width; ++i,++aCell)
		{
			if (aCell->h == aCell->h) //valid cell
			{
				//if we haven't resampled the original cloud, we must add the point
				//corresponding to this non-empty cell
				if (!resampleInputCloud || aCell->nbPoints == 0)
				{
					CCVector3 Pf(	static_cast<PointCoordinateType>(Px),
									static_cast<PointCoordinateType>(Py),
									static_cast<PointCoordinateType>(aCell->h) );

					cloudGrid->addPoint(Pf);
				}

				//fill the associated SFs
				assert(exportedSFs.size() >= exportedFields.size());
				assert(!inputCloud || nonEmptyCellIndex < inputCloud->size());
				for (size_t i=0; i<exportedSFs.size(); ++i)
				{
					CCLib::ScalarField* sf = exportedSFs[i];
					ScalarType sVal = NAN_VALUE;
					switch (exportedFields[i])
					{
					case PER_CELL_HEIGHT:
						sVal = static_cast<ScalarType>(aCell->h);
						break;
					case PER_CELL_COUNT:
						sVal = static_cast<ScalarType>(aCell->nbPoints);
						break;
					case PER_CELL_MIN_HEIGHT:
						sVal = static_cast<ScalarType>(aCell->minHeight);
						break;
					case PER_CELL_MAX_HEIGHT:
						sVal = static_cast<ScalarType>(aCell->maxHeight);
						break;
					case PER_CELL_AVG_HEIGHT:
						sVal = static_cast<ScalarType>(aCell->avgHeight);
						break;
					case PER_CELL_HEIGHT_STD_DEV:
						sVal = static_cast<ScalarType>(aCell->stdDevHeight);
						break;
					case PER_CELL_HEIGHT_RANGE:
						sVal = static_cast<ScalarType>(aCell->maxHeight - aCell->minHeight);
						break;
					default:
						assert(false);
						break;
					}
					if (resampleInputCloud)
						sf->setValue(nonEmptyCellIndex,sVal);
					else
						sf->addElement(sVal);
				}
				++nonEmptyCellIndex;
			}
			else if (fillEmptyCells) //empty cell
			{
				//even if we have resampled the original cloud, we must add the point
				//corresponding to this empty cell
				{
					CCVector3 Pf(	static_cast<PointCoordinateType>(Px),
									static_cast<PointCoordinateType>(Py),
									static_cast<PointCoordinateType>(emptyCellsHeight) );
					cloudGrid->addPoint(Pf);
				}

				assert(exportedSFs.size() == exportedFields.size());
				for (size_t i=0; i<exportedSFs.size(); ++i)
				{
					if (!exportedSFs[i])
					{
						continue;
					}
					
					if (exportedFields[i] == PER_CELL_HEIGHT)
					{
						//we set the point height to the default height
						ScalarType s = static_cast<ScalarType>(emptyCellsHeight);
						exportedSFs[i]->addElement(s);
					}
					else
					{
						exportedSFs[i]->addElement(NAN_VALUE);
					}
				}
			}

			Px += m_grid.gridStep;
		}

		Py += m_grid.gridStep;
	}

	assert(exportedSFs.size() == exportedFields.size());
	for (size_t i=0; i<exportedSFs.size(); ++i)
	{
		CCLib::ScalarField* sf = exportedSFs[i];
		if (sf)
		{
			sf->computeMinAndMax();
		}
	}

	//take care of former scalar fields
	if (!resampleInputCloud)
	{
		if (interpolateSF && inputCloud && inputCloud->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(inputCloud);
			for (size_t k=0; k<m_grid.scalarFields.size(); ++k)
			{
				double* _sfGrid = m_grid.scalarFields[k];
				if (_sfGrid) //valid SF grid
				{
					//the corresponding SF should exist on the input cloud
					ccScalarField* formerSf = static_cast<ccScalarField*>(pc->getScalarField(static_cast<int>(k)));
					assert(formerSf);

					//we try to create an equivalent SF on the output grid
					int sfIdx = cloudGrid->addScalarField(formerSf->getName());
					if (sfIdx < 0) //if we aren't lucky, the input cloud already had a SF with CC_HEIGHT_GRID_FIELD_NAME as name
						sfIdx = cloudGrid->addScalarField(qPrintable(QString(formerSf->getName()).append(".old")));

					if (sfIdx < 0)
					{
						ccLog::Warning("[Rasterize] Couldn't allocate a new scalar field for storing SF '%s' values! Try to free some memory ...",formerSf->getName());
					}
					else
					{
						ccScalarField* sf = static_cast<ccScalarField*>(cloudGrid->getScalarField(sfIdx));
						assert(sf);
						//set sf values
						unsigned n = 0;
						const ScalarType emptyCellSFValue = CCLib::ScalarField::NaN();
						for (unsigned j=0; j<m_grid.height; ++j)
						{
							const RasterCell* aCell = m_grid.data[j];
							for (unsigned i=0; i<m_grid.width; ++i, ++_sfGrid, ++aCell)
							{
								if (aCell->nbPoints)
								{
									ScalarType s = static_cast<ScalarType>(*_sfGrid);
									sf->setValue(n++,s);
								}
								else if (fillEmptyCells)
								{
									sf->setValue(n++,emptyCellSFValue);
								}
							}
						}
						sf->computeMinAndMax();
						sf->importParametersFrom(formerSf);
						assert(sf->currentSize() == pointsCount);
					}
				}
			}
		}
	}
	else
	{
		for (size_t k=0; k<cloudGrid->getNumberOfScalarFields(); ++k)
		{
			CCLib::ScalarField* sf = cloudGrid->getScalarField(static_cast<int>(k));
			sf->resize(cloudGrid->size(),true,NAN_VALUE);
		}
	}

	QString gridName = QString("raster(%1)").arg(m_grid.gridStep);
	if (inputCloud)
	{
		gridName.prepend(inputCloud->getName() + QString("."));
	}
	cloudGrid->setName(gridName);

	return cloudGrid;
}

cc2Point5DimEditor::EmptyCellFillOption cc2Point5DimEditor::getFillEmptyCellsStrategy(QComboBox* comboBox) const
{
	if (!comboBox)
	{
		assert(false);
		return LEAVE_EMPTY;
	}

	switch (comboBox->currentIndex())
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
