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
//#                   COPYRIGHT: The CloudCompare project                  #
//#                                                                        #
//##########################################################################

#include "ccContourLinesGenerator.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccProgressDialog.h>
#include <ccRasterGrid.h>
#include <ccScalarField.h>

//System
#include <cassert>

#ifndef CC_GDAL_SUPPORT

#include "ccIsolines.h" //old alternative code to generate contour lines (doesn't work very well :( )

//Qt
#include <QCoreApplication>

#else

//GDAL
#include <cpl_string.h>
#include <gdal.h>
#include <gdal_alg.h>

struct ContourGenerationParameters
{
	std::vector<ccPolyline*> contourLines;
	const ccRasterGrid* grid = nullptr;
	bool projectContourOnAltitudes = false;
};

static CPLErr ContourWriter(	double dfLevel,
								int nPoints,
								double *padfX,
								double *padfY,
								void * userData)
{
	if (nPoints < 2)
	{
		//nothing to do
		assert(false);
		return CE_None;
	}

	ContourGenerationParameters* params = reinterpret_cast<ContourGenerationParameters*>(userData);
	if (!params || !params->grid)
	{
		assert(false);
		return CE_Failure;
	}

	ccPointCloud* vertices = nullptr;
	ccPolyline* poly = nullptr;

	unsigned subIndex = 0;
	for (int i = 0; i < nPoints; ++i)
	{
		CCVector3 P(padfX[i], padfY[i], dfLevel);

		if (params->projectContourOnAltitudes)
		{
			int xi = std::min(std::max(static_cast<int>(padfX[i]), 0), static_cast<int>(params->grid->width) - 1);
			int yi = std::min(std::max(static_cast<int>(padfY[i]), 0), static_cast<int>(params->grid->height) - 1);
			double h = params->grid->rows[yi][xi].h;
			if (std::isfinite(h))
			{
				P.z = static_cast<PointCoordinateType>(h);
			}
			else
			{
				//DGM: we stop the current polyline
				if (poly)
				{
					if (poly->size() < 2)
					{
						delete poly;
						params->contourLines.pop_back();
					}
					poly = nullptr;
					vertices = nullptr;
				}
				continue;
			}
		}

		if (!poly)
		{
			//we need to instantiate a new polyline
			vertices = new ccPointCloud("vertices");
			vertices->setEnabled(false);
			poly = new ccPolyline(vertices);
			poly->addChild(vertices);
			poly->setMetaData(ccContourLinesGenerator::MetaKeySubIndex(), ++subIndex);
			poly->setClosed(false);

			//add the 'const altitude' meta-data as well
			poly->setMetaData(ccPolyline::MetaKeyConstAltitude(), QVariant(dfLevel));

			if (!vertices->reserve(nPoints - i) || !poly->reserve(nPoints - i))
			{
				//not enough memory
				delete poly;
				poly = nullptr;
				return CE_Failure;
			}

			try
			{
				params->contourLines.push_back(poly);
			}
			catch (const std::bad_alloc&)
			{
				return CE_Failure;
			}
		}

		assert(vertices);
		poly->addPointIndex(vertices->size());
		vertices->addPoint(P);
	}

	return CE_None;
}

#endif //CC_GDAL_SUPPORT

bool ccContourLinesGenerator::GenerateContourLines(	ccRasterGrid* rasterGrid,
													const CCVector2d& gridMinCornerXY,
													const Parameters& params,
													std::vector<ccPolyline*>& contourLines)
{
	if (!rasterGrid || !rasterGrid->isValid())
	{
		ccLog::Error("Need a valid raster/cloud to compute contours!");
		assert(false);
		return false;
	}
	if (params.startAltitude > params.maxAltitude)
	{
		ccLog::Error("Start value is above the layer maximum value!");
		assert(false);
		return false;
	}
	if (params.step < 0)
	{
		ccLog::Error("Invalid step value");
		assert(false);
		return false;
	}
	if (params.minVertexCount < 3)
	{
		ccLog::Error("Invalid input parameter: can't have less than 3 vertices per contour line");
		assert(false);
		return false;
	}

	bool sparseLayer = (params.altitudes && params.altitudes->currentSize() != rasterGrid->height * rasterGrid->width);
	if (sparseLayer && !std::isfinite(params.emptyCellsValue))
	{
		ccLog::Error("Invalid empty cell value (sparse layer)");
		assert(false);
		return false;
	}

	unsigned levelCount = 1;
	if (CCCoreLib::GreaterThanEpsilon(params.step))
	{
		levelCount += static_cast<unsigned>(floor((params.maxAltitude - params.startAltitude) / params.step));
	}

	try
	{
#ifdef CC_GDAL_SUPPORT //use GDAL (more robust) - otherwise we will use an old code found on the Internet (with a strange behavior)

		//invoke the GDAL 'Contour Generator'
		ContourGenerationParameters gdalParams;
		gdalParams.grid = rasterGrid;
		gdalParams.projectContourOnAltitudes = params.projectContourOnAltitudes;
		GDALContourGeneratorH hCG = GDAL_CG_Create(	rasterGrid->width,
													rasterGrid->height,
													std::isnan(params.emptyCellsValue) ? FALSE : TRUE,
													params.emptyCellsValue,
													params.step,
													params.startAltitude,
													ContourWriter,
													&gdalParams);
		if (!hCG)
		{
			ccLog::Error("[GDAL] Failed to create contour generator");
			return false;
		}

		//feed the scan lines
		{
			double* scanline = static_cast<double*>(CPLMalloc(sizeof(double) * rasterGrid->width));
			if (!scanline)
			{
				ccLog::Error("[GDAL] Not enough memory");
				return false;
			}
											
			unsigned layerIndex = 0;

			for (unsigned j = 0; j < rasterGrid->height; ++j)
			{
				const ccRasterGrid::Row& cellRow = rasterGrid->rows[j];
				for (unsigned i = 0; i < rasterGrid->width; ++i)
				{
					if (cellRow[i].nbPoints || !sparseLayer)
					{
						if (params.altitudes)
						{
							ScalarType value = params.altitudes->getValue(layerIndex++);
							scanline[i] = ccScalarField::ValidValue(value) ? value : params.emptyCellsValue;
						}
						else
						{
							scanline[i] = std::isfinite(cellRow[i].h) ? cellRow[i].h : params.emptyCellsValue;
						}
					}
					else
					{
						scanline[i] = params.emptyCellsValue;
					}
				}

				CPLErr error = GDAL_CG_FeedLine(hCG, scanline);
				if (error != CE_None)
				{
					ccLog::Error("[GDAL] An error occurred during contour lines generation");
					break;
				}
			}

			if (scanline)
			{
				CPLFree(scanline);
			}
			scanline = nullptr;

			//have we generated any contour line?
			if (!gdalParams.contourLines.empty())
			{
				//reproject contour lines from raster C.S. to the cloud C.S.
				for (ccPolyline*& poly : gdalParams.contourLines)
				{
					if (static_cast<int>(poly->size()) < params.minVertexCount)
					{
						delete poly;
						poly = nullptr;
						continue;
					}

					double height = std::numeric_limits<double>::quiet_NaN();
					for (unsigned i = 0; i < poly->size(); ++i)
					{
						CCVector3* P2D = const_cast<CCVector3*>(poly->getAssociatedCloud()->getPoint(i));
						if (i == 0)
						{
							height = P2D->z;
						}

						CCVector3 P(	static_cast<PointCoordinateType>((P2D->x - 0.5) * rasterGrid->gridStep + gridMinCornerXY.x),
										static_cast<PointCoordinateType>((P2D->y - 0.5) * rasterGrid->gridStep + gridMinCornerXY.y),
										P2D->z );
						*P2D = P;
					}

					//add contour
					poly->setName(QString("Contour line value = %1 (#%2)").arg(height).arg(poly->getMetaData(ccContourLinesGenerator::MetaKeySubIndex()).toUInt()));
					contourLines.push_back(poly);
				}

				gdalParams.contourLines.clear(); //just in case
			}
		}

		GDAL_CG_Destroy(hCG);
#else
		unsigned xDim = rasterGrid->width;
		unsigned yDim = rasterGrid->height;

		int margin = 0;
		if (!params.ignoreBorders)
		{
			margin = 1;
			xDim += 2;
			yDim += 2;
		}
		std::vector<double> grid(xDim * yDim, 0);

		//fill grid
		{
			unsigned layerIndex = 0;
			for (unsigned j = 0; j < rasterGrid->height; ++j)
			{
				const ccRasterGrid::Row& cellRow = rasterGrid->rows[j];
				double* row = &(grid[(j + margin)*xDim + margin]);
				for (unsigned i = 0; i < rasterGrid->width; ++i)
				{
					if (cellRow[i].nbPoints || !sparseLayer)
					{
						if (params.altitudes)
						{
							ScalarType value = params.altitudes->getValue(layerIndex++);
							row[i] = ccScalarField::ValidValue(value) ? value : params.emptyCellsValue;
						}
						else
						{
							row[i] = std::isfinite(cellRow[i].h) ? cellRow[i].h : params.emptyCellsValue;
						}
					}
					else
					{
						row[i] = params.emptyCellsValue;
					}
				}
			}
		}

		//generate contour lines
		{
			Isolines<double> iso(static_cast<int>(xDim), static_cast<int>(yDim));
			if (!params.ignoreBorders)
			{
				iso.createOnePixelBorder(grid.data(), params.startAltitude - 1.0);
			}

			ccProgressDialog pDlg(true, params.parentWidget);
			pDlg.setMethodTitle(QObject::tr("Contour plot"));
			pDlg.setInfo(QObject::tr("Levels: %1\nCells: %2 x %3").arg(levelCount).arg(rasterGrid->width).arg(rasterGrid->height));
			pDlg.start();
			pDlg.show();
			QCoreApplication::processEvents();
			CCCoreLib::NormalizedProgress nProgress(&pDlg, levelCount);

			for (double v = params.startAltitude; v <= params.maxAltitude; v += params.step)
			{
				//extract contour lines for the current level
				iso.setThreshold(v);
				int lineCount = iso.find(grid.data());

				ccLog::PrintDebug(QString("[Rasterize][Isolines] value=%1 : %2 lines").arg(v).arg(lineCount));

				//convert them to poylines
				int realCount = 0;
				for (int i = 0; i < lineCount; ++i)
				{
					int vertCount = iso.getContourLength(i);
					if (vertCount >= params.minVertexCount)
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
									/*P.u[X] = */P.x = static_cast<PointCoordinateType>((x + 0.5) * rasterGrid->gridStep + gridMinCornerXY.x);
									/*P.u[Y] = */P.y = static_cast<PointCoordinateType>((y + 0.5) * rasterGrid->gridStep + gridMinCornerXY.y);
									if (params.projectContourOnAltitudes)
									{
										int xi = std::min(std::max(static_cast<int>(x), 0), static_cast<int>(rasterGrid->width) - 1);
										int yi = std::min(std::max(static_cast<int>(y), 0), static_cast<int>(rasterGrid->height) - 1);
										double h = rasterGrid->rows[yi][xi].h;
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
									poly->setClosed(isClosed); //if we have less vertices, it means we have 'chopped' the original contour
									vertices->setEnabled(false);

									++realCount;
									poly->setMetaData(ccContourLinesGenerator::MetaKeySubIndex(), realCount);

									//add the 'const altitude' meta-data as well
									poly->setMetaData(ccPolyline::MetaKeyConstAltitude(), QVariant(v));

									//add contour
									poly->setName(QString("Contour line value = %1 (#%2)").arg(v).arg(realCount));
									try
									{
										contourLines.push_back(poly);
									}
									catch (const std::bad_alloc&)
									{
										ccLog::Warning("[ccContourLinesGenerator] Not enough memory");
										return false;
									}
								}
								else
								{
									delete poly;
									poly = nullptr;
								}
							}
							else
							{
								delete poly;
								poly = nullptr;
								ccLog::Warning("Not enough memory!");
								return false;
							}
						}
					}
				}

				if (!nProgress.oneStep())
				{
					//process cancelled by user
					break;
				}
			}
		}
#endif
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccContourLinesGenerator] Not enough memory");
		return false;
	}

	ccLog::Print(QString("[ccContourLinesGenerator] %1 iso-lines generated (%2 levels)").arg(contourLines.size()).arg(levelCount));
	return true;
}

