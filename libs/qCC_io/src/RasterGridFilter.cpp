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

#ifdef CC_GDAL_SUPPORT

#include "RasterGridFilter.h"

// qCC_db
#include <ccMesh.h>
#include <ccPlane.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>

// GDAL
#include <cpl_conv.h> // for CPLMalloc()
#include <gdal_priv.h>

// Qt
#include <QMessageBox>

// System
#include <cstring> //for memset

RasterGridFilter::RasterGridFilter()
    : FileIOFilter({"_Raster Grid Filter",
                    16.0f, // priority
                    QStringList{"tif", "tiff", "adf"},
                    "tif",
                    QStringList{"RASTER grid (*.*)"},
                    QStringList(),
                    Import | BuiltIn})
{
}

CC_FILE_ERROR RasterGridFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	GDALAllRegister();
	ccLog::PrintDebug("(GDAL drivers: %i)", GetGDALDriverManager()->GetDriverCount());

	try
	{
		GDALDataset* poDataset = static_cast<GDALDataset*>(GDALOpen(qUtf8Printable(filename), GA_ReadOnly));
		if (poDataset != nullptr)
		{
			ccLog::Print(QString("Raster file: '%1'").arg(filename));
			ccLog::Print("Driver: %s/%s",
			             poDataset->GetDriver()->GetDescription(),
			             poDataset->GetDriver()->GetMetadataItem(GDAL_DMD_LONGNAME));

			int rasterCount = poDataset->GetRasterCount();
			int rasterX     = poDataset->GetRasterXSize();
			int rasterY     = poDataset->GetRasterYSize();
			ccLog::Print("Size is %dx%dx%d", rasterX, rasterY, rasterCount);

			if (poDataset->GetProjectionRef() != nullptr)
			{
				ccLog::Print("Projection is '%s'", poDataset->GetProjectionRef());
			}

			// See https://gdal.org/user/raster_data_model.html
			// Xgeo = adfGeoTransform(0) + Xpixel * adfGeoTransform(1) + Yline * adfGeoTransform(2)
			// Ygeo = adfGeoTransform(3) + Xpixel * adfGeoTransform(4) + Yline * adfGeoTransform(5)
			double adfGeoTransform[6] = {0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

			if (poDataset->GetGeoTransform(adfGeoTransform) == CE_None)
			{
				ccLog::Print("Origin = (%.6f,%.6f)", adfGeoTransform[0], adfGeoTransform[3]);
				ccLog::Print("Pixel Size = (%.6f,%.6f)", adfGeoTransform[1], adfGeoTransform[5]);
			}

			if (adfGeoTransform[1] == 0 || adfGeoTransform[5] == 0)
			{
				ccLog::Warning("Invalid pixel size! Forcing it to (1,1)");
				adfGeoTransform[1] = adfGeoTransform[5] = 1.0;
			}

			const char* aeraOrPoint = poDataset->GetMetadataItem("AREA_OR_POINT");
			ccLog::Print(QString("Pixel Type = ") + (aeraOrPoint ? aeraOrPoint : "AREA")); // Area by default

			// first check if the raster actually has 'color' bands
			int               colorBands             = 0;
			bool              hasUndefinedColorBands = false;
			std::vector<bool> isColorBand(rasterCount, false);
			{
				int undefinedColorBandCount = 0;
				for (int i = 1; i <= rasterCount; ++i)
				{
					GDALRasterBand* poBand      = poDataset->GetRasterBand(i);
					GDALColorInterp colorInterp = poBand->GetColorInterpretation();

					switch (colorInterp)
					{
					case GCI_RedBand:
					case GCI_GreenBand:
					case GCI_BlueBand:
					case GCI_AlphaBand:
						isColorBand[i - 1] = true;
						++colorBands;
						break;
					case GCI_Undefined:
					case GCI_GrayIndex:
					{
						// Sometimes GDAL fails to recognize color bands as such, or it can flag a Z band as gray values if altitudes are between 0 and 255...
						int    bGotMin = 0, bGotMax = 0;
						double adfMinMax[2]{poBand->GetMinimum(&bGotMin), poBand->GetMaximum(&bGotMax)};
						if (0 == bGotMin || 0 == bGotMax)
						{
							// DGM FIXME: if the file is corrupted (e.g. ASCII ArcGrid with missing rows) this method will enter in a infinite loop!
							GDALComputeRasterMinMax((GDALRasterBandH)poBand, TRUE, adfMinMax);
						}

						int minVal = static_cast<int>(adfMinMax[0]);
						int maxVal = static_cast<int>(adfMinMax[1]);
						if (adfMinMax[0] == static_cast<double>(minVal)
						    && adfMinMax[1] == static_cast<double>(maxVal)
						    && minVal >= 0
						    && maxVal <= 255)
						{
							// integer range limits within [0 255]?
							// (we will still load this band as a scalar field though!)
							isColorBand[i - 1] = true;
							if (colorInterp == GCI_Undefined)
							{
								++undefinedColorBandCount;
							}
						}
					}
					break;

					default:
						break;
					}
				}
				hasUndefinedColorBands = (colorBands == 0 && (undefinedColorBandCount == 3 || undefinedColorBandCount == 4));
			}

			bool loadAsTexturedQuad = false;
			if (colorBands >= 3)
			{
				loadAsTexturedQuad = false;
				if (parameters.parentWidget) // otherwise it means we are in command line mode --> no popup
				{
					loadAsTexturedQuad = (QMessageBox::question(parameters.parentWidget,
					                                            "Result type",
					                                            "Import raster as a cloud (yes) or a texture quad? (no)",
					                                            QMessageBox::Yes,
					                                            QMessageBox::No)
					                      == QMessageBox::No);
				}
			}

			ccPointCloud* pc = new ccPointCloud();

			CCVector3d origin(adfGeoTransform[0], adfGeoTransform[3], 0.0);
			CCVector3d Pshift(0, 0, 0);
			// check for 'big' coordinates
			{
				bool preserveCoordinateShift = true;
				if (HandleGlobalShift(origin, Pshift, preserveCoordinateShift, parameters))
				{
					if (pc && preserveCoordinateShift)
					{
						pc->setGlobalShift(Pshift);
					}
					ccLog::Warning("[RasterFilter::loadFile] Raster has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
				}
			}

			// create blank raster 'grid'
			ccMesh* quad = nullptr;
			QImage  quadTexture;
			if (loadAsTexturedQuad)
			{
				quad = new ccMesh(pc);
				quad->addChild(pc);
				pc->setName("vertices");
				pc->setEnabled(false);

				// reserve memory
				quadTexture = QImage(rasterX, rasterY, QImage::Format_RGB32);
				if (!pc->reserve(4) || !quad->reserve(2) || quadTexture.size() != QSize(rasterX, rasterY))
				{
					delete quad;
					return CC_FERR_NOT_ENOUGH_MEMORY;
				}

				// B ------ C
				// |        |
				// A ------ D
				CCVector3d B = origin + Pshift; // origin is 'top left'
				CCVector3d C = B + CCVector3d(rasterX * adfGeoTransform[1], rasterX * adfGeoTransform[4], 0);
				CCVector3d A = B + CCVector3d(rasterY * adfGeoTransform[2], rasterY * adfGeoTransform[5], 0);
				CCVector3d D = C + CCVector3d(rasterY * adfGeoTransform[2], rasterY * adfGeoTransform[5], 0);

				pc->addPoint(A.toPC());
				pc->addPoint(B.toPC());
				pc->addPoint(C.toPC());
				pc->addPoint(D.toPC());

				quad->addTriangle(0, 2, 1); // A C B
				quad->addTriangle(0, 3, 2); // A D C
			}
			else
			{
				if (!pc->reserve(static_cast<unsigned>(rasterX * rasterY)))
				{
					delete pc;
					return CC_FERR_NOT_ENOUGH_MEMORY;
				}

				double z = 0.0 + Pshift.z;
				for (int j = 0; j < rasterY; ++j)
				{
					for (int i = 0; i < rasterX; ++i)
					{
						// Xgeo = adfGeoTransform(0) + Xpixel * adfGeoTransform(1) + Yline * adfGeoTransform(2)
						// Ygeo = adfGeoTransform(3) + Xpixel * adfGeoTransform(4) + Yline * adfGeoTransform(5)
						double    x = adfGeoTransform[0] + (i + 0.5) * adfGeoTransform[1] + (j + 0.5) * adfGeoTransform[2] + Pshift.x;
						double    y = adfGeoTransform[3] + (i + 0.5) * adfGeoTransform[4] + (j + 0.5) * adfGeoTransform[5] + Pshift.y;
						CCVector3 P(static_cast<PointCoordinateType>(x), static_cast<PointCoordinateType>(y), static_cast<PointCoordinateType>(z));
						pc->addPoint(P);
					}
				}
				QVariant xVar = QVariant::fromValue<int>(rasterX);
				QVariant yVar = QVariant::fromValue<int>(rasterY);
				pc->setMetaData("raster_width", xVar);
				pc->setMetaData("raster_height", yVar);
			}

			// fetch raster bands
			bool                      zRasterProcessed = false;
			CCCoreLib::ReferenceCloud validPoints(pc);
			double                    zMinMax[2]{0, 0};

			for (int i = 1; i <= rasterCount; ++i)
			{
				ccLog::Print("[GDAL] Reading band #%i", i);
				GDALRasterBand* poBand = poDataset->GetRasterBand(i);

				GDALColorInterp colorInterp = poBand->GetColorInterpretation();

				int nBlockXSize = 0, nBlockYSize = 0;
				poBand->GetBlockSize(&nBlockXSize, &nBlockYSize);
				ccLog::Print("[GDAL] Block=%dx%d, Type=%s, ColorInterp=%s", nBlockXSize, nBlockYSize, GDALGetDataTypeName(poBand->GetRasterDataType()), GDALGetColorInterpretationName(colorInterp));

				// fetching raster scan-line
				int nXSize = poBand->GetXSize();
				int nYSize = poBand->GetYSize();
				assert(nXSize == rasterX);
				assert(nYSize == rasterY);

				int    bGotMin = 0, bGotMax = 0;
				double adfMinMax[2]{poBand->GetMinimum(&bGotMin), poBand->GetMaximum(&bGotMax)};
				if (!bGotMin || !bGotMax)
				{
					// DGM FIXME: if the file is corrupted (e.g. ASCII ArcGrid with missing rows) this method will enter in a infinite loop!
					poBand->ComputeRasterMinMax(FALSE, adfMinMax);
				}
				ccLog::Print("[GDAL] Min=%.3fd, Max=%.3f", adfMinMax[0], adfMinMax[1]);

				GDALColorTable* colTable = poBand->GetColorTable();
				if (colTable != nullptr)
				{
					ccLog::Print("[GDAL] Band has a color table with %d entries", colTable->GetColorEntryCount());
				}

				if (poBand->GetOverviewCount() > 0)
				{
					ccLog::Print("[GDAL] Band has %d overviews", poBand->GetOverviewCount());
				}

				if (colorInterp == GCI_GrayIndex && !isColorBand[i - 1])
				{
					// Sometimes, GDAL flags a band as 'gray index' just because its values are between 0 and 255...
					colorInterp = GCI_Undefined;
				}

				if (!zRasterProcessed
				    && colorInterp == GCI_Undefined // probably heights? DGM: not sufficient since GDAL is sometimes lost if the bands are coded with 64 bits values :(
				    && (!hasUndefinedColorBands || !isColorBand[i - 1])
				    && !loadAsTexturedQuad
				    /*&& !colTable*/)
				{
					ccLog::Print("[GDAL] We will load this band to set Z coordinates");

					zRasterProcessed = true;
					zMinMax[0]       = adfMinMax[0];
					zMinMax[1]       = adfMinMax[1];

					size_t  byteCount = sizeof(double) * static_cast<size_t>(nXSize);
					double* scanline  = static_cast<double*>(CPLMalloc(byteCount));
					memset(scanline, 0, byteCount);

					if (!validPoints.reserve(pc->capacity()))
					{
						assert(!quad);
						delete pc;
						CPLFree(scanline);
						GDALClose(poDataset);
						return CC_FERR_READING;
					}

					int    hasNoDataValue = 0;
					double noDataValue    = poBand->GetNoDataValue(&hasNoDataValue);
					if (hasNoDataValue != 0)
					{
						ccLog::Print(QString("[GDAL] No data value = %1").arg(noDataValue));
					}

					for (int j = 0; j < nYSize; ++j)
					{
						poBand->SetColorInterpretation(GCI_Undefined);

						if (poBand->RasterIO(GF_Read,
						                     /*xOffset=*/0,
						                     /*yOffset=*/j,
						                     /*xSize=*/nXSize,
						                     /*ySize=*/1,
						                     /*buffer=*/scanline,
						                     /*bufferSizeX=*/nXSize,
						                     /*bufferSizeY=*/1,
						                     /*bufferType=*/GDT_Float64,
						                     /*x_offset=*/0,
						                     /*y_offset=*/0)
						    != CE_None)
						{
							assert(!quad);
							delete pc;
							CPLFree(scanline);
							GDALClose(poDataset);
							return CC_FERR_READING;
						}

						for (int k = 0; k < nXSize; ++k)
						{
							double   z          = static_cast<double>(scanline[k]) + Pshift.z;
							unsigned pointIndex = static_cast<unsigned>(k + j * rasterX);
							if (pointIndex <= pc->size())
							{
								const_cast<CCVector3*>(pc->getPoint(pointIndex))->z = static_cast<PointCoordinateType>(z);
								if (z >= zMinMax[0] && z <= zMinMax[1])
								{
									validPoints.addPointIndex(pointIndex);
								}
							}
						}
					}

					// update bounding-box
					pc->invalidateBoundingBox();

					if (scanline)
					{
						CPLFree(scanline);
					}
					scanline = 0;
				}
				else // colors or scalars
				{
					bool isRGB     = false;
					bool isScalar  = false;
					bool isPalette = false;

					switch (colorInterp)
					{
					case GCI_Undefined:
					case GCI_GrayIndex:
						isScalar = true;
						break;
					case GCI_PaletteIndex:
						isPalette = true;
						break;
					case GCI_RedBand:
					case GCI_GreenBand:
					case GCI_BlueBand:
						isRGB = true;
						break;
					case GCI_AlphaBand:
						if (adfMinMax[0] != adfMinMax[1])
						{
							if (loadAsTexturedQuad)
								isRGB = true;
							else
								isScalar = true; // we can't load the alpha band as a cloud color (transparency is not handled yet)
						}
						else
						{
							ccLog::Warning(QString("Alpha band ignored as it has a unique value (%1)").arg(adfMinMax[0]));
						}
						break;
					default:
						isScalar = true;
						break;
					}

					if (isRGB || isPalette)
					{
						// first check that a palette exists if the band is a palette index
						if (isPalette && !colTable)
						{
							ccLog::Warning(QString("Band is declared as a '%1' but no palette is associated!").arg(GDALGetColorInterpretationName(colorInterp)));
						}
						else
						{
							// instantiate memory for RBG colors if necessary
							if (!loadAsTexturedQuad && !pc->hasColors() && !pc->setColor(ccColor::white))
							{
								ccLog::Warning(QString("Failed to instantiate memory for storing color band '%1'!").arg(GDALGetColorInterpretationName(colorInterp)));
							}
							else
							{
								assert(poBand->GetRasterDataType() <= GDT_Int32);

								size_t byteCount  = sizeof(int) * static_cast<size_t>(nXSize);
								int*   colIndexes = static_cast<int*>(CPLMalloc(byteCount));
								memset(colIndexes, 0, byteCount);

								for (int j = 0; j < nYSize; ++j)
								{
									if (poBand->RasterIO(GF_Read,
									                     /*xOffset=*/0,
									                     /*yOffset=*/j,
									                     /*xSize=*/nXSize,
									                     /*ySize=*/1,
									                     /*buffer=*/colIndexes,
									                     /*bufferSizeX=*/nXSize,
									                     /*bufferSizeY=*/1,
									                     /*bufferType=*/GDT_Int32,
									                     /*x_offset=*/0,
									                     /*y_offset=*/0)
									    != CE_None)
									{
										CPLFree(colIndexes);
										if (quad)
											delete quad;
										else
											delete pc;
										return CC_FERR_READING;
									}

									for (int k = 0; k < nXSize; ++k)
									{
										unsigned pointIndex = static_cast<unsigned>(k + j * rasterX);
										if (loadAsTexturedQuad || pointIndex <= pc->size())
										{
											ccColor::Rgba C;
											if (loadAsTexturedQuad)
											{
												QRgb origColor = quadTexture.pixel(k, j);
												C              = ccColor::FromQRgba(origColor);
											}
											else
											{
												C = pc->getPointColor(pointIndex);
											}

											switch (colorInterp)
											{
											case GCI_PaletteIndex:
												assert(colTable);
												{
													GDALColorEntry col;
													colTable->GetColorEntryAsRGB(colIndexes[k], &col);
													C.r = static_cast<ColorCompType>(col.c1 & 255);
													C.g = static_cast<ColorCompType>(col.c2 & 255);
													C.b = static_cast<ColorCompType>(col.c3 & 255);
												}
												break;

											case GCI_RedBand:
												C.r = static_cast<ColorCompType>(colIndexes[k] & 255);
												break;
											case GCI_GreenBand:
												C.g = static_cast<ColorCompType>(colIndexes[k] & 255);
												break;
											case GCI_BlueBand:
												C.b = static_cast<ColorCompType>(colIndexes[k] & 255);
												break;

											case GCI_AlphaBand:
												C.a = static_cast<ColorCompType>(colIndexes[k] & 255);
												break;

											default:
												assert(false);
												break;
											}

											if (loadAsTexturedQuad)
											{
												quadTexture.setPixel(k, j, qRgba(C.r, C.g, C.b, C.a));
											}
											else
											{
												pc->setPointColor(pointIndex, C);
											}
										}
									}
								}

								if (colIndexes)
								{
									CPLFree(colIndexes);
								}
								colIndexes = nullptr;
							}
						}
					}
					else if (isScalar && !loadAsTexturedQuad)
					{
						QString        sfName = QString("band #%1 (%2)").arg(i).arg(GDALGetColorInterpretationName(colorInterp)); // SF names really need to be unique!
						ccScalarField* sf     = new ccScalarField(sfName.toStdString());
						if (!sf->resizeSafe(pc->size(), true, CCCoreLib::NAN_VALUE))
						{
							ccLog::Warning(QString("Failed to instantiate memory for storing '%1' as a scalar field!").arg(QString::fromStdString(sf->getName())));
							sf->release();
							sf = nullptr;
						}
						else
						{
							size_t  byteCount = sizeof(double) * static_cast<size_t>(nXSize);
							double* colValues = static_cast<double*>(CPLMalloc(byteCount));
							memset(colValues, 0, byteCount);

							for (int j = 0; j < nYSize; ++j)
							{
								if (poBand->RasterIO(GF_Read,
								                     /*xOffset=*/0,
								                     /*yOffset=*/j,
								                     /*xSize=*/nXSize,
								                     /*ySize=*/1,
								                     /*buffer=*/colValues,
								                     /*bufferSizeX=*/nXSize,
								                     /*bufferSizeY=*/1,
								                     /*bufferType=*/GDT_Float64,
								                     /*x_offset=*/0,
								                     /*y_offset=*/0)
								    != CE_None)
								{
									CPLFree(colValues);
									sf->release();
									delete pc;
									return CC_FERR_READING;
								}

								for (int k = 0; k < nXSize; ++k)
								{
									unsigned pointIndex = static_cast<unsigned>(k + j * rasterX);
									if (pointIndex <= pc->size())
									{
										ScalarType s = static_cast<ScalarType>(colValues[k]);
										sf->setValue(pointIndex, s);
									}
								}
							}

							if (colValues)
							{
								CPLFree(colValues);
							}
							colValues = nullptr;

							sf->computeMinAndMax();
							pc->addScalarField(sf);
							if (pc->getNumberOfScalarFields() == 1)
							{
								pc->setCurrentDisplayedScalarField(0);
							}
						}
					}
				}
			}

			if (quad)
			{
				ccPlane::SetQuadTexture(quad, quadTexture.mirrored());
				container.addChild(quad);
			}
			else if (pc)
			{
				if (!zRasterProcessed)
				{
					ccLog::Warning("Raster has no height (Z) information: you can convert one of its scalar fields to Z with 'Edit > Scalar Fields > Set SF as coordinate(s)'");
				}
				else if (validPoints.size() != 0 && validPoints.size() < pc->size())
				{
					// shall we remove the points with invalid heights?
					static bool s_alwaysRemoveInvalidHeights = false;
					int         result                       = QMessageBox::Yes;
					if (parameters.parentWidget) // otherwise it means we are in command line mode --> no popup
					{
						result = (s_alwaysRemoveInvalidHeights
						              ? QMessageBox::Yes
						              : QMessageBox::question(nullptr,
						                                      "Remove invalid points?",
						                                      "This raster has pixels with invalid heights. Shall we remove them?",
						                                      QMessageBox::Yes,
						                                      QMessageBox::YesToAll,
						                                      QMessageBox::No));
					}
					if (result != QMessageBox::No) // Yes = let's remove them
					{
						if (result == QMessageBox::YesToAll)
						{
							s_alwaysRemoveInvalidHeights = true;
						}

						ccLog::PrintDebug(QString("Removing %1 invalid points out of %2 points...").arg(pc->size() - validPoints.size()).arg(pc->size()));
						ccPointCloud* newPC = pc->partialClone(&validPoints);
						if (newPC)
						{
							delete pc;
							pc = newPC;
						}
						else
						{
							ccLog::Error("Not enough memory to remove the points with invalid heights!");
						}
					}
				}
				container.addChild(pc);

				// we give the priority to colors!
				if (pc->hasColors())
				{
					pc->showColors(true);
					pc->showSF(false);
				}
				else if (pc->hasScalarFields())
				{
					pc->showSF(true);
				}
			}

			GDALClose(poDataset);
		}
		else
		{
			return CC_FERR_UNKNOWN_FILE;
		}
	}
	catch (...)
	{
		return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
	}

	return CC_FERR_NO_ERROR;
}

#endif
