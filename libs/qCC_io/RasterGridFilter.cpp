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

#ifdef CC_GDAL_SUPPORT

#include "RasterGridFilter.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>

//GDAL
#include <gdal_priv.h>
#include <cpl_conv.h> // for CPLMalloc()

//Qt
#include <QMessageBox>

//System
#include <string.h> //for memset

bool RasterGridFilter::canLoadExtension(QString upperCaseExt) const
{
	return (	upperCaseExt == "TIF"
			||	upperCaseExt == "TIFF"
			||	upperCaseExt == "ADF");
}

bool RasterGridFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	//not supported yet
	return false;
}

CC_FILE_ERROR RasterGridFilter::loadFile(QString filename, ccHObject& container, LoadParameters& parameters)
{
	GDALAllRegister();
	ccLog::PrintDebug("(GDAL drivers: %i)", GetGDALDriverManager()->GetDriverCount());

	try
	{
		GDALDataset* poDataset = static_cast<GDALDataset*>(GDALOpen( qPrintable(filename), GA_ReadOnly ));

		if( poDataset != NULL )
		{
			ccLog::Print(QString("Raster file: '%1'").arg(filename));
			ccLog::Print( "Driver: %s/%s",
				poDataset->GetDriver()->GetDescription(), 
				poDataset->GetDriver()->GetMetadataItem( GDAL_DMD_LONGNAME ) );

			int rasterCount = poDataset->GetRasterCount();
			int rasterX = poDataset->GetRasterXSize();
			int rasterY = poDataset->GetRasterYSize();
			ccLog::Print( "Size is %dx%dx%d", rasterX, rasterY, rasterCount );

			ccPointCloud* pc = new ccPointCloud();
			if (!pc->reserve(static_cast<unsigned>(rasterX * rasterY)))
			{
				delete pc;
				return CC_FERR_NOT_ENOUGH_MEMORY;
			}

			if( poDataset->GetProjectionRef() != NULL )
				ccLog::Print( "Projection is `%s'", poDataset->GetProjectionRef() );

			double adfGeoTransform[6] = {	 0, //top left x
											 1, //w-e pixel resolution (can be negative)
											 0, //0
											 0, //top left y
											 0, //0
											 1  //n-s pixel resolution (can be negative)
			};

			if( poDataset->GetGeoTransform( adfGeoTransform ) == CE_None )
			{
				ccLog::Print( "Origin = (%.6f,%.6f)", adfGeoTransform[0], adfGeoTransform[3] );
				ccLog::Print( "Pixel Size = (%.6f,%.6f)", adfGeoTransform[1], adfGeoTransform[5] );
			}

			if (adfGeoTransform[1] == 0 || adfGeoTransform[5] == 0)
			{
				ccLog::Warning("Invalid pixel size! Forcing it to (1,1)");
				adfGeoTransform[1] = adfGeoTransform[5] = 1;
			}

			CCVector3d origin( adfGeoTransform[0], adfGeoTransform[3], 0.0 );
			CCVector3d Pshift(0,0,0);
			//check for 'big' coordinates
			{
				if (HandleGlobalShift(origin,Pshift,parameters))
				{
					pc->setGlobalShift(Pshift);
					ccLog::Warning("[RasterFilter::loadFile] Raster has been recentered! Translation: (%.2f,%.2f,%.2f)",Pshift.x,Pshift.y,Pshift.z);
				}
			}

			//create blank raster 'grid'
			{
				double z = 0.0 /*+ Pshift.z*/;
				for (int j=0; j<rasterY; ++j)
				{
					for (int i=0; i<rasterX; ++i)
					{
						double x = adfGeoTransform[0] + static_cast<double>(i) * adfGeoTransform[1] + static_cast<double>(j) * adfGeoTransform[2] + Pshift.x;
						double y = adfGeoTransform[3] + static_cast<double>(i) * adfGeoTransform[4] + static_cast<double>(j) * adfGeoTransform[5] + Pshift.y;
						CCVector3 P(static_cast<PointCoordinateType>(x), static_cast<PointCoordinateType>(y), static_cast<PointCoordinateType>(z));
						pc->addPoint(P);
					}
				}
				QVariant xVar = QVariant::fromValue<int>(rasterX);
				QVariant yVar = QVariant::fromValue<int>(rasterY);
				pc->setMetaData("raster_width",xVar);
				pc->setMetaData("raster_height",yVar);
			}

			//fetch raster bands
			bool zRasterProcessed = false;
			unsigned zInvalid = 0;
			double zMinMax[2] = {0, 0};

			for (int i=1; i<=rasterCount; ++i)
			{
				ccLog::Print( "Reading band #%i", i);
				GDALRasterBand* poBand = poDataset->GetRasterBand(i);

				GDALColorInterp colorInterp = poBand->GetColorInterpretation();

				int nBlockXSize, nBlockYSize;
				poBand->GetBlockSize( &nBlockXSize, &nBlockYSize );
				ccLog::Print( "Block=%dx%d Type=%s, ColorInterp=%s", nBlockXSize, nBlockYSize, GDALGetDataTypeName(poBand->GetRasterDataType()), GDALGetColorInterpretationName(colorInterp) );

				//fetching raster scan-line
				int nXSize = poBand->GetXSize();
				int nYSize = poBand->GetYSize();
				assert(nXSize == rasterX);
				assert(nYSize == rasterY);
			
				int bGotMin, bGotMax;
				double adfMinMax[2] = {0, 0};
				adfMinMax[0] = poBand->GetMinimum( &bGotMin );
				adfMinMax[1] = poBand->GetMaximum( &bGotMax );
				if (!bGotMin || !bGotMax )
					//DGM FIXME: if the file is corrupted (e.g. ASCII ArcGrid with missing rows) this method will enter in a infinite loop!
					GDALComputeRasterMinMax((GDALRasterBandH)poBand, TRUE, adfMinMax);
				ccLog::Print( "Min=%.3fd, Max=%.3f", adfMinMax[0], adfMinMax[1] );

				GDALColorTable* colTable = poBand->GetColorTable();
				if( colTable != NULL )
					printf( "Band has a color table with %d entries", colTable->GetColorEntryCount() );

				if( poBand->GetOverviewCount() > 0 )
					printf( "Band has %d overviews", poBand->GetOverviewCount() );

				if (colorInterp == GCI_Undefined && !zRasterProcessed/*&& !colTable*/) //probably heights?
				{
					zRasterProcessed = true;
					zMinMax[0] = adfMinMax[0];
					zMinMax[1] = adfMinMax[1];

					double* scanline = (double*) CPLMalloc(sizeof(double)*nXSize);
					//double* scanline = new double[nXSize];
					memset(scanline,0,sizeof(double)*nXSize);

					for (int j=0; j<nYSize; ++j)
					{
						if (poBand->RasterIO( GF_Read, /*xOffset=*/0, /*yOffset=*/j, /*xSize=*/nXSize, /*ySize=*/1, /*buffer=*/scanline, /*bufferSizeX=*/nXSize, /*bufferSizeY=*/1, /*bufferType=*/GDT_Float64, /*x_offset=*/0, /*y_offset=*/0 ) != CE_None)
						{
							delete pc;
							CPLFree(scanline);
							GDALClose(poDataset);
							return CC_FERR_READING;
						}

						for (int k=0; k<nXSize; ++k)
						{
							double z = static_cast<double>(scanline[k]) + Pshift[2];
							unsigned pointIndex = static_cast<unsigned>(k + j * rasterX);
							if (pointIndex <= pc->size())
							{
								if (z < zMinMax[0] || z > zMinMax[1])
								{
									z = zMinMax[0] - 1.0;
									++zInvalid;
								}
								const_cast<CCVector3*>(pc->getPoint(pointIndex))->z = static_cast<PointCoordinateType>(z);
							}
						}
					}

					//update bounding-box
					pc->invalidateBoundingBox();

					if (scanline)
						CPLFree(scanline);
					scanline = 0;
				}
				else //colors
				{
					bool isRGB = false;
					bool isScalar = false;
					bool isPalette = false;
				
					switch(colorInterp)
					{
					case GCI_Undefined:
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
							isScalar = true;
						else
							ccLog::Warning(QString("Alpha band ignored as it has a unique value (%1)").arg(adfMinMax[0]));
						break;
					default:
						isScalar = true;
						break;
					}


					if (isRGB || isPalette)
					{
						//first check that a palette exists if the band is a palette index
						if (isPalette && !colTable)
						{
							ccLog::Warning(QString("Band is declared as a '%1' but no palette is associated!").arg(GDALGetColorInterpretationName(colorInterp)));
							isPalette = false;
						}
						else
						{
							//instantiate memory for RBG colors if necessary
							if (!pc->hasColors() && !pc->setRGBColor(ccColor::MAX,ccColor::MAX,ccColor::MAX))
							{
								ccLog::Warning(QString("Failed to instantiate memory for storing color band '%1'!").arg(GDALGetColorInterpretationName(colorInterp)));
							}
							else
							{
								assert(bandType <= GDT_Int32);

								int* colIndexes = (int*) CPLMalloc(sizeof(int)*nXSize);
								//double* scanline = new double[nXSize];
								memset(colIndexes,0,sizeof(int)*nXSize);

								for (int j=0; j<nYSize; ++j)
								{
									if (poBand->RasterIO( GF_Read, /*xOffset=*/0, /*yOffset=*/j, /*xSize=*/nXSize, /*ySize=*/1, /*buffer=*/colIndexes, /*bufferSizeX=*/nXSize, /*bufferSizeY=*/1, /*bufferType=*/GDT_Int32, /*x_offset=*/0, /*y_offset=*/0 ) != CE_None)
									{
										CPLFree(colIndexes);
										delete pc;
										return CC_FERR_READING;
									}

									for (int k=0; k<nXSize; ++k)
									{
										unsigned pointIndex = static_cast<unsigned>(k + j * rasterX);
										if (pointIndex <= pc->size())
										{
											colorType* C = const_cast<colorType*>(pc->getPointColor(pointIndex));

											switch(colorInterp)
											{
											case GCI_PaletteIndex:
												assert(colTable);
												{
													GDALColorEntry col;
													colTable->GetColorEntryAsRGB(colIndexes[k],&col);
													C[0] = static_cast<colorType>(col.c1 & ccColor::MAX);
													C[1] = static_cast<colorType>(col.c2 & ccColor::MAX);
													C[2] = static_cast<colorType>(col.c3 & ccColor::MAX);
												}
												break;

											case GCI_RedBand:
												C[0] = static_cast<colorType>(colIndexes[k] & ccColor::MAX);
												break;
											case GCI_GreenBand:
												C[1] = static_cast<colorType>(colIndexes[k] & ccColor::MAX);
												break;
											case GCI_BlueBand:
												C[2] = static_cast<colorType>(colIndexes[k] & ccColor::MAX);
												break;

											default:
												assert(false);
												break;
											}
										}
									}
								}

								if (colIndexes)
									CPLFree(colIndexes);
								colIndexes = 0;

								pc->showColors(true);
							}
						}
					}
					else if (isScalar)
					{
						ccScalarField* sf = new ccScalarField(GDALGetColorInterpretationName(colorInterp));
						if (!sf->resize(pc->size(),true,NAN_VALUE))
						{
							ccLog::Warning(QString("Failed to instantiate memory for storing '%1' as a scalar field!").arg(sf->getName()));
							sf->release();
							sf = 0;
						}
						else
						{
							double* colValues = (double*) CPLMalloc(sizeof(double)*nXSize);
							//double* scanline = new double[nXSize];
							memset(colValues,0,sizeof(double)*nXSize);

							for (int j=0; j<nYSize; ++j)
							{
								if (poBand->RasterIO( GF_Read, /*xOffset=*/0, /*yOffset=*/j, /*xSize=*/nXSize, /*ySize=*/1, /*buffer=*/colValues, /*bufferSizeX=*/nXSize, /*bufferSizeY=*/1, /*bufferType=*/GDT_Float64, /*x_offset=*/0, /*y_offset=*/0 ) != CE_None)
								{
									CPLFree(colValues);
									delete pc;
									return CC_FERR_READING;
								}

								for (int k=0; k<nXSize; ++k)
								{
									unsigned pointIndex = static_cast<unsigned>(k + j * rasterX);
									if (pointIndex <= pc->size())
									{
										ScalarType s = static_cast<ScalarType>(colValues[k]);
										sf->setValue(pointIndex,s);
									}
								}
							}

							if (colValues)
								CPLFree(colValues);
							colValues = 0;

							sf->computeMinAndMax();
							pc->addScalarField(sf);
							if (pc->getNumberOfScalarFields() == 1)
								pc->setCurrentDisplayedScalarField(0);
							pc->showSF(true);
						}
					}
				}
			}

			if (pc)
			{
				if (!zRasterProcessed)
				{
					ccLog::Warning("Raster has no height (Z) information: you can convert one of its scalar fields to Z with 'Edit > Scalar Fields > Set SF as coordinate(s)'");
				}
				else if (zInvalid != 0 && zInvalid < pc->size())
				{
					//shall we remove the points with invalid heights?
					if (QMessageBox::question(0,"Remove NaN points?","This raster has pixels with invalid heights. Shall we remove them?",QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
					{
						CCLib::ReferenceCloud validPoints(pc);
						unsigned count = pc->size();
						bool error = true;
						if (validPoints.reserve(count-zInvalid))
						{
							for (unsigned i=0; i<count; ++i)
							{
								if (pc->getPoint(i)->z >= zMinMax[0])
									validPoints.addPointIndex(i);
							}

							if (validPoints.size() > 0)
							{
								validPoints.resize(validPoints.size());
								ccPointCloud* newPC = pc->partialClone(&validPoints);
								if (newPC)
								{
									delete pc;
									pc = newPC;
									error = false;
								}
							}
							else
							{
								assert(false);
							}
						}

						if (error)
						{
							ccLog::Error("Not enough memory to remove the points with invalid heights!");
						}
					}
				}
				container.addChild(pc);
			}

			GDALClose(poDataset);
		}
		else
		{
			return CC_FERR_UNKNOWN_FILE;
		}
	}
	catch(...)
	{
		return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
	}

	return CC_FERR_NO_ERROR;
}

#endif
