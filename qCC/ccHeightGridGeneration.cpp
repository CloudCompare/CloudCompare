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

//system
#include <assert.h>

//CClib 
#include <ScalarField.h>

//qCC
#include "ccCommon.h"
#include "ccHeightGridGeneration.h"

//qCC_db
#include <ccLog.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>

//Qt
#include <QImage>
#include <QDir>
#include <QFileDialog>
#include <QApplication>
#include <QImageWriter>
#include <QSettings>
#include <QMessageBox>

#ifdef CC_GDAL_SUPPORT
//GDAL
#include <gdal.h>
#include <gdal_priv.h>
#include <cpl_string.h>

//local
#include "ui_rasterExportOptionsDlg.h"
#include <QDialog>

class RasterExportOptionsDlg : public QDialog, public Ui::RasterExportOptionsDialog
{
public:

	RasterExportOptionsDlg(QWidget* parent = 0)
		: QDialog(parent)
		, Ui::RasterExportOptionsDialog()
	{
		setupUi(this);
		setWindowFlags(Qt::Tool);
	}
};

#endif

using namespace std;

//! Cell of a regular 2D height grid (height map)
struct HeightGridCell
{
	//! Default constructor
	HeightGridCell()
		: height(0)
		, nbPoints(0)
		, pointIndex(0)
	{}

	//! Value
	PointCoordinateType height;
	//! Number of points projected in this cell
	unsigned nbPoints;
	//! Nearest point index (if any)
	unsigned pointIndex;
};

//************************************************************************************************************************
ccPointCloud* ccHeightGridGeneration::Compute(	ccGenericPointCloud* cloud,
	double grid_step,
	const ccBBox& customBox,
	unsigned char proj_dimension,
	ProjectionType projectionType,
	EmptyCellFillOption fillEmptyCells/*=LEAVE_EMPTY*/,
	ProjectionType sfInterpolation/*=INVALID_PROJECTION_TYPE*/,
	double customEmptyCellsHeight/*=-1.0*/,
	bool generateCloud/*=true*/,
	bool generateImage/*=false*/,
	bool generateRaster/*=false*/,
	bool generateASCII/*=false*/,
	bool generateCountSF/*=false*/,
	bool resampleOriginalCloud/*=false*/,
	CCLib::GenericProgressCallback* progressCb/*=0*/)
{
	if (progressCb)
	{
		progressCb->reset();
		progressCb->setMethodTitle("Height grid generation");
		progressCb->start();
	}

	//we can't use the 'resample origin cloud' option with 'average height' projection
	if (projectionType == PROJ_AVERAGE_HEIGHT)
		resampleOriginalCloud = false;

	//=========================================================================================================
	ccLog::Print("[ccHeightGridGeneration] 1 - Initialization");
	ccLog::Print(QString("Input cloud: '%1' (%2 points)").arg(cloud->getName()).arg(cloud->size()));

	assert(proj_dimension<3);
	const unsigned char Z = proj_dimension;
	const unsigned char X = (Z==2 ? 0 : Z+1);
	const unsigned char Y = (X==2 ? 0 : X+1);

	ccBBox box = (customBox.isValid() ? customBox : cloud->getMyOwnBB());
	CCVector3d boxDiag(	static_cast<double>(box.maxCorner().x) - static_cast<double>(box.minCorner().x),
						static_cast<double>(box.maxCorner().y) - static_cast<double>(box.minCorner().y),
						static_cast<double>(box.maxCorner().z) - static_cast<double>(box.minCorner().z) );
	if (boxDiag.u[X] <= 0 || boxDiag.u[Y] <= 0)
	{
		ccLog::Error("[ccHeightGridGeneration] Invalid cloud bounding box!");
		return 0;
	}

	ccLog::Print(QString("\tX in [%1 - %2] (%3)").arg(box.minCorner().u[X]).arg(box.maxCorner().u[X]).arg(boxDiag.u[X]));
	ccLog::Print(QString("\tY in [%1 - %2] (%3)").arg(box.minCorner().u[Y]).arg(box.maxCorner().u[Y]).arg(boxDiag.u[Y]));

	// Initialization of the height grid :
	unsigned grid_size_X = (unsigned)ceil(boxDiag.u[X] / grid_step);
	unsigned grid_size_Y = (unsigned)ceil(boxDiag.u[Y] / grid_step);
	unsigned grid_total_size = grid_size_X * grid_size_Y;

	ccLog::Print(QString("\tGrid size: [%1 x %2]").arg(grid_size_X).arg(grid_size_Y));
	ccLog::Print(QString("\tCell count: %1").arg(grid_total_size));
	if (grid_total_size == 1)
	{
		if (QMessageBox::question(0,"Unexpected grid size","The generated grid will only have 1 cell! Do you want to proceed anyway?",QMessageBox::Yes,QMessageBox::No) == QMessageBox::No)
			return 0;
	}
	else if (grid_total_size > (1<<24)) //2^24 = 16 Mo
	{
		ccLog::Warning(QString("[ccHeightGridGeneration] The grid that will be generated is pretty huge (%1 million of cells). The process may take some time...").arg(static_cast<double>(grid_total_size)/1.0e6));
	}

	// memory allocation of the height grid
	bool memError = false;
	std::vector<HeightGridCell*> grid;
	try
	{
		grid.resize(grid_size_Y,0);
	}
	catch(std::bad_alloc)
	{
		memError = true;
	}

	if (!memError)
	{
		for (unsigned i=0; i<grid_size_Y; ++i)
		{
			grid[i] = new HeightGridCell[grid_size_X];
			if (!grid[i])
			{
				for (unsigned j=0; j<i; ++j)
					delete[] grid[j];
				memError = true;
				break;
			}
		}
	}

	if (memError)
	{
		ccLog::Error("[ccHeightGridGeneration] Not enough memory!");
		return 0;
	}

	//do we need to interpolate scalar fields?
	ccPointCloud* pc = (cloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(cloud) : 0);
	std::vector<double*> gridScalarFields;
	bool interpolateSF = (sfInterpolation != INVALID_PROJECTION_TYPE) && ((generateCloud && pc && pc->hasScalarFields()) || generateRaster);
	if (!memError && interpolateSF)
	{
		unsigned sfCount = pc->getNumberOfScalarFields();
		gridScalarFields.resize(sfCount,0);
		for (unsigned i=0; i<sfCount; ++i)
		{
			gridScalarFields[i] = new double[grid_total_size];
			if (!gridScalarFields[i])
			{
				ccLog::Error(QString("[ccHeightGridGeneration] Failed to allocate memory for SF '%1' (and potentially the next ones)!").arg(pc->getScalarField(i)->getName()));
				break;
			}
			//not necessary
			//memset(gridScalarFields[i],0,sizeof(double)*grid_total_size);
		}
	}

	//=========================================================================================================
	ccLog::Print("[ccHeightGridGeneration] 2 - Filling the height grid...");

	unsigned count = cloud->size();
	bool cancel = false;
	for (unsigned n=0; n<count; ++n)
	{
		const CCVector3* thePoint = cloud->getPoint(n);

		CCVector3d relativePos(	static_cast<double>(thePoint->x) - static_cast<double>(box.minCorner().x),
								static_cast<double>(thePoint->y) - static_cast<double>(box.minCorner().y),
								static_cast<double>(thePoint->z) - static_cast<double>(box.minCorner().z) );

		int i = static_cast<int>(relativePos.u[X]/grid_step);
		int j = static_cast<int>(relativePos.u[Y]/grid_step);

		//if we fall exactly on the max corner of the grid box
		if (i == static_cast<int>(grid_size_X) && relativePos.u[X] == grid_step * static_cast<double>(grid_size_X))
			--i;
		if (j == static_cast<int>(grid_size_Y) && relativePos.u[Y] == grid_step * static_cast<double>(grid_size_Y))
			--j;

		//we skip points outside the box!
		if (i < 0 || i >= static_cast<int>(grid_size_X) || j < 0 || j >= static_cast<int>(grid_size_Y))
			continue;

		assert(i >= 0 && j >= 0);

		HeightGridCell* aCell = grid[j]+i;
		unsigned& pointsInCell = aCell->nbPoints;
		if (pointsInCell)
		{
			switch (projectionType)
			{
			case PROJ_MINIMUM_HEIGHT:
				// Set the minimum height
				if (thePoint->u[Z] < aCell->height)
				{
					aCell->height = thePoint->u[Z];
					aCell->pointIndex = n;
				}
				break;
			case PROJ_MAXIMUM_HEIGHT:
				// Set the maximum height
				if (thePoint->u[Z] > aCell->height)
				{
					aCell->height = thePoint->u[Z];
					aCell->pointIndex = n;
				}
				break;
			case PROJ_AVERAGE_HEIGHT:
				// Sum the points heights
				aCell->height += thePoint->u[Z];
				break;
			default:
				assert(false);
				break;
			}
		}
		else
		{
			//for the first point, we simply have to store its height (in any case)
			aCell->height = thePoint->u[Z];
			aCell->pointIndex = n;
		}

		//scalar fields
		if (interpolateSF)
		{
			int pos = j*static_cast<int>(grid_size_X)+i; //pos in 2D SF grid(s)
			assert(pos < static_cast<int>(grid_total_size));
			for (unsigned k=0; k<gridScalarFields.size(); ++k)
			{
				if (gridScalarFields[k])
				{
					CCLib::ScalarField* sf = pc->getScalarField(k);
					assert(sf);
					ScalarType sfValue = sf->getValue(n);
					ScalarType formerValue = static_cast<ScalarType>(gridScalarFields[k][pos]);

					if (pointsInCell && ccScalarField::ValidValue(formerValue))
					{
						if (ccScalarField::ValidValue(sfValue))
						{
							switch (sfInterpolation)
							{
							case PROJ_MINIMUM_HEIGHT:
								// keep the minimum value
								gridScalarFields[k][pos] = std::min<double>(formerValue,sfValue);
								break;
							case PROJ_MAXIMUM_HEIGHT:
								// keep the maximum value
								gridScalarFields[k][pos] = std::max<double>(formerValue,sfValue);
								break;
							case PROJ_AVERAGE_HEIGHT:
								//we sum all values (we will divide them later)
								gridScalarFields[k][pos] += sfValue;
								break;
							default:
								break;
							}
						}
					}
					else
					{
						//for the first (vaild) point, we simply have to store its SF value (in any case)
						gridScalarFields[k][pos] = sfValue;
					}
				}
			}
		}

		pointsInCell++;

		if (progressCb)
		{
			progressCb->update(30.0f * static_cast<float>(n) / static_cast<float>(cloud->size()));
			if (progressCb->isCancelRequested())
			{
				cancel = true;
				break;
			}
		}
	}

	//update grids for 'average' cases
	if (!cancel)
	{
		if (sfInterpolation == PROJ_AVERAGE_HEIGHT)
		{
			for (size_t k=0; k<gridScalarFields.size(); ++k)
			{
				if (gridScalarFields[k])
				{
					double* _gridSF = gridScalarFields[k];
					for (unsigned j=0;j<grid_size_Y;++j)
					{
						HeightGridCell* cell = grid[j];
						for (unsigned i=0;i<grid_size_X;++i,++cell,++_gridSF)
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
		if (projectionType == PROJ_AVERAGE_HEIGHT)
		{
			for (unsigned j=0; j<grid_size_Y; ++j)
			{
				HeightGridCell* cell = grid[j];
				for (unsigned i=0; i<grid_size_X; ++i,++cell)
					if (cell->nbPoints > 1)
						cell->height /= static_cast<PointCoordinateType>(cell->nbPoints);
			}
		}
	}

	//=========================================================================================================
	ccLog::Print("[ccHeightGridGeneration] 3 - Computation of the average and extreme height values in the grid...");
	double minHeight=0.0, maxHeight=0.0, meanHeight=0.0;
	unsigned nonEmptyCells = 0; //non empty cells count
	if (!cancel)
	{
		for (unsigned i=0; i<grid_size_Y; ++i)
		{
			for (unsigned j=0; j<grid_size_X; ++j)
			{
				if (grid[i][j].nbPoints) //non empty cell
				{
					double h = static_cast<double>(grid[i][j].height);

					if (nonEmptyCells++)
					{
						if (h < minHeight)
							minHeight = h;
						else if (h > maxHeight)
							maxHeight = h;
						meanHeight += h;
					}
					else
					{
						meanHeight = minHeight = maxHeight = h;
					}
				}
			}
		}
	}

	//default output
	ccPointCloud* cloudGrid(0);

	if (cancel)
	{
		//simply skip it
	}
	else if (!nonEmptyCells)
	{
		ccLog::Warning("[ccHeightGridGeneration] grid is empty!");
	}
	else
	{
		//persistent settings
		QSettings settings;
		settings.beginGroup("HeightGridGeneration");

		meanHeight /= static_cast<double>(nonEmptyCells);

		ccLog::Print("\tMinimal height = %f", minHeight);
		ccLog::Print("\tAverage height = %f", meanHeight);
		ccLog::Print("\tMaximal height = %f", maxHeight);

		if (generateASCII || generateImage || generateRaster || generateCloud)
		{
			//=========================================================================================================
			if (fillEmptyCells == FILL_AVERAGE_HEIGHT)
			{
				//we set the custom height as the average one (so we will be able to ignore this strategy afterwards!)
				fillEmptyCells = FILL_CUSTOM_HEIGHT;
				customEmptyCellsHeight = meanHeight;
			}

			double empty_cell_height = 0.0;
			switch (fillEmptyCells)
			{
			case LEAVE_EMPTY:
				//nothing to do
				break;
			case FILL_MINIMUM_HEIGHT:
				empty_cell_height = minHeight;
				break;
			case FILL_MAXIMUM_HEIGHT:
				empty_cell_height = maxHeight;
				break;
			case FILL_CUSTOM_HEIGHT:
				//update min and max height by the way!
				if (customEmptyCellsHeight <= minHeight)
					minHeight = customEmptyCellsHeight;
				else if (customEmptyCellsHeight >= maxHeight)
					maxHeight = customEmptyCellsHeight;
				empty_cell_height = customEmptyCellsHeight;
				break;
			case FILL_AVERAGE_HEIGHT:
			default:
				assert(false);
			}

			if (fillEmptyCells != LEAVE_EMPTY)
			{
				ccLog::Print("[ccHeightGridGeneration] Empty cells (containing no projected point) will be filled with the followin value:");
				ccLog::Print("\tempty_cell_value = %f",minHeight);
			}

			//=========================================================================================================
			if (generateASCII)
			{
				ccLog::Print("[ccHeightGridGeneration] Saving the height grid as a text file...");

				QString asciiGridSavePath = settings.value("savePathASCIIGrid",QApplication::applicationDirPath()).toString();

				//open file saving dialog
				QString filter("ASCII file (*.txt)");
				QString outputFilename = QFileDialog::getSaveFileName(0,"Save height grid as ASCII file",asciiGridSavePath+QString("/height_grid_text_file.txt"),filter);
				if (!outputFilename.isNull())
				{
					FILE* pFile = fopen(qPrintable(outputFilename),"wt");
					if (pFile)
					{
						ccLog::Print(QString("\tOutput file: ") + outputFilename);

						for (unsigned j=0; j<grid_size_Y; ++j)
						{
							if (progressCb)
								progressCb->update(30.0f + 50.0f*(static_cast<float>(j)/static_cast<float>(grid_size_Y)));

							const HeightGridCell* aCell = grid[j];
							for (unsigned i=0; i<grid_size_X; ++i,++aCell)
								fprintf(pFile,"%.8f ", aCell->nbPoints ? aCell->height : empty_cell_height);

							fprintf(pFile,"\n");
						}

						fclose(pFile);
						pFile = 0;

						//save current export path to persistent settings
						settings.setValue("savePathASCIIGrid",QFileInfo(outputFilename).absolutePath());
					}
					else
					{
						ccLog::Warning(QString("[ccHeightGridGeneration] Failed to write '%1' file!").arg(outputFilename));
					}
				}
			}

			//=========================================================================================================
			if (generateImage)
			{
				ccLog::Print("[ccHeightGridGeneration] Saving the height grid as an image...");

				QImage bitmap8(grid_size_X,grid_size_Y,QImage::Format_Indexed8);
				if (!bitmap8.isNull())
				{
					// Build a custom palette (gray scale)
					QVector<QRgb> palette(256);
					for (unsigned i = 0; i < 256; i++)
						palette[i] = qRgba(i,i,i,255);
					double maxColorComp = 255.99; //.99 --> to avoid round-off issues later!

					if (fillEmptyCells == LEAVE_EMPTY)
					{
						palette[255] = qRgba(255,0,255,0); //magenta/transparent color for empty cells (in place of pure white)
						maxColorComp = 254.99;
					}

					bitmap8.setColorTable(palette);
					//bitmap8.fill(255);

					unsigned empty_cell_color_index = 0;
					switch (fillEmptyCells)
					{
					case LEAVE_EMPTY:
						empty_cell_color_index = 255; //should be transparent!
						break;
					case FILL_MINIMUM_HEIGHT:
						empty_cell_color_index = 0;
						break;
					case FILL_MAXIMUM_HEIGHT:
						empty_cell_color_index = 255;
						break;
					case FILL_CUSTOM_HEIGHT:
						{
							double normalizedHeight = (customEmptyCellsHeight-minHeight)/(maxHeight-minHeight);
							//min and max should have already been updated with custom empty cell height!
							assert(normalizedHeight>=0.0 && normalizedHeight<=1.0);
							empty_cell_color_index = (unsigned)floor(normalizedHeight*maxColorComp);
						}
						break;
					case FILL_AVERAGE_HEIGHT:
					default:
						assert(false);
					}

					ccLog::Print("\tempty_cell_color_index = %u",empty_cell_color_index);

					double range = maxHeight - minHeight;
					if (range < ZERO_TOLERANCE)
						range = 1.0;

					// Filling the image with grid values
					for (unsigned j=0; j<grid_size_Y; ++j)
					{
						const HeightGridCell* aCell = grid[j];
						for (unsigned i=0; i<grid_size_X; ++i,++aCell)
						{
							if (aCell->nbPoints)
							{
								double normalized_height = (static_cast<double>(aCell->height) - minHeight)/range;
								assert(normalized_height >= 0.0 && normalized_height <= 1.0);
								unsigned char val = static_cast<unsigned char>(floor(normalized_height*maxColorComp));
								bitmap8.setPixel(i,grid_size_Y-1-j,val);
							}
							else
							{
								bitmap8.setPixel(i,grid_size_Y-1-j,empty_cell_color_index);
							}
						}

						if (progressCb)
							progressCb->update(80.0f + 10.0f*(static_cast<float>(j)/static_cast<float>(grid_size_Y)));
					}

					//open file saving dialog
					{
						//add images output file filters
						QString filters;

						//we grab the list of supported image file formats (writing)
						QList<QByteArray> formats = QImageWriter::supportedImageFormats();
						if (formats.empty())
						{
							ccLog::Warning("No image format supported by your system?!\n(check that the 'imageformats' directory is alongside CC executable)");
						}
						else
						{
							//we convert this list into a proper "filters" string
							for (int i=0; i<formats.size(); ++i)
								filters.append(QString("%1 image (*.%2)\n").arg(QString(formats[i].data()).toUpper()).arg(formats[i].data()));

							QString imageSavePath = settings.value("savePathImage",QApplication::applicationDirPath()).toString();
							QString outputFilename = QFileDialog::getSaveFileName(0,"Save height grid image",imageSavePath+QString("/height_grid_image.%1").arg(formats[0].data()),filters);

							if (!outputFilename.isNull())
							{
								if (bitmap8.save(outputFilename))
								{
									ccLog::Print(QString("\tOutput file: ")+outputFilename);

									//save current export path to persistent settings
									settings.setValue("savePathImage",QFileInfo(outputFilename).absolutePath());
								}
								else
								{
									ccLog::Error("Failed to save image file! (check the access permissions\nand make sure that the 'imageformats' directory alongside CC executable contains the right DLL)");
								}
							}
						}
					}
				}
				else
				{
					ccLog::Error("[ccHeightGridGeneration] Failed to create output image! (not enough memory?)");
				}
			}

			//=========================================================================================================
			if (generateRaster)
			{
#ifdef CC_GDAL_SUPPORT
				ccLog::Print("[ccHeightGridGeneration] Saving the height grid as a raster...");

				bool keepGoing = true;

				GDALAllRegister();
				ccLog::PrintDebug("(GDAL drivers: %i)", GetGDALDriverManager()->GetDriverCount());

				const char *pszFormat = "GTiff";
				GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
				if (!poDriver)
				{
					ccLog::Warning("[GDAL] Driver %s is not supported", pszFormat);
					keepGoing = false;
				}

				if (keepGoing)
				{
					char** papszMetadata = poDriver->GetMetadata();
					if( !CSLFetchBoolean( papszMetadata, GDAL_DCAP_CREATE, FALSE ) )
					{
						ccLog::Warning("[GDAL] Driver %s doesn't support Create() method", pszFormat);
						keepGoing = false;
					}
				}

				QString outputFilename;
				if (keepGoing)
				{
					QString imageSavePath = settings.value("savePathImage",QApplication::applicationDirPath()).toString();
					outputFilename = QFileDialog::getSaveFileName(0,"Save height grid raster",imageSavePath+QString("/height_grid_raster.tif"),"geotiff (*.tif)");

					if (outputFilename.isNull())
						keepGoing = false;
					else
						//save current export path to persistent settings
						settings.setValue("savePathImage",QFileInfo(outputFilename).absolutePath());
				}

				//which (and how many) bands shall we create?
				bool heightBand = true; //height by default
				bool densityBand = false;
				bool allSFBands = false;
				int sfBandIndex = -1; //scalar field index
				int totalBands = 0;

				if (keepGoing)
				{
					bool hasSF = !gridScalarFields.empty();
					RasterExportOptionsDlg reoDlg;
					reoDlg.dimensionsLabel->setText(QString("%1 x %2").arg(grid_size_X).arg(grid_size_Y));
					reoDlg.exportHeightsCheckBox->setChecked(heightBand);
					reoDlg.exportDensityCheckBox->setChecked(densityBand);
					reoDlg.exportDisplayedSFCheckBox->setEnabled(hasSF);
					reoDlg.exportAllSFCheckBox->setEnabled(hasSF);
					reoDlg.exportAllSFCheckBox->setChecked(allSFBands);

					if (!reoDlg.exec())
						keepGoing = false;

					heightBand = reoDlg.exportHeightsCheckBox->isChecked();
					densityBand = reoDlg.exportDensityCheckBox->isChecked();
					if (hasSF)
					{
						allSFBands = reoDlg.exportAllSFCheckBox->isChecked();
						if (!allSFBands && reoDlg.exportDisplayedSFCheckBox->isChecked())
						{
							sfBandIndex = pc->getCurrentDisplayedScalarFieldIndex();
							if (sfBandIndex < 0)
								ccLog::Warning("[ccHeightGridGeneration] Cloud has no currently displayed SF!");
						}
					}

					totalBands = heightBand ? 1 : 0;
					if (densityBand)
						++totalBands;
					if (allSFBands)
						totalBands += static_cast<int>(gridScalarFields.size());
					else if (sfBandIndex >= 0)
						totalBands++;

					if (totalBands == 0)
					{
						ccLog::Warning("[ccHeightGridGeneration] Warning, can't output a raster with no band! (check export parameters)");
						keepGoing = false;
					}
				}

				if (keepGoing)
				{
					//data type
					GDALDataType dataType = (std::max(sizeof(PointCoordinateType),sizeof(ScalarType)) > 4 ? GDT_Float64 : GDT_Float32);

					char **papszOptions = NULL;
					GDALDataset* poDstDS = poDriver->Create(qPrintable(outputFilename),
						static_cast<int>(grid_size_X),
						static_cast<int>(grid_size_Y),
						totalBands,
						dataType, 
						papszOptions);

					if (poDstDS)
					{
						double shiftX = box.minCorner().u[X];
						double shiftY = box.minCorner().u[Y];

						double stepX = grid_step;
						double stepY = grid_step;
						if (cloud->isA(CC_TYPES::POINT_CLOUD))
						{
							const CCVector3d& shift = static_cast<ccPointCloud*>(cloud)->getGlobalShift();
							shiftX -= shift.u[X];
							shiftY -= shift.u[Y];

							double scale = cloud->getGlobalScale();
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

						double* scanline = (double*) CPLMalloc(sizeof(double)*grid_size_X);
						bool error = false;
						int currentBand = 0;

						//exort height band?
						if (heightBand)
						{
							GDALRasterBand* poBand = poDstDS->GetRasterBand(++currentBand);
							assert(poBand);
							poBand->SetColorInterpretation(GCI_Undefined);

							double empty_cell_height = 0;
							switch (fillEmptyCells)
							{
							case LEAVE_EMPTY:
								empty_cell_height = minHeight-1.0;
								poBand->SetNoDataValue(empty_cell_height); //should be transparent!
								break;
							case FILL_MINIMUM_HEIGHT:
								empty_cell_height = minHeight;
								break;
							case FILL_MAXIMUM_HEIGHT:
								empty_cell_height = maxHeight;
								break;
							case FILL_CUSTOM_HEIGHT:
								empty_cell_height = customEmptyCellsHeight;
								break;
							case FILL_AVERAGE_HEIGHT:
								empty_cell_height = meanHeight;
								break;
							default:
								assert(false);
							}
							//ccLog::Print("\tempty_cell_height = %f",empty_cell_height);

							for (unsigned j=0; j<grid_size_Y; ++j)
							{
								const HeightGridCell* aCell = grid[j];
								for (unsigned i=0; i<grid_size_X; ++i,++aCell)
								{
									scanline[i] = aCell->nbPoints ? static_cast<double>(aCell->height) : empty_cell_height;
								}

								if (poBand->RasterIO( GF_Write, 0, static_cast<int>(j), static_cast<int>(grid_size_X), 1, scanline, static_cast<int>(grid_size_X), 1, GDT_Float64, 0, 0 ) != CE_None)
								{
									ccLog::Error("[GDAL] An error occurred while writing the height band!");
									error = true;
									break;
								}
							}
						}

						//export density band
						if (!error && densityBand)
						{
							GDALRasterBand* poBand = poDstDS->GetRasterBand(++currentBand);
							assert(poBand);
							poBand->SetColorInterpretation(GCI_Undefined);
							for (unsigned j=0; j<grid_size_Y; ++j)
							{
								const HeightGridCell* aCell = grid[j];
								for (unsigned i=0; i<grid_size_X; ++i,++aCell)
								{
									scanline[i] = static_cast<double>(aCell->nbPoints);
								}

								if (poBand->RasterIO( GF_Write, 0, static_cast<int>(j), static_cast<int>(grid_size_X), 1, scanline, static_cast<int>(grid_size_X), 1, GDT_Float64, 0, 0 ) != CE_None)
								{
									ccLog::Error("[GDAL] An error occurred while writing the height band!");
									error = true;
									break;
								}
							}
						}

						//export SF bands
						if (!error && (allSFBands || sfBandIndex >= 0))
						{
							for (size_t k=0; k<gridScalarFields.size(); ++k)
							{
								double* _sfGrid = gridScalarFields[k];
								if (_sfGrid && (allSFBands || sfBandIndex == static_cast<int>(k))) //valid SF grid
								{
									GDALRasterBand* poBand = poDstDS->GetRasterBand(++currentBand);

									double sfNanValue = static_cast<double>(CCLib::ScalarField::NaN());
									poBand->SetNoDataValue(sfNanValue); //should be transparent!
									assert(poBand);
									poBand->SetColorInterpretation(GCI_Undefined);

									for (unsigned j=0; j<grid_size_Y; ++j)
									{
										const HeightGridCell* aCell = grid[j];
										for (unsigned i=0; i<grid_size_X; ++i,++_sfGrid,++aCell)
										{
											scanline[i] = aCell->nbPoints ? *_sfGrid : sfNanValue;
										}

										if (poBand->RasterIO( GF_Write, 0, static_cast<int>(j), static_cast<int>(grid_size_X), 1, scanline, static_cast<int>(grid_size_X), 1, GDT_Float64, 0, 0 ) != CE_None)
										{
											//the corresponding SF should exist on the input cloud
											CCLib::ScalarField* formerSf = pc->getScalarField(static_cast<int>(k));
											assert(formerSf);
											ccLog::Error(QString("[GDAL] An error occurred while writing the '%1' scalar field band!").arg(formerSf->getName()));
											error = true;
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
					}
					else
					{
						ccLog::Error("[GDAL] Failed to create output raster (not enough memory?)");
					}
				}
				else
				{
					ccLog::Error("[ccHeightGridGeneration] Failed to create output raster! (not enough memory?)");
				}
#else
				ccLog::Warning("[ccHeightGridGeneration] GDAL not supported by this version! Can't generate a raster...");
#endif
			}

			//=========================================================================================================
			if (generateCloud)
			{
				ccLog::Print("[ccHeightGridGeneration] Saving the height grid as a cloud...");

				unsigned pointsCount = (fillEmptyCells != LEAVE_EMPTY ? grid_size_X*grid_size_Y : nonEmptyCells);
				if (resampleOriginalCloud)
				{
					CCLib::ReferenceCloud refCloud(cloud);
					if (refCloud.reserve(cloud->size()))
					{
						for (unsigned j=0; j<grid_size_Y; ++j)
						{
							const HeightGridCell* aCell = grid[j];
							for (unsigned i=0; i<grid_size_X; ++i,++aCell)
							{
								if (aCell->nbPoints) //non empty cell
								{
									refCloud.addPointIndex(aCell->pointIndex);
								}
							}
						}

						if (refCloud.size() == 0)
						{
							ccLog::Warning("[ccHeightGridGeneration] No point fell in the grid?!");
						}
						else
						{
							cloudGrid = cloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(cloud)->partialClone(&refCloud) : ccPointCloud::From(&refCloud);
						}
					}
				}
				else
				{
					cloudGrid = new ccPointCloud("grid");
				}

				//even if we have already resampled the original cloud
				//we may have to create new points and/or scalar fields
				if (cloudGrid)
				{
					//per-point height SF
					CCLib::ScalarField* heightSF = 0;
					int heightSFIdx = -1;
					{
						heightSFIdx = cloudGrid->addScalarField(CC_HEIGHT_GRID_FIELD_NAME);
						if (heightSFIdx < 0)
						{
							ccLog::Warning("[ccHeightGridGeneration] Couldn't allocate a new scalar field for storing height grid values! Try to free some memory ...");
						}
						else
						{
							heightSF = cloudGrid->getScalarField(heightSFIdx);
							assert(heightSF);
						}
					}

					//shall we save per-cell population as well?
					CCLib::ScalarField* countSF = 0;
					int countSFIdx = -1;
					if (generateCountSF)
					{
						countSFIdx = cloudGrid->addScalarField("Per-cell population");
						if (countSFIdx < 0)
						{
							ccLog::Warning("[ccHeightGridGeneration] Couldn't allocate a new scalar field for storing per-cell population count! Try to free some memory ...");
						}
						else
						{
							countSF = cloudGrid->getScalarField(countSFIdx);
							assert(countSF);
						}
					}

					//the resampled cloud already contains the points corresponding to 'filled' cells so we will only
					//need to add the empty ones (if requested)
					if ((!resampleOriginalCloud || fillEmptyCells != LEAVE_EMPTY) && !cloudGrid->reserve(pointsCount))
					{
						ccLog::Warning("[ccHeightGridGeneration] Couldn't allocate memory for output cloud!");
						assert(cloudGrid->size() == 0);
						delete cloudGrid;
						cloudGrid = 0;
					}
					else
					{
						//we work with doubles as grid step can be much smaller than the cloud coordinates!
						double Py = static_cast<double>(box.minCorner().u[Y]);

						//as the 'non empty cells points' are already in the cloud
						//we must take care were we put the scalar fields values!
						unsigned nonEmptyCellIndex = 0;

						for (unsigned j=0; j<grid_size_Y; ++j)
						{
							const HeightGridCell* aCell = grid[j];
							double Px = static_cast<double>(box.minCorner().u[X]);
							for (unsigned i=0; i<grid_size_X; ++i,++aCell)
							{
								if (aCell->nbPoints) //non empty cell
								{
									//if we haven't resampled the original cloud, we must add the point
									//corresponding to this non-empty cell
									if (!resampleOriginalCloud)
									{
										double Pz = static_cast<double>(aCell->height);

										CCVector3 Pf(	static_cast<PointCoordinateType>(Px),
														static_cast<PointCoordinateType>(Py),
														static_cast<PointCoordinateType>(Pz) );

										cloudGrid->addPoint(Pf);
									}

									//if a SF is available, we set the point height equal to this cell's height
									assert(nonEmptyCellIndex < cloud->size());
									if (heightSF)
									{
										ScalarType h = static_cast<ScalarType>(aCell->height);
										if (resampleOriginalCloud)
											heightSF->setValue(nonEmptyCellIndex,h);
										else
											heightSF->addElement(h);
									}
									//same thing for the 'per-cell population' SF
									if (countSF)
									{
										ScalarType pop = static_cast<ScalarType>(aCell->nbPoints);
										if (resampleOriginalCloud)
											countSF->setValue(nonEmptyCellIndex,pop);
										else
											countSF->addElement(pop);
									}
									++nonEmptyCellIndex;
								}
								else if (fillEmptyCells != LEAVE_EMPTY) //empty cell
								{
									//even if we have resampled the original cloud, we must add the point
									//corresponding to this empty cell
									{
										CCVector3 Pf(	static_cast<PointCoordinateType>(Px),
														static_cast<PointCoordinateType>(Py),
														static_cast<PointCoordinateType>(empty_cell_height) );
										cloudGrid->addPoint(Pf);
									}

									//if a SF is available, we set the point height to the default height
									if (heightSF)
									{
										ScalarType s = static_cast<ScalarType>(empty_cell_height);
										heightSF->addElement(s);
									}
									if (countSF)
									{
										countSF->addElement(NAN_VALUE);
									}
								}

								Px += grid_step;
							}

							Py += grid_step;
						}

						if (heightSF)
						{
							heightSF->computeMinAndMax();
							cloudGrid->setCurrentDisplayedScalarField(heightSFIdx);
						}
						if (countSF)
						{
							countSF->computeMinAndMax();
							if (!heightSF)
								cloudGrid->setCurrentDisplayedScalarField(countSFIdx);
						}
						cloudGrid->showSF(heightSF || countSF);

						//take care of former scalar fields
						if (!resampleOriginalCloud)
						{
							for (size_t k=0; k<gridScalarFields.size(); ++k)
							{
								double* _sfGrid = gridScalarFields[k];
								if (_sfGrid) //valid SF grid
								{
									//the corresponding SF should exist on the input cloud
									CCLib::ScalarField* formerSf = pc->getScalarField(static_cast<int>(k));
									assert(formerSf);

									//we try to create an equivalent SF on the output grid
									int sfIdx = cloudGrid->addScalarField(formerSf->getName());
									if (sfIdx < 0) //if we aren't lucky, the input cloud already had a SF with CC_HEIGHT_GRID_FIELD_NAME as name
										sfIdx = cloudGrid->addScalarField(qPrintable(QString(formerSf->getName()).append(".old")));

									if (sfIdx < 0)
										ccLog::Warning("[ccHeightGridGeneration] Couldn't allocate a new scalar field for storing SF '%s' values! Try to free some memory ...",formerSf->getName());
									else
									{
										CCLib::ScalarField* sf = cloudGrid->getScalarField(sfIdx);
										assert(sf);
										//set sf values
										unsigned n = 0;
										const ScalarType emptyCellSFValue = CCLib::ScalarField::NaN();
										for (unsigned j=0; j<grid_size_Y; ++j)
										{
											const HeightGridCell* aCell = grid[j];
											for (unsigned i=0; i<grid_size_X; ++i, ++_sfGrid, ++aCell)
											{
												if (aCell->nbPoints)
												{
													ScalarType s = static_cast<ScalarType>(*_sfGrid);
													sf->setValue(n++,s);
												}
												else if (fillEmptyCells != LEAVE_EMPTY)
												{
													sf->setValue(n++,emptyCellSFValue);
												}
											}
										}
										sf->computeMinAndMax();
										assert(sf->currentSize() == pointsCount);
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
					}
				}
				else if (!cloudGrid)
				{
					ccLog::Warning("[ccHeightGridGeneration] Couldn't create cloud! (not enough memory)");
					delete cloudGrid;
					cloudGrid = 0;
				}
			}
		}
	}

	ccLog::Print("This is the end my friend...");

	// release the height grid...
	{
		for (size_t i=0; i<grid.size(); ++i)
			if (grid[i])
				delete[] grid[i];
		grid.clear();
	}

	//...and the scalar fields
	{
		for (unsigned i=0; i<gridScalarFields.size(); ++i)
		{
			if (gridScalarFields[i])
				delete[] gridScalarFields[i];
		}
		gridScalarFields.clear();
	}

	if (progressCb)
		progressCb->stop();

	return cloudGrid;
}
