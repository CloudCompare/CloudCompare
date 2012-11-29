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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2274                                                              $
//$LastChangedDate:: 2012-10-17 19:17:38 +0200 (mer., 17 oct. 2012)        $
//**************************************************************************
//

//system
#include <assert.h>

//CClib 
#include <ScalarField.h>

//qCC
#include "ccConsole.h"
#include "ccCommon.h"
#include "ccHeightGridGeneration.h"

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>

//Qt
#include <QImage>
#include <QDir>

using namespace std;

//! Cell of a regular 2D height grid (height map)
struct hgCell
{
    //! Default constructor
    hgCell()
    {
        height=0.0;
        nbPoints=0;
    }

    //! Value
    float height;
    //! Number of points projected in this cell
    unsigned nbPoints;
};

//************************************************************************************************************************
void ccHeightGridGeneration::Compute(ccGenericPointCloud* cloud,
                                     float grid_step,
									 unsigned char proj_dimension,
                                     ProjectionType projectionType,
                                     EmptyCellFillOption fillEmptyCells/*=LEAVE_EMPTY*/,
									 ProjectionType sfInterpolation/*=INVALID_PROJECTION_TYPE*/,
                                     double customEmptyCellsHeight/*=-1.0*/,
                                     bool generateImage /*= true*/,
                                     bool generateASCII /*= false*/,
                                     ccPointCloud* cloudGrid/*=0*/,
                                     CCLib::GenericProgressCallback* progressCb/*=0*/)
{
    if (progressCb)
    {
        progressCb->reset();
        progressCb->setMethodTitle("Height grid generation");
        progressCb->start();
    }

    //=========================================================================================================
    ccConsole::Print("[ccHeightGridGeneration] 1 - Initialization");
	ccConsole::Print(QString("Input cloud: '%1' (%2 points)").arg(cloud->getName()).arg(cloud->size()));

	assert(proj_dimension<3);
	const unsigned char Z = proj_dimension;
	const unsigned char X  = (Z==2 ? 0 : Z+1);
	const unsigned char Y  = (X==2 ? 0 : X+1);

    PointCoordinateType Mins[3], Maxs[3];
    cloud->getBoundingBox(Mins, Maxs);
    double delta_X = Maxs[X]-Mins[X]; // calcul de la longueur d'un côté du cube englobant la scène
    double delta_Y = Maxs[Y]-Mins[Y];
	if (delta_X<=0 || delta_Y<=0)
	{
		ccLog::Error("[ccHeightGridGeneration] Invalid cloud bounding box!");
		return;
	}

    ccConsole::Print("\tX max = %8.1f - X min = %8.1f",Maxs[X],Mins[Y]);
    ccConsole::Print("\tdelta_X = %8.1f",delta_X);
    ccConsole::Print("\tY max = %8.1f - Y min = %8.1f",Maxs[X],Mins[Y]);
    ccConsole::Print("\tdelta_Y = %8.1f",delta_Y);

    // Initialization of the height grid :
    unsigned grid_size_X = (unsigned)(ceil(delta_X / grid_step));
    unsigned grid_size_Y = (unsigned)(ceil(delta_Y / grid_step));
    unsigned grid_total_size = grid_size_X * grid_size_Y;

    ccConsole::Print("\tgrid_size_X = %i",grid_size_X);
    ccConsole::Print("\tgrid_size_Y = %i",grid_size_Y);
    ccConsole::Print("\tgrid_total_size = %i", grid_total_size);
    if (grid_total_size > (1<<24)) //2^24 = 16 Mo
        ccConsole::Warning("[ccHeightGridGeneration] The grid that will be generated is pretty huge (%i millions of cells) and this may take some time...", (double)grid_total_size/1e6);

	// memory allocation of the height grid
	bool memError=true;
    hgCell** grid = new hgCell*[grid_size_Y];
	if (grid)
	{
		memset(grid,0,sizeof(grid_size_Y)*sizeof(void*));
		memError=false;
		for (unsigned i=0; i<grid_size_Y; ++i)
		{
			grid[i] = new hgCell[grid_size_X];
			if (!grid[i])
			{
				for (unsigned j=0; j<i; ++j)
					delete[] grid[j];
				delete[] grid;
				memError=true;
				break;
			}
		}
	}

	if (memError)
	{
		ccLog::Error("[ccHeightGridGeneration] Not enough memory!");
		return;
	}

	//do we need to interpolate scalar fields?
	ccPointCloud* pc = (cloud->isA(CC_POINT_CLOUD) ? static_cast<ccPointCloud*>(cloud) : 0);
	std::vector<double*> gridScalarFields;
	bool interpolateSF = (sfInterpolation != INVALID_PROJECTION_TYPE) && cloudGrid && pc && pc->hasScalarFields();
	if (!memError && interpolateSF)
	{
		unsigned sfCount = pc->getNumberOfScalarFields();
		gridScalarFields.resize(sfCount,0);
		for (unsigned i=0;i<sfCount;++i)
		{
			gridScalarFields[i] = new double[grid_total_size];
			if (gridScalarFields[i])
			{
				//init grid (DGM: not necessary)
				//DistanceType emptyValue = pc->getScalarField(i)->isPositive() ? HIDDEN_VALUE : BIG_VALUE;
				//double* _grid = gridScalarFields[i];
				//for (unsigned j=0;j<grid_total_size;++j)
				//	*_grid++ = emptyValue;
			}
			else
			{
				ccLog::Error(QString("[ccHeightGridGeneration] Failed to allocate memory for SF '%1' (and potentially the next ones)!").arg(cloudGrid->getScalarField(i)->getName()));
				break;
			}
		}
	}

    //=========================================================================================================
    ccConsole::Print("[ccHeightGridGeneration] 2 - Filling the height grid...");

	unsigned count=cloud->size();
    for (unsigned n=0;n<count;++n)
    {
        const CCVector3 *thePoint = cloud->getPoint(n);

        int j = (int)((thePoint->u[Y]-Mins[Y])/grid_step);
        if (j==(int)grid_size_Y)
            --j;
        int i = (int)((thePoint->u[X]-Mins[X])/grid_step);
        if (i==(int)grid_size_X)
            --i;

		assert((i>=0) && (j>=0));

		hgCell* aCell = grid[j]+i;
        unsigned& pointsInCell = aCell->nbPoints;
		if (pointsInCell)
		{
			switch (projectionType)
			{
			case PROJ_MINIMUM_HEIGHT:
				// Set the minimum height
				if (thePoint->u[Z] < aCell->height)
					aCell->height = thePoint->u[Z];
				break;
			case PROJ_MAXIMUM_HEIGHT:
				// Set the maximum height
				if (thePoint->u[Z] > aCell->height)
					aCell->height = thePoint->u[Z];
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
		}

		//scalar fields
		if (interpolateSF)
		{
			int pos = j*(int)grid_size_X+i; //pos in 2D SF grid(s)
			assert(pos<(int)grid_total_size);
			for (unsigned k=0;k<gridScalarFields.size(); ++k)
			{
				if (gridScalarFields[k])
				{
					CCLib::ScalarField* sf = pc->getScalarField(k);
					assert(sf);
					DistanceType sfValue = sf->getValue(n);
					DistanceType formerValue = gridScalarFields[k][pos];
					bool formerIsNaN = (sf->isPositive() ? formerValue==HIDDEN_VALUE : formerValue==BIG_VALUE);

					if (pointsInCell && !formerIsNaN)
					{
						bool isNaN = (sf->isPositive() ? sfValue==HIDDEN_VALUE : sfValue==BIG_VALUE);
						if (!isNaN)
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
            progressCb->update(30.0 * (float)n / (float)cloud->size());
    }

	//update grids for 'average' cases
	{
		if (sfInterpolation == PROJ_AVERAGE_HEIGHT)
		{
			for (unsigned k=0;k<gridScalarFields.size(); ++k)
			{
				if (gridScalarFields[k])
				{
					CCLib::ScalarField* sf = pc->getScalarField(k);
					assert(sf);
					double* _gridSF = gridScalarFields[k];
					for (unsigned j=0;j<grid_size_Y;++j)
					{
						hgCell* cell = grid[j];
						for (unsigned i=0;i<grid_size_X;++i,++cell,++_gridSF)
						{
							if (cell->nbPoints)
							{
								bool isNaN = (sf->isPositive() ? *_gridSF==HIDDEN_VALUE : *_gridSF==BIG_VALUE);
								if (!isNaN)
									*_gridSF /= (double)cell->nbPoints;
							}
						}
					}
				}
			}
		}

		//we need to finish the average height computation
		if (projectionType == PROJ_AVERAGE_HEIGHT)
		{
			for (unsigned j=0;j<grid_size_Y;++j)
			{
				hgCell* cell = grid[j];
				for (unsigned i=0;i<grid_size_X;++i,++cell)
					if (cell->nbPoints>1)
						cell->height /= (double)cell->nbPoints;
			}
		}
	}

    //=========================================================================================================
    ccConsole::Print("[ccHeightGridGeneration] 3 - Computation of the average and extreme height values in the grid...");
    double minHeight=0.0, maxHeight=0.0, meanHeight=0.0;
    unsigned nonEmptyCells=0; //non empty cells count
    for (unsigned i=0; i<grid_size_Y; ++i)
    {
        for (unsigned j=0; j<grid_size_X; ++j)
        {
            if (grid[i][j].nbPoints) //non empty cell
            {
                double h = (double)grid[i][j].height;

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
                    minHeight = maxHeight = h;
                }
            }
        }
    }

    if (!nonEmptyCells)
    {
        ccConsole::Warning("[ccHeightGridGeneration] grid is empty!");
    }
	else
    {
        ccConsole::Print("\tMinimal height = %f", minHeight);
        meanHeight /= (double)nonEmptyCells;
        ccConsole::Print("\tAverage height = %f", meanHeight);
        ccConsole::Print("\tMaximal height = %f", maxHeight);

		if (generateASCII || generateImage || cloudGrid)
		{
			//=========================================================================================================
			if (fillEmptyCells == FILL_AVERAGE_HEIGHT)
			{
				//we set the custom height as the average one (so we will be able to ignore this strategy aftewards!)
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
				ccConsole::Print("[ccHeightGridGeneration] Empty cells (containing no projected point) will be filled with the followin value:");
				ccConsole::Print("\tempty_cell_value = %f",minHeight);
			}

			QString outputFilePath = QDir::currentPath()+QString("/");

			//=========================================================================================================
			if (generateASCII)
			{
				ccConsole::Print("[ccHeightGridGeneration] Saving the height grid as a text file...");

				const char gridFilenameTXT[] = "height_grid_text_file.txt";
				FILE* pFile = fopen(gridFilenameTXT,"wt");
				if (pFile)
				{
					ccConsole::Print(QString("\tOutput file: %1").arg(outputFilePath+QString(gridFilenameTXT)));

					for (unsigned j=0; j<grid_size_Y; ++j)
					{
						if (progressCb)
							progressCb->update(30.0 + 50.0*((float)j/(float)grid_size_Y));

						const hgCell* aCell = grid[j];
						for (unsigned i=0; i<grid_size_X; ++i,++aCell)
							fprintf(pFile,"%.8f ", aCell->nbPoints ? aCell->height : empty_cell_height);

						fprintf(pFile,"\n");
					}

					fclose (pFile);
				}
				else
				{
					ccConsole::Warning(QString("[ccHeightGridGeneration] Failed to write '%1' file!").arg(outputFilePath+QString(gridFilenameTXT)));
				}
			}

			//=========================================================================================================
			if (generateImage)
			{
				ccConsole::Print("[ccHeightGridGeneration] Saving the height grid as an image...");

				QImage bitmap8(grid_size_X,grid_size_Y,QImage::Format_Indexed8);
				if (!bitmap8.isNull())
				{
					// Build a custom palette (greyscale)
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

					ccConsole::Print("\tempty_cell_color_index = %1",empty_cell_color_index);

					double range = maxHeight - minHeight;
					if (range < ZERO_TOLERANCE)
						range = 1.0;

					// Filling the image with grid values
					for (unsigned j=0; j<grid_size_Y; ++j)
					{
						const hgCell* aCell = grid[j];
						for (unsigned i=0; i<grid_size_X; ++i,++aCell)
						{
							if (aCell->nbPoints)
							{
								double normalized_height = ((double)aCell->height - minHeight)/range;
								assert(normalized_height>=0.0 && normalized_height<=1.0);
								unsigned char val = (unsigned char)floor(normalized_height*maxColorComp);
								bitmap8.setPixel(i,grid_size_Y-1-j,val);
							}
							else
							{
								bitmap8.setPixel(i,grid_size_Y-1-j,empty_cell_color_index);
							}
						}

						if (progressCb)
							progressCb->update(80.0f + 10.0f*((float)j/(float)grid_size_Y));
					}

					static QString outputFilenameTIF("height_grid_image.tif");
					if (bitmap8.save(outputFilenameTIF))
					{
						ccConsole::Print(QString("\tOutput file: ")+outputFilePath+outputFilenameTIF);
					}
					else
					{
						ccConsole::Error("Failed to save TIF file! (check that 'qtiff4.dll' is in the 'imageformats' directory alongside CC executable)");
						static QString outputFilenamePNG("height_grid_image.png");
						if (bitmap8.save(outputFilenamePNG))
							ccConsole::Print(QString("\tOutput file: ")+outputFilePath+outputFilenamePNG);
						else
							ccConsole::Error("Failed to save PNG file as well! (hum, the problem must be more serious;)");
					}
				}
				else
				{
					ccConsole::Error("[ccHeightGridGeneration] Failed to create output image! (not enough memory?)");
				}
			}

			//=========================================================================================================
			if (cloudGrid)
			{
				ccConsole::Print("[ccHeightGridGeneration] Saving the height grid as a cloud...");

				int sfIdx = -1;
				CCLib::ScalarField* sf = 0;

#ifdef _DEBUG
				//the input point cloud should be empty!
				sfIdx = cloudGrid->getScalarFieldIndexByName(CC_HEIGHT_GRID_FIELD_NAME);
				assert(sfIdx<0);
#endif
				sfIdx = cloudGrid->addScalarField(CC_HEIGHT_GRID_FIELD_NAME, minHeight >= 0.0);
				if (sfIdx<0)
				{
					ccConsole::Warning("[ccHeightGridGeneration] Couldn't allocate a new scalar field for storing height grid values! Try to free some memory ...");
				}
				else
				{
					cloudGrid->setCurrentInScalarField(sfIdx);
					sf = cloudGrid->getCurrentInScalarField();
					assert(sf);
				}

				unsigned pointsCount = (fillEmptyCells != LEAVE_EMPTY ? grid_size_X*grid_size_Y : nonEmptyCells);
				if (cloudGrid->reserve(pointsCount))
				{
					CCVector3 P(0.0);
					P.u[Y]=Mins[Y];
	                
					unsigned n=0;
					for (unsigned j=0; j<grid_size_Y; ++j)
					{
						const hgCell* aCell = grid[j];
						P.u[X] = Mins[X];
						for (unsigned i=0; i<grid_size_X; ++i,++aCell)
						{
							if (aCell->nbPoints) //non empty cell
							{
								P.u[Z] = aCell->height;
								cloudGrid->addPoint(P);

								//if a SF is available, we set the point height as its associated scalar
								if (sf)
									cloudGrid->setPointScalarValue(n++,aCell->height);
							}
							else if (fillEmptyCells != LEAVE_EMPTY) //empty cell
							{
								P.u[Z] = empty_cell_height;
								cloudGrid->addPoint(P);

								//if a SF is available, we set the point height as the default one
								if (sf)
									cloudGrid->setPointScalarValue(n++,empty_cell_height);
							}
	                        
							P.u[X] += grid_step;
						}

						P.u[Y] += grid_step;
					}

					if (sf)
					{
						sf->computeMinAndMax();
						cloudGrid->setCurrentDisplayedScalarField(sfIdx);
						cloudGrid->showSF(true);
					}

					//former scalar fields
					for (unsigned k=0;k<gridScalarFields.size(); ++k)
					{
						double* _sfGrid = gridScalarFields[k];
						if (_sfGrid) //valid SF grid
						{
							//the input point cloud should be empty!
							CCLib::ScalarField* formerSf = pc->getScalarField(k);
							assert(formerSf);

							//we try to create an equivalent SF on the output grid
							int sfIdx = cloudGrid->addScalarField(formerSf->getName(),formerSf->isPositive());
							if (sfIdx<0) //if we aren't lucky, the input cloud already had a SF with CC_HEIGHT_GRID_FIELD_NAME as name
								sfIdx = cloudGrid->addScalarField(qPrintable(QString(formerSf->getName()).append(".old")),formerSf->isPositive());

							if (sfIdx<0)
								ccConsole::Warning("[ccHeightGridGeneration] Couldn't allocate a new scalar field for storing SF '%s' values! Try to free some memory ...",formerSf->getName());
							else
							{
								CCLib::ScalarField* sf = cloudGrid->getScalarField(sfIdx);
								assert(sf);
								//set sf values
								DistanceType emptyCellSFValue = (formerSf->isPositive() ? HIDDEN_VALUE : BIG_VALUE);
								for (unsigned j=0; j<grid_size_Y; ++j)
								{
									const hgCell* aCell = grid[j];
									for (unsigned i=0; i<grid_size_X; ++i, ++_sfGrid, ++aCell)
										if (aCell->nbPoints)
											sf->addElement(*_sfGrid);
										else if (fillEmptyCells != LEAVE_EMPTY)
											sf->addElement(emptyCellSFValue);
								}
								sf->computeMinAndMax();
								assert(sf->currentSize()==pointsCount);
							}
						}
					}
				}
				else
				{
					ccConsole::Warning("[ccHeightGridGeneration] Coudln't create cloud! (not enough memory)");
				}
			}
		}
    }

    ccConsole::Print("This is the end my friend...");

    // unallocation of the height grid...
    if (grid)
    {
        for (unsigned i=0; i<grid_size_Y; ++i)
            if (grid[i])
                delete[] grid[i];
        delete[] grid;
		grid=0;
    }

	//...and of the scalar fields
	{
		for (unsigned i=0;i<gridScalarFields.size();++i)
		{
			if (gridScalarFields[i])
				delete[] gridScalarFields[i];
		}
		gridScalarFields.clear();
	}

    if (progressCb)
        progressCb->stop();
}
