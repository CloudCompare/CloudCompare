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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include "SaitoSquaredDistanceTransform.h"

//Local
#include "DistanceComputationTools.h"

//system
#include <algorithm>
#include <cstdint>

using namespace CCLib;

bool SaitoSquaredDistanceTransform::EDT_1D(GridElement* slice, std::size_t r, std::size_t c)
{
	GridElement *row = slice;
	
	for (std::size_t j = 0; j < r; j++, row += c)
	{
		GridElement b = 1;
		for (std::size_t i = 1; i < c; i++)
		{
			GridElement limit = row[i - 1] + b;
			if (row[i] > limit)
			{
				row[i] = limit;
				b += 2;
			}
			else
			{
				b = 1;
			}
		}

		b = 1;
		for (std::size_t i = 1; i < c; i++)
		{
			std::size_t colIndex = c - i;
			GridElement limit = row[colIndex] + b;
			if (row[colIndex - 1] > limit)
			{
				row[colIndex - 1] = limit;
				b += 2;
			}
			else
			{
				b = 1;
			}
		}
	}

	return true;
}

//:
// Assumes given a Lookup table of integer squares.
// Also assumes the image \a im already has infinity in all non-zero points.
bool SaitoSquaredDistanceTransform::SDT_2D(Grid3D<GridElement>& grid, std::size_t sliceIndex, const std::vector<GridElement>& sq)
{
	const Tuple3ui& gridSize = grid.size();
	std::size_t r = gridSize.y;
	std::size_t c = gridSize.x;
	std::size_t voxelCount = r*c;

	GridElement* sliceData = grid.data() + sliceIndex * voxelCount;

	// 1st step: vertical row-wise EDT
	{
		if (!EDT_1D(sliceData, r, c))
		{
			return false;
		}
	}
	
	// 2nd step: horizontal scan
	{
		std::vector<GridElement> colData;
		try
		{
			colData.resize(r);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			return false;
		}

		//for each column
		for (std::size_t i = 0; i < c; ++i)
		{
			//fill buffer with column values
			{
				GridElement* pt = sliceData + i;
				for (std::size_t j = 0; j < r; ++j, pt += c)
					colData[j] = *pt;
			}

			//forward scan
			GridElement* pt = sliceData + i + c;
			{
				GridElement a = 0;
				GridElement buffer = colData[0];

				for (std::size_t j = 1; j < r; ++j, pt += c)
				{
					std::size_t rowIndex = j;
					if (a != 0)
						--a;
					if (colData[rowIndex] > buffer + 1)
					{
						GridElement b = (colData[rowIndex] - buffer - 1) / 2;
						if (rowIndex + b + 1 > r)
							b = static_cast<GridElement>(r - 1 - rowIndex);

						GridElement* npt = pt + a*c;
						for (GridElement l = a; l <= b; ++l)
						{
							GridElement m = buffer + sq[l + 1];
							if (colData[rowIndex + l] <= m)
							{
								//proceed to next column
								break;
							}
							if (m < *npt)
								*npt = m;
							npt += c;
						}
						a = b;
					}
					else
					{
						a = 0;
					}
					buffer = colData[rowIndex];
				}
			}

			//backward scan
			pt -= 2 * c;
			{
				GridElement a = 0;
				GridElement buffer = colData[r - 1];

				for (std::size_t j = 1; j < r; ++j, pt -= c)
				{
					std::size_t rowIndex = r - j - 1;
					if (a != 0)
						--a;
					if (colData[rowIndex] > buffer + 1)
					{
						GridElement b = (colData[rowIndex] - buffer - 1) / 2;
						if (rowIndex < b)
							b = static_cast<GridElement>(rowIndex);

						GridElement* npt = pt - a*c;
						for (GridElement l = a; l <= b; ++l)
						{
							GridElement m = buffer + sq[l + 1];
							if (colData[rowIndex - l] <= m)
							{
								//proceed to next column
								break;
							}
							if (m < *npt)
								*npt = m;
							npt -= c;
						}
						a = b;
					}
					else
					{
						a = 0;
					}
					buffer = colData[rowIndex];
				}
			}
		}
	}

	return true;
}

bool SaitoSquaredDistanceTransform::SDT_3D(Grid3D<GridElement>& grid, GenericProgressCallback* progressCb/*=0*/)
{
	const Tuple3ui& gridSize = grid.size();
	std::size_t r = gridSize.y;
	std::size_t c = gridSize.x;
	std::size_t p = gridSize.z;
	std::size_t voxelCount = r*c*p;

	std::size_t diag = static_cast<std::size_t>(ceil(sqrt(static_cast<double>(r*r + c*c + p*p))) - 1);
	std::size_t nsqr = 2 * (diag + 1);

	std::vector<GridElement> sq;
	try
	{
		sq.resize(nsqr);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	for (std::size_t i = 0; i < nsqr; ++i)
	{
		sq[i] = static_cast<GridElement>(i*i);
	}

	const GridElement maxDistance = std::numeric_limits<GridElement>::max() - static_cast<GridElement>(r*r + c*c + p*p) - 1;

	NormalizedProgress normProgress(progressCb, static_cast<unsigned>(p + r));
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("Saito Distance Transform");
			char buffer[256];
			sprintf(buffer, "Box: [%u x %u x %u]", gridSize.x, gridSize.y, gridSize.z);
			progressCb->setInfo(buffer);
		}
		progressCb->update(0);
		progressCb->start();
	}

	GridElement* data = grid.data();
	{
		for (std::size_t i = 0; i < voxelCount; ++i)
		{
			//DGM: warning we must invert the input image here!
			if (data[i] == 0)
				data[i] = maxDistance;
			else
				data[i] = 0;
		}
	}

	// 2D EDT for each slice
	for (std::size_t k = 0; k < p; ++k)
	{
		if (!SDT_2D(grid, k, sq))
		{
			return false;
		}

		if (progressCb && !normProgress.oneStep())
		{
			//process cancelled by user
			return false;
		}
	}

	// Now, for each pixel, compute final distance by searching along Z direction
	std::size_t rc = r*c;
	std::vector<GridElement> colData;
	try
	{
		colData.resize(p);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	for (std::size_t j = 0; j < r; ++j, data += c)
	{
		for (std::size_t i = 0; i < c; ++i)
		{
			GridElement* pt = data + i;

			for (std::size_t k = 0; k < p; ++k, pt += rc)
				colData[k] = *pt;

			pt = data + i + rc;
			GridElement a = 0;
			GridElement buffer = colData[0];

			for (std::size_t k = 1; k < p; ++k, pt += rc)
			{
				if (a != 0)
					--a;
				if (colData[k] > buffer + 1)
				{
					GridElement b = (colData[k] - buffer - 1) / 2;
					if (k + b + 1 > p)
						b = static_cast<GridElement>(p - 1 - k);

					GridElement* npt = pt + a*rc;
					for (GridElement l = a; l <= b; ++l)
					{
						GridElement m = buffer + sq[l + 1];
						if (colData[k + l] <= m)
							break;   // go to next plane k
						if (m < *npt)
							*npt = m;
						npt += rc;
					}
					a = b;
				}
				else
				{
					a = 0;
				}
				buffer = colData[k];
			}

			a = 0;
			pt -= 2 * rc;
			buffer = colData[p - 1];

			for (std::size_t k = p - 2; k != static_cast<std::size_t>(-1); --k, pt -= rc)
			{
				if (a != 0)
					--a;
				if (colData[k] > buffer + 1)
				{
					GridElement b = (colData[k] - buffer - 1) / 2;
					if (k < b)
						b = static_cast<GridElement>(k);

					GridElement* npt = pt - a*rc;
					for (GridElement l = a; l <= b; ++l)
					{
						GridElement m = buffer + sq[l + 1];
						if (colData[k - l] <= m)
							break;   // go to next column k
						if (m < *npt)
							*npt = m;
						npt -= rc;
					}
					a = b;
				}
				else
				{
					a = 0;
				}
				buffer = colData[k];
			}
		}

		if (progressCb && !normProgress.oneStep())
		{
			//process cancelled by user
			return false;
		}
	}

	return true;
}
