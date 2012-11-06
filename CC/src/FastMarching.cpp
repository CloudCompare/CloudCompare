//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
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
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#include "FastMarching.h"

#include "DgmOctree.h"

#include <assert.h>
#include <string.h>

using namespace CCLib;

FastMarching::FastMarching()
{
	theGrid =		0;
	theOctree =		0;
	initialized =	false;
	gridSize =		0;
	activeCells.clear();
}

FastMarching::~FastMarching()
{
	if (theGrid)
	{
		if (initialized)
		{
			Cell** _theGrid = theGrid;
			for (unsigned i=0;i<gridSize;++i)
			{
				if (*_theGrid) delete (*_theGrid);
				++_theGrid;
			}
		}

		delete[] theGrid;
	}
}

float FastMarching::getTime(int pos[], bool absoluteCoordinates)
{
	unsigned index=0;

	if (absoluteCoordinates)
		index = FM_pos2index(pos);
	else
		index = unsigned(pos[0]+1)+unsigned(pos[1]+1)*decY+unsigned(pos[2]+1)*decZ;

	assert(theGrid[index]);

	return theGrid[index]->T;
}

int FastMarching::initGrid()
{
	if (!theOctree) return -2;

	minFillIndexes = theOctree->getMinFillIndexes(gridLevel);
	maxFillIndexes = theOctree->getMaxFillIndexes(gridLevel);

	dx = maxFillIndexes[0]-minFillIndexes[0]+1;
	dy = maxFillIndexes[1]-minFillIndexes[1]+1;
	dz = maxFillIndexes[2]-minFillIndexes[2]+1;

	cellSize = theOctree->getCellSize(gridLevel);

	decY = dx+2;
	decZ = decY*(dy+2);
	gridSize = decZ*(dz+2);
	indexDec = 1+decY+decZ;

	for (int i=0;i<CC_FM_NUMBER_OF_NEIGHBOURS;++i)
	{
		neighboursIndexShift[i] = neighboursPosShift[i*3]+
									neighboursPosShift[i*3+1]*int(decY)+
									neighboursPosShift[i*3+2]*int(decZ);

		//printf("shift (%i,%i,%i) -> %i\n",neighboursPosShift[i*3],neighboursPosShift[i*3+1],neighboursPosShift[i*3+2],neighboursIndexShift[i]);

		neighboursDistance[i] = sqrt(float(neighboursPosShift[i*3]*neighboursPosShift[i*3]+
									neighboursPosShift[i*3+1]*neighboursPosShift[i*3+1]+
									neighboursPosShift[i*3+2]*neighboursPosShift[i*3+2]))*cellSize;
	}

	activeCells.clear();

	if (!instantiateGrid(gridSize))
        return -3;

	return 0;
}

bool FastMarching::instantiateGrid(unsigned size)
{
    assert(theGrid==0);

	theGrid = new Cell*[size];
	if (!theGrid)
        return false;

	memset(theGrid,0,size*sizeof(Cell*));

	return true;
}

void FastMarching::setSeedCell(int pos[])
{
	unsigned index = FM_pos2index(pos);

	assert(index<gridSize);

	Cell* aCell = theGrid[index];

	if (!aCell)
	{
		//printf("cell(%i,%i,%i) --> %i\n",pos[0],pos[1],pos[2],index);
		//printf("Warning ! No Cell defined at position (%i,%i,%i)\n",pos[0],pos[1],pos[2]);
		return;
	}

	if (aCell->state != Cell::ACTIVE_CELL)
	{
		//on rajoute la cellule au groupe "ACTIVES"
		aCell->state = Cell::ACTIVE_CELL;
		aCell->T = 0.0;
		activeCells.push_back(index);
	}
}

void FastMarching::initTrialCells()
{
	Cell *aCell,*nCell;
	int i,j,index,nIndex;

	for (j=0;j<(int)activeCells.size();++j)
	{
		index = activeCells[j];
		aCell = theGrid[index];

		assert(aCell != 0);

		for (i=0;i<CC_FM_NUMBER_OF_NEIGHBOURS;++i)
		{
			nIndex = index + neighboursIndexShift[i];
			//pointeur vers la cellule voisine
			nCell = theGrid[nIndex];

			//si elle est définie
			if (nCell)
			{
				//et si elle n'est pas encore dans un groupe
				if (nCell->state==Cell::FAR_CELL)
				{
					nCell->state = Cell::TRIAL_CELL;
					nCell->T = neighboursDistance[i]*computeTCoefApprox(aCell,nCell);

					addTrialCell(nIndex,nCell->T);

					//Console::print("Cell %i added to TRIAL\n",nIndex);
				}
			}
		}
	}
}
