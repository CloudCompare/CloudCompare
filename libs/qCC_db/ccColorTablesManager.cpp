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
//$Rev:: 1992                                                              $
//$LastChangedDate:: 2012-01-18 12:17:49 +0100 (mer., 18 janv. 2012)       $
//**************************************************************************
//

#include "ccColorTablesManager.h"

#include <assert.h>

//unique instance
static ccColorTablesManager* s_uniqueInstance = 0;

ccColorTablesManager* ccColorTablesManager::GetUniqueInstance()
{
    if (!s_uniqueInstance)
        s_uniqueInstance = new ccColorTablesManager();

    return s_uniqueInstance;
}

void ccColorTablesManager::ReleaseUniqueInstance()
{
    if (s_uniqueInstance)
        delete s_uniqueInstance;
    s_uniqueInstance=0;
}

ccColorTablesManager::ccColorTablesManager()
{
	theColorRamps = new colorType*[COLOR_RAMPS_NUMBER];
	for (unsigned i=0;i<COLOR_RAMPS_NUMBER;++i)
	{
		theColorRamps[i] = CreateGradientColorTable(DEFAULT_COLOR_RAMP_SIZE,COLOR_RAMPS_ENUMS[i]);
	}
}

ccColorTablesManager::~ccColorTablesManager()
{
    if (!theColorRamps)
        return;

	for (unsigned i=0;i<COLOR_RAMPS_NUMBER;++i)
        if (theColorRamps[i])
            delete[] theColorRamps[i];
	delete[] theColorRamps;
}

colorType* ccColorTablesManager::CreateGradientColorTable(unsigned n, CC_COLOR_RAMPS cr)
{
    //n must be a mutliple of 4
    assert(n>3 && (n%4)==0);

	colorType* table = new colorType[n<<2];
	colorType* pTable = table;

	unsigned i,k;
	float R,G,B,inc;

	switch(cr)
	{
	//GREY RAMP
	case GREY:
		inc = float(MAX_COLOR_COMP) / float(n-1);
		R=0.0;
		G=0.0;
		B=0.0;

		for (i=0;i<n;++i)
		{
			*pTable++ = colorType(R);
			*pTable++ = colorType(G);
			*pTable++ = colorType(B);
			*pTable++ = MAX_COLOR_COMP;
			R+=inc;
			G+=inc;
			B+=inc;
		}
		break;
	//FROM RED TO YELLOW
	case RY:
		inc=float(MAX_COLOR_COMP) / float(n-1);
		R=float(MAX_COLOR_COMP);
		G=0.0;
		B=0.0;

		for (i=0;i<n;++i)
		{
			*pTable++ = colorType(R);
			*pTable++ = colorType(G);
			*pTable++ = colorType(B);
			*pTable++ = MAX_COLOR_COMP;
			G+=inc;
		}
		break;
	//FROM RED TO WHITE
	case RW:
		inc = float(MAX_COLOR_COMP) / float(n-1);
		R=float(MAX_COLOR_COMP);
		G=B=0.0;

		for (i=0;i<n;++i)
		{
			*pTable++ = colorType(R);
			*pTable++ = colorType(G);
			*pTable++ = colorType(B);
			*pTable++ = MAX_COLOR_COMP;
			G+=inc;
			B+=inc;
		}
		//*/

		/*k = n >> 1; //k>0 !
		inc = (float)MAX_COLOR_COMP / (float)(2*k);
		R = (float)MAX_COLOR_COMP;
		G=B=0.0;

		for (i=0;i<k;++i)
		{
			*pTable++ = colorType(R);
			*pTable++ = colorType(G);
			*pTable++ = colorType(B);
			*pTable++ = MAX_COLOR_COMP;
			R-=inc;
			G+=inc;
			B+=inc;
		}

		R=G=B=(float)MAX_COLOR_COMP/2.0f;

		for (i=0;i<k;++i)
		{
			*pTable++ = colorType(R);
			*pTable++ = colorType(G);
			*pTable++ = colorType(B);
			*pTable++ = MAX_COLOR_COMP;
			R+=inc;
			G+=inc;
			B+=inc;
		}
		//*/

		break;
	//FROM BLUE (<0) TO RED (>0) THROUGH WHITE (0)
	case BWR:
		k = n >> 1; //k>0 !
		inc = (float)MAX_COLOR_COMP / (float)k;
		B = (float)MAX_COLOR_COMP;
		G=R=0.0;

		for (i=0;i<k;++i)
		{
			*pTable++ = colorType(R);
			*pTable++ = colorType(G);
			*pTable++ = colorType(B);
			*pTable++ = MAX_COLOR_COMP;
			R+=inc;
			G+=inc;
		}

		R=G=B=(float)MAX_COLOR_COMP;

		for (i=0;i<k;++i)
		{
			*pTable++ = colorType(R);
			*pTable++ = colorType(G);
			*pTable++ = colorType(B);
			*pTable++ = MAX_COLOR_COMP;
			G-=inc;
			B-=inc;
		}

		break;
	//FROM BLUE TO RED
	case BGYR:
	default:

		k = n >> 2; //k>0 !
		inc = float(MAX_COLOR_COMP)/float(k);
		R=0.0;
		G=0.0;
		B=float(MAX_COLOR_COMP);

		//Gradient bleu --> bleu vert
		for (i=0;i<k;i++)
		{
			*pTable++ = colorType(R);
			*pTable++ = colorType(G);
			*pTable++ = colorType(B);
			*pTable++ = MAX_COLOR_COMP;
			G+=inc;
		}
		//Gradient bleu vert --> vert
		for (i=0;i<k;i++)
		{
			*pTable++ = colorType(R);
			*pTable++ = colorType(G);
			*pTable++ = colorType(B);
			*pTable++ = MAX_COLOR_COMP;
			B-=inc;
		}
		//Gradient vert --> jaune
		for (i=0;i<k;i++)
		{
			*pTable++ = colorType(R);
			*pTable++ = colorType(G);
			*pTable++ = colorType(B);
			*pTable++ = MAX_COLOR_COMP;
			R+=inc;
		}
		//Gradient jaune --> rouge
		for (i=0;i<k;i++)
		{
			*pTable++ = colorType(R);
			*pTable++ = colorType(G);
			*pTable++ = colorType(B);
			*pTable++ = MAX_COLOR_COMP;
			G-=inc;
		}
		break;
	}

	return table;
}
