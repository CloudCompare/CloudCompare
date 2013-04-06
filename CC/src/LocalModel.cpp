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

#include "LocalModel.h"

//local
#include "GenericTriangle.h"
#include "GenericMesh.h"
#include "GenericIndexedMesh.h"
#include "DistanceComputationTools.h"

//system
#include <string.h>
#include <math.h>

using namespace CCLib;

LocalModel::LocalModel(Neighbourhood& Yk, CC_LOCAL_MODEL_TYPES mt, const CCVector3 &center, PointCoordinateType _squareModelSize)
	: delaunayTri(0)
	, lsqPlane(0)
	, hf(0)
	, mtype(NO_MODEL)
	, modelCenter(center)
	, squareModelSize(_squareModelSize)
{
	switch(mt)
	{
	case NO_MODEL:
        break;
	case LS:
		{
			const PointCoordinateType* lsq = Yk.getLSQPlane();
			if (lsq)
			{
				lsqPlane = new PointCoordinateType[4];
				memcpy(lsqPlane,lsq,sizeof(PointCoordinateType)*4);
				mtype = mt;
			}
		}
		break;
	case TRI:
		{
			delaunayTri = Yk.triangulateOnPlane(true); //Yk is associated to a volatile ReferenceCloud, so we must duplicate vertices!
			if (delaunayTri)
			{
				if (delaunayTri->size()!=0)
				{
					mtype = mt;
				}
				else
				{
					delete delaunayTri;
					delaunayTri=0;
				}
			}
		}
		break;
	case HF:
		{
			uchar hfdims[3];
			const PointCoordinateType* heightFunc = Yk.getHeightFunction(hfdims);
			if (heightFunc)
			{
				hf = new PointCoordinateType[6];
				memcpy(hf,heightFunc,sizeof(PointCoordinateType)*6);
				hfX = hfdims[0];
				hfY = hfdims[1];
				hfZ = hfdims[2];

				const CCVector3* GC = Yk.getGravityCenter();
				//if the HF computation succeeded, the gravity center should be ok!
				assert(GC);
				gravityCenter = *GC;

				mtype = mt;
			}
		}
		break;
	}
}

LocalModel::~LocalModel()
{
	if (delaunayTri)
		delete delaunayTri;
	if (lsqPlane)
		delete[] lsqPlane;
	if (hf)
		delete[] hf;
}

ScalarType LocalModel::computeDistanceFromModelToPoint(const CCVector3* aPoint)
{
    switch(mtype)
    {
        case LS:
		{
            return DistanceComputationTools::computePoint2PlaneDistance(aPoint,lsqPlane);
		}
        case TRI:
        {
            ScalarType minDist2 = NAN_VALUE;
			{
				unsigned numberOfTriangles = delaunayTri->size();
				delaunayTri->placeIteratorAtBegining();
				for (unsigned i=0;i<numberOfTriangles;++i)
				{
					GenericTriangle* tri = delaunayTri->_getNextTriangle();
					ScalarType dist2 = DistanceComputationTools::computePoint2TriangleDistance(aPoint,tri,false);
					if (dist2 < minDist2 || i==0)
						minDist2 = dist2;
				}
			}

            //there should be at least one triangle!
			assert(minDist2==minDist2);
            return sqrt(minDist2);
        }
        case HF:
        {
			CCVector3 P = *aPoint - gravityCenter;

            //HF : h0+h1.x+h2.y+h3.x^2+h4.x.y+h5.y^2
            PointCoordinateType z2 = hf[0]+hf[1]*P.u[hfX]+hf[2]*P.u[hfY]+hf[3]*P.u[hfX]*P.u[hfX]+hf[4]*P.u[hfX]*P.u[hfY]+hf[5]*P.u[hfY]*P.u[hfY];

            return (ScalarType)fabs(P.u[hfZ]-z2);
        }
        case NO_MODEL:
			//model computation failed?
            break;
    }

	return NAN_VALUE;
}
