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

#include "ManualSegmentationTools.h"

#include "Matrix.h"
#include "CCTypes.h"
#include "GenericProgressCallback.h"
#include "GenericIndexedCloudPersist.h"
#include "ReferenceCloud.h"
#include "GenericIndexedMesh.h"
#include "SimpleMesh.h"
#include "Polyline.h"

#include <assert.h>

using namespace CCLib;

ReferenceCloud* ManualSegmentationTools::segment(GenericIndexedCloudPersist* aCloud, const Polyline* poly, bool keepInside, const float* viewMat)
{
    assert(poly && aCloud);

	CCLib::SquareMatrix* trans=0;
	CCVector3 P,T(0.0,0.0,0.0);
	CCVector2 P2D;

	if (viewMat)
		trans = new CCLib::SquareMatrix(viewMat);

	ReferenceCloud* Y = new ReferenceCloud(aCloud);

	//we check for each point if it falls inside the polyline
	unsigned i,count=aCloud->size();
	for (i=0;i<count;++i)
	{
		aCloud->getPoint(i,P);

		//we project the point in screen space first if necessary
		if (viewMat)
			trans->apply(P.u);

		bool pointInside = isPointInsidePoly(CCVector2(P.x,P.y),poly);
		if ((keepInside && pointInside) || !(keepInside || pointInside))
                Y->addPointIndex(i);
	}

	if (trans)
        delete trans;

	return Y;
}

//Polyline objects are considered as 2D polylines !
bool ManualSegmentationTools::isPointInsidePoly(const CCVector2& P, const Polyline* poly)
{
	//nombre de sommets
	unsigned verices=poly->size();
	if (verices<2)
		return false;

	bool inside = false;

	CCVector3 A;
	poly->getPoint(0,A);
	for (unsigned i=1;i<=verices;++i)
	{
		CCVector3 B;
		poly->getPoint(i%verices,B);

		//Point Inclusion in Polygon Test (inspired from W. Randolph Franklin - WRF)
		if (((B.y<=P.y) && (P.y<A.y)) ||
             ((A.y<=P.y) && (P.y<B.y)))
		{
			PointCoordinateType ABy = A.y-B.y;
			PointCoordinateType t = (P.x-B.x)*ABy-(A.x-B.x)*(P.y-B.y);
			if (ABy<0)
				t=-t;

			if (t<0)
				inside = !inside;
		}

		A=B;
	}

	return inside;
}


ReferenceCloud* ManualSegmentationTools::segment(GenericIndexedCloudPersist* aCloud, DistanceType minDist, DistanceType maxDist)
{
	if(!aCloud)
		return 0;

	ReferenceCloud* Y = new ReferenceCloud(aCloud);

	DistanceType dist;

	//for each point
	aCloud->placeIteratorAtBegining();
	for (unsigned i=0;i<aCloud->size();++i)
	{
		dist = aCloud->getPointScalarValue(i);
		//we test if its assocaited scalar value falls inside the specified intervale
		if (dist>=minDist && dist<=maxDist)
			Y->addPointIndex(i);
	}

	return Y;
}

GenericIndexedMesh* ManualSegmentationTools::segmentMesh(GenericIndexedMesh* theMesh, ReferenceCloud* pointIndexes, bool pointsWillBeInside, GenericProgressCallback* progressCb, GenericIndexedCloud* destCloud, unsigned indexShift)
{
	if (!theMesh)
		return 0;

	unsigned i;

	//par défaut, on tente une segmentation rapide (qui prend plus de mémoire)
	unsigned numberOfPoints = pointIndexes->getAssociatedCloud()->size();
	unsigned numberOfIndexes = pointIndexes->size();

	//on commence par mettre un marqueur de selection pour chaque point en prevoyant au passage son futur indice dans la liste réduite
	unsigned* newPointIndexes = new unsigned[numberOfPoints];
	if (!newPointIndexes)
		return 0; //not enough memory

	memset(newPointIndexes,0,numberOfPoints*sizeof(unsigned));
	pointIndexes->placeIteratorAtBegining();
	for (i=0;i<numberOfIndexes;++i)
	{
		//0 voudra dire que le point n'est pas retenu, sinon son indice est égal à newPointIndexes-1
		assert(pointIndexes->getCurrentPointGlobalIndex()<numberOfPoints);
		newPointIndexes[pointIndexes->getCurrentPointGlobalIndex()]=i+1;
		pointIndexes->forwardIterator();
	}

	//il faut une étape supplémentaire si on a en entrée les points "en dehors"
	//(une sorte de négatif du tableau qu'on vient de construire en somme !)
	if (!pointsWillBeInside)
	{
		unsigned newIndice = 0;
		unsigned* _newPointIndexes = newPointIndexes;
		for (i=0;i<numberOfPoints;++i)
		{
			//negatif
			*_newPointIndexes = (*_newPointIndexes == 0 ? ++newIndice : 0);
			++_newPointIndexes;
		}
	}
	//on va pouvoir maintenant tester beaucoup plus vite la présence ou non des sommets de chaque triangle du "bon côté"

	unsigned numberOfTriangles = theMesh->size();

	//progress notification
	NormalizedProgress* nprogress = 0;
	if (progressCb)
	{
		progressCb->reset();
		progressCb->setMethodTitle("Extract mesh");
		char buffer[256];
		sprintf(buffer,"New vertex number: %i",numberOfIndexes);
		nprogress = new NormalizedProgress(progressCb,numberOfTriangles);
		progressCb->setInfo(buffer);
		progressCb->start();
	}

	SimpleMesh* newTri = new SimpleMesh(destCloud ? destCloud : pointIndexes->getAssociatedCloud());
	unsigned count=0,maxNumberOfTriangles=0;
	int newVertexIndexes[3];

	theMesh->placeIteratorAtBegining();
	for (i=0;i<numberOfTriangles;++i)
	{
		bool triangleIsOnTheRightSide = true;

		//printf("Triangle #%i",i);
		const TriangleSummitsIndexes* tsi = theMesh->getNextTriangleIndexes(); //DGM: getNextTriangleIndexes is faster for mesh groups!

		//VERSION : ON GARDE LE TRIANGLE UNIQUEMENT SI SES 3 SOMMETS SONT A L'INTERIEUR
		for (uchar j=0;j<3;++j)
		{
			const unsigned& currentVertexFlag = newPointIndexes[tsi->i[j]];

			//si c'est un point qui ne restera pas, on doit supprimer le triangle
			if (currentVertexFlag==0)
			{
				triangleIsOnTheRightSide = false;
				break;
			}
			newVertexIndexes[j]=currentVertexFlag-1;
		}

		//le triangle est du "bon" côté
		if (triangleIsOnTheRightSide)
		{
			if (count==maxNumberOfTriangles)
			{
				maxNumberOfTriangles+=1000;
				if (!newTri->reserve(maxNumberOfTriangles))
				{
					delete newTri;
					if (newPointIndexes)
						delete[] newPointIndexes;
					return 0;
				}
			}
			++count;

			//printf("(IN) -> (%i,%i,%i)\n",newVertexIndexes[0],newVertexIndexes[1],newVertexIndexes[2]);
			newTri->addTriangle(indexShift+newVertexIndexes[0],indexShift+newVertexIndexes[1],indexShift+newVertexIndexes[2]);
		}
		//else printf("(OUT)\n");

		if (nprogress)
		{
			if (!nprogress->oneStep())
				break;
		}
	}

	if (progressCb)
	{
		delete nprogress;
		progressCb->stop();
	}

	if (newPointIndexes)
		delete[] newPointIndexes;
	newPointIndexes=0;

	if (newTri->size()==0)
	{
		delete newTri;
		newTri=0;
	}
	else if (count<maxNumberOfTriangles)
		newTri->resize(count); //should always be ok as count<maxNumberOfTriangles

	return newTri;
}
