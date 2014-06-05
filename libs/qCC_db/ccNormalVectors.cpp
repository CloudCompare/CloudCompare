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

#include "ccNormalVectors.h"

//Local
#include "ccSingleton.h"

//CCLib
#include <CCGeom.h>
#include <DgmOctreeReferenceCloud.h>
#include <Neighbourhood.h>

//System
#include <assert.h>

//unique instance
static ccSingleton<ccNormalVectors> s_uniqueInstance;

//Number of points for local modeling to compute normals with 2D1/2 Delaunay triangulation
#define	NUMBER_OF_POINTS_FOR_NORM_WITH_TRI 6
//Number of points for local modeling to compute normals with least square plane
#define	NUMBER_OF_POINTS_FOR_NORM_WITH_LS 6
//Number of points for local modeling to compute normals with quadratic 'height' function
#define	NUMBER_OF_POINTS_FOR_NORM_WITH_HF 12

ccNormalVectors* ccNormalVectors::GetUniqueInstance()
{
	if (!s_uniqueInstance.instance)
		s_uniqueInstance.instance = new ccNormalVectors();
	return s_uniqueInstance.instance;
}

void ccNormalVectors::ReleaseUniqueInstance()
{
	s_uniqueInstance.release();
}

ccNormalVectors::ccNormalVectors()
	: m_theNormalHSVColors(0)
{
	init(NORMALS_QUANTIZE_LEVEL);
}

ccNormalVectors::~ccNormalVectors()
{
	if (m_theNormalHSVColors)
		delete[] m_theNormalHSVColors;
}

bool ccNormalVectors::enableNormalHSVColorsArray()
{
	if (m_theNormalHSVColors)
		return true;

	if (m_theNormalVectors.empty())
	{
		//'init' should be called first!
		return false;
	}

	m_theNormalHSVColors = new colorType[m_theNormalVectors.size()*3];
	if (!m_theNormalHSVColors)
	{
		//not enough memory
		return false;
	}

	colorType* rgb = m_theNormalHSVColors;
	for (size_t i=0; i<m_theNormalVectors.size(); ++i, rgb+=3)
		ccNormalVectors::ConvertNormalToRGB(m_theNormalVectors[i],rgb[0],rgb[1],rgb[2]);

	return (m_theNormalHSVColors != 0);
}

const colorType* ccNormalVectors::getNormalHSVColor(unsigned index) const
{
	assert(m_theNormalHSVColors);
	assert(index < m_theNormalVectors.size());
	return m_theNormalHSVColors+3*index;
}

const colorType* ccNormalVectors::getNormalHSVColorArray() const
{
	assert(m_theNormalHSVColors);
	return m_theNormalHSVColors;
}

bool ccNormalVectors::init(unsigned quantizeLevel)
{
	unsigned numberOfVectors = (1<<(quantizeLevel*2+3));
	try
	{
		m_theNormalVectors.resize(numberOfVectors);
	}
	catch(std::bad_alloc)
	{
		ccLog::Warning("[ccNormalVectors::init] Not enough memory!");
		return false;
	}

	for (unsigned i=0; i<numberOfVectors; ++i)
	{
		Quant_dequantize_normal(i,quantizeLevel,m_theNormalVectors[i].u);
		m_theNormalVectors[i].normalize();
	}

	return true;
}

void ccNormalVectors::InvertNormal(normsType &code)
{
	//See 'Quant_quantize_normal' for a better understanding
	normsType mask = 1 << 2*NORMALS_QUANTIZE_LEVEL;

	code += ((code & mask) ? -mask : mask);
	mask <<= 1;
	code += ((code & mask) ? -mask : mask);
	mask <<= 1;
	code += ((code & mask) ? -mask : mask);
}

bool ccNormalVectors::UpdateNormalOrientations(	ccGenericPointCloud* theCloud,
												NormsIndexesTableType& theNormsCodes,
												int preferedOrientation)
{
	assert(theCloud);

	if (preferedOrientation < 0 || preferedOrientation > 9)
	{
		ccLog::Warning(QString("[ccNormalVectors::UpdateNormalOrientations] Invalid parameter (prefered orientation = %1)").arg(preferedOrientation));
		return false;
	}

	//prefered orientation
	CCVector3 orientation(0.0,0.0,0.0);
	CCVector3 barycenter(0,0,0);
	bool useBarycenter = false;
	bool positiveSign = true;
	if (preferedOrientation < 6) //0-5 = +/-X,Y,Z
	{
		orientation.u[preferedOrientation>>1]=((preferedOrientation & 1) == 0 ? PC_ONE : -PC_ONE); //odd number --> inverse direction
	}
	else if (preferedOrientation == 6 || preferedOrientation == 7) //+/-gravity center
	{
		barycenter = CCLib::GeometricalAnalysisTools::computeGravityCenter(theCloud);
		ccLog::Print(QString("[ccNormalVectors::UpdateNormalOrientations] Barycenter: (%1,%2,%3)").arg(barycenter.x).arg(barycenter.y).arg(barycenter.z));
		useBarycenter = true;
		positiveSign = (preferedOrientation == 6);
	}
	else //if (preferedOrientation == 8 || preferedOrientation == 9) //+/-Zero
	{
		//barycenter = CCVector3(0,0,0);
		useBarycenter = true;
		positiveSign = (preferedOrientation == 8);
	}

	//we 'compress' each normal (and we check its orientation if necessary)
	for (unsigned i=0; i<theNormsCodes.currentSize(); i++)
	{
		const normsType& nCode = theNormsCodes.getValue(i);
		CCVector3 N(GetNormal(nCode));

		//we check sign
		if (useBarycenter)
		{
			if (positiveSign)
			{
				orientation = *(theCloud->getPoint(i)) - barycenter;
			}
			else
			{
				orientation = barycenter - *(theCloud->getPoint(i));
			}
		}

		if (N.dot(orientation) < 0)
		{
			//inverse normal and re-compress it
			N *= -1;
			theNormsCodes.setValue(i,ccNormalVectors::GetNormIndex(N.u));
		}
	}

	return true;
}

bool ccNormalVectors::ComputeCloudNormals(	ccGenericPointCloud* theCloud,
											NormsIndexesTableType& theNormsCodes,
											CC_LOCAL_MODEL_TYPES method,
											PointCoordinateType radius,
											int preferedOrientation/*=-1*/,
											CCLib::GenericProgressCallback* progressCb/*=0*/,
											CCLib::DgmOctree* inputOctree/*=0*/)
{
	assert(theCloud);

	unsigned n = theCloud->size();
	if (n<3)
		return false;

	CCLib::DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new CCLib::DgmOctree(theCloud);
		if (theOctree->build() == 0)
		{
			delete theOctree;
			return false;
		}
	}

	//on reserve la memoire pour stocker les normales
	if (!theNormsCodes.isAllocated() || theNormsCodes.currentSize()<n)
		if (!theNormsCodes.resize(n))
		{
			if (!inputOctree)
				delete theOctree;
			return false;
		}

	//we instantiate 'n' 3D normal vectors
	NormsTableType* theNorms = new NormsTableType;
	CCVector3 blankN(0.0,0.0,0.0);
	if (!theNorms->resize(n,true,blankN.u))
	{
		theNormsCodes.clear();
		if (!inputOctree)
			delete theOctree;
		return false;
	}
	//theNorms->fill(0);

	void* additionalParameters[2] = { (void*)theNorms, (void*)&radius };

	unsigned processedCells = 0;
	switch(method)
	{
	case LS:
		{
		uchar level=theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(radius);
#ifndef ENABLE_MT_OCTREE
		processedCells = theOctree->executeFunctionForAllCellsAtLevel(level,
#else
		processedCells = theOctree->executeFunctionForAllCellsAtLevel_MT(level,
#endif
															&(ComputeNormsAtLevelWithLS),
															additionalParameters,
															progressCb,
															"Normals Computation[LS]");
		}
		break;
	case TRI:
		{
		uchar level=theOctree->findBestLevelForAGivenPopulationPerCell(NUMBER_OF_POINTS_FOR_NORM_WITH_TRI);
#ifndef ENABLE_MT_OCTREE
		processedCells = theOctree->executeFunctionForAllCellsAtStartingLevel(level,
#else
		processedCells = theOctree->executeFunctionForAllCellsAtStartingLevel_MT(level,
#endif
															&(ComputeNormsAtLevelWithTri),
															additionalParameters,
															NUMBER_OF_POINTS_FOR_NORM_WITH_TRI/2,
															NUMBER_OF_POINTS_FOR_NORM_WITH_TRI*3,
															progressCb,
															"Normals Computation[TRI]");
		}
		break;
	case HF:
		{
		uchar level=theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(radius);
#ifndef ENABLE_MT_OCTREE
		processedCells = theOctree->executeFunctionForAllCellsAtLevel(level,
#else
		processedCells = theOctree->executeFunctionForAllCellsAtLevel_MT(level,
#endif
															&(ComputeNormsAtLevelWithHF),
															additionalParameters,
															progressCb,
															"Normals Computation[HF]");
		}
		break;

	default:
		break;
	}

	//error or canceled by user?
	if (processedCells == 0 || (progressCb && progressCb->isCancelRequested()))
	{
		theNormsCodes.clear();
		return false;
	}

	//we 'compress' each normal (and we check its orientation if necessary)
	theNormsCodes.fill(0);
	theNorms->placeIteratorAtBegining();
	for (unsigned i=0; i<n; i++)
	{
		const PointCoordinateType* N = theNorms->getCurrentValue();
		normsType nCode = GetNormIndex(N);
		theNormsCodes.setValue(i,nCode);
		theNorms->forwardIterator();
	}

	theNorms->release();
	theNorms = 0;

	//prefered orientation
	if (preferedOrientation >= 0)
		UpdateNormalOrientations(theCloud,theNormsCodes,preferedOrientation);

	if (!inputOctree)
	{
		delete theOctree;
		theOctree = 0;
	}

	return true;
}

bool ccNormalVectors::ComputeNormsAtLevelWithHF(const CCLib::DgmOctree::octreeCell& cell,
												void** additionalParameters,
												CCLib::NormalizedProgress* nProgress/*=0*/)
{
	//additional parameters
	NormsTableType* theNorms	= static_cast<NormsTableType*>(additionalParameters[0]);
	PointCoordinateType radius	= *static_cast<PointCoordinateType*>(additionalParameters[1]);

	CCLib::DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level												= cell.level;
	nNSS.prepare(radius,cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	//we already know which points are lying in the current cell
	unsigned n = cell.points->size();
	nNSS.pointsInNeighbourhood.resize(n);
	CCLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
	for (unsigned j=0; j<n; ++j,++it)
	{
		it->point = cell.points->getPointPersistentPtr(j);
		it->pointIndex = cell.points->getPointGlobalIndex(j);
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	for (unsigned i=0; i<n; ++i)
	{
		cell.points->getPoint(i,nNSS.queryPoint);

		//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (k)!
		unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,false);
		if (k >= NUMBER_OF_POINTS_FOR_NORM_WITH_LS)
		{
			CCLib::DgmOctreeReferenceCloud neighbours(&nNSS.pointsInNeighbourhood,k);
			CCLib::Neighbourhood Z(&neighbours);

			uchar hfDims[3];
			const PointCoordinateType* h = Z.getHeightFunction(hfDims);
			if (h)
			{
				const CCVector3* gv = Z.getGravityCenter();
				assert(gv);

				const uchar& iX = hfDims[0];
				const uchar& iY = hfDims[1];
				const uchar& iZ = hfDims[2];

				PointCoordinateType lX = nNSS.queryPoint.u[iX] - gv->u[iX];
				PointCoordinateType lY = nNSS.queryPoint.u[iY] - gv->u[iY];

				PointCoordinateType N[3];
				N[iX] = h[1] + (2 * h[3] * lX) + (h[4] * lY);
				N[iY] = h[2] + (2 * h[5] * lY) + (h[4] * lX);
				N[iZ] = -1;

				//on normalise
				CCVector3::vnormalize(N);

				theNorms->setValue(cell.points->getPointGlobalIndex(i),N);
			}
		}

		if (nProgress && !nProgress->oneStep())
			return false;
	}

	return true;
}

bool ccNormalVectors::ComputeNormsAtLevelWithLS(const CCLib::DgmOctree::octreeCell& cell,
												void** additionalParameters,
												CCLib::NormalizedProgress* nProgress/*=0*/)
{
	//additional parameters
	NormsTableType* theNorms	= static_cast<NormsTableType*>(additionalParameters[0]);
	PointCoordinateType radius	= *static_cast<PointCoordinateType*>(additionalParameters[1]);

	CCLib::DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level												= cell.level;
	nNSS.prepare(radius,cell.parentOctree->getCellSize(nNSS.level));
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	//we already know which points are lying in the current cell
	unsigned n = cell.points->size();
	nNSS.pointsInNeighbourhood.resize(n);
	{
		CCLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
		for (unsigned j=0; j<n; ++j,++it)
		{
			it->point = cell.points->getPointPersistentPtr(j);
			it->pointIndex = cell.points->getPointGlobalIndex(j);
		}
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	for (unsigned i=0; i<n; ++i)
	{
		cell.points->getPoint(i,nNSS.queryPoint);

		//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (k)!
		unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS,radius,false);
		if (k >= NUMBER_OF_POINTS_FOR_NORM_WITH_HF)
		{
			CCLib::DgmOctreeReferenceCloud neighbours(&nNSS.pointsInNeighbourhood,k);
			CCLib::Neighbourhood Z(&neighbours);

			//compute best fit plane
			const CCVector3* lsqPlaneNormal = Z.getLSQPlaneNormal();
			if (lsqPlaneNormal) //should already be unit!
			{
				theNorms->setValue(cell.points->getPointGlobalIndex(i),lsqPlaneNormal->u);
			}
		}

		if (nProgress && !nProgress->oneStep())
			return false;
	}

	return true;
}

bool ccNormalVectors::ComputeNormsAtLevelWithTri(	const CCLib::DgmOctree::octreeCell& cell,
													void** additionalParameters,
													CCLib::NormalizedProgress* nProgress/*=0*/)
{
	//additional parameters
	NormsTableType* theNorms = static_cast<NormsTableType*>(additionalParameters[0]);

	CCLib::DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level												= cell.level;
	nNSS.minNumberOfNeighbors								= NUMBER_OF_POINTS_FOR_NORM_WITH_TRI;
	cell.parentOctree->getCellPos(cell.truncatedCode,cell.level,nNSS.cellPos,true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos,cell.level,nNSS.cellCenter);

	//we already know which points are lying in the current cell
	unsigned n = cell.points->size();
	nNSS.pointsInNeighbourhood.resize(n);
	CCLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
	{
		for (unsigned j=0; j<n; ++j,++it)
		{
			it->point = cell.points->getPointPersistentPtr(j);
			it->pointIndex = cell.points->getPointGlobalIndex(j);
		}
	}
	nNSS.alreadyVisitedNeighbourhoodSize = 1;

	for (unsigned i=0; i<n; ++i)
	{
		cell.points->getPoint(i,nNSS.queryPoint);

		unsigned k = cell.parentOctree->findNearestNeighborsStartingFromCell(nNSS);
		if (k > NUMBER_OF_POINTS_FOR_NORM_WITH_TRI)
		{
			if (k > NUMBER_OF_POINTS_FOR_NORM_WITH_TRI*3)
				k = NUMBER_OF_POINTS_FOR_NORM_WITH_TRI*3;
			CCLib::DgmOctreeReferenceCloud neighbours(&nNSS.pointsInNeighbourhood,k);
			CCLib::Neighbourhood Z(&neighbours);

			//we mesh the neighbour points (2D1/2)
			CCLib::GenericIndexedMesh* theMesh = Z.triangulateOnPlane();
			if (theMesh)
			{
				CCVector3 N(0,0,0);

				unsigned faceCount = theMesh->size();

				//for all triangles
				theMesh->placeIteratorAtBegining();
				for (unsigned j=0; j<faceCount; ++j)
				{
					//we can't use getNextTriangleIndexes (which is faster on mesh groups but not multi-thread compatible) but anyway we'll never get mesh groups here!
					const CCLib::TriangleSummitsIndexes* tsi = theMesh->getTriangleIndexes(j);

					//we look if the central point is one of the triangle's vertices
					if (tsi->i1 == 0 || tsi->i2 == 0 || tsi->i3 == 0)
					{
						const CCVector3 *A = neighbours.getPoint(tsi->i1);
						const CCVector3 *B = neighbours.getPoint(tsi->i2);
						const CCVector3 *C = neighbours.getPoint(tsi->i3);

						CCVector3 no = (*B - *A).cross(*C - *A);
						//no.normalize();
						N += no;
					}
				}

				delete theMesh;
				theMesh = 0;

				//normalize the 'mean' vector
				N.normalize();
				theNorms->setValue(cell.points->getPointGlobalIndex(i),N.u);
			}
		}

		if (nProgress && !nProgress->oneStep())
			return false;
	}

	return true;
}

/************************************************************************/
/* Quantize a normal => 2D problem.                                     */
/* input :																*/
/*	n : a vector (normalized or not)[xyz]								*/
/*	level : the level of the quantization result is 3+2*level bits		*/
/* output :																*/
/*	res : the result - least significant bits are filled !!!			*/
/************************************************************************/
unsigned ccNormalVectors::Quant_quantize_normal(const PointCoordinateType* n, unsigned level)
{
	if (level == 0)
		return 0;

	/// compute in which sector lie the elements
	unsigned res = 0;
	PointCoordinateType x,y,z;
	if (n[0] >= 0) { x = n[0]; } else { res |= 4; x = -n[0]; }
	if (n[1] >= 0) { y = n[1]; } else { res |= 2; y = -n[1]; }
	if (n[2] >= 0) { z = n[2]; } else { res |= 1; z = -n[2]; }

	/// scale the sectored vector - early return for null vector
	PointCoordinateType psnorm = x + y + z;
	if (psnorm == 0)
	{
		res <<= (level<<1);
		return res;
	}
	psnorm = 1 / psnorm;
	x *= psnorm; y *= psnorm; z *= psnorm;

	/// compute the box
	PointCoordinateType box[6] = { 0, 0, 0, 1, 1, 1 };
	/// then for each required level, quantize...
	bool flip = false;
	while (level > 0)
	{
		//next level
		res <<= 2;
		--level;
		PointCoordinateType halfBox[3] = {	(box[0] + box[3])/2,
											(box[1] + box[4])/2,
											(box[2] + box[5])/2 };

		unsigned sector = 3;
		if (flip)
		{
			     if (z < halfBox[2]) sector = 2;
			else if (y < halfBox[1]) sector = 1;
			else if (x < halfBox[0]) sector = 0;
		}
		else
		{
			     if (z > halfBox[2]) sector = 2;
			else if (y > halfBox[1]) sector = 1;
			else if (x > halfBox[0]) sector = 0;
		}
		res |= sector;
		/// do not do the last operation ...
		if (level == 0)
			return res;
		// fd : a little waste for less branching and smaller
		// code.... which one will be fastest ???
		if (flip)
		{
			if (sector != 3)
				psnorm = box[sector];
			box[0] = halfBox[0];
			box[1] = halfBox[1];
			box[2] = halfBox[2];
			if (sector != 3)
			{
				box[3+sector] = box[sector];
				box[sector] = psnorm;
			}
			else
			{
				flip = false;
			}
		}
		else
		{
			if (sector != 3)
				psnorm = box[3+sector];
			box[3] = halfBox[0];
			box[4] = halfBox[1];
			box[5] = halfBox[2];
			if (sector != 3)
			{
				box[sector] = box[3+sector];
				box[3+sector] = psnorm;
			}
			else
			{
				flip = true;
			}
		}
	}

	return 0;
}

/************************************************************************/
/* DeQuantize a normal => 2D problem.                                   */
/* input :                                                              */
/*		q : quantized normal                                            */
/*		level : the level of the quantized normal has 3+2*level bits	*/
/* output :																*/
/*		res : the result : a NON-normalized normal is returned			*/
/************************************************************************/
void ccNormalVectors::Quant_dequantize_normal(unsigned q, unsigned level, PointCoordinateType* res)
{
	/// special case for level = 0
	if (level == 0)
	{
		res[0] = ((q & 4) != 0 ? -PC_ONE : PC_ONE);
		res[1] = ((q & 2) != 0 ? -PC_ONE : PC_ONE);
		res[2] = ((q & 1) != 0 ? -PC_ONE : PC_ONE);
		return;
	}

	bool flip = false;

	/// recompute the box in the sector...
	PointCoordinateType box[6] = { 0, 0, 0, 1, 1, 1 };

	unsigned l_shift = (level<<1);
	for (unsigned k=0; k<level; ++k)
	{
		l_shift -= 2;
		unsigned sector = (q >> l_shift) & 3;
		if (flip)
		{
			PointCoordinateType tmp = box[sector];
			box[0] = (box[0] + box[3]) / 2;
			box[1] = (box[1] + box[4]) / 2;
			box[2] = (box[2] + box[5]) / 2;
			if (sector != 3)
			{
				box[3+sector] = box[sector];
				box[sector] = tmp;
			}
			else
			{
				flip = false;
			}
		}
		else
		{
			PointCoordinateType tmp = box[3+sector];
			box[3] = (box[0] + box[3]) / 2;
			box[4] = (box[1] + box[4]) / 2;
			box[5] = (box[2] + box[5]) / 2;
			if (sector != 3)
			{
				box[sector] = box[3+sector];
				box[3+sector] = tmp;
			}
			else
			{
				flip = true;
			}
		}
	}

	//get the sector
	unsigned sector = q >> (level+level);

	res[0] = ((sector & 4) != 0 ? -(box[3] + box[0]) : box[3] + box[0]);
	res[1] = ((sector & 2) != 0 ? -(box[4] + box[1]) : box[4] + box[1]);
	res[2] = ((sector & 1) != 0 ? -(box[5] + box[2]) : box[5] + box[2]);
}

QString ccNormalVectors::ConvertStrikeAndDipToString(double& strike_deg, double& dip_deg)
{
	int iStrike = static_cast<int>(strike_deg);
	int iDip = static_cast<int>(dip_deg);

	return QString("N%1°E - %2°").arg(iStrike,3,10,QChar('0')).arg(iDip,3,10,QChar('0'));
}

QString ccNormalVectors::ConvertDipAndDipDirToString(PointCoordinateType dip_deg, PointCoordinateType dipDir_deg)
{
	int iDipDir = static_cast<int>(dipDir_deg);
	int iDip = static_cast<int>(dip_deg);

	return QString("Dip direction: %1° - Dip angle: %2°").arg(iDipDir,3,10,QChar('0')).arg(iDip,3,10,QChar('0'));
}

void ccNormalVectors::ConvertNormalToStrikeAndDip(const CCVector3& N, double& strike_deg, double& dip_deg)
{
	/** Adapted from Andy Michael's 'stridip.c':
	Finds strike and dip of plane given normal vector having components n, e, and u
	output is in degrees north of east and then
	uses a right hand rule for the dip of the plane
	//*/
	strike_deg = 180.0 - atan2(N.y,N.x)*CC_RAD_TO_DEG;	//atan2 output is between -180 and 180! So strike is always positive here
	PointCoordinateType x = sqrt(N.x*N.x+N.y*N.y);		//x is the horizontal magnitude
	dip_deg = atan2(x,N.z)*CC_RAD_TO_DEG;
}

void ccNormalVectors::ConvertNormalToDipAndDipDir(const CCVector3& N, PointCoordinateType& dip_deg, PointCoordinateType& dipDir_deg)
{
	//http://en.wikipedia.org/wiki/Structural_geology#Geometries
	double r2 = N.x*N.x+N.y*N.y;
	if (r2 < ZERO_TOLERANCE)
	{
		dip_deg = 0; //purely vertical normal
		dipDir_deg = 0; //anything in fact
		return;
	}

	//"Dip direction is measured in 360 degrees, generally clockwise from North"
	double dipDir_rad = atan2(N.x,N.y); //result in [-pi,+pi]
	if (dipDir_rad < 0)
		dipDir_rad += 2.0*M_PI;

	//Dip
	double dip_rad = atan(fabs(N.z)/sqrt(r2)); //atan's result in [-pi/2,+pi/2] but |N.z|/r >= 0
	dip_rad = (M_PI/2) - dip_rad; //DGM: we always measure the dip downward from horizontal

	dipDir_deg = static_cast<PointCoordinateType>(dipDir_rad * CC_RAD_TO_DEG);
	dip_deg = static_cast<PointCoordinateType>(dip_rad * CC_RAD_TO_DEG);
}

void ccNormalVectors::ConvertNormalToHSV(const CCVector3& N, double& H, double& S, double& V)
{
	PointCoordinateType dip = 0, dipDir = 0;
	ConvertNormalToDipAndDipDir(N,dip,dipDir);

	H = dipDir;
	if (H == 360.0) //H is in [0;360[
		H = 0.0;
	S = dip/90.0; //S is in [0;1]
	V = 1.0;
}

void ccNormalVectors::ConvertHSVToRGB(double H, double S, double V, colorType& R, colorType& G, colorType& B)
{
	int hi = ((static_cast<int>(H)/60) % 6);
	double f = 0;
	modf(H/60.0,&f);
	double l = V*(1.0-S);
	double m = V*(1.0-f*S);
	double n = V*(1.0-(1.0-f)*S);

	double r = 0.0;
	double g = 0.0;
	double b = 0.0;

	switch(hi)
	{
	case 0:
		r=V;g=n;b=l;
		break;
	case 1:
		r=m;g=V;b=l;
		break;
	case 2:
		r=l;g=V;b=n;
		break;
	case 3:
		r=l;g=m;b=V;
		break;
	case 4:
		r=n;g=l;b=V;
		break;
	case 5:
		r=V;g=l;b=m;
		break;
	}

	R = static_cast<colorType>(r * static_cast<double>(MAX_COLOR_COMP));
	G = static_cast<colorType>(g * static_cast<double>(MAX_COLOR_COMP));
	B = static_cast<colorType>(b * static_cast<double>(MAX_COLOR_COMP));
}

void ccNormalVectors::ConvertNormalToRGB(const CCVector3& N, colorType& R, colorType& G, colorType& B)
{
	double H,S,V;
	ConvertNormalToHSV(N,H,S,V);
	ConvertHSVToRGB(H,S,V,R,G,B);
}
