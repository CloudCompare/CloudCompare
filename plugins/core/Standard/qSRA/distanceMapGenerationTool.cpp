//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qSRA                         #
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
//#                           COPYRIGHT: EDF                               #
//#                                                                        #
//##########################################################################

#include "distanceMapGenerationTool.h"

//qCC
#include <ccMainAppInterface.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccMaterialSet.h>
#include <ccPolyline.h>
#include <ccScalarField.h>
#include <ccProgressDialog.h>

//CCLib
#include <Delaunay2dMesh.h>

//Qt
#include <QFile>
#include <QTextStream>
#include <QMainWindow>

//Meta-data key for profile (polyline) origin
const char PROFILE_ORIGIN_KEY[]  = "ProfileOrigin";
//Meta-data key for profile (polyline) axis
const char REVOLUTION_AXIS_KEY[]  = "RevolutionAxis";
//Meta-data key for the profile (polyline) height shift
const char PROFILE_HEIGHT_SHIFT_KEY[] = "ProfileHeightShift";

//shortcuts
static const double M_PI_DIV_2 = M_PI / 2;
static const double M_PI_DIV_4 = M_PI / 4;

//helper
static inline double ComputeLatitude_rad(	PointCoordinateType x,
											PointCoordinateType y,
											PointCoordinateType z )
{
	double r = static_cast<double>(x*x + y*y);

	if (r < 1.0e-8)
	{
		return z < 0.0 ? -M_PI_DIV_2 : M_PI_DIV_2;
	}

	return atan(z / sqrt(static_cast<double>(r)));
}

//helper
static bool GetPolylineMetaVector(const ccPolyline* polyline, const QString& key, CCVector3& P)
{
	assert(polyline);
	if (polyline)
	{
		//we try to get the right meta-data 'vector'
		QVariant x = polyline->getMetaData(key + QString(".x"));
		QVariant y = polyline->getMetaData(key + QString(".y"));
		QVariant z = polyline->getMetaData(key + QString(".z"));
		if (x.isValid() && y.isValid() && z.isValid())
		{
			bool ok[3] = { true, true, true };
			P.x = static_cast<PointCoordinateType>(x.toDouble(ok));
			P.y = static_cast<PointCoordinateType>(y.toDouble(ok + 1));
			P.z = static_cast<PointCoordinateType>(z.toDouble(ok + 2));
			return (ok[0] && ok[1] && ok[2]);
		}
	}

	return false;
}

//helper
static void SetPoylineMetaVector(ccPolyline* polyline, const QString& key, const CCVector3& P)
{
	assert(polyline);
	if (polyline)
	{
		//add revolution axis as meta-data
		polyline->setMetaData(key + QString(".x"), QVariant(P.x));
		polyline->setMetaData(key + QString(".y"), QVariant(P.y));
		polyline->setMetaData(key + QString(".z"), QVariant(P.z));
	}
}

//helper
ccGLMatrix DistanceMapGenerationTool::ProfileMetaData::computeCloudToSurfaceOriginTrans() const
{
	ccGLMatrix cloudToSurfaceOrigin;
	cloudToSurfaceOrigin.setTranslation(-origin);

	//we must take the axis of the Surface of Revolution into account (if any and if it is not colinear with X, Y or Z)
	if (hasAxis && axis.u[revolDim] + std::numeric_limits<PointCoordinateType>::epsilon() < 1.0)
	{
		ccGLMatrix rotation;
		CCVector3 Z(0, 0, 0);
		Z.u[revolDim] = PC_ONE;
		rotation = ccGLMatrix::FromToRotation(axis, Z);
		cloudToSurfaceOrigin = rotation * cloudToSurfaceOrigin;
	}

	return cloudToSurfaceOrigin;
}

//helper
ccGLMatrix DistanceMapGenerationTool::ProfileMetaData::computeCloudToProfileOriginTrans() const
{
	ccGLMatrix cloudToPolylineOrigin = computeCloudToSurfaceOriginTrans();
	
	//add the height shift along the revolution axis (if any)
	cloudToPolylineOrigin.getTranslation()[revolDim] -= heightShift;

	return cloudToPolylineOrigin;
}

void DistanceMapGenerationTool::SetPoylineRevolDim(ccPolyline* polyline, int revolDim)
{
	assert(polyline);
	if (polyline)
	{
		//add revolution dimension as meta-data
		QVariant dim(revolDim);
		polyline->setMetaData(REVOLUTION_AXIS_KEY,dim);
	}
}

int DistanceMapGenerationTool::GetPoylineRevolDim(const ccPolyline* polyline)
{
	assert(polyline);
	if (polyline)
	{
		//we try to get the revolution dimension from the polyline meta-data
		QVariant axis = polyline->getMetaData(REVOLUTION_AXIS_KEY);
		if (axis.isValid())
		{
			bool ok = true;
			int dim = axis.toInt(&ok);
			if (ok && dim >= 0 && dim <= 2)
				return dim;
		}
	}

	return -1;
}

void DistanceMapGenerationTool::SetPoylineOrigin(ccPolyline* polyline, const CCVector3& origin)
{
	SetPoylineMetaVector(polyline, PROFILE_ORIGIN_KEY, origin);
}

bool DistanceMapGenerationTool::GetPoylineOrigin(const ccPolyline* polyline, CCVector3& origin)
{
	//we try to get the profile origin from the polyline meta-data
	return GetPolylineMetaVector(polyline, PROFILE_ORIGIN_KEY, origin);
}

void DistanceMapGenerationTool::SetPoylineAxis(ccPolyline* polyline, const CCVector3& axis)
{
	SetPoylineMetaVector(polyline, REVOLUTION_AXIS_KEY, axis);
}

bool DistanceMapGenerationTool::GetPoylineAxis(const ccPolyline* polyline, CCVector3& axis)
{
	//we try to get the profile origin from the polyline meta-data
	return GetPolylineMetaVector(polyline, REVOLUTION_AXIS_KEY, axis);
}

void DistanceMapGenerationTool::SetPolylineHeightShift(ccPolyline* polyline, PointCoordinateType heightShift)
{
	assert(polyline);
	if (polyline)
	{
		//add 'height shift' as meta-data
		polyline->setMetaData(PROFILE_HEIGHT_SHIFT_KEY, QVariant(heightShift));
	}
}

bool DistanceMapGenerationTool::GetPolylineHeightShift(const ccPolyline* polyline, PointCoordinateType& heightShift)
{
	assert(polyline);
	if (polyline)
	{
		//retrieve the right meta-data
		QVariant shift = polyline->getMetaData(PROFILE_HEIGHT_SHIFT_KEY);
		if (shift.isValid())
		{
			bool ok;
			heightShift = static_cast<PointCoordinateType>(shift.toDouble(&ok));
			return ok;
		}
	}

	return false;
}

bool DistanceMapGenerationTool::GetPoylineMetaData(	const ccPolyline* polyline, ProfileMetaData& data )
{
	if (!polyline)
	{
		assert(false);
		return false;
	}

	data.revolDim = GetPoylineRevolDim(polyline);
	if (data.revolDim < 0 || data.revolDim > 2)
	{
		return false;
	}

	if (!GetPoylineOrigin(polyline, data.origin))
	{
		return false;
	}

	if (!GetPolylineHeightShift(polyline, data.heightShift))
	{
		data.heightShift = 0;
	}

	data.hasAxis = GetPoylineAxis(polyline, data.axis);

	return true;
}

bool DistanceMapGenerationTool::ComputeRadialDist(	ccPointCloud* cloud,
													ccPolyline* profile,
													bool storeRadiiAsSF/*=false*/,
													ccMainAppInterface* app/*=0*/)
{
	//check input cloud and profile/polyline
	if (!cloud || !profile)
	{
		if (app)
			app->dispToConsole(QString("Internal error: invalid input parameters"), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}
	assert(cloud && profile);

	//number of vertices for the profile
	CCLib::GenericIndexedCloudPersist* vertices = profile->getAssociatedCloud();
	unsigned vertexCount = vertices->size();
	if (vertexCount < 2)
	{
		if (app)
			app->dispToConsole(QString("Invalid polyline (not enough vertices)"), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	//profile meta-data
	ProfileMetaData profileDesc;
	if (!GetPoylineMetaData(profile, profileDesc))
	{
		if (app)
			app->dispToConsole(QString("Invalid polyline (bad or missing meta-data)"), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	//reserve a new scalar field (or take the old one if it already exists)
	int sfIdx = cloud->getScalarFieldIndexByName(RADIAL_DIST_SF_NAME);
	if (sfIdx < 0)
		sfIdx = cloud->addScalarField(RADIAL_DIST_SF_NAME);
	if (sfIdx < 0)
	{
		if (app)
			app->dispToConsole(QString("Failed to allocate a new scalar field for computing distances! Try to free some memory ..."), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}
	ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(sfIdx));
	unsigned pointCount = cloud->size();
	sf->resize(pointCount); //should always be ok
	assert(sf);

	ccScalarField* radiiSf = nullptr;
	if (storeRadiiAsSF)
	{
		int sfIdxRadii = cloud->getScalarFieldIndexByName(RADII_SF_NAME);
		if (sfIdxRadii < 0)
			sfIdxRadii = cloud->addScalarField(RADII_SF_NAME);
		if (sfIdxRadii < 0)
		{
			if (app)
				app->dispToConsole(QString("Failed to allocate a new scalar field for storing radii! You should try to free some memory ..."), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			//return false;
		}
		else
		{
			radiiSf = static_cast<ccScalarField*>(cloud->getScalarField(sfIdxRadii));
			radiiSf->resize(pointCount); //should always be ok
		}
	}

	bool success = true;

	//now compute the distance between the cloud and the (implicit) surface of revolution
	{
		ccGLMatrix cloudToProfile = profileDesc.computeCloudToProfileOriginTrans();

		//we deduce the horizontal dimensions from the revolution axis
		const unsigned char dim1 = static_cast<unsigned char>(profileDesc.revolDim < 2 ? profileDesc.revolDim + 1 : 0);
		const unsigned char dim2 = (dim1 < 2 ? dim1 + 1 : 0);

		ccProgressDialog dlg(true, app ? app->getMainWindow() : nullptr);
		dlg.setMethodTitle(QObject::tr("Cloud to profile radial distance"));
		dlg.setInfo(QObject::tr("Polyline: %1 vertices\nCloud: %2 points").arg(vertexCount).arg(pointCount));
		dlg.start();
		CCLib::NormalizedProgress nProgress(static_cast<CCLib::GenericProgressCallback*>(&dlg), pointCount);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3* P = cloud->getPoint(i);

			//relative point position
			CCVector3 Prel = cloudToProfile * (*P);

			//deduce point height and radius (i.e. in profile 2D coordinate system)
			double height = Prel.u[profileDesc.revolDim];
			//TODO FIXME: we assume the surface of revolution is smooth!
			double radius = sqrt(Prel.u[dim1] * Prel.u[dim1] + Prel.u[dim2] * Prel.u[dim2]);

			if (radiiSf)
			{
				ScalarType radiusVal = static_cast<ScalarType>(radius);
				radiiSf->setValue(i, radiusVal);
			}

			//search nearest "segment" in polyline
			ScalarType minDist = NAN_VALUE;
			for (unsigned j = 1; j < vertexCount; ++j)
			{
				const CCVector3* A = vertices->getPoint(j - 1);
				const CCVector3* B = vertices->getPoint(j);

				double alpha = (height - A->y) / (B->y - A->y);
				if (alpha >= 0.0 && alpha <= 1.0)
				{
					//we deduce the right radius by linear interpolation
					double radius_th = A->x + alpha * (B->x - A->x);
					double dist = radius - radius_th;

					//we look at the closest segment (if the polyline is concave!)
					if (!CCLib::ScalarField::ValidValue(minDist) || dist*dist < minDist*minDist)
					{
						minDist = static_cast<ScalarType>(dist);
					}
				}
			}

			sf->setValue(i, minDist);

			if (!nProgress.oneStep())
			{
				//cancelled by user
				for (unsigned j = i; j < pointCount; ++j)
					sf->setValue(j, NAN_VALUE);

				success = false;
				break;
			}

			//TEST
			//*const_cast<CCVector3*>(P) = Prel;
		}

		//TEST
		//cloud->invalidateBoundingBox();
	}

	sf->computeMinAndMax();
	cloud->setCurrentDisplayedScalarField(sfIdx);
	cloud->showSF(true);

	return success;
}

bool DistanceMapGenerationTool::ComputeMinAndMaxLatitude_rad(	ccPointCloud* cloud,
																double& minLat_rad,
																double& maxLat_rad,
																const ccGLMatrix& cloudToSurfaceOrigin, //e.g. translation to the revolution origin
																unsigned char revolutionAxisDim)
{
	minLat_rad = maxLat_rad = 0.0;

	assert(cloud);
	//invalid parameters?
	if (!cloud || revolutionAxisDim > 2)
		return false;

	unsigned count = cloud->size();
	if (count == 0)
		return true;

	//revolution axis
	const unsigned char Z = revolutionAxisDim;
	//we deduce the 2 other ('horizontal') dimensions from the revolution axis
	const unsigned char X = (Z < 2 ? Z + 1 : 0);
	const unsigned char Y = (X < 2 ? X + 1 : 0);

	for (unsigned n=0; n<count; ++n)
	{
		const CCVector3* P = cloud->getPoint(n);
		CCVector3 relativePos = cloudToSurfaceOrigin * (*P);

		//latitude between 0 and pi/2
		double lat_rad = ComputeLatitude_rad(relativePos.u[X], relativePos.u[Y], relativePos.u[Z]);

		if (n)
		{
			if (lat_rad < minLat_rad)
				minLat_rad = lat_rad;
			else if (lat_rad > maxLat_rad)
				maxLat_rad = lat_rad;
		}
		else
		{
			minLat_rad = maxLat_rad = lat_rad;
		}
	}

	return true;
}

double DistanceMapGenerationTool::ConicalProjectN(double phi1, double phi2)
{
	if (phi1 >= phi2)
		return 1.0;

	assert(fabs(phi1) < M_PI_DIV_2);
	assert(fabs(phi2) < M_PI_DIV_2);

	double tan_pl1 = tan(M_PI_DIV_4 - phi1 / 2);
	double tan_pl2 = tan(M_PI_DIV_4 - phi2 / 2);

	return (log(cos(phi1)) - log(cos(phi2))) / (log(tan_pl1) - log(tan_pl2));
}

double DistanceMapGenerationTool::ConicalProject(double phi, double phi1, double n)
{
	assert(phi1 >= -M_PI_DIV_2 && phi1 < M_PI_DIV_2);

	double tan_pl1 = tan(M_PI_DIV_4 - phi1 / 2);
	double tan_pl  = tan(M_PI_DIV_4 - phi  / 2);

	return cos(phi1) * pow(tan_pl / tan_pl1, n) / n;
}

QSharedPointer<DistanceMapGenerationTool::Map> DistanceMapGenerationTool::CreateMap(ccPointCloud* cloud,
																					ccScalarField* sf,
																					const ccGLMatrix& cloudToSurface,
																					unsigned char revolutionAxisDim,
																					double xStep_rad,
																					double yStep,
																					double yMin,
																					double yMax,
																					bool conical,
																					bool counterclockwise,
																					FillStrategyType fillStrategy,
																					EmptyCellFillOption emptyCellfillOption,
																					ccMainAppInterface* app/*=0*/)
{
	assert(cloud && sf);
	if (!cloud || !sf)
	{
		if (app)
			app->dispToConsole(QString("[DistanceMapGenerationTool] Internal error: invalid input structures!"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return QSharedPointer<Map>(nullptr);
	}

	//invalid parameters?
	if (xStep_rad <= 0.0 || yStep <= 0.0 || yMax <= yMin || revolutionAxisDim > 2)
	{
		if (app)
			app->dispToConsole(QString("[DistanceMapGenerationTool] Internal error: invalid grid parameters!"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return QSharedPointer<Map>(nullptr);
	}

	unsigned count = cloud->size();
	if (count == 0)
	{
		if (app)
			app->dispToConsole(QString("[DistanceMapGenerationTool] Cloud is empty! Nothing to do!"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return QSharedPointer<Map>(nullptr);
	}

	//revolution axis
	const unsigned char Z = revolutionAxisDim;
	//we deduce the 2 other ('horizontal') dimensions from the revolution axis
	const unsigned char X = (Z < 2 ? Z + 1 : 0);
	const unsigned char Y = (X < 2 ? X + 1 : 0);

	//grid dimensions
	unsigned xSteps = 0;
	{
		if (xStep_rad > 0)
			xSteps = static_cast<unsigned>(ceil((2 * M_PI) / xStep_rad));
		if (xSteps == 0)
		{
			if (app)
				app->dispToConsole(QString("[DistanceMapGenerationTool] Invalid longitude step/boundaries! Can't generate a proper map!"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return QSharedPointer<Map>(nullptr);
		}
	}

	unsigned ySteps = 0;
	{
		if (yStep > 0)
			ySteps = static_cast<unsigned>(ceil((yMax - yMin) / yStep));
		if (ySteps == 0)
		{
			if (app)
				app->dispToConsole(QString("[DistanceMapGenerationTool] Invalid latitude step/boundaries! Can't generate a proper map!"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return QSharedPointer<Map>(nullptr);
		}
	}

	unsigned cellCount = xSteps * ySteps;
	if (app)
		app->dispToConsole(QString("[DistanceMapGenerationTool] Projected map size: %1 x %2 (%3 cells)").arg(xSteps).arg(ySteps).arg(cellCount),ccMainAppInterface::STD_CONSOLE_MESSAGE);

	//reserve memory for the output matrix
	QSharedPointer<Map> grid(new Map);
	try
	{
		grid->resize(cellCount);
	}
	catch (const std::bad_alloc&)
	{
		if (app)
			app->dispToConsole(QString("[DistanceMapGenerationTool] Not enough memory!"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return QSharedPointer<Map>(nullptr);
	}

	//update grid info ("for the records")
	grid->xSteps = xSteps;
	grid->xMin = 0.0;
	grid->xMax = 2 * M_PI;
	grid->xStep = xStep_rad;
	grid->ySteps = ySteps;
	grid->yMin = yMin;
	grid->yMax = yMax;
	grid->yStep = yStep;
	grid->conical = conical;

	//motion direction
	grid->counterclockwise = counterclockwise;
	double ccw = (counterclockwise ? -1.0 : 1.0);

	for (unsigned n=0; n<count; ++n)
	{
		//we skip invalid values
		const ScalarType& val = sf->getValue(n);
		if (!CCLib::ScalarField::ValidValue(val))
			continue;

		const CCVector3* P = cloud->getPoint(n);
		CCVector3 relativePos = cloudToSurface * (*P);

		//convert to cylindrical or conical (spherical) coordinates
		double x = ccw * atan2(relativePos.u[X], relativePos.u[Y]); //longitude
		if (x < 0.0)
		{
			x += 2 * M_PI;
		}

		double y = 0.0;
		if (conical)
		{
			y = ComputeLatitude_rad(relativePos.u[X], relativePos.u[Y], relativePos.u[Z]); //latitude between 0 and pi/2
		}
		else
		{
			y = relativePos.u[Z]; //height
		}

		int i = static_cast<int>((x - grid->xMin) / grid->xStep);
		int j = static_cast<int>((y - grid->yMin) / grid->yStep);

		//if we fall exactly on the max corner of the grid box
		if (i == static_cast<int>(grid->xSteps))
			--i;
		if (j == static_cast<int>(grid->ySteps))
			--j;

		//we skip points outside the box!
		if (	i < 0 || i >= static_cast<int>(grid->xSteps)
			||	j < 0 || j >= static_cast<int>(grid->ySteps) )
		{
			continue;
		}
		assert(i >= 0 && j >= 0);

		MapCell& cell = (*grid)[j*static_cast<int>(grid->xSteps) + i];
		if (cell.count) //if there's already values projected in this cell
		{
			switch (fillStrategy)
			{
			case FILL_STRAT_MIN_DIST:
				// Set the minimum SF value
				if (val < cell.value)
					cell.value = val;
				break;
			case FILL_STRAT_AVG_DIST:
				// Sum the values
				cell.value += static_cast<double>(val);
				break;
			case FILL_STRAT_MAX_DIST:
				// Set the maximum SF value
				if (val > cell.value)
					cell.value = val;
				break;
			default:
				assert(false);
				break;
			}
		}
		else
		{
			//for the first point, we simply have to store its associated value (whatever the case)
			cell.value = val;
		}
		++cell.count;

		//if (progressCb)
		//	progressCb->update(30.0 * (float)n / (float)cloud->size());
	}

	//we need to finish the average values computation
	if (fillStrategy == FILL_STRAT_AVG_DIST)
	{
		MapCell* cell = &grid->at(0);
		for (unsigned i = 0; i < cellCount; ++i, ++cell)
			if (cell->count > 1)
				cell->value /= static_cast<double>(cell->count);
	}

	//fill empty cells with zero?
	if (emptyCellfillOption == FILL_WITH_ZERO)
	{
		MapCell* cell = &grid->at(0);
		for (unsigned i = 0; i < cellCount; ++i, ++cell)
		{
			if (cell->count == 0)
			{
				cell->value = 0.0;
				cell->count = 1;
			}
		}
	}
	else if (emptyCellfillOption == FILL_INTERPOLATE)
	{
		//convert the non-empty cells to a 2D point cloud
		unsigned fillCount = 0;
		{
			MapCell* cell = &grid->at(0);
			for (unsigned i = 0; i < cellCount; ++i, ++cell)
				if (cell->count != 0)
					++fillCount;
		}

		//do we really need to interpolate empty grid cells?
		if (fillCount)
		{
			std::vector<CCVector2> the2DPoints;
			try
			{
				the2DPoints.reserve(fillCount);
			}
			catch (...)
			{
				//out of memory
				if (app)
					app->dispToConsole(QString("[DistanceMapGenerationTool] Not enough memory to interpolate!"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			}

			if (the2DPoints.capacity() == fillCount)
			{
				//fill 2D vector with non-empty cell indexes
				{
					const MapCell* cell = &grid->at(0);
					for (unsigned j = 0; j < grid->ySteps; ++j)
						for (unsigned i = 0; i < grid->xSteps; ++i, ++cell)
							if (cell->count)
								the2DPoints.push_back(CCVector2(static_cast<PointCoordinateType>(i), static_cast<PointCoordinateType>(j)));
				}

				//mesh the '2D' points
				CCLib::Delaunay2dMesh* dm = new CCLib::Delaunay2dMesh();
				char errorStr[1024];
				if (!dm->buildMesh(the2DPoints, 0, errorStr))
				{
					if (app)
						app->dispToConsole(QString("[DistanceMapGenerationTool] Interpolation failed: Triangle lib. said '%1'").arg(errorStr),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				}
				else
				{
					unsigned triNum = dm->size();
					MapCell* cells = &grid->at(0);
					//now we are going to 'project' all triangles on the grid
					dm->placeIteratorAtBeginning();
					for (unsigned k = 0; k < triNum; ++k)
					{
						const CCLib::VerticesIndexes* tsi = dm->getNextTriangleVertIndexes();
						//get the triangle bounding box (in grid coordinates)
						int P[3][2];
						int xMin = 0;
						int yMin = 0;
						int xMax = 0;
						int yMax = 0;
						{
							for (unsigned j = 0; j < 3; ++j)
							{
								const CCVector2& P2D = the2DPoints[tsi->i[j]];
								P[j][0] = static_cast<int>(P2D.x);
								P[j][1] = static_cast<int>(P2D.y);
							}
							xMin = std::min(std::min(P[0][0], P[1][0]), P[2][0]);
							yMin = std::min(std::min(P[0][1], P[1][1]), P[2][1]);
							xMax = std::max(std::max(P[0][0], P[1][0]), P[2][0]);
							yMax = std::max(std::max(P[0][1], P[1][1]), P[2][1]);
						}
						//now scan the cells
						{
							//pre-computation for barycentric coordinates
							const double& valA = cells[P[0][0] + P[0][1] * grid->xSteps].value;
							const double& valB = cells[P[1][0] + P[1][1] * grid->xSteps].value;
							const double& valC = cells[P[2][0] + P[2][1] * grid->xSteps].value;
							int det = (P[1][1] - P[2][1])*(P[0][0] - P[2][0]) + (P[2][0] - P[1][0])*(P[0][1] - P[2][1]);

							for (int j = yMin; j <= yMax; ++j)
							{
								MapCell* cell = cells + static_cast<unsigned>(j)*grid->xSteps;

								for (int i = xMin; i <= xMax; ++i)
								{
									//if the cell is empty
									if (!cell[i].count)
									{
										//we test if it's included or not in the current triangle
										//Point Inclusion in Polygon Test (inspired from W. Randolph Franklin - WRF)
										bool inside = false;
										for (int ti = 0; ti < 3; ++ti)
										{
											const int* P1 = P[ti];
											const int* P2 = P[(ti + 1) % 3];
											if ((P2[1] <= j &&j < P1[1]) || (P1[1] <= j && j < P2[1]))
											{
												int t = (i - P2[0])*(P1[1] - P2[1]) - (P1[0] - P2[0])*(j - P2[1]);
												if (P1[1] < P2[1])
													t = -t;
												if (t < 0)
													inside = !inside;
											}
										}
										//can we interpolate?
										if (inside)
										{
											double l1 = static_cast<double>((P[1][1] - P[2][1])*(i - P[2][0]) + (P[2][0] - P[1][0])*(j - P[2][1])) / det;
											double l2 = static_cast<double>((P[2][1] - P[0][1])*(i - P[2][0]) + (P[0][0] - P[2][0])*(j - P[2][1])) / det;
											double l3 = 1.0-l1-l2;

											cell[i].count = 1;
											cell[i].value = l1 * valA + l2 * valB + l3 * valC;
										}
									}
								}
							}
						}
					}
				}

				delete dm;
				dm = nullptr;
			}
		}
	}

	//update min and max values
	{
		const MapCell* cell = &grid->at(0);
		grid->minVal = grid->maxVal = cell->value;
		++cell;
		for (unsigned i = 1; i < cellCount; ++i, ++cell)
		{
			if (cell->value < grid->minVal)
				grid->minVal = cell->value;
			else if (cell->value > grid->maxVal)
				grid->maxVal = cell->value;
		}
	}

	//end of process
	return grid;
}

CCVector3 DistanceMapGenerationTool::ProjectPointOnCone(	double lon_rad,
															double lat_rad,
															double latMin_rad,
															double nProj,
															bool counterclockwise)
{
	double theta = nProj * (lon_rad - M_PI);
	double r = ConicalProject(lat_rad, latMin_rad, nProj);

	CCVector3 P(static_cast<PointCoordinateType>((counterclockwise ? -r  : r)* sin(theta)),
				static_cast<PointCoordinateType>(-r * cos(theta)),
				0);
	
	return P;
}												

ccMesh* DistanceMapGenerationTool::ConvertConicalMapToMesh(	const QSharedPointer<Map>& map,
															bool counterclockwise,
															QImage mapTexture/*=QImage()*/)
{
	if (!map)
		return nullptr;

	unsigned meshVertCount = map->xSteps * map->ySteps;
	unsigned meshFaceCount = (map->xSteps-1) * (map->ySteps-1) * 2;
	ccPointCloud* cloud = new ccPointCloud();
	ccMesh* mesh = new ccMesh(cloud);
	mesh->addChild(cloud);
	if (!cloud->reserve(meshVertCount) || !mesh->reserve(meshFaceCount))
	{
		//not enough memory
		delete mesh;
		return nullptr;
	}

	//compute projection constant
	double nProj = ConicalProjectN(map->yMin,map->yMax) * map->conicalSpanRatio;
	assert(nProj >= -1.0 && nProj <= 1.0);

	//create vertices
	{
		double cwSign = (counterclockwise ? -1.0 : 1.0);
		for (unsigned j = 0; j < map->xSteps; ++j)
		{
			//longitude
			double lon_rad = static_cast<double>(j) / map->xSteps * (2.0 * M_PI);

			double theta = nProj * (lon_rad - M_PI); //-Pi shift so that the map is well centered
			double sin_theta = sin(theta);
			double cos_theta = cos(theta);

			for (unsigned i = 0; i < map->ySteps; ++i)
			{
				double lat_rad = map->yMin + static_cast<double>(i)* map->yStep;
				double r = ConicalProject(lat_rad, map->yMin, nProj);

				CCVector3 Pxyz( static_cast<PointCoordinateType>(cwSign * r * sin_theta),
								static_cast<PointCoordinateType>(-r * cos_theta),
								0);
				cloud->addPoint(Pxyz);
			}
		}
	}

	//create facets
	{
		for (unsigned j = 0; j + 1 < map->xSteps; ++j)
		{
			for (unsigned i = 0; i + 1 < map->ySteps; ++i)
			{
				unsigned vertA = j*map->ySteps + i;
				unsigned vertB = vertA + map->ySteps;
				unsigned vertC = vertB + 1;
				unsigned vertD = vertA + 1;

				mesh->addTriangle(vertB, vertC, vertD);
				mesh->addTriangle(vertB, vertD, vertA);
			}
		}
	}

	//do we have a texture as well?
	if (true/*!mapTexture.isNull()*/) //we force tex. coordinates and indexes creation!
	{
		//texture coordinates
		TextureCoordsContainer* texCoords = new TextureCoordsContainer();
		if (!texCoords->reserveSafe(meshVertCount))
		{
			//not enough memory to finish the job!
			delete texCoords;
			return mesh;
		}

		//create default texture coordinates
		for (unsigned j = 0; j < map->xSteps; ++j)
		{
			TexCoords2D T(static_cast<float>(j) / (map->xSteps - 1), 0.0f);
			for (unsigned i = 0; i < map->ySteps; ++i)
			{
				T.ty = static_cast<float>(i) / (map->ySteps - 1);
				texCoords->addElement(T);
			}
		}

		if (!mesh->reservePerTriangleTexCoordIndexes())
		{
			//not enough memory to finish the job!
			delete texCoords;
			return mesh;
		}
		
		//set texture indexes
		{
			for (unsigned j = 0; j + 1 < map->xSteps; ++j)
			{
				for (unsigned i = 0; i + 1 < map->ySteps; ++i)
				{
					unsigned vertA = j*map->ySteps + i;
					unsigned vertB = vertA + map->ySteps;
					unsigned vertC = vertB + 1;
					unsigned vertD = vertA + 1;

					mesh->addTriangleTexCoordIndexes(vertB, vertC, vertD);
					mesh->addTriangleTexCoordIndexes(vertB, vertD, vertA);
				}
			}
		}
	
		//set material indexes
		if (!mesh->reservePerTriangleMtlIndexes())
		{
			//not enough memory to finish the job!
			delete texCoords;
			mesh->removePerTriangleTexCoordIndexes();
			return mesh;
		}
		for (unsigned i = 0; i < meshFaceCount; ++i)
		{
			mesh->addTriangleMtlIndex(0);
		}

		//set material
		{
			ccMaterial::Shared material(new ccMaterial("texture"));
			material->setTexture(mapTexture, QString(), false);

			ccMaterialSet* materialSet = new ccMaterialSet();
			materialSet->addMaterial(material);

			mesh->setMaterialSet(materialSet);
		}

		mesh->setTexCoordinatesTable(texCoords);
		mesh->showMaterials(true);
		mesh->setVisible(true);
	}

	return mesh;
}

bool DistanceMapGenerationTool::ComputeSurfacesAndVolumes(	const QSharedPointer<Map>& map,
															ccPolyline* profile,
															Measures& surface,
															Measures& volume)
{
	if (!map || !profile)
		//invalid input!
		return false;

	CCLib::GenericIndexedCloudPersist* vertices = profile->getAssociatedCloud();
	unsigned vertexCount = vertices ? vertices->size() : 0;
	if (vertexCount < 2)
	{
		//invalid profile!
		return false;
	}

	const ccPointCloud* pcVertices = dynamic_cast<ccPointCloud*>(profile->getAssociatedCloud());
	if (!pcVertices)
	{
		return false;
	}

	//surface measures
	surface = Measures();
	//volume measures
	volume = Measures();

	//theoretical surface and volumes
	{
		double surfaceProd = 0.0;
		double volumeProd = 0.0;
		const double yMax = map->yMin + map->yStep * map->ySteps;
		for (unsigned i = 1; i < pcVertices->size(); ++i)
		{
			const CCVector3* P0 = pcVertices->getPoint(i - 1);
			const CCVector3* P1 = pcVertices->getPoint(i);

			//polyline: X = radius, Y = height
			double r0 = P0->x;
			double y0 = P0->y;
			double r1 = P1->x;
			double y1 = P1->y;
			
			//without loss of generality ;)
			if (y0 > y1)
			{
				std::swap(y0, y1);
				std::swap(r0, r1);
			}

			//segment is totally outside the map?
			if (y1 < map->yMin || y0 > yMax)
			{
				//we skip it
				continue;
			}

			if (y0 < map->yMin)
			{
				//interpolate r0 @ map->yMin
				double alpha = (map->yMin - y0) / (y1 - y0);
				assert(alpha >= 0.0 && alpha <= 1.0);
				r0 = r0 + alpha * (r1 - r0);
				y0 = map->yMin;
			}
			else if (y1 > yMax)
			{
				//interpolate r1 @ map->yMax
				double alpha = (yMax - y0)/(y1 - y0);
				assert(alpha >= 0.0 && alpha <= 1.0);
				r1 = r0 + alpha * (r1 - r0);
				y1 = yMax;
			}

			//product for truncated cone surface (see http://en.wikipedia.org/wiki/Frustum)
			double segmentLength = sqrt((r1 - r0)*(r1 - r0) + (y1 - y0)*(y1 - y0));
			surfaceProd += (r0 + r1) * segmentLength;

			//product for truncated cone volume (see http://en.wikipedia.org/wiki/Frustum)
			volumeProd += (y1 - y0) * (r0*r0 + r1*r1 + r0*r1);
		}

		surface.theoretical = M_PI * surfaceProd;
		volume.theoretical  = M_PI / 3.0 * volumeProd;
	}

	int revolDim = GetPoylineRevolDim(profile);
	if (revolDim < 0)
		return false;

	//constant factors
	const double surfPart = map->xStep / 2.0;				//perimeter of a portion of circle of angle alpha = alpha * r (* height to get the external surface)
	const double volPart = map->yStep * map->xStep / 6.0;	//area of a portion of circle of angle alpha = alpha/2 * r^2 (* height to get the volume)

	const MapCell* cell = &map->at(0);
	//for each row
	for (unsigned j = 0; j < map->ySteps; ++j)
	{
		//corresponding heights
		double height1 = map->yMin + j * map->yStep;
		double height2 = height1 + map->yStep;
		double r_th1 = -1.0;
		double r_th2 = -1.0;

		//search nearest "segment" in polyline
		double height_middle = (height1 + height2) / 2.0;
		for (unsigned k = 1; k < vertexCount; ++k)
		{
			const CCVector3* A = vertices->getPoint(k - 1);
			const CCVector3* B = vertices->getPoint(k);

			double alpha = (height_middle - A->y) / (B->y - A->y);
			if (alpha >= 0.0 && alpha <= 1.0)
			{
				r_th1 = A->x + (height1 - A->y) / (B->y - A->y) * (B->x - A->x);
				r_th2 = A->x + (height2 - A->y) / (B->y - A->y) * (B->x - A->x);
				break; //FIXME: we hope that there's only one segment facing this particular height?!
			}
		}

		if (r_th1 >= 0.0 /* && r_th2 >= 0.0*/)
		{
			//for each column
			for (unsigned i = 0; i < map->xSteps; ++i, ++cell)
			{
				//deviation from theory
				double d = (cell->count != 0 ? cell->value : 0.0);

				//"real" radius
				double r1 = r_th1 + d;	//see ComputeRadialDist --> << double dist = radius - radius_th; >>
				double r2 = r_th2 + d;	//FIXME: works only if the "scalar field" used for map creation was radial distances!!!

				//surface of the element (truncated cone external face)
				{
					double s = sqrt((r2 - r1) * (r2 - r1) + map->yStep * map->yStep);
					double externalSurface = /*surfPart * */(r1 + r2) * s;
					surface.total += externalSurface;
					//dispatch in 'positive' and 'negative' surface
					if (d >= 0.0)
						surface.positive += externalSurface;
					else
						surface.negative += externalSurface;
				}

				//volume of the element
				{
					volume.total += /*volPart * */(r1*r1 + r2*r2 + r1*r2);
					//volume of the gain (or loss) of matter
					double diffVolume = /*volPart * */fabs(3.0*d * (r_th1 + r_th2 + d)); // = (r*r) * part - (r_th*r_th) * part = [(r_th+d)*(r_th+d)-r_th*r_th] * part
					if (d >= 0.0)
						volume.positive += diffVolume;
					else
						volume.negative += diffVolume;
				}
			}
		}
		else
		{
			cell += map->xSteps;
		}
	}

	//don't forget to mult. by constants
	surface.total *= surfPart;
	surface.positive *= surfPart;
	surface.negative *= surfPart;

	volume.total *= volPart;
	volume.positive *= volPart;
	volume.negative *= volPart;

	return true;
}

bool DistanceMapGenerationTool::ConvertCloudToCylindrical(	ccPointCloud* cloud,
															const ccGLMatrix& cloudToSurface, //e.g. translation to the revolution origin
															unsigned char revolutionAxisDim,
															bool counterclockwise/*=false*/)
{
	assert(cloud);
	if (!cloud || cloud->size() == 0)
		return false;

	//revolution axis
	const unsigned char Z = revolutionAxisDim;
	//we deduce the 2 other ('horizontal') dimensions from the revolution axis
	const unsigned char X = (Z < 2 ? Z + 1 : 0);
	const unsigned char Y = (X < 2 ? X + 1 : 0);

	//motion direction
	PointCoordinateType ccw = (counterclockwise ? -PC_ONE : PC_ONE);

	//get projection height
	for (unsigned n = 0; n < cloud->size(); ++n)
	{
		CCVector3* P = const_cast<CCVector3*>(cloud->getPoint(n));
		CCVector3 relativePos = cloudToSurface * (*P);

		//convert to cylindrical coordinates
		double lon_rad = ccw * atan2(relativePos.u[X], relativePos.u[Y]); //longitude
		if (lon_rad < 0.0)
		{
			lon_rad += 2 * M_PI;
		}
		
		PointCoordinateType height = relativePos.u[Z];

		P->x = static_cast<PointCoordinateType>(lon_rad);
		P->y = height;
		P->z = 0;
	}

	cloud->refreshBB();
	if (cloud->getOctree())
	{
		cloud->deleteOctree();
	}
	//TODO FIXME: and kd-trees? etc. We need a better way to handle those cases...

	return true;
}

bool DistanceMapGenerationTool::ConvertCloudToConical(	ccPointCloud* cloud,
														const ccGLMatrix& cloudToSurface, //e.g. translation to the revolution origin
														unsigned char revolutionAxisDim,
														double latMin_rad,
														double latMax_rad,
														double conicalSpanRatio/*=1.0*/,
														bool counterclockwise/*=false*/)
{
	assert(cloud);
	if (!cloud || cloud->size() == 0)
		return false;

	//revolution axis
	const unsigned char Z = revolutionAxisDim;
	//we deduce the 2 other ('horizontal') dimensions from the revolution axis
	const unsigned char X = (Z < 2 ? Z + 1 : 0);
	const unsigned char Y = (X < 2 ? X + 1 : 0);

	//motion direction
	PointCoordinateType ccw = (counterclockwise ? -PC_ONE : PC_ONE);
	//projection factor
	double nProj = ConicalProjectN(latMin_rad, latMax_rad) * conicalSpanRatio;

	//get projection height
	for (unsigned n = 0; n < cloud->size(); ++n)
	{
		CCVector3* P = const_cast<CCVector3*>(cloud->getPoint(n));
		CCVector3 relativePos = cloudToSurface * (*P);

		//convert to cylindrical coordinates
		PointCoordinateType ang_rad = ccw * atan2(relativePos.u[X], relativePos.u[Y]);
		if (ang_rad < 0.0)
			ang_rad += static_cast<PointCoordinateType>(2 * M_PI);

		double lat_rad = ComputeLatitude_rad(	relativePos.u[X],
												relativePos.u[Y],
												relativePos.u[Z] ); //between 0 and pi/2

		*P = ProjectPointOnCone(ang_rad, lat_rad, latMin_rad, nProj, counterclockwise);
	}

	cloud->refreshBB();
	if (cloud->getOctree())
	{
		cloud->deleteOctree();
	}
	//TODO FIXME: and kd-trees? etc. We need a better way to handle those cases...

	return true;
}

bool DistanceMapGenerationTool::SaveMapAsCSVMatrix( const QSharedPointer<Map>& map,
													QString filename,
													QString xUnit,
													QString yUnit,
													double xConversionFactor/*=1.0*/,
													double yConversionFactor/*=1.0*/,
													ccMainAppInterface* app/*=0*/)
{
	if (!map)
	{
		if (app)
			app->dispToConsole(QString("[SaveMapAsCSVMatrix] Internal error: invalid input map!"), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	//write file
	QFile file(filename);
	if (!file.open(QFile::WriteOnly | QFile::Text))
	{
		if (app)
			app->dispToConsole(QString("[SaveMapAsCSVMatrix] Failed to open file for writing! Check access rights"), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}
	QTextStream stream(&file);

	//write CSV header
	{
		//min and max height (for all lines)
		stream << QString("Height min (%1);").arg(yUnit);
		stream << QString("Height max (%1);").arg(yUnit);

		//for each column
		for (unsigned i=0; i<map->xSteps; ++i)
		{
			//min and max angle for the current column
			double minX = xConversionFactor * (map->xMin + i*map->xStep);
			double maxX = xConversionFactor * (map->xMin + (i + 1)*map->xStep);
			stream << QString("%1-%2 (%3);").arg(minX).arg(maxX).arg(xUnit);
		}

		//eol
		stream << QString("\n");
	}

	//for each line
	for (unsigned j = 0; j < map->ySteps; ++j)
	{
		//min and max height (for the current line)
		double minY = yConversionFactor * (map->yMin + (map->ySteps - 1 - j)*map->yStep);
		double maxY = yConversionFactor * (map->yMin + (map->ySteps - j)*map->yStep);
		stream << QString::number(minY) << QString(";");
		stream << QString::number(maxY) << QString(";");

		//for each column
		for (unsigned i = 0; i < map->xSteps; ++i)
		{
			//write the grid value
			stream << QString::number(map->at(i + j*map->xSteps).value) << QString(";");
		}
		//eol
		stream << QString("\n");
	}

	file.close();

	return true;
}

ccMesh* DistanceMapGenerationTool::ConvertProfileToMesh(ccPolyline* profile,
														const ccGLMatrix& cloudToProfile,
														bool counterclockwise,
														unsigned angularSteps/*=36*/,
														QImage mapTexture/*=QImage()*/)
{
	if (!profile || angularSteps < 3)
	{
		return nullptr;
	}

	//profile vertices
	CCLib::GenericIndexedCloudPersist* profileVertices = profile->getAssociatedCloud();
	unsigned profVertCount = profileVertices->size();
	if (profVertCount < 2)
	{
		return nullptr;
	}

	//profile meta-data
	ProfileMetaData profileDesc;
	if (!GetPoylineMetaData(profile, profileDesc))
	{
		assert(false);
		return nullptr;
	}

	unsigned char Z = static_cast<unsigned char>(profileDesc.revolDim);
	//we deduce the 2 other ('horizontal') dimensions
	const unsigned char X = (Z < 2 ? Z + 1 : 0);
	const unsigned char Y = (X < 2 ? X + 1 : 0);

	unsigned meshVertCount = profVertCount * angularSteps;
	unsigned meshFaceCount = (profVertCount - 1) * angularSteps * 2;
	ccPointCloud* cloud = new ccPointCloud("vertices");
	ccMesh* mesh = new ccMesh(cloud);
	if (!cloud->reserve(meshVertCount) || !mesh->reserve(meshFaceCount))
	{
		//not enough memory
		delete cloud;
		delete mesh;
		return nullptr;
	}

	ccGLMatrix profileToCloud = cloudToProfile.inverse();

	//create vertices
	{
		double cwSign = (counterclockwise ? -1.0 : 1.0);
		for (unsigned j = 0; j < angularSteps; ++j)
		{
			double angle_rad = static_cast<double>(j) / angularSteps * (2 * M_PI);

			CCVector3d N(sin(angle_rad) * cwSign,
						 cos(angle_rad),
						 0);

			for (unsigned i=0; i<profVertCount; ++i)
			{
				const CCVector3* P = profileVertices->getPoint(i);
				double radius = static_cast<double>(P->x);

				CCVector3 Pxyz;
				Pxyz.u[X] = static_cast<PointCoordinateType>(radius * N.x);
				Pxyz.u[Y] = static_cast<PointCoordinateType>(radius * N.y);
				Pxyz.u[Z] = P->y;

				profileToCloud.apply(Pxyz);

				cloud->addPoint(Pxyz);
			}
		}
		mesh->addChild(cloud);
	}

	PointCoordinateType h0 = profileVertices->getPoint(0)->y;
	PointCoordinateType dH = profileVertices->getPoint(profVertCount-1)->y - h0;
	bool invertedHeight = (dH < 0);

	//create facets
	{
		for (unsigned j = 0; j < angularSteps; ++j)
		{
			unsigned nextJ = ((j + 1) % angularSteps);
			for (unsigned i = 0; i + 1 < profVertCount; ++i)
			{
				unsigned vertA = j*profVertCount + i;
				unsigned vertB = nextJ*profVertCount + i;
				unsigned vertC = vertB + 1;
				unsigned vertD = vertA + 1;

				if (invertedHeight)
				{
					mesh->addTriangle(vertB, vertC, vertD);
					mesh->addTriangle(vertB, vertD, vertA);
				}
				else
				{
					mesh->addTriangle(vertB, vertD, vertC);
					mesh->addTriangle(vertB, vertA, vertD);
				}
			}
		}
	}

	//do we have a texture as well?
	if (!mapTexture.isNull())
	{
		//texture coordinates
		TextureCoordsContainer* texCoords = new TextureCoordsContainer();
		mesh->addChild(texCoords);
		if (!texCoords->reserveSafe(meshVertCount+profVertCount)) //we add a column for correct wrapping!
		{
			//not enough memory to finish the job!
			return mesh;
		}

		//create default texture coordinates
		for (unsigned j = 0; j <= angularSteps; ++j)
		{
			TexCoords2D T(static_cast<float>(j) / angularSteps, 0.0f);
			for (unsigned i = 0; i < profVertCount; ++i)
			{
				T.ty = (profileVertices->getPoint(i)->y - h0) / dH;
				if (invertedHeight)
					T.ty = 1.0f - T.ty;
				texCoords->addElement(T);
			}
		}

		if (!mesh->reservePerTriangleTexCoordIndexes())
		{
			//not enough memory to finish the job!
			return mesh;
		}
		
		//set texture indexes
		{
			for (unsigned j = 0; j < angularSteps; ++j)
			{
				unsigned nextJ = ((j + 1)/*% angularSteps*/);
				for (unsigned i = 0; i + 1 < profVertCount; ++i)
				{
					unsigned vertA = j*profVertCount + i;
					unsigned vertB = nextJ*profVertCount + i;
					unsigned vertC = vertB + 1;
					unsigned vertD = vertA + 1;

					if (invertedHeight)
					{
						mesh->addTriangleTexCoordIndexes(vertB, vertC, vertD);
						mesh->addTriangleTexCoordIndexes(vertB, vertD, vertA);
					}
					else
					{
						mesh->addTriangleTexCoordIndexes(vertB, vertD, vertC);
						mesh->addTriangleTexCoordIndexes(vertB, vertA, vertD);
					}
				}
			}
		}
	
		//set material indexes
		if (!mesh->reservePerTriangleMtlIndexes())
		{
			//not enough memory to finish the job!
			mesh->removeChild(texCoords);
			mesh->removePerTriangleTexCoordIndexes();
			return mesh;
		}
		for (unsigned i = 0; i < meshFaceCount; ++i)
		{
			mesh->addTriangleMtlIndex(0);
		}

		//set material
		{
			ccMaterial::Shared material(new ccMaterial("texture"));
			material->setTexture(mapTexture, QString(), false);

			ccMaterialSet* materialSet = new ccMaterialSet();
			materialSet->addMaterial(material);

			mesh->setMaterialSet(materialSet);
		}

		mesh->setTexCoordinatesTable(texCoords);
		mesh->showMaterials(true);
		mesh->setVisible(true);
		cloud->setVisible(false);
	}

	return mesh;
}

ccPointCloud* DistanceMapGenerationTool::ConvertMapToCloud(	const QSharedPointer<Map>& map,
															ccPolyline* profile,
															double baseRadius/*=1.0*/,
															bool keepNaNPoints/*=true*/)
{
	if (!map || !profile)
		return nullptr;

	unsigned count = map->ySteps * map->xSteps;

	ccPointCloud* cloud = new ccPointCloud("map");
	ccScalarField* sf = new ccScalarField("values");
	if (!cloud->reserve(count) || !sf->reserveSafe(count))
	{
		//not enough memory
		delete cloud;
		sf->release();
		return nullptr;
	}

	//number of vertices
	CCLib::GenericIndexedCloudPersist* polyVertices = profile->getAssociatedCloud();
	unsigned polyVertCount = polyVertices->size();
	if (polyVertCount < 2)
	{
		delete cloud;
		sf->release();
		return nullptr;
	}

	//profile meta-data
	ProfileMetaData profileDesc;
	if (!GetPoylineMetaData(profile, profileDesc))
	{
		delete cloud;
		sf->release();
		return nullptr;
	}

	unsigned char Z = static_cast<unsigned char>(profileDesc.revolDim);
	//we deduce the 2 other ('horizontal') dimensions
	const unsigned char X = (Z < 2 ? Z + 1 : 0);
	const unsigned char Y = (X < 2 ? X + 1 : 0);

	const double xStep = baseRadius * (2 * M_PI) / static_cast<double>(map->xSteps);

	const MapCell* cell = &map->at(0);
	for (unsigned j = 0; j < map->ySteps; ++j)
	{
		CCVector3 P(0, 0, 0);
		P.u[Z] = static_cast<PointCoordinateType>(map->yMin + (j + 0.5) * map->yStep);

		//for each column
		for (unsigned i=0; i<map->xSteps; ++i, ++cell)
		{
			if (keepNaNPoints || cell->count != 0)
			{
				P.u[X] = static_cast<PointCoordinateType>(map->xMin + (i + 0.5) * xStep);

				//search nearest "segment" in polyline
				for (unsigned k = 1; k < polyVertCount; ++k)
				{
					const CCVector3* A = polyVertices->getPoint(k - 1);
					const CCVector3* B = polyVertices->getPoint(k);

					double alpha = (P.u[Z] - profileDesc.heightShift - A->y) / (B->y - A->y);
					if (alpha >= 0.0 && alpha <= 1.0)
					{
						//we deduce the right radius by linear interpolation
						double radius_th = A->x + alpha * (B->x - A->x);
						//TODO: we take the first radius (even if there are other segments at
						//this particular height, because we can't guess which one is the 'right' one!
						P.u[Y] = static_cast<PointCoordinateType>(radius_th);
						break;
					}
				}

				cloud->addPoint(profileDesc.origin + P);

				ScalarType val = cell->count ? static_cast<ScalarType>(cell->value) : NAN_VALUE;
				sf->addElement(val);
			}
		}
	}

	sf->computeMinAndMax();
	int sfIdx = cloud->addScalarField(sf);
	cloud->setCurrentDisplayedScalarField(sfIdx);
	cloud->showSF(true);
	cloud->resize(cloud->size()); //if we have skipped NaN values!

	return cloud;
}

QImage DistanceMapGenerationTool::ConvertMapToImage(const QSharedPointer<Map>& map,
													ccColorScale::Shared colorScale,
													unsigned colorScaleSteps/*=ccColorScale::MAX_STEPS*/)
{
	if (!map || !colorScale)
		return QImage();

	//create image
	QImage image(QSize(map->xSteps, map->ySteps), QImage::Format_ARGB32);
	if (image.isNull())
	{
		//not enough memory!
		return QImage();
	}

	//convert map cells to pixels
	{
		bool csIsRelative = colorScale->isRelative();

		const MapCell* cell = &map->at(0);
		for (unsigned j = 0; j < map->ySteps; ++j)
		{
			//for each column
			for (unsigned i = 0; i < map->xSteps; ++i, ++cell)
			{
				const ccColor::Rgb* rgb = &ccColor::lightGreyRGB;

				if (cell->count != 0)
				{
					double relativePos = csIsRelative ? (cell->value - map->minVal) / (map->maxVal - map->minVal) : colorScale->getRelativePosition(cell->value);
					if (relativePos < 0.0)
						relativePos = 0.0;
					else if (relativePos > 1.0)
						relativePos = 1.0;
					rgb = colorScale->getColorByRelativePos(relativePos, colorScaleSteps, &ccColor::lightGreyRGB);
				}

				//DGM FIXME: QImage::sePixel is quite slow!
				image.setPixel(	static_cast<int>(i),
								static_cast<int>(j),
								qRgb(rgb->r, rgb->g, rgb->b));
			}
		}
	}

	return image;
}
