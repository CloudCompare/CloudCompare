//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qSRA                         #
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
//#                           COPYRIGHT: EDF                               #
//#                                                                        #
//##########################################################################

#ifndef QSRA_DISTANCE_MAP_GENERATION_TOOL_HEADER
#define QSRA_DISTANCE_MAP_GENERATION_TOOL_HEADER

//Qt
#include <QSharedPointer>
#include <QImage>

//CCLib
#include <CCGeom.h>

//qCC_db
#include <ccColorScale.h>

//system
#include <vector>

class ccPointCloud;
class ccMesh;
class ccPolyline;
class ccScalarField;
class ccMainAppInterface;

//Default radial distance scalar field name
const char RADIAL_DIST_SF_NAME[] = "Radial distance";
//Default radii scalar field name
const char RADII_SF_NAME[] = "Radius";

//! Distance map generation tool (surface of revolution)
class DistanceMapGenerationTool
{
public:

	//! Sets the revolution axis of a given polyline
	/** The revolution axis (dimension) is associated with a specific meta-data.
	**/
	static void SetPoylineAxis(ccPolyline* polyline, int axisDim);

	//! Returns the revolution 'dimension' associated to a given profile (polyline)
	/** Requires the right meta-data to be set (see SetPoylineAxis).
		\retrun 0 (X), 1 (Y), 2 (Z) or -1 (no axis defined)
	**/
	static int GetPoylineAxis(const ccPolyline* polyline);

	//! Computes radial distance between cloud and a profile
	static bool ComputeRadialDist(	ccPointCloud* cloud,
									ccPolyline* profile,
									bool storeRadiiAsSF = false,
									ccMainAppInterface* app = 0);

	//! "Distance" map cell
	struct MapCell
	{
		//! Default constructor
		MapCell()
			: value(0)
			, count(0)
		{}

		//! Value
		double value;
		//! Number of values projected in this cell (for average computation)
		unsigned count;
	};
	
	//! "Distance" map
	class Map : public std::vector<MapCell>
	{
	public:
		Map()
			: xSteps(0)
			, xMin(0.0)
			, xMax(0.0)
			, xStep(1.0)
			, ySteps(0)
			, yMin(0.0)
			, yMax(0.0)
			, yStep(1.0)
			, minVal(0.0)
			, maxVal(0.0)
			, counterclockwise(false)
		{}

		unsigned xSteps;
		double xMin;
		double xMax;
		double xStep;

		unsigned ySteps;
		double yMin;
		double yMax;
		double yStep;

		//min and max values
		double minVal;
		double maxVal;

		//motion direction
		/** Counter-clockwise (true) or clockwise (false)
		**/
		bool counterclockwise;
	};

    //! Grid filling strategy
    enum FillStrategyType {	FILL_STRAT_MIN_DIST			= 0,
							FILL_STRAT_AVG_DIST			= 1,
							FILL_STRAT_MAX_DIST			= 2,
                            INVALID_STRATEGY_TYPE		= 255,
    };

    //! Option for handling empty cells
    enum EmptyCellFillOption {	LEAVE_EMPTY				= 0,
								FILL_WITH_ZERO			= 1,
								FILL_INTERPOLATE		= 2,
    };

	//! Projects a cloud (scalar field) on a revolution surface to generate a 2D map
	/** Projection can be either cylindrical or spherical.
		Warning: for cylindrical projection, the 2D map 'height' will be expressed
		relatively to the 'revolutionOrigin' and not the cloud's (implicit) origin.
	**/
	static QSharedPointer<Map> CreateMap(	ccPointCloud* cloud,
											ccScalarField* sf,
											const CCVector3d& revolutionOrigin,
											unsigned char revolutionAxisDim,
											double angStep_rad,
											double yStep,
											double yMin,
											double yMax,
											bool spherical,
											bool counterclockwise,
											FillStrategyType fillStrategy,
											EmptyCellFillOption emptyCellfillOption,
											ccMainAppInterface* app = 0 );


	//! Creates a conical projection (textured) mesh
	static ccMesh* ConvertConicalMapToMesh(	const QSharedPointer<Map>& map,
											bool counterclockwise,
											double conicalSpanRatio = 1.0,
											QImage mapTexture = QImage());


	//! Returns the constant 'n' factor of a Lambert conform conical projection with two intersecting parallels
	static double ConicalProjectN(double phi1, double phi2);

	//! Projects a given latitude with a Lambert conform conical projection with two intersecting parallels
	static double ConicalProject(double phi, double phi1, double n);

	//! Converts a point cloud coordinates to "cylindrical" ones (in place)
	static bool ConvertCloudToCylindrical(	ccPointCloud* cloud,
											CCVector3d& revolutionCenter,
											unsigned char revolutionAxisDim,
											bool counterclockwise = false);

	//! Converts a point cloud coordinates to "conical" ones (in place)
	/** See ProjectPointOnCone.
	**/
	static bool ConvertCloudToConical(	ccPointCloud* cloud,
										CCVector3d& revolutionCenter,
										unsigned char revolutionAxisDim,
										double latMin_rad,
										double latMax_rad,
										double conicalSpanRatio = 1.0,
										bool counterclockwise = false);

	//! Projects a (longitude,r) couple to a 2D map
	/** For computing 'nProj' see ConicalProjectN.
	**/
	static CCVector3 ProjectPointOnCone(	double lon_rad,
											double lat_rad,
											double latMin_rad,
											double nProj,
											bool counterclockwise);

	//! Computes min and max latitudes of a point cloud (relatively to an axis and an 'origin')
	static bool ComputeMinAndMaxLatitude_rad(	ccPointCloud* cloud,
												double& minLat_rad,
												double& maxLat_rad,
												const CCVector3d& revolutionOrigin,
												unsigned char revolutionAxisDim);

	//! Saves a map as a CSV matrix
	static bool SaveMapAsCSVMatrix( const QSharedPointer<Map>& map,
									QString filename,
									QString xUnit,
									QString yUnit,
									double xConversionFactor = 1.0,
									double yConversionFactor = 1.0,
									ccMainAppInterface* app = 0 );

	//! Converts map to a QImage
	static QImage ConvertMapToImage(	const QSharedPointer<Map>& map,
										ccColorScale::Shared colorScale,
										unsigned colorScaleSteps = ccColorScale::MAX_STEPS);

	//! Converts map to a point cloud
	static ccPointCloud* ConvertMapToCloud(	const QSharedPointer<Map>& map,
											ccPolyline* profile,
											double baseRadius = 1.0,
											bool keepNaNPoints = true);

	//! Converts profile to a (textured) mesh
	static ccMesh* ConvertProfileToMesh(ccPolyline* profile,
										bool counterclockwise,
										unsigned angularSteps = 36,
										QImage mapTexture = QImage());

	//! Various "measures" that can be computed with the map (surfacces, volumes)
	struct Measures
	{
		double total;
		double theoretical;
		double positive;
		double negative;

		Measures() : total(0), theoretical(0), positive(0), negative(0) {}
	};

	//! Computes various surfaces and volumes (see Surfaces & Volumes structures)
	/** Warning: the 'positive' resp. 'negative' volumes coorespond to differences
		of volumes (i.e the volume that is exterior resp. interori to the theoretical
		profile). However, the 'positive' resp. 'negative' surfaces correspond to the
		surface of all the elements that have a positive resp. negative deviation.
	**/
	static bool ComputeSurfacesAndVolumes(	const QSharedPointer<Map>& map,
											ccPolyline* profile,
											Measures& surfaces,
											Measures& volumes);
};

#endif //QSRA_DISTANCE_MAP_GENERATION_TOOL_HEADER

