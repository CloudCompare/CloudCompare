#pragma once

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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//qCC_db
#include <ccPolyline.h>

//CCCoreLib
#include <PointProjectionTools.h>

//! Envelope extractor (with debug GUI)
class ccEnvelopeExtractor
{
public:

	//! Envelope type
	enum EnvelopeType { LOWER, UPPER, FULL };

	//! Extracts a unique closed (2D) envelope polyline of a point cloud
	/** Projects the cloud on its best fitting LS plane first.
		\param points point cloud
		\param allowMultiPass whether to allow multi-pass process (with longer edges potentially generated so as 'disturb' the initial guess)
		\param maxEdgeLength max edge length (ignored if 0, in which case the envelope is the convex hull)
		\param preferredNormDim to specifiy a preferred (normal) direction for the polyline extraction
		\param preferredUpDir to specifiy a preferred up direction for the polyline extraction (preferredNormDim must be defined as well and must be normal to this 'up' direction)
		\param envelopeType to specify a type of envelope (you should define a 'up' direction to get proper lower and upper envelope)
		\param[out] originalPointIndexes to get the indexes (relatively to the input cloud) of the output polyline vertices
		\param enableVisualDebugMode whether to display a (debug) window to represent the algorithm process
		\param maxAngleDeg max angle between segments (angle between 0 and 180, in degrees)
		\return envelope polyline (or 0 if an error occurred)
	**/
	static ccPolyline* ExtractFlatEnvelope(	CCCoreLib::GenericIndexedCloudPersist* points,
											bool allowMultiPass,
											PointCoordinateType maxEdgeLength = 0,
											const PointCoordinateType* preferredNormDim = nullptr,
											const PointCoordinateType* preferredUpDir = nullptr,
											EnvelopeType envelopeType = FULL,
											std::vector<unsigned>* originalPointIndexes = nullptr,
											bool enableVisualDebugMode = false,
											double maxAngleDeg = 0.0);

	//! Extracts one or several parts of the (2D) envelope polyline of a point cloud
	/** Projects the cloud on its best fitting LS plane first.
		\warning output polylines set (parts) may be empty if all the vertices are too far from each other!
		\param points point cloud
		\param allowMultiPass whether to allow multi-pass process (with longer edges potentially generated so as 'disturb' the initial guess)
		\param maxEdgeLength max edge length (ignored if 0, in which case the envelope is the convex hull)
		\param[out] parts output polyline parts
		\param envelopeType envelope type (FULL by default)
		\param allowSplitting whether the polyline can be split or not
		\param preferredNormDim to specifiy a preferred (normal) direction for the polyline extraction
		\param preferredUpDir to specifiy a preferred up direction for the polyline extraction (preferredNormDim must be defined as well and must be normal to this 'up' direction)
		\param enableVisualDebugMode whether to display a (debug) window to represent the algorithm process
		\return success
	**/
	static bool ExtractFlatEnvelope(CCCoreLib::GenericIndexedCloudPersist* points,
									bool allowMultiPass,
									PointCoordinateType maxEdgeLength,
									std::vector<ccPolyline*>& parts,
									EnvelopeType envelopeType = FULL,
									bool allowSplitting = true,
									const PointCoordinateType* preferredNormDim = nullptr,
									const PointCoordinateType* preferredUpDir = nullptr,
									bool enableVisualDebugMode = false);

protected:

	//! Determines the 'concave' hull of a set of points
	/** Inspired from JIN-SEO PARK AND SE-JONG OH, "A New Concave Hull Algorithm
		and Concaveness Measure for n-dimensional Datasets", 2012
		Calls extractConvexHull2D (see associated warnings).
		\note Almost the same method as CCCoreLib::PointProjectionTools::ExtractConcaveHull2D
		but with partial envelope support and visual debug mode.
		\param points input set of points
		\param hullPoints output points (on the convex hull)
		\param envelopeType type of envelope (above / below / full)
		\param allowMultiPass whether to allow multi-pass process (with longer edges potentially generated so as 'disturb' the initial guess)
		\param maxSquareLength maximum square length (ignored if <= 0, in which case the method simply returns the convex hull!)
		\param enableVisualDebugMode whether to display a (debug) window to represent the algorithm process
		\param maxAngleDeg max angle between segments (angle between 0 and 180, in degrees)
		\return success
	**/
	static bool ExtractConcaveHull2D(	std::vector<CCCoreLib::PointProjectionTools::IndexedCCVector2>& points,
										std::list<CCCoreLib::PointProjectionTools::IndexedCCVector2*>& hullPoints,
										EnvelopeType envelopeType,
										bool allowMultiPass,
										PointCoordinateType maxSquareLength = 0,
										bool enableVisualDebugMode = false,
										double maxAngleDeg = 90.0);


};
