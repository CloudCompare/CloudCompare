// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#ifndef CC_EDGE_METRICS_HEADER
#define CC_EDGE_METRICS_HEADER

// Qt
#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>

//! Per-edge metric result (BatGraph F6)
struct EdgeMetricResult
{
	double  distance_3d_m          = 0.0;
	double  horizontal_m           = 0.0;
	double  elevation_change_m     = 0.0;
	double  elevation_gain_m       = 0.0;
	double  slope_deg              = 0.0;
	double  slope_percent          = 0.0;
	double  bearing_deg_from_north = 0.0;
	QString direction_8way;
};

//! XYZ coordinate triplet
struct NodeCoord
{
	double x = 0, y = 0, z = 0;
};

//! Static helpers for the BatGraph F6 Distance & Slope Calculator
class ccEdgeMetrics
{
  public:
	//! Compute all metrics for a single edge given its endpoint coordinates
	/** Coordinate system: X = easting, Y = northing (BNG or similar).
	    Bearing is measured clockwise from north (Y+). **/
	static EdgeMetricResult compute(const NodeCoord& from, const NodeCoord& to);

	//! Load a nodes CSV into a name→coord lookup
	/** Accepted formats:
	 *    label, source_cloud_id, x, y, z   (F1 / F5 export — 5 cols)
	 *    label, x, y, z                    (simple — 4 cols)
	 *  Returns false and sets errorMsg on failure. **/
	static bool loadNodes(const QString& path,
	                      QHash<QString, NodeCoord>& out,
	                      QString& errorMsg);

	//! Process an edges CSV, compute metrics for each row, write output CSV
	/** The output CSV keeps all original columns and appends the 8 metric
	    columns (or updates them if they already exist).
	    Returns false and sets errorMsg on failure. **/
	static bool processEdges(const QString& nodesPath,
	                         const QString& edgesPath,
	                         const QString& outputPath,
	                         int& edgesProcessed,
	                         int& edgesSkipped,
	                         QString& errorMsg);

  private:
	static QString direction8Way(double bearingDeg);
};

#endif
