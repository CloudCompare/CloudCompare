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

#include "ccEdgeMetrics.h"

// Qt
#include <QFile>
#include <QTextStream>

// std
#include <cmath>

static constexpr double kDeg = 180.0 / M_PI;

EdgeMetricResult ccEdgeMetrics::compute(const NodeCoord& from, const NodeCoord& to)
{
	EdgeMetricResult r;

	const double dx = to.x - from.x;
	const double dy = to.y - from.y;
	const double dz = to.z - from.z;

	r.horizontal_m       = std::sqrt(dx * dx + dy * dy);
	r.distance_3d_m      = std::sqrt(dx * dx + dy * dy + dz * dz);
	r.elevation_change_m = dz;
	r.elevation_gain_m   = std::max(0.0, dz);

	if (r.horizontal_m > 1e-9)
	{
		r.slope_deg     = std::atan2(std::abs(dz), r.horizontal_m) * kDeg;
		r.slope_percent = (std::abs(dz) / r.horizontal_m) * 100.0;
	}

	// Bearing: clockwise from North (Y+). atan2(east, north)
	double bearing = std::atan2(dx, dy) * kDeg;
	if (bearing < 0.0)
		bearing += 360.0;
	r.bearing_deg_from_north = bearing;
	r.direction_8way         = direction8Way(bearing);

	return r;
}

QString ccEdgeMetrics::direction8Way(double b)
{
	// 8 sectors of 45°, centred on the cardinal/intercardinal directions
	if (b < 22.5 || b >= 337.5)
		return QStringLiteral("N");
	if (b < 67.5)
		return QStringLiteral("NE");
	if (b < 112.5)
		return QStringLiteral("E");
	if (b < 157.5)
		return QStringLiteral("SE");
	if (b < 202.5)
		return QStringLiteral("S");
	if (b < 247.5)
		return QStringLiteral("SW");
	if (b < 292.5)
		return QStringLiteral("W");
	return QStringLiteral("NW");
}

bool ccEdgeMetrics::loadNodes(const QString& path, QHash<QString, NodeCoord>& out, QString& errorMsg)
{
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		errorMsg = QStringLiteral("Cannot open nodes file: %1").arg(path);
		return false;
	}

	QTextStream in(&f);
	bool        firstLine = true;

	while (!in.atEnd())
	{
		const QString line = in.readLine().trimmed();
		if (line.isEmpty())
			continue;

		if (firstLine)
		{
			firstLine = false;
			if (line.startsWith("label", Qt::CaseInsensitive) || line.startsWith("name", Qt::CaseInsensitive))
				continue;
		}

		const QStringList parts = line.split(',');
		bool              ok1 = false, ok2 = false, ok3 = false;
		NodeCoord         coord;

		if (parts.size() >= 5)
		{
			// label, source_cloud_id, x, y, z
			coord.x = parts[2].trimmed().toDouble(&ok1);
			coord.y = parts[3].trimmed().toDouble(&ok2);
			coord.z = parts[4].trimmed().toDouble(&ok3);
			if (ok1 && ok2 && ok3)
				out.insert(parts[0].trimmed(), coord);
		}
		else if (parts.size() >= 4)
		{
			// label, x, y, z
			coord.x = parts[1].trimmed().toDouble(&ok1);
			coord.y = parts[2].trimmed().toDouble(&ok2);
			coord.z = parts[3].trimmed().toDouble(&ok3);
			if (ok1 && ok2 && ok3)
				out.insert(parts[0].trimmed(), coord);
		}
	}

	if (out.isEmpty())
	{
		errorMsg = QStringLiteral("No valid node rows found in: %1").arg(path);
		return false;
	}
	return true;
}

// Metric column names appended to (or replacing in) the output
static const QStringList kMetricCols = {
    "distance_3d_m",
    "horizontal_m",
    "elevation_change_m",
    "elevation_gain_m",
    "slope_deg",
    "slope_percent",
    "bearing_deg_from_north",
    "direction_8way",
};

bool ccEdgeMetrics::processEdges(const QString& nodesPath,
                                 const QString& edgesPath,
                                 const QString& outputPath,
                                 int&           edgesProcessed,
                                 int&           edgesSkipped,
                                 QString&       errorMsg)
{
	QHash<QString, NodeCoord> nodes;
	if (!loadNodes(nodesPath, nodes, errorMsg))
		return false;

	QFile ef(edgesPath);
	if (!ef.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		errorMsg = QStringLiteral("Cannot open edges file: %1").arg(edgesPath);
		return false;
	}

	QFile out(outputPath);
	if (!out.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		errorMsg = QStringLiteral("Cannot write output file: %1").arg(outputPath);
		return false;
	}

	QTextStream in(&ef);
	QTextStream os(&out);

	edgesProcessed = 0;
	edgesSkipped   = 0;

	// Parse header to find FromNode / ToNode column indices
	// and to determine which metric columns already exist
	QStringList header;
	bool        hasHeader = false;
	int         fromIdx = 0, toIdx = 1; // defaults

	QVector<int> metricColIdx(kMetricCols.size(), -1); // -1 = not present

	if (!in.atEnd())
	{
		const QString headerLine = in.readLine();
		header                   = headerLine.split(',');
		for (int i = 0; i < header.size(); ++i)
		{
			const QString col = header[i].trimmed().toLower();
			if (col == "fromnode" || col == "from")
				fromIdx = i;
			else if (col == "tonode" || col == "to")
				toIdx = i;

			for (int m = 0; m < kMetricCols.size(); ++m)
			{
				if (col == kMetricCols[m].toLower())
					metricColIdx[m] = i;
			}
		}
		hasHeader = true;

		// Build output header: original columns + any new metric columns
		QStringList outHeader = header;
		for (int m = 0; m < kMetricCols.size(); ++m)
		{
			if (metricColIdx[m] < 0)
			{
				metricColIdx[m] = outHeader.size();
				outHeader.append(kMetricCols[m]);
			}
		}
		os << outHeader.join(',') << "\n";
	}

	while (!in.atEnd())
	{
		const QString line = in.readLine().trimmed();
		if (line.isEmpty())
			continue;

		QStringList parts = line.split(',');

		// Ensure enough columns
		while (parts.size() < (hasHeader ? header.size() : 2))
			parts.append(QString());

		// Extend to cover metric columns
		const int needed = *std::max_element(metricColIdx.begin(), metricColIdx.end()) + 1;
		while (parts.size() < needed)
			parts.append(QString());

		const QString fromName = (fromIdx < parts.size()) ? parts[fromIdx].trimmed() : QString();
		const QString toName   = (toIdx < parts.size()) ? parts[toIdx].trimmed() : QString();

		if (!nodes.contains(fromName) || !nodes.contains(toName))
		{
			++edgesSkipped;
			os << parts.join(',') << "\n";
			continue;
		}

		const EdgeMetricResult m = compute(nodes[fromName], nodes[toName]);
		parts[metricColIdx[0]]   = QString::number(m.distance_3d_m, 'f', 4);
		parts[metricColIdx[1]]   = QString::number(m.horizontal_m, 'f', 4);
		parts[metricColIdx[2]]   = QString::number(m.elevation_change_m, 'f', 4);
		parts[metricColIdx[3]]   = QString::number(m.elevation_gain_m, 'f', 4);
		parts[metricColIdx[4]]   = QString::number(m.slope_deg, 'f', 4);
		parts[metricColIdx[5]]   = QString::number(m.slope_percent, 'f', 4);
		parts[metricColIdx[6]]   = QString::number(m.bearing_deg_from_north, 'f', 2);
		parts[metricColIdx[7]]   = m.direction_8way;

		os << parts.join(',') << "\n";
		++edgesProcessed;
	}

	return true;
}
