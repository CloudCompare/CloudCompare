//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: LAS-IO Plugin                      #
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
//#                   COPYRIGHT: Thomas Montaigu                           #
//#                                                                        #
//##########################################################################

#include "LasTiler.h"

#include <QElapsedTimer>
#include <QFileInfo>
#include <ccProgressDialog.h>

CC_FILE_ERROR TileLasReader(laszip_POINTER laszipReader, const QString& originName, const LasTilingOptions& options)
{
	laszip_header* laszipHeader{nullptr};
	laszip_CHAR*   errorMsg{nullptr};

	if (laszip_get_header_pointer(laszipReader, &laszipHeader))
	{
		laszip_get_error(laszipHeader, &errorMsg);
		ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
		laszip_close_reader(laszipReader);
		laszip_clean(laszipReader);
		laszip_destroy(laszipReader);
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	laszip_U64 pointCount;
	if (laszipHeader->version_minor == 4)
	{
		pointCount = laszipHeader->extended_number_of_point_records;
	}
	else
	{
		pointCount = laszipHeader->number_of_point_records;
	}

	laszip_point* laszipPoint{nullptr};
	if (laszip_get_point_pointer(laszipReader, &laszipPoint))
	{
		laszip_get_error(laszipHeader, &errorMsg);
		ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
		laszip_close_reader(laszipReader);
		laszip_clean(laszipReader);
		laszip_destroy(laszipReader);
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	ccLog::Print(QString("Tiles: %1 x %2").arg(options.numTiles0).arg(options.numTiles1));

	std::vector<laszip_POINTER> writers;
	writers.resize(static_cast<size_t>(options.numTiles0) * options.numTiles1, nullptr);

	CC_FILE_ERROR error = CC_FERR_NO_ERROR;

	const size_t index0 = options.index0();
	const size_t index1 = options.index1();

	double cloudMins[3] = {
	    laszipHeader->min_x,
	    laszipHeader->min_y,
	    laszipHeader->min_z,
	};

	double cloudBBox[3] = {
	    laszipHeader->max_x - laszipHeader->min_x,
	    laszipHeader->max_y - laszipHeader->min_y,
	    laszipHeader->max_z - laszipHeader->min_z,
	};

	double tileSize[2] = {
	    cloudBBox[index0] / options.numTiles0,
	    cloudBBox[index1] / options.numTiles1,
	};

	laszip_F64 laszipCoordinates[3] = {0};

	QElapsedTimer timer;
	timer.start();

	QFileInfo originInfo(originName);

	ccProgressDialog progressDialog(true);
	progressDialog.setMethodTitle("Tiling LAS file");
	progressDialog.setInfo("Tiling...");
	CCCoreLib::NormalizedProgress normProgress(&progressDialog, pointCount);
	progressDialog.start();

	for (unsigned i = 0; i < pointCount; ++i)
	{
		if (laszip_read_point(laszipReader))
		{
			laszip_get_error(laszipReader, &errorMsg);
			ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
			error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
			break;
		}

		if (laszip_get_coordinates(laszipReader, laszipCoordinates))
		{
			laszip_get_error(laszipReader, &errorMsg);
			ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
			error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
			break;
		}

		auto tileI = static_cast<size_t>((laszipCoordinates[index0] - cloudMins[index0]) / tileSize[0]);
		tileI      = std::min(tileI, static_cast<size_t>(options.numTiles0 - 1));
		auto tileJ = static_cast<size_t>((laszipCoordinates[index1] - cloudMins[index1]) / tileSize[1]);
		tileJ      = std::min(tileJ, static_cast<size_t>(options.numTiles1 - 1));

		const size_t tileIndex = (tileI * options.numTiles1) + tileJ;

		laszip_POINTER& writer = writers[tileIndex];

		if (writer == nullptr)
		{
			QString outputName;

			if (!options.outputDir.isEmpty())
			{
				outputName += options.outputDir;
				outputName += '/';
			}

			const QString fileName = QString("%1_%2_%3.%4").arg(originInfo.baseName(), QString::number(tileI), QString::number(tileJ), originInfo.suffix());
			outputName += fileName;
			const std::string outputNameStd = outputName.toStdString();

			if (laszip_create(&writer))
			{
				ccLog::Warning("[LAS] Failed to create tile writer");
				error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
				break;
			}

			if (laszip_set_header(writer, laszipHeader))
			{
				laszip_get_error(writer, &errorMsg);
				ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
				error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
				break;
			}

			if (laszip_open_writer(writer, outputNameStd.c_str(), false))
			{
				laszip_get_error(writer, &errorMsg);
				ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
				error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
				break;
			}
		}

		if (laszip_set_point(writer, laszipPoint))
		{
			laszip_get_error(writer, &errorMsg);
			ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
			error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
			break;
		}

		if (laszip_write_point(writer))
		{
			laszip_get_error(writer, &errorMsg);
			ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
			error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
			break;
		}

		if (laszip_update_inventory(writer))
		{
			laszip_get_error(writer, &errorMsg);
			ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
			error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
			break;
		}

		normProgress.oneStep();
	}

	for (laszip_POINTER writer : writers)
	{
		if (writer == nullptr)
		{
			continue;
		}

		laszip_close_writer(writer);
		laszip_clean(writer);
		laszip_destroy(writer);
	}

	laszip_close_reader(laszipReader);
	laszip_clean(laszipReader);
	laszip_destroy(laszipReader);

	if (error == CC_FERR_NO_ERROR)
	{
		timer.elapsed();
		qint64 elapsed_ms = timer.elapsed();
		qint64 minutes    = elapsed_ms / (1000 * 60);
		elapsed_ms -= minutes * (1000 * 60);
		qint64 seconds = elapsed_ms / 1000;
		elapsed_ms -= seconds * 1000;
		ccLog::Print(QString("[LAS] File tiled in %1m%2s%3ms").arg(minutes).arg(seconds).arg(elapsed_ms));
	}

	return error;
}
