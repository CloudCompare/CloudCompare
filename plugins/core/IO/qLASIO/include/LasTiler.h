#pragma once

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

#include <FileIOFilter.h>
#include <QString>
#include <laszip/laszip_api.h>

enum class LasTilingDimensions
{
	XY = 0,
	XZ = 1,
	YZ = 2,
};

struct LasTilingOptions final
{
	QString             outputDir;
	LasTilingDimensions dims      = LasTilingDimensions::XY;
	unsigned            numTiles0 = 0;
	unsigned            numTiles1 = 0;

	inline size_t index0() const
	{
		switch (dims)
		{
		case LasTilingDimensions::XY:
			Q_FALLTHROUGH();
		case LasTilingDimensions::XZ:
			return 0;
		case LasTilingDimensions::YZ:
			return 1;
		}
		return 0;
	}

	inline size_t index1() const
	{
		switch (dims)
		{
		case LasTilingDimensions::XY:
			return 1;
		case LasTilingDimensions::XZ:
			Q_FALLTHROUGH();
		case LasTilingDimensions::YZ:
			return 2;
		}
		return 1;
	}
};

/// Tiles the cloud that the reader reads into a grid described by the options.
///
/// This takes ownership of the reader and takes care of closing and deleting it
CC_FILE_ERROR TileLasReader(laszip_POINTER laszipReader, const QString& originName, const LasTilingOptions& options);
