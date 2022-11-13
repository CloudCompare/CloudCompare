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

// Qt
#include <QByteArray>
#include <QDataStream>
// LASzip
#include <laszip/laszip_api.h>

class ccPointCloud;

struct LasWaveformSaver
{
	LasWaveformSaver(const ccPointCloud& pointCloud) noexcept;

	void handlePoint(size_t index, laszip_point& point);

  private:
	QByteArray          m_array;
	const ccPointCloud& m_pointCloud;
};
