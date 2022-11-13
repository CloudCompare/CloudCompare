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

#include "LasWaveformSaver.h"

#include "LasDetails.h"

#include <ccPointCloud.h>

LasWaveformSaver::LasWaveformSaver(const ccPointCloud& pointCloud) noexcept
    : m_array(29, '\0')
    , m_pointCloud(pointCloud)
{
}

void LasWaveformSaver::handlePoint(size_t index, laszip_point& point)
{
	assert(index < m_pointCloud.size());
	const ccWaveform& w = m_pointCloud.waveforms().at(index);

	{
		QDataStream stream(&m_array, QIODevice::WriteOnly);
		stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);
		stream << w.descriptorID();
		stream << static_cast<quint64>(w.dataOffset() + LasDetails::EvlrHeader::SIZE);
		stream << w.byteCount();
		stream << w.echoTime_ps();
		stream << w.beamDir().x << w.beamDir().y << w.beamDir().z;
	}

	memcpy(point.wave_packet, m_array.constData(), 29);
}
