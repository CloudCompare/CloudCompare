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

#include "LasWaveformLoader.h"

#include "LasDetails.h"

#include <QDataStream>

static bool ParseWavepacketDescriptorVlr(const laszip_vlr_struct& vlr, WaveformDescriptor& descriptor)
{
	if (vlr.record_length_after_header < 26)
	{
		return false;
	}

	auto        data = QByteArray::fromRawData(reinterpret_cast<const char*>(vlr.data), vlr.record_length_after_header);
	QDataStream stream(data);
	stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

	uint8_t compressionType;
	stream >> descriptor.bitsPerSample >> compressionType >> descriptor.numberOfSamples >> descriptor.samplingRate_ps >> descriptor.digitizerGain >> descriptor.digitizerOffset;

	if (descriptor.digitizerGain == 0.0)
	{
		// shouldn't be 0 by default!
		descriptor.digitizerGain = 1.0;
	}

	return true;
}

static ccPointCloud::FWFDescriptorSet ParseWaveformDescriptorVlrs(const laszip_vlr_struct* vlrs,
                                                                  laszip_U32               numVlrs)
{
	ccPointCloud::FWFDescriptorSet descriptors;

	for (laszip_U32 i = 0; i < numVlrs; ++i)
	{
		const laszip_vlr_struct& vlr = vlrs[i];
		if (strcmp(vlr.user_id, "LASF_Spec") == 0 && 99 < vlr.record_id && vlr.record_id < 355)
		{
			WaveformDescriptor descriptor;
			if (!ParseWavepacketDescriptorVlr(vlr, descriptor))
			{
				ccLog::Warning("[LAS] Invalid Descriptor VLR");
			}
			else
			{
				descriptors.insert(vlr.record_id - 99, descriptor);
			}
		}
	}
	return descriptors;
}

LasWaveformLoader::LasWaveformLoader(const laszip_header_struct& laszipHeader,
                                     const QString&              lasFilename,
                                     ccPointCloud&               pointCloud)
    : isPointFormatExtended(laszipHeader.point_data_format >= 6)
{
	descriptors = ParseWaveformDescriptorVlrs(laszipHeader.vlrs, laszipHeader.number_of_variable_length_records);
	ccLog::Print("[LAS] %d Waveform Packet Descriptor VLRs found", descriptors.size());

	QFile fwfDataSource;
	if (laszipHeader.start_of_waveform_data_packet_record != 0)
	{
		ccLog::Print("[LAS] Waveform data is located within the las file");
		fwfDataSource.setFileName(lasFilename);
		if (!fwfDataSource.open(QFile::ReadOnly))
		{
			ccLog::Warning(QString("[LAS] Failed to re open the las file: %1").arg(fwfDataSource.errorString()));
			return;
		}

		if (!fwfDataSource.seek(laszipHeader.start_of_waveform_data_packet_record))
		{
			ccLog::Warning(QString("[LAS] Failed to find the associated waveform data packets header"));
			return;
		}

		QDataStream            stream(&fwfDataSource);
		LasDetails::EvlrHeader evlrHeader;
		stream >> evlrHeader;
		if (stream.status() == QDataStream::Status::ReadPastEnd)
		{
			ccLog::Warning(QString("[LAS] Failed to read the associated waveform data packets"));
			return;
		}

		if (!evlrHeader.isWaveFormDataPackets())
		{
			ccLog::Warning("[LAS] Invalid waveform EVLR");
			return;
		}
		fwfDataCount  = evlrHeader.recordLength;
		fwfDataOffset = LasDetails::EvlrHeader::SIZE;
		if (fwfDataCount == 0)
		{
			ccLog::Warning(QString("[LAS] Invalid waveform data packet size (0). We'll load all the "
			                       "remaining part of the file!"));
			fwfDataCount = fwfDataSource.size() - fwfDataSource.pos();
		}
	}
	else if (laszipHeader.global_encoding & 4)
	{
		QFileInfo info(lasFilename);
		QString   wdpFilename = info.path() + "/" + info.completeBaseName() + ".wdp";
		fwfDataSource.setFileName(wdpFilename);
		if (!fwfDataSource.open(QFile::ReadOnly))
		{
			ccLog::Warning(QString("[LAS] Failed to read the associated waveform data packets file "
			                       "(looking for '%1'): %2")
			                   .arg(wdpFilename)
			                   .arg(fwfDataSource.errorString()));
			return;
		}
		fwfDataCount = fwfDataSource.size();

		if (fwfDataCount > LasDetails::EvlrHeader::SIZE)
		{
			QDataStream            stream(&fwfDataSource);
			LasDetails::EvlrHeader evlrHeader;
			stream >> evlrHeader;

			if (evlrHeader.isWaveFormDataPackets())
			{
				// this is a valid EVLR header, we can skip it
				auto p = fwfDataSource.pos();
				fwfDataCount -= LasDetails::EvlrHeader::SIZE;
				fwfDataOffset = LasDetails::EvlrHeader::SIZE;
			}
			else
			{
				// this doesn't look like a valid EVLR
				fwfDataSource.seek(0);
			}
		}
		ccLog::Print(QString("[LAS] Waveform Data Packets are in an external file located at %1").arg(wdpFilename));
	}

	if (fwfDataSource.isOpen() && fwfDataCount != 0)
	{
		ccPointCloud::FWFDataContainer* container{nullptr};
		try
		{
			container = new ccPointCloud::FWFDataContainer;
			container->resize(fwfDataCount);
			pointCloud.waveforms().resize(pointCloud.capacity());
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning(QString("[LAS] Not enough memory to import the waveform data"));
			delete container;
			return;
		}

		fwfDataSource.read((char*)container->data(), fwfDataCount);
		fwfDataSource.close();

		pointCloud.fwfData() = ccPointCloud::SharedFWFDataContainer(container);
	}
}

void LasWaveformLoader::loadWaveform(ccPointCloud& pointCloud, const laszip_point& currentPoint) const
{
	assert(pointCloud.size() > 0);
	if (fwfDataCount == 0)
	{
		return;
	}

	auto        data = QByteArray::fromRawData(reinterpret_cast<const char*>(currentPoint.wave_packet), 29);
	QDataStream stream(data);
	stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

	uint8_t  descriptorIndex     = 0;
	quint64  byteOffset          = 0;
	uint32_t byteCount           = 0;
	float    returnPointLocation = 0;
	float    x_t = 0, y_t = 0, z_t = 0;

	returnPointLocation = ((float*)&currentPoint.wave_packet[13])[0];
	x_t                 = ((float*)&currentPoint.wave_packet[17])[0];
	y_t                 = ((float*)&currentPoint.wave_packet[21])[0];
	z_t                 = ((float*)&currentPoint.wave_packet[25])[0];

	stream >> descriptorIndex >> byteOffset >> byteCount;

	ccPointCloud::FWFDescriptorSet& cloudDescriptors = pointCloud.fwfDescriptors();
	if (descriptors.contains(descriptorIndex) && !cloudDescriptors.contains(descriptorIndex))
	{
		cloudDescriptors.insert(descriptorIndex, descriptors.value(descriptorIndex));
	}
	else if (!descriptors.contains(descriptorIndex))
	{
		if (byteCount != 0) // otherwise it's just a blank/missing waveform
		{
			ccLog::Warning("[LAS] No valid descriptor vlr for index %d", descriptorIndex);
		}
		return;
	}

	if (byteOffset < fwfDataOffset)
	{
		ccLog::Warning("[LAS] Waveform byte offset is smaller that fwfDataOffset");
		byteOffset = fwfDataOffset;
	}

	byteOffset -= fwfDataOffset;

	if (byteOffset + byteCount > fwfDataCount)
	{
		ccLog::Warning("[LAS] Waveform byte count for point %u is bigger than actual fwf data",
		               pointCloud.size() - 1);
		byteCount = (fwfDataCount - byteOffset);
	}

	ccWaveform& w = pointCloud.waveforms()[pointCloud.size() - 1];

	w.setDescriptorID(descriptorIndex);
	w.setDataDescription(byteOffset, byteCount);
	w.setEchoTime_ps(returnPointLocation);
	w.setBeamDir(CCVector3f(x_t, y_t, z_t));

	if (isPointFormatExtended)
	{
		w.setReturnIndex(currentPoint.extended_return_number);
	}
	else
	{
		w.setReturnIndex(currentPoint.return_number);
	}
}
