#include "ccWaveform.h"

//Qt
#include <QDataStream>
#include <QFile>
#include <QTextStream>

WaveformDescriptor::WaveformDescriptor()
	: numberOfSamples(0)
	, samplingRate_ps(0)
	, digitizerGain(0)
	, digitizerOffset(0)
	, bitsPerSample(0)
{}

bool WaveformDescriptor::operator != (const WaveformDescriptor& d) const
{
	return	d.bitsPerSample != bitsPerSample
		|| d.digitizerGain != digitizerGain
		|| d.digitizerOffset != digitizerOffset
		|| d.numberOfSamples != numberOfSamples
		|| d.samplingRate_ps != samplingRate_ps;
}

bool WaveformDescriptor::toFile(QFile& out) const
{
	QDataStream outStream(&out);

	//dataVersion >= 44
	outStream << numberOfSamples;
	outStream << samplingRate_ps;
	outStream << digitizerGain;
	outStream << digitizerOffset;
	outStream << bitsPerSample;

	return true;
}

bool WaveformDescriptor::fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	QDataStream inStream(&in);

	if (dataVersion < 44)
		return false;

	//dataVersion >= 44
	inStream >> numberOfSamples;
	inStream >> samplingRate_ps;
	inStream >> digitizerGain;
	inStream >> digitizerOffset;
	inStream >> bitsPerSample;

	return true;
}

ccWaveform::ccWaveform(uint8_t descriptorID/*=0*/)
	: m_byteCount(0)
	, m_dataOffset(0)
	, m_beamDir(0, 0, 0)
	, m_echoTime_ps(0)
	, m_descriptorID(descriptorID)
	, m_returnIndex(1)
{
}

void ccWaveform::setDataDescription(uint64_t dataOffset, uint32_t byteCount)
{
	m_dataOffset = dataOffset;
	m_byteCount = byteCount;
}

uint32_t ccWaveform::getRawSample(uint32_t i, const WaveformDescriptor& descriptor, const uint8_t* dataStorage) const
{
	if (!dataStorage)
	{
		assert(false);
		return 0;
	}
	
	const uint8_t* _data = data(dataStorage);
	switch (descriptor.bitsPerSample)
	{
	case 8:
		return _data[i];

	case 16:
		return reinterpret_cast<const uint16_t*>(_data)[i];

	case 24:
	{
		uint32_t v = *reinterpret_cast<const uint32_t*>(_data + 3 * i);
		//'hide' the 4th byte
		static const uint32_t Byte4Mask = 0x0FFF;
		v &= Byte4Mask;
		return v;
	}

	case 32:
		return reinterpret_cast<const uint32_t*>(_data)[i];

	default: //other 'strange' bps values ;)
	{
		uint32_t firstBitIndex = descriptor.bitsPerSample * i;
		uint32_t firstByteIndex = (firstBitIndex >> 3); // = divide by 8

		uint32_t lastBitIndex = firstBitIndex + descriptor.bitsPerSample - 1;
		uint32_t lastByteIndex = (lastBitIndex >> 3); // = divide by 8
		if (lastByteIndex >= m_byteCount)
		{
			assert(false);
			return 0;
		}

		//last byte (may be the first one!)
		uint32_t value = _data[lastByteIndex];
		{
			//number of bits used in the current byte
			uint32_t r = ((lastByteIndex + 1) % 8);
			if (r != 0)
			{
				//we keep only the used bits
				value &= ((1 << r) - 1);
			}
		}

		//other bytes (if any)
		while (lastByteIndex != firstByteIndex)
		{
			--lastByteIndex;
			value <<= 8;
			value |= _data[lastByteIndex];
		}

		//remove the first unused bits (if any)
		{
			uint32_t r = firstBitIndex - 8 * firstByteIndex;
			if (r != 0)
			{
				value >>= r;
			}
		}

		return value;
	}
	}

	//we should never arrive here
	assert(false);
	return 0;
}

double ccWaveform::getSample(uint32_t i, const WaveformDescriptor& descriptor, const uint8_t* dataStorage) const
{
	uint32_t raw = getRawSample(i, descriptor, dataStorage);

	return descriptor.digitizerGain * raw + descriptor.digitizerOffset;
}

bool ccWaveform::decodeSamples(std::vector<double>& values, const WaveformDescriptor& descriptor, const uint8_t* dataStorage) const
{
	try
	{
		values.resize(descriptor.numberOfSamples);
		for (uint32_t i = 0; i < descriptor.numberOfSamples; ++i)
		{
			values[i] = getSample(i, descriptor, dataStorage);
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	return true;
}

bool ccWaveform::toASCII(const QString& filename, const WaveformDescriptor& descriptor, const uint8_t* dataStorage) const
{
	if (descriptor.numberOfSamples == 0)
	{
		assert(false);
		return false;
	}

	std::vector<double> values;
	if (!decodeSamples(values, descriptor, dataStorage))
	{
		ccLog::Warning(QString("[ccWaveform::toASCII] Not enough memory"));
		return false;
	}

	return ToASCII(filename, values, descriptor.samplingRate_ps);
}

bool ccWaveform::ToASCII(const QString& filename, std::vector<double>& values, uint32_t samplingRate_ps)
{
	QFile file(filename);
	if (!file.open(QFile::Text | QFile::WriteOnly))
	{
		ccLog::Warning(QString("[ccWaveform::toASCII] Failed to open file '%1' for writing").arg(filename));
		return false;
	}

	QTextStream stream(&file);
	stream.setRealNumberPrecision(6);
	stream.setRealNumberNotation(QTextStream::FixedNotation);
	stream << "//time(ps);intensity" << endl;

	for (uint32_t i = 0; i < values.size(); ++i)
	{
		stream << i * samplingRate_ps << ";" << values[i] << endl;
	}

	file.close();
	ccLog::Print(QString("[ccWaveform::toASCII] File '%1' has been saved successfully").arg(filename));

	return true;
}

double ccWaveform::getRange(double& minVal, double& maxVal, const WaveformDescriptor& descriptor, const uint8_t* dataStorage) const
{
	if (descriptor.numberOfSamples == 0)
	{
		assert(false);
		minVal = maxVal = std::numeric_limits<double>::quiet_NaN();
		return 0.0;
	}
	else
	{
		minVal = maxVal = getSample(0, descriptor, dataStorage);
	}

	for (uint32_t i = 1; i < descriptor.numberOfSamples; ++i)
	{
		double c = getSample(i, descriptor, dataStorage);
		maxVal = std::max(maxVal, c);
		minVal = std::min(minVal, c);
	}

	return maxVal - minVal;
}

CCVector3 ccWaveform::getSamplePos(float index, const CCVector3& P0, const WaveformDescriptor& descriptor) const
{
	float delta_ps = m_echoTime_ps - index * descriptor.samplingRate_ps;
	return P0 + CCVector3::fromArray(m_beamDir.u) * delta_ps;
}

void ccWaveform::applyRigidTransformation(const ccGLMatrix& trans)
{
	//we apply only the rotation
	CCVector3 u = CCVector3::fromArray(m_beamDir.u);
	trans.applyRotation(u);
	m_beamDir = CCVector3f::fromArray(u.u);
}

bool ccWaveform::toFile(QFile& out) const
{
	QDataStream outStream(&out);

	//dataVersion >= 46
	outStream << m_descriptorID;
	if (m_descriptorID != 0) //no need to save invalid waveforms
	{
		outStream << m_byteCount;
		outStream << static_cast<quint64>(m_dataOffset); //see comment below (in 'fromFile')
		outStream << m_beamDir.x;
		outStream << m_beamDir.y;
		outStream << m_beamDir.z;
		outStream << m_echoTime_ps;
		//dataVersion >= 47
		outStream << m_returnIndex;
	}

	return true;
}

bool ccWaveform::fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	QDataStream inStream(&in);

	if (dataVersion < 46)
		return false;

	//dataVersion >= 46
	inStream >> m_descriptorID;
	if (m_descriptorID != 0)
	{
		inStream >> m_byteCount;

		//for compilation on gcc/clang, we need to be 'more explicit'...
		//(apparently uint64_t is not 'evidently' casted to quint64?!)
		quint64 dataOffset;
		inStream >> dataOffset;
		m_dataOffset = static_cast<uint64_t>(dataOffset);

		inStream >> m_beamDir.x;
		inStream >> m_beamDir.y;
		inStream >> m_beamDir.z;
		inStream >> m_echoTime_ps;

		if (dataVersion > 46)
		{
			//dataVersion >= 47
			inStream >> m_returnIndex;
		}
		else
		{
			m_returnIndex = 1;
		}
	}

	return true;
}
