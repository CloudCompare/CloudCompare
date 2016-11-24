#include "ccWaveform.h"

//Qt
#include <QDataStream>

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

bool WaveformDescriptor::fromFile(QFile& in, short dataVersion, int flags)
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
	, m_data(0)
	, m_beamDir(0, 0, 0)
	, m_echoTime_ps(0)
	, m_descriptorID(descriptorID)
{
}
	
ccWaveform::ccWaveform(const ccWaveform& w)
	: m_byteCount(0)
	, m_beamDir(w.m_beamDir)
	, m_echoTime_ps(w.m_echoTime_ps)
	, m_data(0)
	, m_descriptorID(w.m_descriptorID)
{
	//try to reserve some memory
	if (reserve_bytes(w.m_byteCount))
	{
		//copy the data if successful
		for (uint32_t i = 0; i < m_byteCount; ++i)
		{
			m_data[i] = w.m_data[i];
		}
	}
}

bool ccWaveform::setData(const uint8_t* data, uint32_t byteCount)
{
	if (!reserve_bytes(byteCount))
	{
		//not enough memory
		return false;
	}

	for (uint32_t i = 0; i < byteCount; ++i)
	{
		m_data[i] = data[i];
	}

	return true;
}

void ccWaveform::clear()
{
	if (m_data)
	{
		delete[] m_data;
		m_data = 0;
	}
	m_byteCount = 0;
}

uint32_t ccWaveform::getRawSample(uint32_t i, const WaveformDescriptor& descriptor) const
{
	if (!m_data)
	{
		assert(false);
		return 0;
	}
	
	switch (descriptor.bitsPerSample)
	{
	case 8:
		return m_data[i];

	case 16:
		return reinterpret_cast<uint16_t*>(m_data)[i];

	case 24:
	{
		uint32_t v = *reinterpret_cast<uint32_t*>(m_data + 3 * i);
		//'hide' the 4th byte
		v &= (static_cast<uint32_t>(1 << 24) - 1);
		return v;
	}

	case 32:
		return reinterpret_cast<uint32_t*>(m_data)[i];

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
		uint32_t value = m_data[lastByteIndex];
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
			value |= m_data[lastByteIndex];
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
}

double ccWaveform::getSample(uint32_t i, const WaveformDescriptor& descriptor) const
{
	uint32_t raw = getRawSample(i, descriptor);

	return descriptor.digitizerGain * raw + descriptor.digitizerOffset;
}

CCVector3 ccWaveform::getSamplePos(uint32_t i, const CCVector3& P0, const WaveformDescriptor& descriptor) const
{
	float dist = m_echoTime_ps - i * descriptor.samplingRate_ps;
	return P0 + CCVector3::fromArray(m_beamDir.u) * dist;
}

void ccWaveform::applyRigidTransformation(const ccGLMatrix& trans)
{
	//we apply only the rotation
	CCVector3 u = CCVector3::fromArray(m_beamDir.u);
	trans.applyRotation(u);
	m_beamDir = CCVector3f::fromArray(u.u);
}

bool ccWaveform::reserve_bytes(uint32_t byteCount)
{
	m_data = (uint8_t*)realloc(m_data, byteCount);
	if (!m_data)
	{
		m_byteCount = 0;
		return false;
	}
	m_byteCount = byteCount;
	return true;
}

bool ccWaveform::toFile(QFile& out) const
{
	QDataStream outStream(&out);

	//dataVersion >= 44
	outStream << m_descriptorID;
	if (m_descriptorID != 0) //no need to save invalid waveforms
	{
		outStream << m_byteCount;
		outStream << m_beamDir.x;
		outStream << m_beamDir.y;
		outStream << m_beamDir.z;
		outStream << m_echoTime_ps;
		outStream << QByteArray::fromRawData((const char*)m_data, static_cast<int>(m_byteCount));
	}

	return true;
}

bool ccWaveform::fromFile(QFile& in, short dataVersion, int flags)
{
	QDataStream inStream(&in);

	if (dataVersion < 44)
		return false;

	//dataVersion >= 44
	inStream >> m_descriptorID;
	if (m_descriptorID != 0)
	{
		inStream >> m_byteCount;
		inStream >> m_beamDir.x;
		inStream >> m_beamDir.y;
		inStream >> m_beamDir.z;
		inStream >> m_echoTime_ps;

		QByteArray a;
		inStream >> a;
		if (a.size())
		{
			setData((const uint8_t*)a.data(), static_cast<uint32_t>(a.size()));
		}
	}

	return true;
}
