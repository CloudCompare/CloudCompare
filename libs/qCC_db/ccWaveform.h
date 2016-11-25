#ifndef CC_WAVEFORM_HEADER
#define CC_WAVEFORM_HEADER

//Local
#include "qCC_db.h"
#include "ccGLMatrix.h"
#include "ccSerializableObject.h"

//CCLib
#include <CCGeom.h>

//system
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>

//! Waveform descriptor
class QCC_DB_LIB_API WaveformDescriptor : public ccSerializableObject
{
public:
	
	//! Default constructor
	WaveformDescriptor();

	//! Difference operator
	bool operator != (const WaveformDescriptor& d) const;

	//! Equivalence operator
	bool operator == (const WaveformDescriptor& d) const { return	!(*this != d); }

	//inherited from ccSerializableObject
	virtual bool isSerializable() const override { return true; }
	virtual bool toFile(QFile& out) const override;
	virtual bool fromFile(QFile& in, short dataVersion, int flags) override;

	uint32_t numberOfSamples;	//!< Number of samples
	uint32_t samplingRate_ps;	//!< Sampling rate in pico seconds
	double digitizerGain;		//!< Digitizer gain (VOLTS = OFFSET + GAIN * Raw_Waveform_Amplitude)
	double digitizerOffset;		//!< Digitizer offset (VOLTS = OFFSET + GAIN * Raw_Waveform_Amplitude)
	uint8_t bitsPerSample;		//!< Number of bits per sample
};

//! Waveform
class QCC_DB_LIB_API ccWaveform : public ccSerializableObject
{
public:

	//! Default constructor
	ccWaveform(uint8_t descriptorID = 0);
	
	//! Copy constructor
	ccWaveform(const ccWaveform& w);

	//! Destructor
	virtual ~ccWaveform() { clear(); }

	//! Returns the associated descriptor (ID)
	/** \warning A value of zero indicates that there is no associated waveform data.
	**/
	inline uint8_t descriptorID() const { return m_descriptorID; }

	//! Sets the associated descriptor (ID)
	inline void setDescriptorID(uint8_t id) { m_descriptorID = id; }

	//! Sets the (samples) data
	bool setData(const uint8_t* data, uint32_t byteCount);

	//! Clears the structure
	void clear();

	//! Returns the (raw) value of a given sample
	uint32_t getRawSample(uint32_t i, const WaveformDescriptor& descriptor) const;

	//! Returns the (real) value of a given sample (in volts)
	double getSample(uint32_t i, const WaveformDescriptor& descriptor) const;

	//! Returns the range of (real) samples
	double getRange(double& minVal, double& maxVal, const WaveformDescriptor& descriptor) const;

	//! Expotrs (real) samples to an ASCII file
	bool toASCII(QString filename, const WaveformDescriptor& descriptor) const;

	//! Returns the sample position in 3D
	CCVector3 getSamplePos(uint32_t i, const CCVector3& P0, const WaveformDescriptor& descriptor) const;

	//! Returns the number of allocated bytes
	inline uint32_t byteCount() const { return m_byteCount; }

	//! Gives access to the internal data
	inline const uint8_t* data() const { return m_data; }

	//! Applies a rigid transformation (on the beam direction)
	void applyRigidTransformation(const ccGLMatrix& trans);

	//inherited from ccSerializableObject
	virtual bool isSerializable() const override { return true; }
	virtual bool toFile(QFile& out) const override;
	virtual bool fromFile(QFile& in, short dataVersion, int flags) override;

protected: //methods

	//! Reserves the memory to store the sample values
	bool reserve_bytes(uint32_t byteCount);

protected: //members

	//! Data buffer size (in bytes)
	/** \warning Not necessarily equal to the number of samples!
	**/
	uint32_t m_byteCount;

	//! Samples data
	uint8_t* m_data;

	//! Laser beam direction
	/** Parametric line equation for extrapolating points along the associated waveform:
		X = X0 + X(t)
		Y = Y0 + Y(t)
		Z = Z0 + Z(t)
	**/
	CCVector3f m_beamDir;

	//! Echo time (in picoseconds)
	/** The offset in picoseconds from the first digitized value to the location
		within the waveform packet that the associated return pulse was detected.
	**/
	float m_echoTime_ps;

	//! Associated descriptor ID
	/** \warning A value of zero indicates that there is no associated waveform data.
	**/
	uint8_t m_descriptorID;
};

#endif //CC_WAVEFORM_HEADER
