#ifndef CC_WAVEFORM_HEADER
#define CC_WAVEFORM_HEADER

//Local
#include "qCC_db.h"
#include "ccGLMatrix.h"
#include "ccSerializableObject.h"

//CCLib
#include <CCGeom.h>

//system
#include <cstdint>
#include <cstdlib>

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
	bool isSerializable() const override { return true; }
	bool toFile(QFile& out) const override;
	bool fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

	uint32_t numberOfSamples;	//!< Number of samples
	uint32_t samplingRate_ps;	//!< Sampling rate in pico seconds
	double digitizerGain;		//!< Digitizer gain (VOLTS = OFFSET + GAIN * Raw_Waveform_Amplitude)
	double digitizerOffset;		//!< Digitizer offset (VOLTS = OFFSET + GAIN * Raw_Waveform_Amplitude)
	uint8_t bitsPerSample;		//!< Number of bits per sample
};

//! Waveform
/** \warning Waveforms do not own their data!
**/
class QCC_DB_LIB_API ccWaveform : public ccSerializableObject
{
public:

	//! Default constructor
	ccWaveform(uint8_t descriptorID = 0);
	
	//! Destructor
	~ccWaveform() override = default;

	//! Returns the associated descriptor (ID)
	/** \warning A value of zero indicates that there is no associated waveform data.
	**/
	inline uint8_t descriptorID() const { return m_descriptorID; }

	//! Sets the associated descriptor (ID)
	inline void setDescriptorID(uint8_t id) { m_descriptorID = id; }

	//! Describes the waveform data
	void setDataDescription(uint64_t dataOffset, uint32_t byteCount);

	//! Returns the (raw) value of a given sample
	uint32_t getRawSample(uint32_t i, const WaveformDescriptor& descriptor, const uint8_t* dataStorage) const;

	//! Returns the (real) value of a given sample (in volts)
	double getSample(uint32_t i, const WaveformDescriptor& descriptor, const uint8_t* dataStorage) const;

	//! Returns the range of (real) samples
	double getRange(double& minVal, double& maxVal, const WaveformDescriptor& descriptor, const uint8_t* dataStorage) const;

	//! Decodes the samples and store them in a vector
	bool decodeSamples(std::vector<double>& values, const WaveformDescriptor& descriptor, const uint8_t* dataStorage) const;

	//! Exports (real) samples to an ASCII file
	bool toASCII(const QString& filename, const WaveformDescriptor& descriptor, const uint8_t* dataStorage) const;

	//! Helper: exports a series of values as an ASCII file
	static bool ToASCII(const QString& filename, std::vector<double>& values, uint32_t samplingRate_ps);

	//! Returns the sample position in 3D
	CCVector3 getSamplePos(float i, const CCVector3& P0, const WaveformDescriptor& descriptor) const;

	//! Returns the number of allocated bytes
	inline uint32_t byteCount() const { return m_byteCount; }

	//! Returns the byte offset to waveform data
	inline uint64_t dataOffset() const { return m_dataOffset; }

	//! Sets the byte offset to waveform data
	inline void setDataOffset(uint64_t offset) { m_dataOffset = offset; }

	//! Gives access to the internal data
	inline const uint8_t* data(const uint8_t* dataStorage) const { return dataStorage + m_dataOffset; }

	//! Sets the beam direction
	inline void setBeamDir(const CCVector3f& dir) { m_beamDir = dir; }

	//! Returns the beam direction
	inline const CCVector3f& beamDir() const { return m_beamDir; }

	//! Set the echo time (in picoseconds)
	inline void setEchoTime_ps(float time_ps) { m_echoTime_ps = time_ps; }

	//! Returns the echo time (in picoseconds)
	inline float echoTime_ps() const { return m_echoTime_ps; }

	//! Applies a rigid transformation (on the beam direction)
	void applyRigidTransformation(const ccGLMatrix& trans);

	//! Returns the return index
	uint8_t returnIndex() const { return m_returnIndex; }
	//! Sets the return index
	void setReturnIndex(uint8_t index) { m_returnIndex = index; }

	//inherited from ccSerializableObject
	bool isSerializable() const override { return true; }
	bool toFile(QFile& out) const override;
	bool fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

protected: //members

	//! Waveform packet size in bytes
	/** \warning Not necessarily equal to the number of samples!
	**/
	uint32_t m_byteCount;

	//! Byte offset to waveform data
	uint64_t m_dataOffset;

	//! Laser beam direction
	/** Parametric line equation for extrapolating points along the associated waveform:
		X = X0 + X(t)
		Y = Y0 + Y(t)
		Z = Z0 + Z(t)
	**/
	CCVector3f m_beamDir;

	//! Return Point location (in picoseconds)
	/** The offset in picoseconds from the first digitized value to the location
		within the waveform packet that the associated return pulse was detected.
	**/
	float m_echoTime_ps;

	//! Wave Packet descriptor index
	/** \warning A value of zero indicates that there is no associated waveform data.
	**/
	uint8_t m_descriptorID;

	//! Return index
	uint8_t m_returnIndex;
};

//! Waveform proxy
/** For easier access to the waveform data
**/
class QCC_DB_LIB_API ccWaveformProxy
{
public:

	//! Default constructor
	ccWaveformProxy(const ccWaveform& w, const WaveformDescriptor& d, const uint8_t* storage)
		: m_w(w)
		, m_d(d)
		, m_storage(storage)
	{}

	//! Returns whether the waveform (proxy) is valid or not
	inline bool isValid() const { return m_storage && m_w.descriptorID() != 0 && m_d.numberOfSamples != 0; }

	//! Returns the associated descriptor (ID)
	/** \warning A value of zero indicates that there is no associated waveform data.
	**/
	inline uint8_t descriptorID() const { return m_w.descriptorID(); }

	//! Returns the (raw) value of a given sample
	inline uint32_t getRawSample(uint32_t i) const { return m_w.getRawSample(i, m_d, m_storage); }

	//! Returns the (real) value of a given sample (in volts)
	inline double getSample(uint32_t i) const { return m_w.getSample(i, m_d, m_storage); }

	//! Returns the range of (real) samples
	inline double getRange(double& minVal, double& maxVal) const { return m_w.getRange(minVal, maxVal, m_d, m_storage); }

	//! Decodes the samples and store them in a vector
	inline bool decodeSamples(std::vector<double>& values) const { return m_w.decodeSamples(values, m_d, m_storage); }

	//! Exports (real) samples to an ASCII file
	inline bool toASCII(const QString& filename) const { return m_w.toASCII(filename, m_d, m_storage); }

	//! Returns the sample position in 3D
	inline CCVector3 getSamplePos(float i, const CCVector3& P0) const { return m_w.getSamplePos(i, P0, m_d); }

	//! Returns the number of allocated bytes
	inline uint32_t byteCount() const { return m_w.byteCount(); }

	//! Gives access to the internal data
	inline const uint8_t* data() const { return m_w.data(m_storage); }

	//! Returns the beam direction
	inline const CCVector3f& beamDir() const { return m_w.beamDir(); }

	//! Returns the echo time (in picoseconds)
	inline float echoTime_ps() const { return m_w.echoTime_ps(); }

	//! Returns the number of samples
	inline uint32_t numberOfSamples() const { return m_d.numberOfSamples; }

	//! Returns the descriptor
	inline const WaveformDescriptor& descriptor() const { return m_d; }
	//! Returns the waveform
	inline const ccWaveform& waveform() const { return m_w; }

protected: //members

	//! Associated ccWaveform instance
	const ccWaveform& m_w;
	//! Associated descriptor
	const WaveformDescriptor& m_d;
	//! Associated storage data
	const uint8_t* m_storage;
};

#endif //CC_WAVEFORM_HEADER
