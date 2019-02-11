//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCANUPO                       #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#      COPYRIGHT: UEB (UNIVERSITE EUROPEENNE DE BRETAGNE) / CNRS         #
//#                                                                        #
//##########################################################################

#ifndef CC_POINT_DESCRIPTOR_HEADER
#define CC_POINT_DESCRIPTOR_HEADER

//Qt
#include <QByteArray>
#include <QString>

//CCLib
#include <ReferenceCloud.h>

//system
#include <vector>
#include <math.h>

// Descriptors' ID must be declared here and then implemented in 'ccPointDescriptor.cpp'.
// 
// WARNING: the ID must be unique and must not be changed once set!
// 
// Once declared (and implemented) in ccPointDescriptor.cpp, the descriptor must be added
// to the static 'vault' (see ScaleParamsComputerVault's constructor).

static const unsigned DESC_INVALID				=	0;	// Unknown descritpr
static const unsigned DESC_DIMENSIONALITY		=	1;	// Original CANUPO's "Dimensionality" descriptor
static const unsigned DESC_DIMENSIONALITY_SF	=	2;	// "Dimensionality" + scalar field descriptor
static const unsigned DESC_CURVATURE			=	3;	// Test: Gaussian curvature descriptor
//static const unsigned DESC_CUSTOM				=	?;	// Example of custom descriptor (to be reimplemented)

//! Generic parameters 'computer' class (at a given scale)
/** Must be inherited by any custom computer.
**/
class ScaleParamsComputer
{
public:

	//! Vault: returns the computer corresponding to the given ID
	static ScaleParamsComputer* GetByID(unsigned descID);

public:

	//! Returns the number of available 'descriptors'
	static unsigned AvailableCount();

	//! Vault: returns the ith computer
	static ScaleParamsComputer* GetByIndex(unsigned index);

public:
	virtual ~ScaleParamsComputer() = default;
	
	//! Returns the associated descriptor ID
	virtual unsigned getID() const = 0;

	//! Returns the associated descriptor name
	virtual QString getName() const = 0;

	//! Returns the number of dimensions per scale for this descriptor
	virtual unsigned dimPerScale() const = 0;

	//! Returns whether the computer requires a scalar field or not
	virtual bool needSF() const { return false; }

	//! Called once before computing parameters at first scale
	virtual void reset() = 0;
	
	//! Computes the parameters at a given scale
	/** Scales are always called in decreasing order.
		\param[in] neighbors the set of neighbors at the current scale
		\param[in] radius current radius (half scale) value
		\param[out] params the computed parameters
		\param[out] invalidScale whether this scale is 'invalid' (i.e. parameters couldn't be computed, default one have been returned instead)
		\return false if an error occurred (e.g. not enough memory)
	**/
	virtual bool computeScaleParams(CCLib::ReferenceCloud& neighbors, double radius, float params[], bool& invalidScale) = 0;

protected:
};

//! Set of descriptors
/** Typically computed on (core) points
**/
struct CorePointDesc
{
	//! Parameters array (its size should be 'number of scales' * 'dimensions per scale')
	std::vector<float> params;
};

class ccPointCloud;

//! Set of (core) point descriptors
class CorePointDescSet : public std::vector<CorePointDesc>
{
public:

	CorePointDescSet() : m_descriptorID(0), m_dimPerScale(0) {}
	CorePointDescSet(size_t sz) : std::vector<CorePointDesc>(sz), m_descriptorID(0), m_dimPerScale(0) {}
	CorePointDescSet(size_t sz, const CorePointDesc& defaultVal) : std::vector<CorePointDesc>(sz,defaultVal), m_descriptorID(0), m_dimPerScale(0) {}
	CorePointDescSet(const CorePointDescSet& descSet) : std::vector<CorePointDesc>(descSet), m_descriptorID(descSet.m_descriptorID), m_dimPerScale(descSet.m_dimPerScale) {}

	//! Converts structure to a byte array
	QByteArray toByteArray() const;
	//! Inits structure from a byte array
	bool fromByteArray(const QByteArray& data);

	//! Loads structure of descriptors from an ".msc" file (see Brodu's version)
	bool loadFromMSC(QString filename, QString& error, ccPointCloud* corePoints = 0);

	//! Returns associated scales
	inline const std::vector<float>& scales() const { return m_scales; }

	//! Sets associated scales
	/** \warning Automatically resize the descriptors 'params' vector
		\warning Call this AFTER having resized the vector!
		\return success
	**/
	bool setScales(const std::vector<float>& scales);

	//! Returns associated descriptor ID
	inline const unsigned descriptorID() const { return m_descriptorID; }
	//! Sets associated descriptor ID
	inline void setDescriptorID(unsigned ID) { m_descriptorID = ID; }

	//! Returns the number of dimensions per scale
	inline const unsigned dimPerScale() const { return m_dimPerScale; }
	//! Sets associated descriptor ID
	inline void setDimPerScale(unsigned dim) { m_dimPerScale = dim; }

protected:

	//! Associated scales
	std::vector<float> m_scales;

	//! Associated descriptor ID
	unsigned m_descriptorID;

	//! Dimensions per scale
	unsigned m_dimPerScale;
};

#endif 