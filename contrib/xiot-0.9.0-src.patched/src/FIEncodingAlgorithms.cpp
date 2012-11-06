#include <xiot/FIEncodingAlgorithms.h>
#include <xiot/FITypes.h>
#include <xiot/FIConstants.h>

#include <iostream>
#include <cassert>

#define SWAP_2(x) ( (((x) & 0xff) << 8) | ((unsigned short)(x) >> 8) )
#define SWAP_4(x) ( ((x) << 24) | \
	(((x) << 8) & 0x00ff0000) | \
	(((x) >> 8) & 0x0000ff00) | \
	((x) >> 24) )
#define FIX_SHORT(x) (*(unsigned short *)&(x) = SWAP_2(*(unsigned short *)&(x)))
#define FIX_INT(x)   (*(unsigned int *)&(x)   = SWAP_4(*(unsigned int *)&(x)))
#define FIX_FLOAT(x) static_cast<float>(FIX_INT(x))


namespace FI {


	std::string FloatEncodingAlgorithm::decodeToString(const FI::NonEmptyOctetString &octets) const
	{
		std::vector<float> floatArray;
		decodeToFloatArray(octets, floatArray);
		
		std::stringstream ss;
		std::vector<float>::const_iterator I; 
		for(I = floatArray.begin(); I != floatArray.end() -1; I++)
		{
			ss << (*I) << " ";
		}
		ss << (*I);
		return ss.str();
	}

	void FloatEncodingAlgorithm::decodeToFloatArray(const FI::NonEmptyOctetString &octets, std::vector<float> &vec)
	{
		assert(octets.size() % 4 == 0);
		size_t length = octets.size() / 4;

		const unsigned char*pOctets = octets.c_str();
		std::vector<float> result(length);
		for (size_t i = 0; i < length; i++)
		{
			result[i] = Tools::readFloat(pOctets);
			pOctets += 4;
		}
		std::swap(result, vec);
	}

	void FloatEncodingAlgorithm::encode(const float* values, size_t size, FI::NonEmptyOctetString &octets)
	{
		Tools::float_to_unsigned_int_to_bytes u;
		for(size_t i = 0; i < size; i++)
		{
			float f = values[i];
			// Avoid -0
			if (f == 0x80000000)
			{
				f = 0;
			}
			u.ui = Tools::reverseBytes(reinterpret_cast<int*>(&f));
			octets.insert(octets.end(), u.ub, u.ub+4);
		}
	}

	std::string IntEncodingAlgorithm::decodeToString(const FI::NonEmptyOctetString &octets) const
	{
		std::stringstream ss;
		std::vector<int> intArray;
		decodeToIntArray(octets, intArray);


		std::vector<int>::const_iterator I;
		for(I = intArray.begin(); I != intArray.end() -1; I++)
		{
			ss << (*I) << " ";
		}
		ss << (*I);
		return ss.str();
	}

	void IntEncodingAlgorithm::decodeToIntArray(const FI::NonEmptyOctetString &octets, std::vector<int> &vec)
	{
		assert(octets.size() % 4 == 0);
		size_t length = octets.size() / 4;

		const unsigned char *pOctets = octets.c_str();
		std::vector<int> result(length);
		for (size_t i = 0; i < length; i++)
		{
			result[i] = Tools::readUInt(pOctets);
			pOctets += 4;
		}
		std::swap(result, vec);
	}

	void IntEncodingAlgorithm::encode(const int* values, size_t size, FI::NonEmptyOctetString &octets)
	{
		Tools::float_to_unsigned_int_to_bytes u;
		for(size_t i = 0; i < size; i++)
		{
			u.ui = FIX_INT(values[i]);
			octets.insert(octets.end(), u.ub, u.ub+4);
		}
	}

	std::string BooleanEncodingAlgorithm::decodeToString(const FI::NonEmptyOctetString &) const
	{
		throw std::runtime_error("BooleanEncodingAlgorithm not implemented (yet)");
		/*std::vector<bool> floatArray = decodeToBoolArray(octets);
		std::stringstream ss;
		std::vector<bool>::const_iterator I; 
		for(I = floatArray.begin(); I != floatArray.end() -1; I++)
		{
		ss << (*I) << " ";
		}
		ss << (*I);
		return ss.str();*/
	}

	void BooleanEncodingAlgorithm::decodeToBoolArray(const FI::NonEmptyOctetString &, std::vector<bool> &)
	{
		throw std::runtime_error("BooleanEncodingAlgorithm not implemented (yet)");
	}

}

