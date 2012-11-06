#include <xiot/X3DFICompressionTools.h>

#include <cassert>
#include <string>
#include <cstring>
#include <stdexcept>

#define EXPONENT_MASK_32 0x7f800000
#define MANTISSA_MASK_32 0x007fffff
#define EXPONENT_BIAS_32 127
#define MANTISSA_BITS_32 23
#define SIGN_SHIFT_32    31
#define PACKER_BUFFER_SIZE 1024
namespace XIOT {
	namespace FITools {

BitPacker::BitPacker() {
	buffer.resize(PACKER_BUFFER_SIZE, 0);
    next_bit_to_write = 0;
}


// Fast version of pack()
const int BITS_PER_WORD = 32;
void BitPacker::pack(unsigned long value, unsigned long num_bits_to_pack) {
	assert(num_bits_to_pack <= 32);
	assert((value & ((1 << num_bits_to_pack) - 1)) == value);

    // Scoot the value bits up to the top of the word; this makes
    // them easier to work with.

    value <<= (BITS_PER_WORD - num_bits_to_pack);

    // First we do the hard part: pack bits into the first unsigned char,
    // which may already have bits in it.

    int byte_index = (next_bit_to_write / 8);
    int bit_index = (next_bit_to_write % 8);
    int empty_space_this_byte = (8 - bit_index) & 0x7;

    // Update next_bit_to_write for the next call; we don't need 
    // the old value any more.

    next_bit_to_write += num_bits_to_pack;

	unsigned char *dest = &buffer.front() + byte_index;

    if (empty_space_this_byte) {
        unsigned int to_copy = empty_space_this_byte;
	    int align = 0;

	    if (to_copy > num_bits_to_pack) {
	        // We don't have enough bits to fill up this unsigned char.
	        align = to_copy - num_bits_to_pack;
	        to_copy = num_bits_to_pack;
	    }

	    unsigned long fill_bits = value >> (BITS_PER_WORD - empty_space_this_byte);
	    *dest |= fill_bits;

	    num_bits_to_pack -= to_copy;
	    dest++;
	    value <<= to_copy;
    }

    // Now we do the fast and easy part for what is hopefully
    // the bulk of the data.

    while (value) {
        *dest++ = value >> (BITS_PER_WORD - 8);
	    value <<= 8;
    }
}

void BitPacker::get_result(char **data_return, int *len_return) {
    int len_in_bytes = (next_bit_to_write + 7) / 8;
    *len_return = len_in_bytes;

    assert(*len_return <= PACKER_BUFFER_SIZE);

	*data_return = (char *)&buffer.front();
}


BitUnpacker::BitUnpacker(unsigned char *data, size_t len) {
	if (len > buffer.size())
		buffer.resize(len);

    memcpy(&buffer.front(), data, len);
    num_bits_remaining = static_cast<unsigned int>(len * 8);
    next_bit_to_read = 0;
}

unsigned long BitUnpacker::unpack(unsigned long num_bits_to_unpack) {
	unsigned long result = 0;

    assert(num_bits_to_unpack <= num_bits_remaining);

	while (num_bits_to_unpack) {
		unsigned long byte_index = (next_bit_to_read / 8);
		unsigned long bit_index = (next_bit_to_read % 8);
		
		unsigned long src_mask = (1 << (7 - bit_index));
		unsigned long dest_mask = (1 << (num_bits_to_unpack - 1));

		if (buffer[byte_index] & src_mask) result |= dest_mask;
		num_bits_to_unpack--;
		next_bit_to_read++;
	}

    num_bits_remaining -= num_bits_to_unpack;

    return result;
}

FloatPacker::FloatPacker(unsigned long exponentBits, unsigned long mantissaBits) :
_exponentBits(exponentBits),
_mantissaBits(mantissaBits)
{
	if (_exponentBits > 8) {
		throw std::range_error("Exponent bits out of range, max 8" );
	}
	if (_mantissaBits > 23) {
		throw std::range_error("Too many mantissa bits, max: 23" );
	}

	_exponent_bias = (1 << (_exponentBits - 1)) - 1;
	 _sign_shift = _exponentBits + _mantissaBits;
	_sign_mask = 1 << _sign_shift;
	 _exponent_mask = ((1 << _exponentBits) - 1) << _mantissaBits;
	 _mantissa_mask = (1 << _mantissaBits) - 1;
	_exponent_max = (1 << (_exponentBits - 1)) - 1;
	_exponent_min = -_exponent_max - 1;
}

float FloatPacker::decode(unsigned long src, bool )
{
	if (src == 0)
		return 0.0f;

	int mantissa  = (src & _mantissa_mask);
	int exponent  = (int) (src & _exponent_mask) >> _mantissaBits;
	long sign     = (src >> _sign_shift);
	
	exponent += _exponent_min;
	exponent += EXPONENT_BIAS_32;
	
	mantissa = mantissa << (MANTISSA_BITS_32 - _mantissaBits);
	unsigned int result = (sign << SIGN_SHIFT_32) | (exponent << MANTISSA_BITS_32) | (mantissa);

	union float_to_unsigned_int_to_bytes
		{
			float f;
			unsigned int ui;
			unsigned char ub[4]; // unsigned bytes
		};
	
	float_to_unsigned_int_to_bytes v;
	v.ui = result;
	return v.f;
}

} // end namespace FI
} // end namespace X3D
