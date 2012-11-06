/*=========================================================================
     This file is part of the XIOT library.

     Copyright (C) 2008-2009 EDF R&D
     Author: Kristian Sons (xiot@actor3d.com)

     This library is free software; you can redistribute it and/or modify
     it under the terms of the GNU Lesser Public License as published by
     the Free Software Foundation; either version 2.1 of the License, or
     (at your option) any later version.

     The XIOT library is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU Lesser Public License for more details.

     You should have received a copy of the GNU Lesser Public License
     along with XIOT; if not, write to the Free Software
     Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
     MA 02110-1301  USA
=========================================================================*/
#ifndef FI_FIENCODER_H
#define FI_FIENCODER_H

#include <xiot/FITypes.h>
#include <ostream>
#include <cassert>

namespace FI {

/**  
 *  Decoder for FI files. 
 */
class OPENFI_EXPORT FIEncoder
{
public:
	FIEncoder(void);
	virtual ~FIEncoder(void);

    // Resets stream and positions
    void reset();
	
	// Set the stream for output
    void setStream(std::ostream &stream);

	void encodeHeader(bool encodeXmlDecl);
	void encodeInitialVocabulary();

	void encodeDocumentTermination();

	void encodeLineFeed();

	// ITU C.14 Encoding of the NonIdentifyingStringOrIndex 
	// type starting on the first bit of an octet
	//void encodeNonIdentifyingStringOnFirstBit();

	// ITU C.19 Encoding of the EncodedCharacterString type starting 
	// on the third bit of an octet
	void encodeCharacterString3(const std::string &value);

	// ITU C.22 Encoding of the NonEmptyOctetString type starting 
	// on the second bit of an octet
	void encodeNonEmptyOctetString2(const NonEmptyOctetString &value);

	// ITU C.23: Encoding of the NonEmptyByteString starting
	// on the fifth bit of an byte
	void encodeNonEmptyByteString5(const NonEmptyOctetString &value);

	// ITU C.25: Encoding of integers in the range 1 to 2^20
	// starting on the second bit of an byte
	void encodeInteger2(int value);

	// ITU C.27: Encoding of integers in the range 1 to 2^20
    // starting on the third bit of an byte
	void encodeInteger3(int value);

	virtual void encodeAttributeIntegerArray(const int* values, size_t size);
	virtual void encodeAttributeFloatArray(const float* values, size_t size);

  // Puts a bitstring to the current byte bit by bit
  void putBits(const std::string &bitstring) {
	for(std::string::const_iterator I = bitstring.begin(); I != bitstring.end(); I++)
      putBit((*I) == '1');
  }

  // Puts the integer value to the stream using count bits
  // for encoding
  void putBits(unsigned int value, unsigned char count)
  {
	// Can be optimized
    while (count > 0)
    {
      count = this->append(value, count);
    }
  }


  // Puts on bit to the current byte true = 1, false = 0
  inline void putBit(bool on) {
	  assert(_currentBytePos < 8);
	  if (on)
		{
		unsigned char pos = _currentBytePos;
		unsigned char mask = (unsigned char)(0x80 >> pos);
		_currentByte |= mask;
		}
		_currentBytePos++;
		tryFlush();
  }

  // Puts whole bytes to the file stream. CurrentBytePos must
  // be 0 for this
  void putBytes(const unsigned char* bytes, size_t length) {
	if(_currentBytePos == 0)
		_stream->write(reinterpret_cast<const char*>(bytes), length);
	else {
	    assert(false);
		throw std::runtime_error("Wrong position in FiEncode::PutBytes");
    }
  }
  
  // Fills up the current byte with 0 values
  inline void fillByte() {
	while (_currentBytePos)
	    putBit(0);
  };
protected:

  inline void encodeEncodingAlgorithmStart(int algorithmID)
  {
    // ITU C.19.3.4: If the alternative encoding-algorithm is present, 
    // then the two bits '11' (discriminant) are appended
    putBits("11");
    //C.29.2 The value, minus the lower bound of the range, is encoded as an unsigned integer in a field of eight bits and
    //appended to the bit stream.
    putBits(algorithmID-1, 8);
  }

  inline void writeOctet(const NonEmptyOctetString& value)
  {
	  putBytes(value.c_str(), value.length());
  }

  inline unsigned char append(unsigned int value, unsigned char count)
  {
    assert(_currentBytePos < 8);
	while ((_currentBytePos < 8) && count > 0)
    {
		// Value and der Stelle i
		unsigned int mask = 1;
		bool isSet = !(((mask << (count - 1)) & value) == 0);
		if (isSet)
		{
			_currentByte |= static_cast<unsigned char>(0x80 >> _currentBytePos);
      }
    _currentBytePos++;
    count--;
    }
  tryFlush();
  return count;
}


   // This is the current byte to fill
  unsigned char _currentByte;
  // This is the current byte position. Range: 0-7
  unsigned char _currentBytePos;
  bool _firstLineFeed;
private:

  inline void tryFlush() {
    assert(_stream);
	if (_currentBytePos == 8) {
		_stream->put(_currentByte);
		_currentByte = 0;
		_currentBytePos = 0;
    }
  }
  std::ostream* _stream;


};

} // namespace FI

#endif // FI_FIENCODER_H

