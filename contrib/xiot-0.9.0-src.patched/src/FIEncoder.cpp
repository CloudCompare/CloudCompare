#include <xiot/FIEncoder.h>
#include <xiot/FIConstants.h>
#include <xiot/FIEncodingAlgorithms.h>

using namespace std;

namespace FI {

FIEncoder::FIEncoder(void): _stream(NULL)
{
	reset();
}

FIEncoder::~FIEncoder(void)
{
}

void FIEncoder::setStream(std::ostream &stream)
{
	_stream = &stream;
}

void FIEncoder::reset()
{
	_currentByte = 0;
	_currentBytePos = 0;
	_firstLineFeed = true;
	if(_stream && _stream->good())
		_stream->seekp(0, ios_base::beg);
}

void FIEncoder::encodeHeader(bool )
{
  // ITU 12.6: 1110000000000000
  putBits("1110000000000000");
  // ITU 12.7 / 12.9: Version of standard: 1 as 16bit
  putBits("0000000000000001");
  // ITU 12.8: The bit '0' (padding) shall then be appended to the bit stream
  putBit(0);
}

void FIEncoder::encodeInitialVocabulary()
{
  const std::string external_voc = "urn:external-vocabulary";
  
  // ITU C.2.3 
  putBit(0); // additional-data
  putBit(1); // initial-vocabulary
  putBit(0); // notations
  putBit(0); // unparsed-entities 
  putBit(0); // character-encoding-scheme
  putBit(0); // standalone
  putBit(0); // and version
  // ITU C.2.5: padding '000' for optional component initial-vocabulary
  putBits("000");
  // ITU C.2.5.1: For each of the thirteen optional components:
  // presence ? 1 : 0
  putBits("1000000000000"); // 'external-vocabulary'
  // ITU C.2.5.2: external-vocabulary is present
  putBit(0); 
  encodeNonEmptyOctetString2(NonEmptyOctetString(external_voc.begin(), external_voc.end()));
  // Write "urn:external-vocabulary"
  // ITU C.22.3.1: Length is < 65
  //putBit(0); 
  //Writer->PutBits("010110"); // = strlen(external_voc) - 1
  //putBits(static_cast<unsigned int>(external_voc.length() - 1), 6);
  //putBytes(external_voc);
}



void FIEncoder::encodeDocumentTermination()
{
  // ITU C.2.12: The four bits '1111' (termination) are appended
  putBits("1111");
}

void FIEncoder::encodeLineFeed()
{
  fillByte();
  if (_firstLineFeed)
  {
	putBits("1001000000001010");
    _firstLineFeed = false;
  }
  else
  {
    putBits("10100000");
  }
}



/// C.19 Encoding of the EncodedCharacterString type starting 
/// on the third bit of an octet
void FIEncoder::encodeCharacterString3(const std::string &value)
{
     // We want to start at position 3
    assert(_currentBytePos == 2);

    // ITU C.19.3.1 If the alternative utf-8 is present, then the two bits '00' 
    // are appended to the bit stream.
    putBits("00");
    // ITU C.19.4: The component bytes is encoded as described in C.23.
	encodeNonEmptyByteString5(NonEmptyOctetString(value.begin(), value.end()));
}

// ITU C.22 Encoding of the NonEmptyOctetString type starting 
// on the second bit of an octet
void FIEncoder::encodeNonEmptyOctetString2(const NonEmptyOctetString &value)
{
  // We want to start at position 2
  assert(_currentBytePos == 1);
  // Non-empty bytes
  assert(!value.empty());

  size_t length = value.size();
  if (length <= 64)
  {
    putBit(0);
    putBits(static_cast<int>(length) - 1, 6);
  }
  else if (length <= 320)
  {
    putBits("10");
    putBits(static_cast<int>(length) - 65, 8);
  }
  else
  {
    putBits("1100");
    putBits(static_cast<int>(length) - 321, 32);
  }
  writeOctet(value);
}

// ITU C.23: Encoding of the NonEmptyByteString starting
// on the fifth bit of an byte
void FIEncoder::encodeNonEmptyByteString5(const NonEmptyOctetString &value)
{
	// We want to start at position 5
	assert(_currentBytePos == 4);
	// Non-empty bytes
	assert(!value.empty());

	size_t length = value.size();
    if (length <= 8)
    {
      putBit(0);
      putBits(static_cast<int>(length) - 1, 3);
    }
    else if (length <= 264)
    {
      putBits("1000");
      putBits(static_cast<int>(length) - 9, 8);
    }
    else
    {
      putBits("1100");
      putBits(static_cast<int>(length) - 265, 32);
    }
	writeOctet(value);
}

// ITU C.25: Encoding of integers in the range 1 to 2^20
// starting on the second bit of an byte
void FIEncoder::encodeInteger2(int value) 
{
 // We want to start at position 2
    assert(_currentBytePos == 1);

    if (value <= 64) // ITU  C.25.2
    {
      putBits("0");
      putBits(value - 1, 6);
    }
    else if (value <= 8256) // ITU C.25.3
    {
      putBits("10");
      putBits(value - 65, 13);
    }
    else // ITU C.25.4
    {
      putBits("110");
      putBits(value - 8257, 20);
    }
}

// ITU C.27: Encoding of integers in the range 1 to 2^20
// starting on the third bit of an byte
void FIEncoder::encodeInteger3(int value) 
{
  // We want to start at position 3
    assert(_currentBytePos == 2);

    if (value <= 32) // ITU  C.27.2
    {
      putBit(0);
      putBits(value - 1, 5);
	}
    else if (value <= 2080) // ITU C.27.3
    {
      putBits("100");
      putBits(value - 33, 11);
    }
    else if (value < 526368) // ITU C.27.4
    {
      putBits("101");
      putBits(value - 2081, 19);
    }
    else // ITU C.27.5
    {
      putBits("1100000000");
      putBits(value - 526369, 20);
    }
}

void FIEncoder::encodeAttributeFloatArray(const float* values, size_t size)
{
  // We want to start at position 3
  assert(_currentBytePos == 2);

  // ITU 10.8.1: This encoding algorithm has a vocabulary table index of 7
  encodeEncodingAlgorithmStart(7);
  
  NonEmptyOctetString octets;
  FloatEncodingAlgorithm::encode(values, size, octets);
  encodeNonEmptyByteString5(octets);
}

void FIEncoder::encodeAttributeIntegerArray(const int* values, size_t size)
{
  // We want to start at position 3
  assert(_currentBytePos == 2);
  
  // ITU 10.8.1: This encoding algorithm has a vocabulary table index of 4
  encodeEncodingAlgorithmStart(4);
  
  NonEmptyOctetString octets;
  IntEncodingAlgorithm::encode(values, size, octets);
  encodeNonEmptyByteString5(octets);
}


} // namespace FI

