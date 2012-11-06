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
#ifndef FI_DECODER_H
#define FI_DECODER_H

#include <xiot/X3DFIAttributes.h>
#include <xiot/FITypes.h>
#include <xiot/FIParserVocabulary.h>
#include <string>
#include <fstream>

namespace FI {

/**  
 *  Decoder for FI files. 
 *
 *  The class implements the binary parsing of FI document
 *  It currently just implemets a subset of the Fast Infoset standard
 *  and will throw exception in all cases a not implemented part
 *  of the standard is found.
 *
 *  Normally a client will not use the Decoder directly but one of
 *  it`s derived classes, i.e. the until now only derived class FI::SAXParser.
 *  
 *  @see SAXParser
 */
class OPENFI_EXPORT Decoder 
{
public:
  /// Constructor.
  Decoder();
  /// Destructor.
  virtual ~Decoder();
 
  void setStream(std::istream* stream);

  /**
   *  C.1 Fast infoset document.
   *  This method validates the header of a Fast Infoset document
   */
  bool detectFIDocument();

  /**
   * C.2 Encoding of the Document type
   */
  void getDocument(FI::Document &document);


  /**
   * C.3 Encoding of the Element type
   */
  void getElement(FI::Element &element);

  /**
   * C.4 Encoding of the Attribute type
   */
  void getAttribute(FI::Attribute &value);

  /**
   * C.7 Encoding of the CharacterChunk type
   */
  void getCharacterChunk(FI::CharacterChunk &chunk); 

  /**
   * C.13 Encoding of the IdentifyingStringOrIndex type
   */
  void getIdentifyingStringOrIndex(FI::IdentifyingStringOrIndex &value); 

  /**
   * C.14 Encoding of the NonIdentifyingStringOrIndex type starting on the first bit of an octet
   */
  void getNonIdentifyingStringOrIndex1(FI::NonIdentifyingStringOrIndex &value);

  /**
   * C.15 Encoding of the NonIdentifyingStringOrIndex type starting on the third bit of an octet
   */
  void getNonIdentifyingStringOrIndex3(FI::NonIdentifyingStringOrIndex &value); 

  /**
   * C.17 Encoding of the QualifiedNameOrIndex type starting on the second bit of an octet
   */
  void getQualifiedNameOrIndex2(FI::QualifiedNameOrIndex& value);

  /**
   * C.18 Encoding of the QualifiedNameOrIndex type starting on the third bit of an octet
   */
  void getQualifiedNameOrIndex3(FI::QualifiedNameOrIndex& value);

  /**
   * C.19 Encoding of the EncodedCharacterString type starting on the third bit of an octet
   */
  void getEncodedCharacterString3(FI::EncodedCharacterString &value);
  
  /**
   * C.20 Encoding of the EncodedCharacterString type starting on the fifth bit of an octet
   */
  void getEncodedCharacterString5(FI::EncodedCharacterString &value);

  /**
   * C.22 Encoding of the NonEmptyOctetString type starting on the second bit of an octet
   */
  void getNonEmptyOctetString2(FI::NonEmptyOctetString &value); 

  /**
   * C.23 Encoding of the NonEmptyOctetString starting on the fifth bit of an octet
   */
  void getNonEmptyOctetString5(FI::NonEmptyOctetString& value);

  /**
   * C.24 Encoding of the NonEmptyOctetString type starting on the seventh bit of an octet
   */
  void getNonEmptyOctetString7(FI::NonEmptyOctetString& value);
  
  /**
   * C.25 Encoding of integers in the range 1 to 2^20 starting on the second bit of an octet
   * C.26 Encoding of integers in the range 0 to 2^20 starting on the second bit of an octet
   */
  unsigned int getInteger2();
  
  /**
   * C.27 Encoding of integers in the range 1 to 2^20 starting on the third bit of an octet
   */
  unsigned int getInteger3();

  /**
   * C.28 Encoding of integers in the range 1 to 2^20 starting on the fourth bit of an octet
   */
  unsigned int getInteger4();

  /**
   * C.29 Encoding of integers in the range 1 to 256
   * Starting at the 5th bit of the current byte
   */
  unsigned int getSmallInteger5();
  
  /**
   * C.29 Encoding of integers in the range 1 to 256
   * Starting at the 7th bit of the current byte
   */
  unsigned int getSmallInteger7();

  /**
   * This method adds an external vocabulary to the intenal map of
   * external vocabularies. If a external vocabulary is refernced inside
   * the parsed document, the parser will look up for it using the
   * give URI.
   *
   * @uri The URI to identifiy the given ParserVocabulary
   * @parserVocabulary An external ParserVocabulary
   */
  virtual void addExternalVocabularies(const std::string &uri, ParserVocabulary* parserVocabulary); 

protected:
  void processDocumentProperties();

  void decodeAdditionalData();
  void decodeInitialVocabulary();
  void decodeNotations();
  void decodeUnparsedEntities();
  void decodeCharacterEncodingScheme();
  void decodeStandalone();
  void decodeVersion();

  void decodeExternalVocabularyURI();

  bool readChildren(); // C.2
  bool readAttributes(FI::Element& element); // C.4

  int checkBit(unsigned char c, unsigned char iPos);

protected:
  unsigned char _b;
  ParserVocabulary* _vocab;

  std::map<std::string, ParserVocabulary*> _externalVocabularies;
  std::istream* _stream;
};

}

#endif
