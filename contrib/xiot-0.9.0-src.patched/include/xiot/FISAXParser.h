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
#ifndef FI_SAXPARSER_H
#define FI_SAXPARSER_H

#include <xiot/FITypes.h>
#include <xiot/FIDecoder.h>
#include <string>
#include <fstream>
#include <map>

namespace FI {

class ContentHandler;
class ParserVocabulary;

/**
 * The SAXParser is a specialization of the Decoder that implements callback
 * at those states defined by the SAX mechanism. The given ContentHandler can
 * process those callbacks. The Parser also gives a reference to the current 
 * vocabulary.
 *
 * @see ContentHandler
 */
class OPENFI_EXPORT SAXParser : public Decoder
{
public:
  /// Constructor.
  SAXParser() {};
  /// Destructor.
  virtual ~SAXParser();

  virtual void parse();

  void setContentHandler(ContentHandler* handler);

protected:
  virtual void processDocument();
  virtual void processElement();
  virtual void processAttributes();
  virtual void processCharacterChunk();

  /**
    * Reference to content handler.
    */
  ContentHandler* _contentHandler;

private:
  bool _terminated;
  bool _doubleTerminated;
  Attributes _attributes;
};

}

#endif
