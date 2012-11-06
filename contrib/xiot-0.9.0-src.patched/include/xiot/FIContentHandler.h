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
#ifndef FI_PROCESSOR_H
#define FI_PROCESSOR_H

#include <istream>
#include <xiot/FITypes.h>

namespace FI {

class ParserVocabulary;

/**
 * This is the interface for the content handler used by the FISAXParser.
 *
 * It's similar to other common SAX content handlers but additionally includes a 
 * pointer to the current ParserVocabulary that can help to resolve the Element and
 * attribute content.
 * @see FISAXParser
 */
class OPENFI_EXPORT ContentHandler
{
public:
  virtual void startDocument() = 0;
  
  virtual void endDocument() = 0;

  virtual void startElement(const ParserVocabulary* vocab, const Element &element, const Attributes &attributes) = 0;
  virtual void endElement(const ParserVocabulary* vocab, const Element &element) = 0;
  
  virtual void characters(const ParserVocabulary* vocab, const CharacterChunk &chunk) = 0;
};

/**
 * The DefaultContent handler is a default implementation of the ContentHandler
 *
 * It implements all the callbacks defined in the ContentHandler interface. It does nothing for these
 * callbacks, so clients can implement just the needed methods.
 */
class OPENFI_EXPORT DefaultContentHandler : public ContentHandler
{
public:
  /// Destructor.
	virtual ~DefaultContentHandler() {};

  virtual void startDocument();
  virtual void endDocument();

  virtual void startElement(const ParserVocabulary* vocab, const Element &element, const Attributes &attributes);
  virtual void endElement(const ParserVocabulary* vocab, const Element &element);

  virtual void characters(const ParserVocabulary* vocab, const CharacterChunk &chunk);

};

}

#endif
