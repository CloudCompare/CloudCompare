#include <xiot/FIContentHandler.h>
#include <xiot/FIParserVocabulary.h>
#include <iostream>
#include <sstream>

using namespace std;

namespace FI {

void DefaultContentHandler::startDocument()
{
}

void DefaultContentHandler::endDocument()
{
}

void DefaultContentHandler::startElement(const ParserVocabulary* , const Element &, const Attributes &)
{
}

void DefaultContentHandler::endElement(const ParserVocabulary* , const Element &)
{
}

void DefaultContentHandler::characters(const ParserVocabulary* , const CharacterChunk &)
{
}


}
