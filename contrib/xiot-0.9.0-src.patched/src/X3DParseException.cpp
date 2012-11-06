#include <xiot/X3DParseException.h>


namespace XIOT {

X3DParseException::X3DParseException(const std::string &message, int lineNumber, int columnNumber) : 
_message(message), _lineNumber(lineNumber), _columnNumber(columnNumber)
{
}

X3DParseException::X3DParseException(const std::string &message) : 
_message(message), _lineNumber(0), _columnNumber(0)
{
}


}

