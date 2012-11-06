#ifndef PDMS_PARSER_HEADER
#define PDMS_PARSER_HEADER

#include "PdmsTools.h"

using namespace PdmsTools;

#include <string>
#include <map>

//! PDMS Lexer
/** PDMS sessions are made to provide an input stream to the parser (file, keyboard, pipe, ...)
**/
class PdmsLexer
{
public:
	
	PdmsLexer();
	virtual ~PdmsLexer() {}

	virtual bool initializeSession();
	bool gotoNextToken();
	void finish() {stop = true;}
	virtual void closeSession(bool destroyLoadedObject=false);

	PointCoordinateType valueFromBuffer();
	const char* nameFromBuffer() const;

	virtual void printWarning(const char* str) = 0;

	Token getCurrentToken() const {return currentToken;}
	PdmsObjects::GenericItem* getLoadedObject() const {return loadedObject;}
	const char* getBufferContent() const {return tokenBuffer;}
	void setLoadedObject(PdmsObjects::GenericItem *o) {loadedObject = o;}
	
protected:

	static const int c_max_buff_size = 2048;

	PdmsObjects::GenericItem *loadedObject;
	Token currentToken;
	char tokenBuffer[c_max_buff_size], nextBuffer[c_max_buff_size];
	std::map<std::string, Token> dictionnary;
	bool stop;
	char metaGroupMask;

	void pushIntoDictionnary(const char *str, Token token, int minSize=0);
	virtual void parseCurrentToken();
	virtual void skipComment() = 0;
	virtual void skipHandleCommand() = 0;
	virtual bool moveForward();
};

//Pdms file session
class PdmsFileSession : public PdmsLexer
{
protected:
	static const int s_max_file_name_length = 1024;
	char filename[s_max_file_name_length];
	unsigned currentLine;
	bool eol, eof;
	FILE *file;

public:
	PdmsFileSession(const char* fileName);
	virtual ~PdmsFileSession() {closeSession();}
	virtual bool initializeSession();
	virtual void closeSession(bool destroyLoadedObject=false);
	virtual void printWarning(const char* str);
protected:
	virtual void parseCurrentToken();
	virtual bool moveForward();
	virtual void skipComment();
	virtual void skipHandleCommand();
};

//PDMS Parser
/** Use this parser the following way:
	1- create any Pdms session
	2- link parser with the session
	3- parse the current session content. if the session content changes, you can parse it as many
	times as you wish (the loaded object is overwritten each time)
	4- get the result (either in session or in parser)
**/
class PdmsParser
{
public:
	//! Default constructor
	PdmsParser();
	~PdmsParser();

	void reset();
	void linkWithSession(PdmsLexer *s);
	bool parseSessionContent();
	PdmsObjects::GenericItem *getLoadedObject(bool forgetIt = true);

protected:

	bool processCurrentToken();

	PdmsLexer *session;
	PdmsCommands::Command *currentCommand;
	PdmsObjects::GenericItem *currentItem, *root;
};

#endif
