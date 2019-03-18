//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef PDMS_PARSER_HEADER
#define PDMS_PARSER_HEADER

#include "PdmsTools.h"

//system
#include <map>
#include <string>

using namespace PdmsTools;

//! PDMS Lexer
/** PDMS sessions are made to provide an input stream to the parser (file, keyboard, pipe, ...)
**/
class PdmsLexer
{
public:
	PdmsLexer();
	virtual ~PdmsLexer() = default;

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
	/** \warning Never pass a 'constant initializer' by reference
	**/
	static const int c_max_buff_size = 2048;

	PdmsObjects::GenericItem *loadedObject;
	Token currentToken;
	char tokenBuffer[c_max_buff_size];
	char nextBuffer[c_max_buff_size];
	std::map<std::string, Token> dictionary;
	bool stop;
	char metaGroupMask;

	void pushIntoDictionary(const char *str, Token token, int minSize=0);
	virtual void parseCurrentToken();
	virtual void skipComment() = 0;
	virtual void skipHandleCommand() = 0;
	virtual bool moveForward();
};

//Pdms file session
class PdmsFileSession : public PdmsLexer
{
protected:
	std::string m_filename;
	int m_currentLine;
	bool m_eol;
	bool m_eof;
	FILE* m_file;

public:
	PdmsFileSession(const std::string &filename);
	~PdmsFileSession() override { closeSession(); }
	bool initializeSession() override;
	void closeSession(bool destroyLoadedObject=false) override;
	void printWarning(const char* str) override;

protected:
	void parseCurrentToken() override;
	bool moveForward() override;
	void skipComment() override;
	void skipHandleCommand() override;
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
	PdmsObjects::GenericItem *currentItem;
	PdmsObjects::GenericItem *root;
};

#endif //PDMS_PARSER_HEADER
