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

#include "PdmsParser.h"

//system
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <assert.h>

//////////// STRING HANDLING ////////////////////
inline void upperStr(char *s) {while(*s) {if(((*s)>='a')&&((*s)<='z')) (*s)+='A'-'a'; s++;}}

////////////////////////////
// PDMS LEXER
////////////////////////////

PdmsLexer::PdmsLexer()
	: loadedObject(0)
	, currentToken(PDMS_INVALID_TOKEN)
	, stop(false)
	, metaGroupMask(0)
{
	tokenBuffer[0] = 0;
	nextBuffer[0] = 0;
}

bool PdmsLexer::initializeSession()
{
	loadedObject = NULL;
	currentToken = PDMS_INVALID_TOKEN;
	stop = false;
	memset(tokenBuffer, 0, c_max_buff_size);
	memset(nextBuffer, 0, c_max_buff_size);
	metaGroupMask = 0;

	dictionnary.clear();
	pushIntoDictionnary("NEW",          PDMS_CREATE,            3);
	pushIntoDictionnary("AND",          PDMS_AND,               3);
	pushIntoDictionnary("IS",           PDMS_IS,                2);
	pushIntoDictionnary("WRT",          PDMS_WRT,               3);
	pushIntoDictionnary("LAST",         PDMS_LAST,              4);
	pushIntoDictionnary("GROUP",        PDMS_GROUP,             2);
	pushIntoDictionnary("WORLD",        PDMS_WORLD,             4);
	pushIntoDictionnary("SITE",         PDMS_SITE,              3);
	pushIntoDictionnary("ZONE",         PDMS_ZONE,              3);
	pushIntoDictionnary("EQUIPMENT",    PDMS_EQUIPMENT,         3);
	pushIntoDictionnary("STRUCTURE",    PDMS_STRUCTURE,         3);
	pushIntoDictionnary("SUBSTRUCTURE", PDMS_SUBSTRUCTURE,      4);
	pushIntoDictionnary("END",          PDMS_END,               3);
	pushIntoDictionnary("NAME",         PDMS_NAME,              4);
	pushIntoDictionnary("SLCYLINDER",   PDMS_SCYLINDER,         3);
	pushIntoDictionnary("CYLINDER",   	PDMS_SCYLINDER,         3);
	pushIntoDictionnary("CTORUS",      	PDMS_CTORUS,            4);
	pushIntoDictionnary("RTORUS",      	PDMS_RTORUS,            4);
	pushIntoDictionnary("DISH",        	PDMS_DISH,              3);
	pushIntoDictionnary("CONE",         PDMS_CONE,              3);
	pushIntoDictionnary("BOX",          PDMS_BOX,               3);
	pushIntoDictionnary("NBOX",         PDMS_NBOX,              4);
	pushIntoDictionnary("PYRAMID",      PDMS_PYRAMID,           4);
	pushIntoDictionnary("SNOUT",        PDMS_SNOUT,             4);
	pushIntoDictionnary("EXTRUSION",    PDMS_EXTRU,             5);
	pushIntoDictionnary("NXTRUSION",    PDMS_NEXTRU,            5);
	pushIntoDictionnary("LOOP",         PDMS_LOOP,              4);
	pushIntoDictionnary("VERTEX",       PDMS_VERTEX,            4);
	pushIntoDictionnary("EST",          PDMS_EST,               1);
	pushIntoDictionnary("NORTH",        PDMS_NORTH,             1);
	pushIntoDictionnary("UP",           PDMS_UP,                1);
	pushIntoDictionnary("WEST",         PDMS_WEST,              1);
	pushIntoDictionnary("SOUTH",        PDMS_SOUTH,             1);
	pushIntoDictionnary("DOWN",         PDMS_DOWN,              1);
	pushIntoDictionnary("X",            PDMS_X,                 1);
	pushIntoDictionnary("Y",            PDMS_Y,                 1);
	pushIntoDictionnary("Z",            PDMS_Z,                 1);
	pushIntoDictionnary("DIAMETER",     PDMS_DIAMETER,          3);
	pushIntoDictionnary("RADIUS",       PDMS_RADIUS,            3);
	pushIntoDictionnary("HEIGHT",       PDMS_HEIGHT,            3);
	pushIntoDictionnary("XTSHEAR",      PDMS_X_TOP_SHEAR,       4);
	pushIntoDictionnary("XBSHEAR",      PDMS_X_BOTTOM_SHEAR,    4);
	pushIntoDictionnary("YTSHEAR",      PDMS_Y_TOP_SHEAR,       4);
	pushIntoDictionnary("YBSHEAR",      PDMS_Y_BOTTOM_SHEAR,    4);
	pushIntoDictionnary("XBOTTOM",      PDMS_X_BOTTOM,          4);
	pushIntoDictionnary("YBOTTOM",      PDMS_Y_BOTTOM,          4);
	pushIntoDictionnary("XTOP",         PDMS_X_TOP,             4);
	pushIntoDictionnary("YTOP",         PDMS_Y_TOP,             4);
	pushIntoDictionnary("XOFF",         PDMS_X_OFF,             4);
	pushIntoDictionnary("YOFF",         PDMS_Y_OFF,             4);
	pushIntoDictionnary("RINSIDE",      PDMS_INSIDE_RADIUS,     4);
	pushIntoDictionnary("ROUTSIDE",     PDMS_OUTSIDE_RADIUS,    4);
	pushIntoDictionnary("XLENGTH",      PDMS_XLENGTH,           4);
	pushIntoDictionnary("YLENGTH",      PDMS_YLENGTH,           4);
	pushIntoDictionnary("ZLENGTH",      PDMS_ZLENGTH,           4);
	pushIntoDictionnary("ANGLE",        PDMS_ANGLE,             4);
	pushIntoDictionnary("DTOP",         PDMS_TOP_DIAMETER,      4);
	pushIntoDictionnary("DBOTTOM",      PDMS_BOTTOM_DIAMETER,   5);
	pushIntoDictionnary("AT",           PDMS_POSITION,          2);
	pushIntoDictionnary("POSITION",     PDMS_POSITION,          3);
	pushIntoDictionnary("ORIENTED",     PDMS_ORIENTATION,       3);
	pushIntoDictionnary("METRE",        PDMS_METRE,             1);
	pushIntoDictionnary("MILLIMETRE",   PDMS_MILLIMETRE,        3);
	pushIntoDictionnary("MM",           PDMS_MILLIMETRE,        2);
	pushIntoDictionnary("OWNER",        PDMS_OWNER,             3);
	pushIntoDictionnary("RETURN",       PDMS_RETURN,            6);

	return true;
}

void PdmsLexer::closeSession(bool destroyLoadedObject)
{
	dictionnary.clear();
	if (destroyLoadedObject && loadedObject)
	{
		PdmsObjects::Stack::Detroy(loadedObject);
	}
}

bool PdmsLexer::gotoNextToken()
{
	const int enter_meta_group_mask = 1;
	const int leave_meta_group_mask = 100;

	//Special case: in meta group, the lexer splits Meta Group comments into appropriated tokens
	if(metaGroupMask)
	{
		metaGroupMask++;
		switch(metaGroupMask)
		{
		case enter_meta_group_mask+1: currentToken = PDMS_CREATE; return true;
		case enter_meta_group_mask+2: currentToken = PDMS_GROUP; return true;
		case enter_meta_group_mask+3: currentToken = PDMS_NAME_STR; return true;
		case leave_meta_group_mask+1: currentToken = PDMS_END; return true;
		case leave_meta_group_mask+2: currentToken = PDMS_GROUP; return true;
		default: metaGroupMask=0; break;
		}
	}

	//Usual cases
	currentToken = PDMS_INVALID_TOKEN;
	if(stop) return false;
	while(currentToken==PDMS_INVALID_TOKEN)
	{
		if(!moveForward())
			currentToken = PDMS_EOS;
		else{
			parseCurrentToken();
			switch(currentToken)
			{
			case PDMS_COMMENT_LINE:
			case PDMS_COMMENT_BLOCK:
				skipComment();
				if(currentToken == PDMS_ENTER_METAGROUP)
				{
					metaGroupMask = enter_meta_group_mask;
					break;
				}
				if(currentToken == PDMS_LEAVE_METAGROUP)
				{
					metaGroupMask = leave_meta_group_mask;
					break;
				}
			case PDMS_UNUSED:
				currentToken = PDMS_INVALID_TOKEN;
				break;
			default:
				break;
			}
		}
	}

	if(metaGroupMask)
		return gotoNextToken();

	return (currentToken != PDMS_EOS);
}

void PdmsLexer::parseCurrentToken()
{
	currentToken = PDMS_UNKNOWN;
	if(tokenBuffer[0] == '/')
		currentToken = PDMS_NAME_STR;
	else if(strncmp(tokenBuffer, "$*", 2)==0)
		currentToken = PDMS_COMMENT_LINE;
	else if(strncmp(tokenBuffer, "$(", 2)==0)
		currentToken = PDMS_COMMENT_BLOCK;
	else if(tokenBuffer[0]=='-' || ('0'<=tokenBuffer[0] && tokenBuffer[0]<='9'))
		currentToken = PDMS_NUM_VALUE;
	else if(strcmp(tokenBuffer, "ENDHANDLE")==0)
		currentToken = PDMS_UNUSED;
	else if(strncmp(tokenBuffer, "HANDLE", 6)==0)
	{
		skipHandleCommand();
		currentToken = PDMS_UNUSED;
	}
	else
	{
		std::map<std::string, Token>::const_iterator location;
		location = dictionnary.find(std::string(tokenBuffer));
		if(location != dictionnary.end())
			currentToken = location->second;
	}
}

PointCoordinateType PdmsLexer::valueFromBuffer()
{
	size_t index = strlen(tokenBuffer);
	size_t length = 0;
	while (index > 0) //go back until we meet a number symbol
	{
		if ((tokenBuffer[index-1]>='0' && tokenBuffer[index-1]<='9') || tokenBuffer[index-1]=='.')
			break;
		index--;
		length++;
	}

	//Read units
	if (length > 0)
	{
		strcpy(nextBuffer, &(tokenBuffer[index]));
		memset(&(tokenBuffer[index]), 0, length);
	}

	//Replace comma
	length = strlen(tokenBuffer);
	for (index=0; index<length; index++)
		if (tokenBuffer[index]==',')
			tokenBuffer[index]='.';

	//convert value
	PointCoordinateType value = (PointCoordinateType)atof(tokenBuffer);

	return value;
}

const char* PdmsLexer::nameFromBuffer() const
{
	return &tokenBuffer[1];
}

bool PdmsLexer::moveForward()
{
	if(strlen(nextBuffer))
	{
		strcpy(tokenBuffer, nextBuffer);
		memset(nextBuffer, 0, c_max_str_length);
		return true;
	}
	return false;
}

void PdmsLexer::pushIntoDictionnary(const char *str, Token token, int minSize)
{
	int n = (int)strlen(str);
	if (minSize == 0 || minSize > n)
		minSize= n ;
	for (; minSize<=n; minSize++)
		dictionnary[std::string(str).substr(0, minSize)] = token;
}

PdmsFileSession::PdmsFileSession(const std::string &filename)
    : m_filename(filename)
    , m_currentLine(-1)
    , m_eol(false)
    , m_eof(false)
    , m_file(0)
{}

bool PdmsFileSession::initializeSession()
{
	PdmsLexer::initializeSession();
	m_file = fopen(m_filename.c_str(), "r");
	if (!m_file)
		return false;
	
	m_currentLine = 1;
	m_eol = false;
	m_eof = false;
	
	return true;
}

void PdmsFileSession::closeSession(bool destroyLoadedObject)
{
	if (m_file)
	{
		fclose(m_file);
		m_file = NULL;
	}

	PdmsLexer::closeSession(destroyLoadedObject);
}

void PdmsFileSession::parseCurrentToken()
{
	if(m_eof && strlen(tokenBuffer) == 0)
		currentToken = PDMS_EOS;
	else
		PdmsLexer::parseCurrentToken();
}

bool PdmsFileSession::moveForward()
{
	if (PdmsLexer::moveForward())
		return true;

	m_eol = false;
	bool tokenFilled = false;
	unsigned n = 0;
	while (!tokenFilled)
	{
		int car = getc(m_file);
		switch(car)
		{
		case '\n':
			if(n > 0)
			{
				tokenFilled = true;
				m_eol = true;
			}
			m_currentLine++;
			break;
		case ' ':
		case '\t':
			if(n > 0)
				tokenFilled = true;
			break;
		case EOF:
			tokenFilled = true;
			m_eof = true;
			break;
		default:
			if(n >= c_max_buff_size)
			{
				printWarning("Buffer overflow");
				return false;
			}
			tokenBuffer[n] = car;
			n++;
			break;
		}
	}
	tokenBuffer[n] = '\0';
	if(tokenBuffer[0] != '/')
		upperStr(tokenBuffer);
	return (n > 0);
}

void PdmsFileSession::skipComment()
{
	switch(currentToken)
	{
	case PDMS_COMMENT_LINE:
		//skip line only if the end of line has not been read in current buffer
		if(!m_eol)
		{
			int n = 0;
			int car = 0;
			do{
				car = getc(m_file);
				if(car=='\t') car=' ';
				tokenBuffer[n] = car;
				if(((n+1)<c_max_buff_size) && ((car!=' ') || (n>0 && tokenBuffer[n-1]!=' '))) n++;
			} while(car!=EOF && (char)car!='\n');
			if(car == '\n')
				m_currentLine++;
			tokenBuffer[n-1] = '\0';
		}
		m_eol = false;
		break;
	case PDMS_COMMENT_BLOCK:
		{
		//comment block opening symbol has been met. Search for comment block ending symbol
		//don't forget that some other comments could be embeded in this comment
		bool commentSymb = false;
		int commentBlockLevel = 1;
		int n = 0;
		int car = 0;
		do{
			car = getc(m_file);
			if(car=='\n') m_currentLine++;
			if(car=='\n' || car=='\t') car = ' ';
			if(car=='$') commentSymb = true;
			else if(car=='(' && commentSymb) commentBlockLevel++;
			else if(car==')' && commentSymb) commentBlockLevel--;
			else
			{
				commentSymb = false;
				tokenBuffer[n] = car;
				if(((n+1)<c_max_buff_size) && ((car!=' ') || (n>0 && tokenBuffer[n-1]!=' '))) n++;
			}
		} while(car!=EOF && commentBlockLevel>0);
		tokenBuffer[n-1] = '\0';
		m_eol = false;
		}
		break;
	default:
		break;
	}

	upperStr(tokenBuffer);
	if(strncmp(tokenBuffer, "ENTERING IN GROUP:", 18)==0)
	{
		currentToken = PDMS_ENTER_METAGROUP;
		//The meta group name starts after the "entering in group:" statement, after the last slash
		//But we still store the whole path
		char* ptr2 = &(tokenBuffer[18]);
		while((*ptr2)==' ') {ptr2++;}
		//Copy the meta group name at the begining of tokenbuffer
		tokenBuffer[0] = '/';
		char* ptr1 = &(tokenBuffer[1]);
		while((*ptr2) && (*ptr2)!=' ')
		{
			*ptr1 = *ptr2;
			ptr1++;
			ptr2++;
		}
		*ptr1 = '\0';
		metaGroupMask = 0;
	}
	else if(strncmp(tokenBuffer, "LEAVING GROUP", 13)==0)
	{
		currentToken = PDMS_LEAVE_METAGROUP;
		metaGroupMask = 0;
	}
}

void PdmsFileSession::skipHandleCommand()
{
	int opened=0, state=0;

	//Search for "HANDLE(...)" and remove this string from tokenBuffer
	for(unsigned i=0; i<strlen(tokenBuffer); i++)
	{
		if(tokenBuffer[i]=='(') {opened++; state++;}
		else if(tokenBuffer[i]==')') state--;
		if(opened>0 && state==0)
			return;
	}
	//"HANDLE(...) does not lie in tokenBuffer, then search it in the file
	while(!(opened>0 && state==0))
	{
		int car = getc(m_file);
		if(car=='(') {opened++; state++;}
		else if(car==')') state--;
	}
	memset(tokenBuffer, 0, c_max_str_length);
}

void PdmsFileSession::printWarning(const char* str)
{
	if(currentToken == PDMS_EOS)
		std::cerr << "[" << m_filename << "]@postprocessing : " << str << std::endl;
	else
		std::cerr << "[" << m_filename << "]@[line " << m_currentLine << "]::[" << tokenBuffer << "] : " << str << std::endl;
}

///////////////////////////
// PDMS PARSER
///////////////////////////
PdmsParser::PdmsParser()
	: session(NULL)
	, currentCommand(NULL)
	, currentItem(NULL)
	, root(NULL)
{
}

PdmsParser::~PdmsParser()
{
	if (currentCommand)
	{
		delete currentCommand;
		currentCommand = 0;
	}
	
	if (currentItem)
	{
		currentItem = currentItem->getRoot();
		PdmsObjects::Stack::Detroy(currentItem);
	}

	PdmsObjects::Stack::Clear();
}

void PdmsParser::linkWithSession(PdmsLexer *s)
{
	session = s;
	currentCommand = NULL;
	currentItem = NULL;
	root = NULL;
	PdmsCommands::DistanceValue::setWorkingUnit(PDMS_MILLIMETRE);
}

bool PdmsParser::processCurrentToken()
{
	if (!session)
	{
		assert(false);
		return false;
	}

	Token currentToken = session->getCurrentToken();
	switch(currentToken)
	{
	case PDMS_UNUSED:
		break;
	
	case PDMS_UNKNOWN:
		session->printWarning("Unknown token");
		return false;
	
	case PDMS_EOS:
		break;
	
	case PDMS_NUM_VALUE:
		//There should be a active command, and it should handle the value
		if (!currentCommand || !currentCommand->handle(session->valueFromBuffer()))
		{
			session->printWarning("Unexpected numerical value");
			return false;
		}
		break;
	
	case PDMS_NAME_STR:
		//There should be a active command, and it should handle the char string
		if (!currentCommand || !currentCommand->handle(session->nameFromBuffer()))
		{
			session->printWarning("Last token cannot be associated with a name");
			return false;
		}
		break;
	
	default:
		//If there is an active command
		if (currentCommand)
		{
			//If the active command can handle the new token, it's ok
			if (currentCommand->handle(currentToken))
				return true;
			
			//Else, the token must be a new command. We execute the active command and delete it
			PdmsObjects::GenericItem* item = currentItem;
			bool success = currentCommand->execute(item);
			delete currentCommand;
			currentCommand = NULL;
			if (!success)
			{
				assert(false);
				session->printWarning("Unable to resolve previous command (this token may be unexpected in current command)");
				return false;
			}
			//The command execution could have changed the current item
			if (item)
			{
				currentItem = item;
			}
			else if (currentItem)
			{
				if (!root)
				{
					root = currentItem->getRoot();
					currentItem = NULL;
				}
				else
				{
					assert(false);
					session->printWarning("Trying to create a second root for elements hierarchy");
					return false;
				}
			}
		}
		if (currentToken == PDMS_RETURN)
		{
			session->finish();
		}
		else
		{
			//Here, we must create a new command from the token speciffied by the lexer
			currentCommand = PdmsCommands::Command::Create(currentToken);
			if (!currentCommand)
			{
				session->printWarning("Unknown command");
				assert(false);
				return false;
			}
		}
		break;
	}
	return true;
}

bool PdmsParser::parseSessionContent()
{
	PdmsObjects::Stack::Init();

	if (!session  || !session->initializeSession())
	{
		return false;
	}

	while (session->gotoNextToken())
	{
		if (!processCurrentToken())
		{
			session->closeSession(true);
			return false;
		}
	}
	//If the hierarchy root has not yet been computed, do it now.
	if (!root)
		root = currentItem->getRoot();
	//else check that the current item doesn't belong to a new root
	else if (currentItem->getRoot() != root)
		session->printWarning("there could be several hierarchy root specified in this file");
	if (root)
		root->convertCoordinateSystem();
	session->setLoadedObject(root);
	session->closeSession(false);
	return true;
}

PdmsObjects::GenericItem* PdmsParser::getLoadedObject(bool forgetIt)
{
	PdmsObjects::GenericItem *result = NULL;
	if(session)
		result = session->getLoadedObject();
	if(forgetIt)
	{
		currentItem = root = NULL;
	}
	return result;
}
