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

#include "ccLog.h"

//CCLib
#include <CCPlatform.h>

//System
#include <cassert>
#include <vector>

#if !defined(CC_WINDOWS)
#define _vsnprintf vsnprintf
#endif

/***************
 *** Globals ***
 ***************/

//buffer for formatted string generation
static const size_t s_bufferMaxSize = 4096;
static char s_buffer[s_bufferMaxSize];

//! Message
struct Message
{
	Message(const QString& t, int f)
		: text(t)
		, flags(f)
	{}

	QString text;
	int flags;
};

//message backup system
static bool s_backupEnabled;
//backuped messages
static std::vector<Message> s_backupMessages;

//unique console instance
static ccLog* s_instance = nullptr;

ccLog* ccLog::TheInstance()
{
	return s_instance;
}

void ccLog::EnableMessageBackup(bool state)
{
	s_backupEnabled = state;
}

void ccLog::LogMessage(const QString& message, int level)
{
#ifndef QT_DEBUG
	//skip debug messages in release mode as soon as possible
	if (level & LOG_DEBUG)
	{
		return;
	}
#endif

	if (s_instance)
	{
		s_instance->logMessage(message, level);
	}
	else if (s_backupEnabled)
	{
		try
		{
			s_backupMessages.emplace_back(message, level);
		}
		catch (const std::bad_alloc&)
		{
			//nothing to do, the message will be lost...
		}
	}
}

void ccLog::RegisterInstance(ccLog* logInstance)
{
	s_instance = logInstance;
	if (s_instance)
	{
		//if we have a valid instance, we can now flush the backuped messages
		for (const Message& message : s_backupMessages)
		{
			s_instance->logMessage(message.text, message.flags);
		}
		s_backupMessages.clear();
	}
}

//Conversion from '...' parameters to QString so as to call ccLog::logMessage
//(we get the "..." parameters as "printf" would do)
#define LOG_ARGS(flags)\
	if (s_instance || s_backupEnabled)\
	{\
		va_list args;\
		va_start(args, format);\
		_vsnprintf(s_buffer, s_bufferMaxSize, format, args);\
		va_end(args);\
		LogMessage(QString(s_buffer), flags);\
	}\

bool ccLog::Print(const char* format, ...)
{
	LOG_ARGS(LOG_STANDARD)
	return true;
}

bool ccLog::Warning(const char* format, ...)
{
	LOG_ARGS(LOG_WARNING)
	return false;
}

bool ccLog::Error(const char* format, ...)
{
	LOG_ARGS(LOG_ERROR)
	return false;
}

bool ccLog::PrintDebug(const char* format, ...)
{
#ifdef QT_DEBUG
	LOG_ARGS(LOG_STANDARD | LOG_DEBUG)
#endif
	return true;
}

bool ccLog::WarningDebug(const char* format, ...)
{
#ifdef QT_DEBUG
	LOG_ARGS(LOG_WARNING | LOG_DEBUG)
#endif
	return false;
}

bool ccLog::ErrorDebug(const char* format, ...)
{
#ifdef QT_DEBUG
	LOG_ARGS(LOG_ERROR | LOG_DEBUG)
#endif
	return false;
}
