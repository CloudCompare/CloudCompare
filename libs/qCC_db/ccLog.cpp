//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccLog.h"

//CCLib
#include <CCPlatform.h>

//System
#include <assert.h>

#if !defined(CC_WINDOWS)
#define _vsnprintf vsnprintf
#endif

/***************
 *** Globals ***
 ***************/

//buffer for formated string generation
static char s_buffer[4096];

//unique console instance
static ccLog* s_instance = 0;

ccLog* ccLog::TheInstance()
{
	return s_instance;
}

void ccLog::RegisterInstance(ccLog* logInstance)
{
	s_instance = logInstance;
}


bool ccLog::Print(const char* format, ...)
{
	if (s_instance)
	{
		//we get the "..." parameters as "printf" would do
		va_list args;
		va_start(args, format);
		_vsnprintf(s_buffer, (size_t)4096, format, args);
		va_end(args);

		s_instance->displayMessage(QString(s_buffer), LOG_STANDARD);
	}

	return true;
}

bool ccLog::Print(const QString& message)
{
	if (s_instance)
		s_instance->displayMessage(message, LOG_STANDARD);

	return true;
}

bool ccLog::Warning(const char* format, ...)
{
	if (s_instance)
	{
		//we get the "..." parameters as "printf" would do
		va_list args;
		va_start(args, format);
		_vsnprintf(s_buffer, (size_t)4096, format, args);
		va_end(args);

		s_instance->displayMessage(QString(s_buffer), LOG_WARNING);
	}

	return false;
}

bool ccLog::Warning(const QString& message)
{
	if (s_instance)
		s_instance->displayMessage(message, LOG_WARNING);

	return false;
}

bool ccLog::Error(const char* format, ...)
{
	if (s_instance)
	{
		//we get the "..." parameters as "printf" would do
		va_list args;
		va_start(args, format);
		_vsnprintf(s_buffer, (size_t)4096, format, args);
		va_end(args);

		s_instance->displayMessage(QString(s_buffer), LOG_ERROR);
	}

	return false;
}

bool ccLog::Error(const QString& message)
{
	if (s_instance)
		s_instance->displayMessage(message, LOG_ERROR);

	return false;
}

bool ccLog::PrintDebug(const char* format, ...)
{
	if (s_instance)
	{
		//we get the "..." parameters as "printf" would do
		va_list args;
		va_start(args, format);
		_vsnprintf(s_buffer, (size_t)4096, format, args);
		va_end(args);

		s_instance->displayMessage(QString(s_buffer), LOG_STANDARD_DEBUG);
	}

	return true;
}

bool ccLog::PrintDebug(const QString& message)
{
	if (s_instance)
		s_instance->displayMessage(message, LOG_STANDARD_DEBUG);

	return true;
}

bool ccLog::WarningDebug(const char* format, ...)
{
	if (s_instance)
	{
		//we get the "..." parameters as "printf" would do
		va_list args;
		va_start(args, format);
		_vsnprintf(s_buffer, (size_t)4096, format, args);
		va_end(args);

		s_instance->displayMessage(QString(s_buffer), LOG_WARNING_DEBUG);
	}

	return false;
}

bool ccLog::WarningDebug(const QString& message)
{
	if (s_instance)
		s_instance->displayMessage(message, LOG_WARNING_DEBUG);

	return false;
}

bool ccLog::ErrorDebug(const char* format, ...)
{
	if (s_instance)
	{
		//we get the "..." parameters as "printf" would do
		va_list args;
		va_start(args, format);
		_vsnprintf(s_buffer, (size_t)4096, format, args);
		va_end(args);

		s_instance->displayMessage(QString(s_buffer), LOG_ERROR_DEBUG);
	}

	return false;
}

bool ccLog::ErrorDebug(const QString& message)
{
	if (s_instance)
		s_instance->displayMessage(message, LOG_ERROR_DEBUG);

	return false;
}
