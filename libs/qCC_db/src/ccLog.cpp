// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "../include/ccLog.h"

// CCCoreLib
#include <CCPlatform.h>

// Qt
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QMutex>

// System
#include <cassert>
#include <vector>

#if !defined(CC_WINDOWS)
#define _vsnprintf vsnprintf
#endif

/***************
 *** Globals ***
 ***************/

// message backup system
static bool s_backupEnabled;

// message verbosity level
#ifdef QT_DEBUG
static int s_verbosityLevel = ccLog::LOG_VERBOSE;
#else
static int s_verbosityLevel = ccLog::LOG_STANDARD;
#endif

// backed up messages
static std::vector<ccLog::Message> s_backupMessages;

// timer
QMutex               s_timerMutex;
static QElapsedTimer s_timer;
static QTime         s_startTime;
static bool          s_timerStarted = false;

// unique console instance
static ccLog* s_instance = nullptr;

bool ccLog::Start()
{
	if (s_timerStarted)
	{
		// already started
		return true;
	}

	if (!qApp) // we need a valid Qt application to use QElapsedTimer
	{
		return false;
	}

	s_timerMutex.lock();
	s_startTime = QTime::currentTime();
	s_timer.start();
	s_timerStarted = true;
	s_timerMutex.unlock();

	return true;
}

ccLog* ccLog::TheInstance()
{
	return s_instance;
}

void ccLog::EnableMessageBackup(bool state)
{
	s_backupEnabled = state;
}

int ccLog::VerbosityLevel()
{
	return s_verbosityLevel;
}

void ccLog::SetVerbosityLevel(int level)
{
	s_verbosityLevel = std::min(level, static_cast<int>(LOG_ERROR)); // can't ignore error messages
}

void ccLog::LogMessage(const QString& message, int level, qint64 time_ns/*=-1*/)
{
	// skip messages below the current 'verbosity' level
	if ((level & 7) < s_verbosityLevel)
	{
		return;
	}

	if (time_ns < 0)
	{
		if (s_timerStarted)
		{
			s_timerMutex.lock();
			time_ns = s_timer.nsecsElapsed();
			s_timerMutex.unlock();
		}
		else
		{
			assert(false);
			time_ns = 0;
		}
	}

	if (s_instance)
	{
		s_instance->logMessage({message, level, time_ns});
	}
	else if (s_backupEnabled)
	{
		try
		{
			s_backupMessages.emplace_back(message, level, time_ns);
		}
		catch (const std::bad_alloc&)
		{
			// nothing to do, the message will be lost...
		}
	}
}

void ccLog::RegisterInstance(ccLog* logInstance)
{
	s_instance = logInstance;

	if (s_instance)
	{
		if (!s_timerStarted)
		{
			Start();
		}
		s_instance->setStartTime(s_startTime);

		// if we have a valid instance, we can now flush the backed up messages
		for (const Message& message : s_backupMessages)
		{
			s_instance->logMessage(message);
		}
		s_backupMessages.clear();
	}
}

// Conversion from '...' parameters to QString so as to call ccLog::logMessage
//(we get the "..." parameters as "printf" would do)
#define LOG_ARGS(flags) \
	if (s_instance || s_backupEnabled) \
	{ \
		s_timerMutex.lock(); \
		qint64 timestamp_ns = s_timer.nsecsElapsed(); /*capture timestamp as early as possible*/ \
		s_timerMutex.unlock(); \
		va_list args; \
		va_start(args, format); \
		static const size_t s_bufferMaxSize = 4096; \
		static char s_buffer[s_bufferMaxSize]; \
		_vsnprintf(s_buffer, s_bufferMaxSize, format, args); \
		va_end(args); \
		LogMessage(QString(s_buffer), flags, timestamp_ns); \
	}

bool ccLog::PrintVerbosef(const char* format, ...)
{
	LOG_ARGS(LOG_VERBOSE)
	return true;
}

bool ccLog::PrintVerbose(const QString& message)
{
	LogMessage(message, LOG_VERBOSE);
	return true;
}

bool ccLog::Printf(const char* format, ...)
{
	LOG_ARGS(LOG_STANDARD)
	return true;
}

bool ccLog::Print(const QString& message)
{
	LogMessage(message, LOG_STANDARD);
	return true;
}

bool ccLog::PrintHighf(const char* format, ...)
{
	LOG_ARGS(LOG_IMPORTANT)
	return true;
}

bool ccLog::PrintHigh(const QString& message)
{
	LogMessage(message, LOG_IMPORTANT);
	return true;
}

bool ccLog::Warningf(const char* format, ...)
{
	LOG_ARGS(LOG_WARNING)
	return false;
}

bool ccLog::Warning(const QString& message)
{
	LogMessage(message, LOG_WARNING);
	return false;
}

bool ccLog::Error(const char* format, ...)
{
	LOG_ARGS(LOG_ERROR)
	return false;
}

bool ccLog::Error(const QString& message)
{
	LogMessage(message, LOG_ERROR);
	return false;
}

bool ccLog::PrintDebugf(const char* format, ...)
{
#ifdef QT_DEBUG
	LOG_ARGS(LOG_STANDARD | DEBUG_FLAG)
#endif
	return false;
}

bool ccLog::PrintDebug(const QString& message)
{
#ifdef QT_DEBUG
	LogMessage(message, LOG_STANDARD | DEBUG_FLAG);
#endif
	return false;
}

bool ccLog::WarningDebugf(const char* format, ...)
{
#ifdef QT_DEBUG
	LOG_ARGS(LOG_WARNING)
#endif
	return false;
}

bool ccLog::WarningDebug(const QString& message)
{
#ifdef QT_DEBUG
	LogMessage(message, LOG_WARNING | DEBUG_FLAG);
#endif
	return false;
}

bool ccLog::ErrorDebugf(const char* format, ...)
{
#ifdef QT_DEBUG
	LOG_ARGS(LOG_ERROR)
#endif
	return false;
}

bool ccLog::ErrorDebug(const QString& message)
{
#ifdef QT_DEBUG
	LogMessage(message, LOG_ERROR | DEBUG_FLAG);
#endif
	return false;
}
