#pragma once

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

// Local
#include "qCC_db.h"

// Qt
#include <QString>
#include <QTime>

//! Main log interface
/** This interface is meant to be used as a unique (static) instance.
    It should be thread safe!
**/
class QCC_DB_LIB_API ccLog
{
  public:
	//! Destructor
	virtual ~ccLog() = default;

	//! Starts the logger (timer)
	/** \return success (a Qt application must be alive)
	 **/
	static bool Start();

	//! Returns the static and unique instance
	static ccLog* TheInstance();

	//! Registers a unique instance
	static void RegisterInstance(ccLog* logInstance);

	//! Enables the message backup system
	/** Stores the messages until a valid logging instance is registered.
	 **/
	static void EnableMessageBackup(bool state);

	//! Message level
	enum MessageLevelFlags
	{
		LOG_VERBOSE   = 0, /**< Verbose message (Debug) **/
		LOG_STANDARD  = 1, /**< Standard message (Print) **/
		LOG_IMPORTANT = 2, /**< Important messages (PrintHigh) **/
		LOG_WARNING   = 3, /**< Warning message (Warning) **/
		LOG_ERROR     = 4, /**< Error message (Error) **/

		DEBUG_FLAG = 8 /**< Debug flag (reserved) **/
	};

	//! Returns the current verbosity level
	static int VerbosityLevel();

	//! Sets the verbosity level
	static void SetVerbosityLevel(int level);

	//! Message
	struct Message
	{
		//! Default constructor
		Message(const QString& _text, int _level, qint64 _time_ns)
		    : text(_text)
		    , level(_level)
		    , time_ns(_time_ns)
		{
		}

		QString text;    //!< message text
		int     level;   //!< message level (see ccLog::MessageLevelFlags)
		qint64  time_ns; //!< time stamp (in ns) when the message was generated
	};

	//! Static shortcut to ccLog::logMessage
	static void LogMessage(const QString& message, int level, qint64 time_ns = -1);

	//! Prints out a verbose formatted message in console
	/** Works just like the 'printf' command.
	    \return always 'true'
	**/
	static bool PrintVerbosef(const char* format, ...);

	//! QString version of ccLog::PrintVerbose
	static bool PrintVerbose(const QString& message);

	//! Prints out a formatted message in console
	/** Works just like the 'printf' command.
	    \return always 'true'
	**/
	static bool Printf(const char* format, ...);

	//! QString version of ccLog::Print
	static bool Print(const QString& message);

	//! Prints out an important formatted message in console
	/** Works just like the 'printf' command.
	    \return always 'true'
	**/
	static bool PrintHighf(const char* format, ...);

	//! QString version of ccLog::PrintHigh
	static bool PrintHigh(const QString& message);

	//! Same as Print, but works only in Debug mode
	/** Works just like the 'printf' command.
	    \return always 'true'
	**/
	static bool PrintDebugf(const char* format, ...);

	//! QString version of ccLog::PrintDebug
	static bool PrintDebug(const QString& message);

	//! Prints out a formatted warning message in console
	/** Works just like the 'printf' command.
	    \return always 'false'
	**/
	static bool Warningf(const char* format, ...);

	//! QString version of ccLog::Warning
	static bool Warning(const QString& message);

	//! Same as Warning, but works only in Debug mode
	/** Works just like the 'printf' command.
	    \return always 'false'
	**/
	static bool WarningDebugf(const char* format, ...);

	//! QString version of ccLog::WarningDebug
	static bool WarningDebug(const QString& message);

	//! Display an error dialog with formatted message
	/** Works just like the 'printf' command.
	    \return always 'false'
	**/
	static bool Error(const char* format, ...);

	//! QString version of 'Error'
	static bool Error(const QString& message);

	//! Same as Error, but works only in Debug mode
	/** Works just like the 'printf' command.
	    \return always 'false'
	**/
	static bool ErrorDebugf(const char* format, ...);

	//! QString version of ccLog::ErrorDebug
	static bool ErrorDebug(const QString& message);

	//! Sets the start time (for time stamp)
	void setStartTime(const QTime& time)
	{
		m_startTime = time;
	}

  protected:
	//! Generic message logging method
	/** To be implemented by child class.
	    \warning MUST BE THREAD SAFE!
	    \param message message
	**/
	virtual void logMessage(const Message& message) = 0;

	//! Start time (for time stamp)
	QTime m_startTime;
};
