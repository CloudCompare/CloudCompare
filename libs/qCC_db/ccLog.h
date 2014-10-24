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

#ifndef CC_LOG_HEADER
#define CC_LOG_HEADER

//Local
#include "qCC_db.h"

//system
#include <stdio.h>
#include <string.h>

//Qt
#include <QString>

//! Main log interface
/** This interface is meant to be used as a unique (static) instance.
	It should be thread safe!
**/
class QCC_DB_LIB_API ccLog
{
public:

	//! Destructor
	virtual ~ccLog() {};

	//! Returns the static and unique instance
	static ccLog* TheInstance();

	//! Registers a unique instance
	static void RegisterInstance(ccLog* logInstance);

	//! Prints out a formated message in console
	/** Works just like the 'printf' command.
		\return always return 'true'
	**/
	static bool Print(const char *format, ...);

	//! QString version of 'Print'
	static bool Print(const QString& message);

	//! Same as Print, but works only in debug mode
	/** Works just like the 'printf' command.
		\return always return 'true'
	**/
	static bool PrintDebug(const char *format, ...);

	//! QString version of 'PrintDebug'
	static bool PrintDebug(const QString& message);

	//! Prints out a formated warning message in console
	/** Works just like the 'printf' command.
		\return always return 'false'
	**/
	static bool Warning(const char *format, ...);

	//! QString version of 'Warning'
	static bool Warning(const QString& message);

	//! Same as Warning, but works only in debug mode
	/** Works just like the 'printf' command.
		\return always return 'false'
	**/
	static bool WarningDebug(const char *format, ...);

	//! QString version of 'WarningDebug'
	static bool WarningDebug(const QString& message);

	//! Display an error dialog with formated message
	/** Works just like the 'printf' command.
		\return always return 'false'
	**/
	static bool Error(const char *format, ...);

	//! QString version of 'Error'
	static bool Error(const QString& message);

	//! Same as Error, but works only in debug mode
	/** Works just like the 'printf' command.
		\return always return 'false'
	**/
	static bool ErrorDebug(const char *format, ...);

	//! QString version of 'ErrorDebug'
	static bool ErrorDebug(const QString& message);

protected:

	//! Message level
	enum MessageLevel
	{
		LOG_STANDARD		= 0, /**< Standard message (Print) **/
		LOG_STANDARD_DEBUG	= 1, /**< Standard message - debug only (PrintDebug) **/
		LOG_WARNING			= 2, /**< Warning message (Warning) **/
		LOG_WARNING_DEBUG	= 3, /**< Warning message - debug only (WarningDebug) **/
		LOG_ERROR			= 4, /**< Error message (Error) **/
		LOG_ERROR_DEBUG		= 5, /**< Error message - debug only (ErrorDebug) **/
	};

	//! Generic message display method
	/** To be implemented by child class.
		WARNING: MUST BE THREAD SAFE!
		\param message message
		\param level message severity
	**/
	virtual void displayMessage(const QString& message, MessageLevel level) = 0;

};

#endif //CC_LOG_HEADER
