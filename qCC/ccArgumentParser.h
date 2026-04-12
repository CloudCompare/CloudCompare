#pragma once
// ##########################################################################
// #                                                                        #
// #                            CLOUDCOMPARE                                #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 of the License.               #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                   COPYRIGHT: CloudCompare project                      #
// #                                                                        #
// ##########################################################################

#include <ccLog.h>

// Qt
#include <QObject>
#include <QStringList>

// System
#include <initializer_list>
#include <optional>
#include <utility>

//! Argument parser for command line arguments
/** Meant to simplify parsing.
    Functions will log errors when necessary
**/
class ccArgumentParser
{
  public:
	explicit ccArgumentParser(QStringList& arguments);

	//! Returns the next argument without consuming it, or nullptr if there are none
	const QString peek() const;
	//! Skips the next argument, to be used with `peek`
	void skip();
	//! Returns true if there are no arguments left
	bool isEmpty() const;

	//! Returns the next argument in the list, or a null string if there are none
	QString takeNext();
	//! Returns the next argument in the list parsed as a float, or std::nullopt if there are none
	/** Logs errors
	 **/
	std::optional<float> takeFloat(const QString& context);
	//! Checks if the next argument is `-OPTION` (case insensitive)
	/** - if yes: consumes it and returns true
	    - if not: returns false without consuming
	**/
	bool tryConsumeOption(const QString& option);

	//! Returns the next argument parsed as an enum
	/** Takes a list that matches strings to enum values
	    strings should be `UPPER_CASE`, the user value will not be case sensitive
	**/
	template <typename T>
	std::optional<T> takeEnum(const std::initializer_list<std::pair<const char*, T>>& mapping, const QString& context)
	{
		if (m_arguments.isEmpty())
		{
			ccLog::Error(QObject::tr("Missing parameter: %1").arg(context));
			return std::nullopt;
		}

		const QString arg = m_arguments.takeFirst().toUpper();

		for (const auto& [key, value] : mapping)
		{
			if (key == arg)
			{
				return value;
			}
		}

		QStringList valid;
		for (const auto& [key, value] : mapping)
		{
			valid << key;
		}

		ccLog::Error(QObject::tr("Invalid %1: '%2' (expected one of: %3)")
		                 .arg(context, arg, valid.join(", ")));

		return std::nullopt;
	}

	//! Parses a float from a string
	/** Logs an error on failure
	 **/
	static std::optional<float> ParseFloat(const QString& arg, const QString& name);

  private:
	QStringList& m_arguments; //!< Command line arguments (reference to)
};
