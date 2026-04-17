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
	//! Returns the numbers of arguments left
	size_t size() const
	{
		return m_arguments.size();
	}

	//! Returns the next argument in the list, or a null string if there are none
	QString takeNext();

	//! Returns the next argument in the list parsed as a float, or std::nullopt if there are none
	/** Logs errors
	 **/
	std::optional<float> takeFloat(const QString& context);

	std::optional<double> takeDouble(const QString& context, double min = std::numeric_limits<double>::lowest(), double max = std::numeric_limits<double>::max());

	std::optional<int> takeInt(const QString& context, int min = std::numeric_limits<int>::min(), int max = std::numeric_limits<int>::max());

	//! Checks if the next argument is `-OPTION` (case insensitive)
	/** option string should not contain the `-`
	    - if yes: consumes it and returns true
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

		const QString arg = m_arguments.takeFirst();
		return ParseEnum(arg, mapping, context);
	}

	template <typename T>
	static std::optional<T> ParseEnum(const QString& arg, const std::initializer_list<std::pair<const char*, T>>& mapping, const QString& context)
	{
		const QString upper = arg.toUpper();
		for (const auto& [key, value] : mapping)
		{
			if (key == upper)
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

	//! Parses a double from a string
	/** Logs an error on failure:
	 * - if arg is not a number
	 * - if arg is < min
	 * - if arg is > max
	 **/
	static std::optional<double> ParseDouble(const QString& arg, const QString& name, double min = std::numeric_limits<double>::lowest(), double max = std::numeric_limits<double>::max());

	//! Parses an int from a string
	/** Logs an error on failure:
	 * - if arg is not a number
	 * - if arg is < min
	 * - if arg is > max
	 **/
	static std::optional<int> ParseInt(const QString& arg, const QString& name, int min = std::numeric_limits<int>::min(), int max = std::numeric_limits<int>::max());

  private:
	QStringList& m_arguments; //!< Command line arguments (reference to)
};
