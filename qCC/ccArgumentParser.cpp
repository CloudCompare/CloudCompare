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
#include "ccArgumentParser.h"

#include <cassert>
#include <ccLog.h>

ccArgumentParser::ccArgumentParser(QStringList& arguments)
    : m_arguments(arguments)
{
}

const QString ccArgumentParser::peek() const
{
	if (m_arguments.isEmpty())
	{
		return {};
	}
	return m_arguments.first();
}

void ccArgumentParser::skip()
{
	assert(!m_arguments.isEmpty());
	if (!m_arguments.isEmpty())
	{
		m_arguments.removeFirst();
	}
}

bool ccArgumentParser::isEmpty() const
{
	return m_arguments.isEmpty();
}

QString ccArgumentParser::takeNext()
{
	if (m_arguments.isEmpty())
	{
		return {};
	}
	return m_arguments.takeFirst();
}

std::optional<float> ccArgumentParser::takeFloat(const QString& context)
{
	if (m_arguments.isEmpty())
	{
		ccLog::Error(QObject::tr("Missing parameter: %1").arg(context));
		return std::nullopt;
	}

	const QString arg = m_arguments.takeFirst();
	return ParseFloat(arg, context);
}

std::optional<double> ccArgumentParser::takeDouble(const QString& context, double min, double max)
{
	if (m_arguments.isEmpty())
	{
		ccLog::Error(QObject::tr("Missing parameter: %1").arg(context));
		return std::nullopt;
	}

	const QString arg = m_arguments.takeFirst();
	return ParseDouble(arg, context, min, max);
}

std::optional<int> ccArgumentParser::takeInt(const QString& context, int min, int max)
{
	if (m_arguments.isEmpty())
	{
		ccLog::Error(QObject::tr("Missing parameter: %1").arg(context));
		return std::nullopt;
	}

	const QString arg = m_arguments.takeFirst();
	return ParseInt(arg, context, min, max);
}

bool ccArgumentParser::tryConsumeOption(const QString& option)
{
	if (isEmpty())
	{
		return false;
	}

	const QString arg = m_arguments.first();

	if (arg.startsWith("-") && arg.mid(1).toUpper() == option)
	{
		m_arguments.removeFirst();
		return true;
	}

	return false;
}

std::optional<float> ccArgumentParser::ParseFloat(const QString& arg, const QString& name)
{
	bool        ok;
	const float value = arg.toFloat(&ok);
	if (!ok)
	{
		ccLog::Error(QObject::tr("Invalid float value '%1' for %2").arg(arg, name));
		return std::nullopt;
	}

	return value;
}

std::optional<double> ccArgumentParser::ParseDouble(const QString& arg, const QString& name, double min, double max)
{
	bool         ok;
	const double value = arg.toDouble(&ok);
	if (!ok)
	{
		ccLog::Error(QObject::tr("Invalid double value '%1' for %2").arg(arg, name));
		return std::nullopt;
	}

	if (value < min)
	{
		ccLog::Error(QObject::tr("%1 (=%2) must be >= %3").arg(name, arg, QString::number(min)));
		return std::nullopt;
	}

	if (value > max)
	{
		ccLog::Error(QObject::tr("%1 (=%2) must be <= %3").arg(name, arg, QString::number(max)));
		return std::nullopt;
	}

	return value;
}

std::optional<int> ccArgumentParser::ParseInt(const QString& arg, const QString& name, int min, int max)
{
	bool      ok;
	const int value = arg.toInt(&ok);
	if (!ok)
	{
		ccLog::Error(QObject::tr("Invalid int value '%1' for %2").arg(arg, name));
		return std::nullopt;
	}

	if (value < min)
	{
		ccLog::Error(QObject::tr("%1 (=%2) must be >= %3").arg(name, arg, QString::number(min)));
		return std::nullopt;
	}

	if (value > max)
	{
		ccLog::Error(QObject::tr("%1 (=%2) must be <= %3").arg(name, arg, QString::number(max)));
		return std::nullopt;
	}

	return value;
}
