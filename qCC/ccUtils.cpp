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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include "ccConsole.h"

#include "ccUtils.h"

//Qt
#include <QClipboard>
#include <QApplication>

namespace ccUtils
{
	void    DisplayLockedVerticesWarning(const QString& meshName, bool displayAsError)
	{
		QString message = QString("Vertices of mesh '%1' are locked (they may be shared by multiple entities for instance).\nYou should call this method directly on the vertices cloud.\n(warning: all entities depending on this cloud will be impacted!)").arg(meshName);
		
		if (displayAsError)
			ccConsole::Error(message);
		else
			ccConsole::Warning(message);
	}

	bool GetVectorFromClipboard(CCVector3d& vector, bool sendErrors/*=true*/)
	{
		const QClipboard* clipboard = QApplication::clipboard();
		if (!clipboard)
		{
			if (sendErrors)
			{
				ccLog::Error("Clipboard not available");
			}
			return false;
		}
		QString text = clipboard->text().trimmed();
		if (text.startsWith('[') || text.startsWith('{') || text.startsWith('('))
		{
			text = text.right(text.length() - 1);
		}
		if (text.endsWith(']') || text.endsWith('}') || text.endsWith(')'))
		{
			text = text.left(text.length() - 1);
		}

		if (text.isEmpty())
		{
			return false;
		}

		QStringList tokens = text.split(QRegExp("\\s+"), QString::SkipEmptyParts);
		if (tokens.size() != 3)
		{
			tokens = text.split(';', QString::SkipEmptyParts);
			if (tokens.size() != 3)
			{
				tokens = text.split(',', QString::SkipEmptyParts);
				if (tokens.size() != 3)
				{
					if (text.length() > 64)
					{
						text.truncate(61);
						text += "...";
					}
					if (sendErrors)
					{
						ccLog::Error("Unrecognized format: " + text);
					}
					return false;
				}
			}
		}

		for (unsigned char i = 0; i < 3; ++i)
		{
			bool ok = false;
			vector.u[i] = tokens[i].toDouble(&ok);
			if (!ok)
			{
				if (sendErrors)
				{
					ccLog::Error("Invalid value: " + tokens[i]);
				}
				return false;
			}
		}

		return true;
	}

}
