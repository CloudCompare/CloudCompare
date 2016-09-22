//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE LIGHT VIEWER                            #
//#                                                                        #
//#  This project has been initiated under funding from ANR/CIFRE          #
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
//#      +++ COPYRIGHT: EDF R&D + TELECOM ParisTech (ENST-TSI) +++         #
//#                                                                        #
//##########################################################################

#ifndef CCVIEWER_LOG_HEADER
#define CCVIEWER_LOG_HEADER

//Qt
#include <QMessageBox>
#include <QMainWindow>

//qCC_db
#include <ccLog.h>

//! Minimalist logger (only displays error messages)
class ccViewerLog : public ccLog
{
public:
	//! Default constructor
	explicit ccViewerLog(QMainWindow* parentWindow = 0) : ccLog(), m_parentWindow(parentWindow) {}

protected:
	//inherited from ccLog
	virtual void logMessage(const QString& message, int level)
	{
		if (level & LOG_ERROR)
		{
			QMessageBox::warning(m_parentWindow, "Error", message);
		}
	}

	//! Associated window
	QMainWindow* m_parentWindow;
};

#endif // CCVIEWER_LOG_HEADER
