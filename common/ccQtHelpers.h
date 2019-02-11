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

#ifndef CC_QT_HELPERS_HEADER
#define CC_QT_HELPERS_HEADER

//Qt
#include <QAbstractButton>

class ccQtHelpers
{
public:

	//! Sets a button background color
	inline static void SetButtonColor(QAbstractButton* button, const QColor &col)
	{
		if (button)
			button->setStyleSheet(QString("* { background-color: rgb(%1,%2,%3) }").arg(col.red()).arg(col.green()).arg(col.blue()));
	}

};

#endif //CC_QT_HELPERS_HEADER