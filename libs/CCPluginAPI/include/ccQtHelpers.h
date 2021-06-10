#pragma once
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

#include <QAbstractButton>

class ccQtHelpers
{
public:
	
	//! Sets a button background color
	static void SetButtonColor( QAbstractButton* button, const QColor &col )
	{
		if ( button != nullptr )
		{
			button->setStyleSheet( QStringLiteral( "* { background-color: rgb(%1,%2,%3) }" )
								   .arg( col.red() )
								   .arg( col.green() )
								   .arg( col.blue() )
								   );
		}
	}
	
};
