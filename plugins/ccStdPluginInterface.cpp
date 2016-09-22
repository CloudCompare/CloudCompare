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

#include "ccStdPluginInterface.h"

//qCC_db
#include <ccObject.h>

void ccStdPluginInterface::setMainAppInterface(ccMainAppInterface* app)
{
	m_app = app;

	if (m_app)
	{
		//we use the same 'unique ID' generator in plugins as in the main
		//application (otherwise we'll have issues with 'unique IDs'!)
		ccObject::SetUniqueIDGenerator(m_app->getUniqueIDGenerator());
	}
}
