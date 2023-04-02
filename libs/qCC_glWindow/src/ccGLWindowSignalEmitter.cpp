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

#include "../include/ccGLWindowSignalEmitter.h"

#include "../include/ccGLWindowInterface.h"

ccGLWindowSignalEmitter::ccGLWindowSignalEmitter(ccGLWindowInterface* associatedWindow, QObject* parent)
	: QObject(parent)
	, m_associatedWindow(associatedWindow)
{
	setObjectName(QString("Signal emitter @ GL Window #%1").arg(associatedWindow->getUniqueID()));
}
