//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This project has been initated under funding from ANR/CIFRE           #
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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2279                                                              $
//$LastChangedDate:: 2012-10-18 21:54:33 +0200 (jeu., 18 oct. 2012)        $
//**************************************************************************
//

#include "ccCommon.h"

#include <QString>

#define CC_VER_NUM  2.4
#define CC_VER_DATE "01/13/2013"

//! Returns current version as string
const char* ccCommon::GetCCVersion()
{
#if !defined(_WIN32) && !defined(WIN32)
    return qPrintable(QString::number(CC_VER_NUM)+QString(".Qt/Linux - %2").arg(CC_VER_DATE));
#else
#ifdef _MSC_VER
    return qPrintable(QString::number(CC_VER_NUM)+QString(".Qt/MSVC - %2").arg(CC_VER_DATE));
#else
    return qPrintable(QString::number(CC_VER_NUM)+QString(".Qt/MinGW - %2").arg(CC_VER_DATE));
#endif
#endif
};
