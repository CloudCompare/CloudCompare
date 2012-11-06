//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#               COPYRIGHT: Luca Penasa                                   #
//#                                                                        #
//##########################################################################
//
#ifndef Q_PCL_PLUGIN_LOADPCD_HEADER
#define Q_PCL_PLUGIN_LOADPCD_HEADER

#include "BaseFilter.h"

//Qt
#include <QStringList>

/** \brief LoadPCD filter
 * \author Luca Penasa
 * Enables loading of some of the types defined in PCL \n
 * This class is an implementation of the PCLFilter base class. \n
 * Overridden methods are init(), openDialog() and compute() \n
 * \note conversion is made using the function sensorToCC()
 */
class LoadPCD: public BaseFilter
{
    Q_OBJECT

public:

	//! Default constructor
    LoadPCD();

protected:

	//inherited from BaseFilter
    int checkSelected();
    int openDialog();
    int compute();
    QString getErrorMessage(int errorCode);

    //! Filename(s) to open
    QStringList m_filenames;

};

#endif
