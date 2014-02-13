//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qBlur                       #
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

#include <QtGui>
#include <QInputDialog>

#include "qBlur.h"
#include <ccBilateralFilter.h>

QIcon qBlur::getIcon() const
{
    return QIcon();
}

ccGlFilter* qBlur::getFilter()
{
    ccBilateralFilter* filter = new ccBilateralFilter();

    bool ok=false;
    double sigma = QInputDialog::getDouble(0,"Bilateral filter","Sigma (pixel)",1.0,0.1,8.0,1,&ok);

    if (!ok)
        return 0;

    int filterSize = 1+2*static_cast<int>(ceil(2.5*sigma));
    filter->setParameters(filterSize,static_cast<float>(sigma),0);

    return filter;
}

Q_EXPORT_PLUGIN2(qBlur,qBlur);
