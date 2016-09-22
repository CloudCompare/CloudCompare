//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qBlur                       #
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

#include <ccBilateralFilter.h>

//Qt
#include <QtGui>
#include <QInputDialog>

#include "qBlur.h"

QIcon qBlur::getIcon() const
{
    return QIcon();
}

ccGlFilter* qBlur::getFilter()
{
	bool ok = false;
	double sigma = QInputDialog::getDouble(0, "Bilateral filter", "Sigma (pixel)", 1.0, 0.1, 8.0, 1, &ok);

	if (!ok || sigma < 0)
		return 0;

	unsigned halfFilterSize = static_cast<unsigned>(ceil(2.5*sigma));

	ccBilateralFilter* filter = new ccBilateralFilter();
	filter->setParams(halfFilterSize, static_cast<float>(sigma), 0);

	return filter;
}
