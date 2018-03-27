//##########################################################################
//#                                                                        #
//#            CLOUDCOMPARE PLUGIN: qExampleGLPlugin                       #
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

#include <QInputDialog>
#include <QtMath>

#include "ccBilateralFilter.h"

#include "qExampleGLPlugin.h"


qExampleGLPlugin::qExampleGLPlugin( QObject *parent ) :
	QObject( parent )
  , ccGLFilterPluginInterface( ":/CC/plugin/qExampleGLPlugin/info.json" )
{
}

ccGlFilter *qExampleGLPlugin::getFilter()
{
	bool ok = false;
	double sigma = QInputDialog::getDouble( 0,
											"Bilateral filter",
											"Sigma (pixel)",
											1.0,
											0.1, 8.0,
											1,
											&ok );

	if (!ok || sigma < 0)
	{
		return nullptr;
	}

	unsigned int halfFilterSize = static_cast<unsigned int>(qCeil( 2.5 * sigma ));

	ccBilateralFilter* filter = new ccBilateralFilter;
	
	filter->setParams( halfFilterSize, static_cast<float>(sigma), 0.0f );

	return filter;
}
