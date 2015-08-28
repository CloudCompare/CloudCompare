//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qFacets                       #
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
//#                           COPYRIGHT: BRGM                              #
//#                                                                        #
//##########################################################################

#ifndef QFACET_CLASSIFICATION_PARAMS_DLG_HEADER
#define QFACET_CLASSIFICATION_PARAMS_DLG_HEADER

#include "ui_classificationParamsDlg.h"

//! Dialog for orientation-based classification of facets (qFacets plugin)
class ClassificationParamsDlg : public QDialog, public Ui::ClassificationParamsDlg
{
public:

	//! Default constructor
	ClassificationParamsDlg(QWidget* parent = 0)
		: QDialog(parent)
		, Ui::ClassificationParamsDlg()
	{
		setupUi(this);
		setWindowFlags(Qt::Tool);
	}
};

#endif //QFACET_CLASSIFICATION_PARAMS_DLG_HEADER
