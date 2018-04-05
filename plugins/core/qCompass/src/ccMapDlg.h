//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: ccCompass                      #
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
//#                     COPYRIGHT: Sam Thiele  2017                        #
//#                                                                        #
//##########################################################################

#ifndef CC_MAP_DIALOG_HEADER
#define CC_MAP_DIALOG_HEADER

//Qt
#include <QDialog>
#include <QList>
#include <QAction>

//CC
#include <ccGLWindow.h>
#include <ccOverlayDialog.h>

//Local
#include <ui_mapDlg.h>
#include "ccTrace.h"

//class encapsulating the map-mode overlay dialog
class ccMapDlg : public ccOverlayDialog, public Ui::mapDlg
{
	Q_OBJECT

public:
	//! Default constructor
	explicit ccMapDlg(QWidget* parent = 0);

	//menus
	QMenu *m_createObject_menu;

	//actions
	QAction *m_create_geoObject; //create a normal GeoObject
	QAction *m_create_geoObjectSS; //create a single surface GeoObject

};

#endif
