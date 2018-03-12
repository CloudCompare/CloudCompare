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

#include "ccMapDlg.h"

//Local
#include "ccGLWindow.h"

//qCC_db
#include <ccLog.h>

//Qt
#include <QEvent>
#include <QKeyEvent>
#include <QApplication>
#include <qmenu.h>
#include <qaction.h>

//system
#include <assert.h>

ccMapDlg::ccMapDlg(QWidget* parent/*=0*/)
	: ccOverlayDialog(parent)
	, Ui::mapDlg()
{
	setupUi(this);

	//set background color
	QPalette p;
	p.setColor(backgroundRole(), QColor(240, 240, 240, 200));
	setPalette(p);
	setAutoFillBackground(true);

	//create menus
	m_createObject_menu = new QMenu(this);
	addObjectButton->setMenu(m_createObject_menu);

	//create actions
	m_create_geoObject = new QAction("GeoObject", this);
	m_create_geoObjectSS = new QAction("Single Surface GeoObject", this);

	//assign tool tips
	m_create_geoObject->setToolTip("Create a GeoObject with upper and lower surfaces and an interior.");
	m_create_geoObjectSS->setToolTip("Create a GeoObject with only a single surface ('interior').");

	//add to menu
	m_createObject_menu->addAction(m_create_geoObject);
	m_createObject_menu->addAction(m_create_geoObjectSS);
}
