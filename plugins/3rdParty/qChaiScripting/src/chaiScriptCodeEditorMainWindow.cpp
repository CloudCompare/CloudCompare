//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: ChaiScriptingPlugin                #
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
//#                     COPYRIGHT: Chris S Brown                           #
//#                                                                        #
//##########################################################################

#include "chaiScriptCodeEditorMainWindow.h"

//qCC
#include "ccMainAppInterface.h"

//Qt
#include <QMainWindow>

//global static pointer (as there should only be one instance of chaiScriptCodeEditorMainWindow!)
static chaiScriptCodeEditorMainWindow* s_instance = nullptr;

chaiScriptCodeEditorMainWindow::chaiScriptCodeEditorMainWindow() : Ui::chaiScriptCodeEditorMainWindow()
{
	setupUi(this);
}

chaiScriptCodeEditorMainWindow* chaiScriptCodeEditorMainWindow::TheInstance()
{
	if (!s_instance)
		s_instance = new chaiScriptCodeEditorMainWindow();
	return s_instance;
}