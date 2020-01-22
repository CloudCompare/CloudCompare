#ifndef CHAISCRIPT_CODE_EDITOR_MAIN_WINDOW
#define CHAISCRIPT_CODE_EDITOR_MAIN_WINDOW

//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: ChaiScripting                      #
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

#include <ui_chaiScriptCodeEditorMainWindow.h>


//Qt
#include <QMainWindow>
#include <QSettings>

class ccMainAppInterface;


//! chaiScritping plugin's main window
class chaiScriptCodeEditorMainWindow : public QMainWindow, public Ui::chaiScriptCodeEditorMainWindow
{
	Q_OBJECT

public:

	//! Default constructor
	chaiScriptCodeEditorMainWindow();

	static chaiScriptCodeEditorMainWindow* TheInstance();

protected slots:

	

protected: //methods


protected: //members

	ccMainAppInterface* m_app;

};

#endif //Q_M3C2_DIALOG_HEADER
