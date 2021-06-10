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

class CodeEditor;
QT_BEGIN_NAMESPACE
class QAction;
class QMenu;
class QMdiArea;
class QMdiSubWindow;
QT_END_NAMESPACE



class ccMainAppInterface;


//! chaiScritping plugin's main window
class chaiScriptCodeEditorMainWindow : public QMainWindow, public Ui::chaiScriptCodeEditorMainWindow
{
	Q_OBJECT

public:

	//! Default constructor
	chaiScriptCodeEditorMainWindow();
	void changeEvent(QEvent* e) override;
	static chaiScriptCodeEditorMainWindow* TheInstance();
	bool openFile(const QString& fileName);
	static QString settingsApplicationName();

protected:
	void closeEvent(QCloseEvent* event) override;
signals:
	void executionCalled(const std::string& evalFileName, const std::string &evalStatement);
	void reset_Chai_to_initial_state();
	void save_Chai_state();
	void reset_chai_to_last_save();
	void destroy_chai();


protected slots:
	void newFile();
	void open();
	void save();
	void saveAs();
	void updateRecentFileActions();
	void openRecentFile();
	bool eventFilter(QObject* obj, QEvent* e) override;
#ifndef QT_NO_CLIPBOARD
	void cut();
	void copy();
	void paste();
#endif
	void comment();
	void uncomment();
	void indentMore();
	void indentLess();
	void about();
	void updateMenus();
	void updateWindowMenu();
	CodeEditor* createChildCodeEditor();

private:
	enum { MaxRecentFiles = 10 };

	void createActions();
	void runExecute();
	void createStatusBar();
	void readSettings();
	void writeSettings();
	bool loadFile(const QString& fileName);
	static bool hasRecentFiles();
	void prependToRecentFiles(const QString& fileName);
	void setRecentFilesVisible(bool visible);
	CodeEditor* activeChildCodeEditor() const;
	QMdiSubWindow* findChildCodeEditor(const QString& fileName) const;

	QMdiArea* mdiArea;

	QMenu* windowMenu;

	QAction* recentFileActs[MaxRecentFiles];
	QAction* recentFileSeparator;
	QAction* recentFileSubMenuAct;

	QAction* closeAct;
	QAction* closeAllAct;
	QAction* tileAct;
	QAction* cascadeAct;
	QAction* nextAct;
	QAction* previousAct;
	QAction* windowMenuSeparatorAct;

};

#endif // CHAISCRIPT_CODE_EDITOR_MAIN_WINDOW
