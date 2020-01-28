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
#include "codeEditor.h"

//qCC
#include "ccMainAppInterface.h"

//Qt
#include <QMainWindow>
#include <QtWidgets>

//global static pointer (as there should only be one instance of chaiScriptCodeEditorMainWindow!)
static chaiScriptCodeEditorMainWindow* s_instance = nullptr;

chaiScriptCodeEditorMainWindow::chaiScriptCodeEditorMainWindow() : Ui::chaiScriptCodeEditorMainWindow()
{
	setupUi(this);
	mdiArea = new QMdiArea(this);
	setCentralWidget(mdiArea);
	mdiArea->showMaximized();
	//PBtreeView->showMaximized();
	connect(mdiArea, &QMdiArea::subWindowActivated,
		this, &chaiScriptCodeEditorMainWindow::updateMenus);

	createActions();
	createStatusBar();
	updateMenus();

	readSettings();
	newFile();
}

void chaiScriptCodeEditorMainWindow::closeEvent(QCloseEvent* event)
{
	mdiArea->closeAllSubWindows();
	if (mdiArea->currentSubWindow()) 
	{
		event->ignore();
	}
	else 
	{
		emit destroy_chai();
		writeSettings();
		event->accept();
	}
}

void chaiScriptCodeEditorMainWindow::newFile()
{
	CodeEditor* child = createChildCodeEditor();
	child->newFile();
	child->show();
}

void chaiScriptCodeEditorMainWindow::open()
{
	const QString fileName = QFileDialog::getOpenFileName(this);
	if (!fileName.isEmpty())
	{
		openFile(fileName);
	}
}

void chaiScriptCodeEditorMainWindow::changeEvent(QEvent* e)
{
	QWidget::changeEvent(e);
	switch (e->type())
	{
	case QEvent::LanguageChange:
		retranslateUi(this);
		break;
	default:
		break;
	}
}

chaiScriptCodeEditorMainWindow* chaiScriptCodeEditorMainWindow::TheInstance()
{
	if (!s_instance)
	{
		s_instance = new chaiScriptCodeEditorMainWindow();
	}
	return s_instance;
}

bool chaiScriptCodeEditorMainWindow::openFile(const QString& fileName)
{
	if (QMdiSubWindow* existing = findChildCodeEditor(fileName))
	{
		mdiArea->setActiveSubWindow(existing);
		return true;
	}
	const bool succeeded = loadFile(fileName);
	if (succeeded)
	{
		statusBar()->showMessage(tr("File loaded"), 2000);
	}
	return succeeded;
}

bool chaiScriptCodeEditorMainWindow::loadFile(const QString& fileName)
{
	CodeEditor* child = createChildCodeEditor();
	const bool succeeded = child->loadFile(fileName);
	if (succeeded)
	{
		child->show();
	}
	else
	{
		child->close();
	}
	chaiScriptCodeEditorMainWindow::prependToRecentFiles(fileName);
	return succeeded;
}

static inline QString recentFilesKey() { return QStringLiteral("recentFileList"); }
static inline QString fileKey() { return QStringLiteral("file"); }


static QStringList readRecentFiles(QSettings& settings)
{
	QStringList result;
	const int count = settings.beginReadArray(recentFilesKey());
	for (int i = 0; i < count; ++i)
	{
		settings.setArrayIndex(i);
		result.append(settings.value(fileKey()).toString());
	}
	settings.endArray();
	return result;
}

static void writeRecentFiles(const QStringList& files, QSettings& settings)
{
	const int count = files.size();
	settings.beginWriteArray(recentFilesKey());
	for (int i = 0; i < count; ++i)
	{
		settings.setArrayIndex(i);
		settings.setValue(fileKey(), files.at(i));
	}
	settings.endArray();
}

bool chaiScriptCodeEditorMainWindow::hasRecentFiles()
{
	QSettings settings(QCoreApplication::organizationName(), settingsApplicationName());
	const int count = settings.beginReadArray(recentFilesKey());
	settings.endArray();
	return count > 0;
}

void chaiScriptCodeEditorMainWindow::prependToRecentFiles(const QString& fileName)
{
	QSettings settings(QCoreApplication::organizationName(), settingsApplicationName());

	const QStringList oldRecentFiles = readRecentFiles(settings);
	QStringList recentFiles = oldRecentFiles;
	recentFiles.removeAll(fileName);
	recentFiles.prepend(fileName);
	if (oldRecentFiles != recentFiles)
		writeRecentFiles(recentFiles, settings);

	setRecentFilesVisible(!recentFiles.isEmpty());
}


void chaiScriptCodeEditorMainWindow::setRecentFilesVisible(bool visible)
{
	recentFileSubMenuAct->setVisible(visible);
	recentFileSeparator->setVisible(visible);
}

void chaiScriptCodeEditorMainWindow::updateRecentFileActions()
{
	QSettings settings(QCoreApplication::organizationName(), settingsApplicationName());

	const QStringList recentFiles = readRecentFiles(settings);
	const int count = qMin(int(MaxRecentFiles), recentFiles.size());
	int i = 0;
	for (; i < count; ++i)
	{
		const QString fileName = QFileInfo(recentFiles.at(i)).fileName();
		recentFileActs[i]->setText(tr("&%1 %2").arg(i + 1).arg(fileName));
		recentFileActs[i]->setData(recentFiles.at(i));
		recentFileActs[i]->setVisible(true);
	}
	for (; i < MaxRecentFiles; ++i)
	{
		recentFileActs[i]->setVisible(false);
	}
}

void chaiScriptCodeEditorMainWindow::openRecentFile()
{
	if (const QAction* action = qobject_cast<const QAction*>(sender()))
	{
		openFile(action->data().toString());
	}
}

void chaiScriptCodeEditorMainWindow::save()
{
	if (activeChildCodeEditor() && activeChildCodeEditor()->save())
	{
		statusBar()->showMessage(tr("File saved"), 2000);
	}
}

void chaiScriptCodeEditorMainWindow::saveAs()
{
	CodeEditor* child = activeChildCodeEditor();
	if (child && child->saveAs())
	{
		statusBar()->showMessage(tr("File saved"), 2000);
		chaiScriptCodeEditorMainWindow::prependToRecentFiles(child->currentFile());
	}
}

#ifndef QT_NO_CLIPBOARD
void chaiScriptCodeEditorMainWindow::cut()
{
	if (activeChildCodeEditor())
	{
		activeChildCodeEditor()->cut();
	}
}

void chaiScriptCodeEditorMainWindow::copy()
{
	if (activeChildCodeEditor())
	{
		activeChildCodeEditor()->copy();
	}
}

void chaiScriptCodeEditorMainWindow::paste()
{
	if (activeChildCodeEditor())
	{
		activeChildCodeEditor()->paste();
	}
}
#endif

void chaiScriptCodeEditorMainWindow::about()
{

}

void chaiScriptCodeEditorMainWindow::createActions()
{
	connect(actionNew, &QAction::triggered, this, &chaiScriptCodeEditorMainWindow::newFile);
	connect(actionSave, &QAction::triggered, this, &chaiScriptCodeEditorMainWindow::save);
	connect(actionOpen, &QAction::triggered, this, &chaiScriptCodeEditorMainWindow::open);
	connect(actionSave_As, &QAction::triggered, this, &chaiScriptCodeEditorMainWindow::saveAs);
	connect(actionRun, &QAction::triggered, this, &chaiScriptCodeEditorMainWindow::runExecute);
	connect(actionSave_Chai_state, &QAction::triggered, this, [=]() { emit save_Chai_state(); });
	connect(actionReset_Chai_to_initial_state, &QAction::triggered, this, [=]() { emit reset_Chai_to_initial_state(); });
	connect(actionReset_chai_to_last_save, &QAction::triggered, this, [=]() { emit reset_chai_to_last_save(); });

	menuFile->addSeparator();

	QMenu* recentMenu = menuFile->addMenu(tr("Recent..."));
	connect(recentMenu, &QMenu::aboutToShow, this, &chaiScriptCodeEditorMainWindow::updateRecentFileActions);
	recentFileSubMenuAct = recentMenu->menuAction();

	for (int i = 0; i < MaxRecentFiles; ++i)
	{
		recentFileActs[i] = recentMenu->addAction(QString(), this, &chaiScriptCodeEditorMainWindow::openRecentFile);
		recentFileActs[i]->setVisible(false);
	}

	recentFileSeparator = menuFile->addSeparator();

	setRecentFilesVisible(chaiScriptCodeEditorMainWindow::hasRecentFiles());
	menuFile->addSeparator();

	actionCu_t->setShortcuts(QKeySequence::Cut);
	action_Copy->setShortcuts(QKeySequence::Copy);
	action_Paste->setShortcuts(QKeySequence::Paste);
	connect(actionCu_t, &QAction::triggered, this, &chaiScriptCodeEditorMainWindow::cut);
	connect(action_Copy, &QAction::triggered, this, &chaiScriptCodeEditorMainWindow::copy);
	connect(action_Paste, &QAction::triggered, this, &chaiScriptCodeEditorMainWindow::paste);

	windowMenu = menuBar()->addMenu(tr("&Window"));
	connect(windowMenu, &QMenu::aboutToShow, this, &chaiScriptCodeEditorMainWindow::updateWindowMenu);

	closeAct = new QAction(tr("Cl&ose"), this);
	closeAct->setStatusTip(tr("Close the active window"));
	connect(closeAct, &QAction::triggered,
		mdiArea, &QMdiArea::closeActiveSubWindow);

	closeAllAct = new QAction(tr("Close &All"), this);
	closeAllAct->setStatusTip(tr("Close all the windows"));
	connect(closeAllAct, &QAction::triggered, mdiArea, &QMdiArea::closeAllSubWindows);

	tileAct = new QAction(tr("&Tile"), this);
	tileAct->setStatusTip(tr("Tile the windows"));
	connect(tileAct, &QAction::triggered, mdiArea, &QMdiArea::tileSubWindows);

	cascadeAct = new QAction(tr("&Cascade"), this);
	cascadeAct->setStatusTip(tr("Cascade the windows"));
	connect(cascadeAct, &QAction::triggered, mdiArea, &QMdiArea::cascadeSubWindows);

	nextAct = new QAction(tr("Ne&xt"), this);
	nextAct->setShortcuts(QKeySequence::NextChild);
	nextAct->setStatusTip(tr("Move the focus to the next window"));
	connect(nextAct, &QAction::triggered, mdiArea, &QMdiArea::activateNextSubWindow);

	previousAct = new QAction(tr("Pre&vious"), this);
	previousAct->setShortcuts(QKeySequence::PreviousChild);
	previousAct->setStatusTip(tr("Move the focus to the previous "
		"window"));
	connect(previousAct, &QAction::triggered, mdiArea, &QMdiArea::activatePreviousSubWindow);

	windowMenuSeparatorAct = new QAction(this);
	windowMenuSeparatorAct->setSeparator(true);

	updateWindowMenu();

	menuBar()->addSeparator();

	QMenu* helpMenu = menuBar()->addMenu(tr("&Help"));

	QAction* aboutAct = helpMenu->addAction(tr("&About"), this, &chaiScriptCodeEditorMainWindow::about);
	aboutAct->setStatusTip(tr("Show the application's About box"));


}



void chaiScriptCodeEditorMainWindow::createStatusBar()
{
	statusBar()->showMessage(tr("Ready"));
}





void chaiScriptCodeEditorMainWindow::updateMenus()
{
	bool hasChildCodeEditor = (activeChildCodeEditor() != 0);
	actionSave->setEnabled(hasChildCodeEditor);
	actionSave_As->setEnabled(hasChildCodeEditor);
	actionRun->setEnabled(hasChildCodeEditor);
	closeAct->setEnabled(hasChildCodeEditor);
	closeAllAct->setEnabled(hasChildCodeEditor);
	tileAct->setEnabled(hasChildCodeEditor);
	cascadeAct->setEnabled(hasChildCodeEditor);
	nextAct->setEnabled(hasChildCodeEditor);
	previousAct->setEnabled(hasChildCodeEditor);
	windowMenuSeparatorAct->setVisible(hasChildCodeEditor);

#ifndef QT_NO_CLIPBOARD
	action_Paste->setEnabled(hasChildCodeEditor);
	bool hasSelection = (activeChildCodeEditor() &&
		activeChildCodeEditor()->textCursor().hasSelection());
	actionCu_t->setEnabled(hasSelection);
	action_Copy->setEnabled(hasSelection);
#endif
}

void chaiScriptCodeEditorMainWindow::updateWindowMenu()
{
	windowMenu->clear();
	windowMenu->addAction(closeAct);
	windowMenu->addAction(closeAllAct);
	windowMenu->addSeparator();
	windowMenu->addAction(tileAct);
	windowMenu->addAction(cascadeAct);
	windowMenu->addSeparator();
	windowMenu->addAction(nextAct);
	windowMenu->addAction(previousAct);
	windowMenu->addAction(windowMenuSeparatorAct);

	QList<QMdiSubWindow*> windows = mdiArea->subWindowList();
	windowMenuSeparatorAct->setVisible(!windows.isEmpty());

	for (int i = 0; i < windows.size(); ++i) {
		QMdiSubWindow* mdiSubWindow = windows.at(i);
		CodeEditor* child = qobject_cast<CodeEditor*>(mdiSubWindow->widget());

		QString text;
		if (i < 9) {
			text = tr("&%1 %2").arg(i + 1)
				.arg(child->userFriendlyCurrentFile());
		}
		else {
			text = tr("%1 %2").arg(i + 1)
				.arg(child->userFriendlyCurrentFile());
		}
		QAction* action = windowMenu->addAction(text, mdiSubWindow, [this, mdiSubWindow]() {
			mdiArea->setActiveSubWindow(mdiSubWindow);
			});
		action->setCheckable(true);
		action->setChecked(child == activeChildCodeEditor());
	}
}


CodeEditor* chaiScriptCodeEditorMainWindow::createChildCodeEditor()
{
	CodeEditor* child = new CodeEditor;
	mdiArea->addSubWindow(child);

#ifndef QT_NO_CLIPBOARD
	connect(child, &QPlainTextEdit::copyAvailable, actionCu_t, &QAction::setEnabled);
	connect(child, &QPlainTextEdit::copyAvailable, action_Copy, &QAction::setEnabled);
#endif

	return child;
}


void chaiScriptCodeEditorMainWindow::readSettings()
{
	QSettings settings(QCoreApplication::organizationName(), settingsApplicationName());
	const QByteArray geometry = settings.value("geometry", QByteArray()).toByteArray();
	if (geometry.isEmpty())
	{
		const QRect availableGeometry = QApplication::desktop()->availableGeometry(this);
		resize(availableGeometry.width() / 3, availableGeometry.height() / 2);
		move((availableGeometry.width() - width()) / 2,
			(availableGeometry.height() - height()) / 2);
	}
	else
	{
		restoreGeometry(geometry);
	}
}

void chaiScriptCodeEditorMainWindow::writeSettings()
{
	QSettings settings(QCoreApplication::organizationName(), settingsApplicationName());
	settings.setValue("geometry", saveGeometry());
}

CodeEditor* chaiScriptCodeEditorMainWindow::activeChildCodeEditor() const
{
	if (QMdiSubWindow* activeSubWindow = mdiArea->activeSubWindow())
	{
		return qobject_cast<CodeEditor*>(activeSubWindow->widget());
	}
	return nullptr;
}

QMdiSubWindow* chaiScriptCodeEditorMainWindow::findChildCodeEditor(const QString& fileName) const
{
	QString canonicalFilePath = QFileInfo(fileName).canonicalFilePath();

	foreach(QMdiSubWindow * window, mdiArea->subWindowList())
	{
		CodeEditor* mdiChild = qobject_cast<CodeEditor*>(window->widget());
		if (mdiChild->currentFile() == canonicalFilePath)
		{
			return window;
		}
	}
	return nullptr;
}

QString chaiScriptCodeEditorMainWindow::settingsApplicationName()
{

	return QString(QCoreApplication::applicationName()).append(":ChaiScriptPlugin");
}

void chaiScriptCodeEditorMainWindow::runExecute()
{
	if (activeChildCodeEditor())
	{
		emit executionCalled(activeChildCodeEditor()->toPlainText().toUtf8().constData());
	}
}

