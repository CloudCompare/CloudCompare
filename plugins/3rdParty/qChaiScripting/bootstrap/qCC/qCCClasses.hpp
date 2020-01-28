#ifndef CHAISCRIPTING_BOOTSTRAP_QCC_CLASSES_HPP
#define CHAISCRIPTING_BOOTSTRAP_QCC_CLASSES_HPP

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


#include <chaiscript/chaiscript.hpp>
#include <chaiscript/utility/utility.hpp>
#include <ccMainAppInterface.h>
#include <ccHObject.h>
#include <QtGui>
#include <QString>
#include <ccGLWindow.h>
#include <ccOverlayDialog.h>
#include <ccColorScalesManager.h>
#include <ccPickingHub.h>

namespace chaiscript
{
	namespace cloudCompare
	{
		namespace qCC
		{
			
			ModulePtr bs_ccMainAppInterface(ModulePtr m = std::make_shared<Module>())
			{
				m->add(chaiscript::user_type<ccMainAppInterface>(), "ccMainAppInterface");
				m->add(fun([](ccMainAppInterface* ma, ccHObject* p1) {ma->addToDB(p1); }), "addToDB");
				m->add(fun([](ccMainAppInterface* ma, ccHObject* p1, bool b1) {ma->addToDB(p1, b1); }), "addToDB");
				m->add(fun([](ccMainAppInterface* ma, ccHObject* p1, bool b1, bool b2) {ma->addToDB(p1, b1, b2); }), "addToDB");
				m->add(fun([](ccMainAppInterface* ma, ccHObject* p1, bool b1, bool b2, bool b3) {ma->addToDB(p1, b1, b2, b3); }), "addToDB");
				m->add(fun([](ccMainAppInterface* ma, ccHObject* p1, bool b1, bool b2, bool b3, bool b4) {ma->addToDB(p1, b1, b2, b3, b4); }), "addToDB");
				m->add(fun([](ccMainAppInterface* ma, const QStringList& filenames, QString fileFilter, ccGLWindow* destWin) {ma->addToDB(filenames, fileFilter, destWin); }), "addToDB");
				m->add(fun([](ccMainAppInterface* ma, const QStringList& filenames, QString fileFilter) {ma->addToDB(filenames, fileFilter); }), "addToDB");
				m->add(fun([](ccMainAppInterface* ma, const QStringList& filenames) {ma->addToDB(filenames); }), "addToDB");
				m->add(fun([](ccMainAppInterface* ma, const std::string& filename) {ma->addToDB(QStringList(QString::fromUtf8(filename.c_str()))); }), "addToDB");
				//m->add(fun([](ccMainAppInterface* ma, const std::vector<std::string&> filenames, QString fileFilter, ccGLWindow* destWin) {ma->addToDB(filenames, fileFilter, destWin); }), "addToDB");
				//m->add(fun([](ccMainAppInterface* ma, const std::vector<std::string&> filenames, QString fileFilter) {ma->addToDB(filenames, fileFilter, null_ptr); }), "addToDB");
				//m->add(fun([](ccMainAppInterface* ma, const std::vector<std::string&> {ma->addToDB(filenames, QString(), null_ptr); }), "addToDB");
				m->add(fun([](ccMainAppInterface* ma, std::string str) { ma->dispToConsole(QString::fromUtf8(str.c_str())); }), "dispToConsole");
				m->add(fun([](ccMainAppInterface* ma, std::string str, ccMainAppInterface::ConsoleMessageLevel ml) { ma->dispToConsole(QString::fromUtf8(str.c_str()), ml); }), "dispToConsole");
				m->add(fun(&ccMainAppInterface::dispToConsole), "dispToConsole");
				m->add(fun(&ccMainAppInterface::getMainWindow), "getMainWindow");
				m->add(fun(&ccMainAppInterface::getActiveGLWindow), "getActiveGLWindow");
				m->add(fun(&ccMainAppInterface::createGLWindow), "createGLWindow");
				m->add(fun(&ccMainAppInterface::destroyGLWindow), "destroyGLWindow");
				m->add(fun(&ccMainAppInterface::registerOverlayDialog), "registerOverlayDialog");
				m->add(fun(&ccMainAppInterface::unregisterOverlayDialog), "unregisterOverlayDialog");
				m->add(fun(&ccMainAppInterface::updateOverlayDialogsPlacement), "updateOverlayDialogsPlacement");
				m->add(fun(&ccMainAppInterface::getUniqueIDGenerator), "getUniqueIDGenerator");
				m->add(fun(&ccMainAppInterface::registerOverlayDialog), "registerOverlayDialog");
				m->add(fun([](ccMainAppInterface* ma, ccHObject* p1, bool b1) {ma->removeFromDB(p1, b1); }), "removeFromDB");
				m->add(fun([](ccMainAppInterface* ma, ccHObject* p1) {ma->removeFromDB(p1); }), "removeFromDB");
				m->add(fun(&ccMainAppInterface::removeObjectTemporarilyFromDBTree), "removeObjectTemporarilyFromDBTree");
				m->add(fun(&ccMainAppInterface::putObjectBackIntoDBTree), "putObjectBackIntoDBTree");
				m->add(fun(&ccMainAppInterface::setSelectedInDB), "setSelectedInDB");
				m->add(fun(&ccMainAppInterface::getSelectedEntities), "getSelectedEntities");
				m->add(fun(&ccMainAppInterface::haveSelection), "haveSelection");
				m->add(fun(&ccMainAppInterface::haveOneSelection), "haveOneSelection");
				m->add(fun(&ccMainAppInterface::forceConsoleDisplay), "forceConsoleDisplay");
				m->add(fun(&ccMainAppInterface::dbRootObject), "dbRootObject");
				m->add(fun(&ccMainAppInterface::redrawAll), "redrawAll");
				m->add(fun([](ccMainAppInterface* ma) { ma->redrawAll(); }), "redrawAll");
				m->add(fun(&ccMainAppInterface::refreshAll), "refreshAll");
				m->add(fun([](ccMainAppInterface* ma) { ma->refreshAll(); }), "refreshAll");
				m->add(fun(&ccMainAppInterface::enableAll), "enableAll");
				m->add(fun(&ccMainAppInterface::disableAll), "disableAll");
				m->add(fun(&ccMainAppInterface::disableAllBut), "disableAllBut");
				m->add(fun(&ccMainAppInterface::updateUI), "updateUI");
				m->add(fun(&ccMainAppInterface::freezeUI), "freezeUI");
				m->add(fun(&ccMainAppInterface::getColorScalesManager), "getColorScalesManager");
				m->add(fun(&ccMainAppInterface::spawnHistogramDialog), "spawnHistogramDialog");
				m->add(fun(&ccMainAppInterface::pickingHub), "pickingHub");
				m->add(fun(&ccMainAppInterface::setView), "setView");
				m->add(fun(&ccMainAppInterface::toggleActiveWindowCenteredPerspective), "toggleActiveWindowCenteredPerspective");
				m->add(fun(&ccMainAppInterface::toggleActiveWindowCustomLight), "toggleActiveWindowCustomLight");
				m->add(fun(&ccMainAppInterface::toggleActiveWindowSunLight), "toggleActiveWindowSunLight");
				m->add(fun(&ccMainAppInterface::toggleActiveWindowViewerBasedPerspective), "toggleActiveWindowViewerBasedPerspective");
				m->add(fun(&ccMainAppInterface::zoomOnSelectedEntities), "zoomOnSelectedEntities");
				m->add(fun(&ccMainAppInterface::setGlobalZoom), "setGlobalZoom");
				m->add(fun(&ccMainAppInterface::increasePointSize), "increasePointSize");
				m->add(fun(&ccMainAppInterface::decreasePointSize), "decreasePointSize");
				return m;
			}



			ModulePtr bootstrap_classes(ModulePtr m = std::make_shared<Module>())
			{
				bs_ccMainAppInterface(m);
				return m;
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_QCC_CLASSES_HPP