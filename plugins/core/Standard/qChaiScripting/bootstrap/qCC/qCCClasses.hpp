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
#include <ccCommandLineInterface.h>
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
				m->add(chaiscript::user_type<ccMainAppInterface::ccHObjectContext>(), "ccHObjectContext");
				m->add(fun(&ccMainAppInterface::ccHObjectContext::parent), "parent");
				m->add(fun(&ccMainAppInterface::ccHObjectContext::childFlags), "childFlags");
				m->add(fun(&ccMainAppInterface::ccHObjectContext::parentFlags), "parentFlags");
				return m;
			}

			ModulePtr bs_ccCommandLineInterface(ModulePtr m = std::make_shared<Module>())
			{
				m->add(chaiscript::user_type<CLEntityDesc>(), "CLEntityDesc");
				m->add(fun(&CLEntityDesc::basename), "basename");
				m->add(fun(&CLEntityDesc::path), "path");
				m->add(fun(&CLEntityDesc::indexInFile), "indexInFile");
				m->add(fun(static_cast<ccHObject*(CLEntityDesc::*)()>(&CLEntityDesc::getEntity)), "getEntity");
				m->add(fun(static_cast<const ccHObject*(CLEntityDesc::*)()const>(&CLEntityDesc::getEntity)), "getEntity");

				m->add(chaiscript::user_type<CLGroupDesc>(), "CLGroupDesc");
				m->add(chaiscript::constructor<CLGroupDesc(ccHObject*, const QString&, const QString&)>(), "CLGroupDesc");
				m->add(fun(&CLGroupDesc::groupEntity), "groupEntity");
				m->add(fun(static_cast<ccHObject * (CLGroupDesc::*)()>(&CLGroupDesc::getEntity)), "getEntity");
				m->add(fun(static_cast<const ccHObject * (CLGroupDesc::*)()const>(&CLGroupDesc::getEntity)), "getEntity");


				m->add(chaiscript::user_type<CLCloudDesc>(), "CLCloudDesc");
				m->add(chaiscript::constructor<CLCloudDesc()>(), "CLCloudDesc");
				m->add(chaiscript::constructor<CLCloudDesc(ccPointCloud*, const QString&, int)>(), "CLCloudDesc");
				m->add(chaiscript::constructor<CLCloudDesc(ccPointCloud*, const QString&, const QString&, int)>(), "CLCloudDesc");
				m->add(fun(&CLCloudDesc::pc), "pc");
				m->add(fun(static_cast<ccHObject * (CLCloudDesc::*)()>(&CLCloudDesc::getEntity)), "getEntity");
				m->add(fun(static_cast<const ccHObject * (CLCloudDesc::*)()const>(&CLCloudDesc::getEntity)), "getEntity");


				m->add(chaiscript::user_type<CLMeshDesc>(), "CLMeshDesc");
				m->add(chaiscript::constructor<CLMeshDesc()>(), "CLMeshDesc");
				m->add(chaiscript::constructor<CLMeshDesc(ccGenericMesh*, const QString&, int)>(), "CLMeshDesc");
				m->add(chaiscript::constructor<CLMeshDesc(ccGenericMesh*, const QString&, const QString&, int)>(), "CLMeshDesc");
				m->add(fun(&CLMeshDesc::mesh), "mesh");
				m->add(fun(static_cast<ccHObject * (CLMeshDesc::*)()>(&CLMeshDesc::getEntity)), "getEntity");
				m->add(fun(static_cast<const ccHObject * (CLMeshDesc::*)()const>(&CLMeshDesc::getEntity)), "getEntity");


				m->add(chaiscript::user_type<ccCommandLineInterface>(), "ccCommandLineInterface"); //TODO ADD CONSTRUCTOR
					m->add(chaiscript::user_type<ccCommandLineInterface::Command>(), "Command");
					
					m->add(chaiscript::user_type<QSharedPointer<ccCommandLineInterface::Command>>(), "SharedCommand");
					m->add(fun(&ccCommandLineInterface::Command::process), "process");
					m->add(fun(&ccCommandLineInterface::Command::m_name), "m_name");
					m->add(fun(&ccCommandLineInterface::Command::m_keyword), "m_keyword");
				m->add(fun(&ccCommandLineInterface::IsCommand), "IsCommand");
				m->add(fun(&ccCommandLineInterface::registerCommand), "registerCommand");
				m->add(fun(&ccCommandLineInterface::getExportFilename), "getExportFilename"); //TODO ADD DEFAULT PARAMS
				m->add(fun(&ccCommandLineInterface::exportEntity), "exportEntity"); //TODO ADD DEFAULT PARAMS
				m->add(fun(&ccCommandLineInterface::saveClouds), "saveClouds"); //TODO ADD DEFAULT PARAMS
				m->add(fun(&ccCommandLineInterface::saveMeshes), "saveMeshes"); //TODO ADD DEFAULT PARAMS
				m->add(fun(&ccCommandLineInterface::removeClouds), "removeClouds"); //TODO ADD DEFAULT PARAMS
				m->add(fun(&ccCommandLineInterface::removeMeshes), "removeMeshes"); //TODO ADD DEFAULT PARAMS
				m->add(fun(static_cast<QStringList&(ccCommandLineInterface::*)()>(&ccCommandLineInterface::arguments)), "arguments");
				m->add(fun(static_cast<const QStringList&(ccCommandLineInterface::*)()const>(&ccCommandLineInterface::arguments)), "arguments");
				m->add(fun(&ccCommandLineInterface::progressDialog), "progressDialog");
				m->add(fun(&ccCommandLineInterface::widgetParent), "widgetParent");
					m->add(chaiscript::user_type<ccCommandLineInterface::CLLoadParameters>(), "CLLoadParameters");
					m->add(chaiscript::constructor<ccCommandLineInterface::CLLoadParameters()>(), "CLLoadParameters");
					m->add(fun(&ccCommandLineInterface::CLLoadParameters::m_coordinatesShiftEnabled), "m_coordinatesShiftEnabled");
					m->add(fun(&ccCommandLineInterface::CLLoadParameters::m_coordinatesShift), "m_coordinatesShift");				
				m->add(fun(&ccCommandLineInterface::fileLoadingParams), "fileLoadingParams");
				m->add(fun(&ccCommandLineInterface::importFile), "importFile"); //TODO ADD DEFAULT PARAMS
				m->add(fun(&ccCommandLineInterface::cloudExportFormat), "cloudExportFormat");
				m->add(fun(&ccCommandLineInterface::cloudExportExt), "cloudExportExt");
				m->add(fun(&ccCommandLineInterface::meshExportFormat), "meshExportFormat");
				m->add(fun(&ccCommandLineInterface::meshExportExt), "meshExportExt");
				m->add(fun(&ccCommandLineInterface::setCloudExportFormat), "setCloudExportFormat");
				m->add(fun(&ccCommandLineInterface::setMeshExportFormat), "setMeshExportFormat");
				m->add(fun(&ccCommandLineInterface::print), "print");
				m->add(fun(&ccCommandLineInterface::warning), "warning");
				m->add(fun(&ccCommandLineInterface::error), "error");
				m->add(fun(static_cast<std::vector<CLCloudDesc>&(ccCommandLineInterface::*)()>(&ccCommandLineInterface::clouds)), "clouds");
				m->add(fun(static_cast<const std::vector<CLCloudDesc>&(ccCommandLineInterface::*)()const>(&ccCommandLineInterface::clouds)), "clouds");
				m->add(fun(static_cast<std::vector<CLMeshDesc> & (ccCommandLineInterface::*)()>(&ccCommandLineInterface::meshes)), "meshes");
				m->add(fun(static_cast<const std::vector<CLMeshDesc> & (ccCommandLineInterface::*)()const>(&ccCommandLineInterface::meshes)), "meshes");
				m->add(fun(&ccCommandLineInterface::toggleSilentMode), "toggleSilentMode");
				m->add(fun(&ccCommandLineInterface::silentMode), "silentMode");
				m->add(fun(&ccCommandLineInterface::toggleAutoSaveMode), "toggleAutoSaveMode");
				m->add(fun(&ccCommandLineInterface::autoSaveMode), "autoSaveMode");
				m->add(fun(&ccCommandLineInterface::toggleAddTimestamp), "toggleAddTimestamp");
				m->add(fun(&ccCommandLineInterface::addTimestamp), "addTimestamp");
				m->add(fun(&ccCommandLineInterface::setNumericalPrecision), "setNumericalPrecision");
				m->add(fun(&ccCommandLineInterface::numericalPrecision), "numericalPrecision");
				m->add(fun(&ccCommandLineInterface::coordinatesShiftWasEnabled), "coordinatesShiftWasEnabled");
				m->add(fun(&ccCommandLineInterface::formerCoordinatesShift), "formerCoordinatesShift");
				m->add(fun(&ccCommandLineInterface::storeCoordinatesShiftParams), "storeCoordinatesShiftParams");
				m->add(fun(&ccCommandLineInterface::nextCommandIsGlobalShift), "nextCommandIsGlobalShift");
				m->add(fun(&ccCommandLineInterface::processGlobalShiftCommand), "processGlobalShiftCommand");



				return m;
			}

			ModulePtr bs_class_relationships(ModulePtr m = std::make_shared<Module>())
			{
				m->add(chaiscript::base_class<CLEntityDesc, CLGroupDesc>());
				m->add(chaiscript::base_class<CLEntityDesc, CLCloudDesc>());
				m->add(chaiscript::base_class<CLEntityDesc, CLMeshDesc>());

				return m;
			}


			ModulePtr bootstrap_classes(ModulePtr m = std::make_shared<Module>())
			{
				bs_ccMainAppInterface(m);
				bs_ccCommandLineInterface(m);

				bs_class_relationships(m);
				return m;
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_QCC_CLASSES_HPP