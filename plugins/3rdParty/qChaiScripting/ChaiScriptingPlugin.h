#ifndef CHAISCRIPTING_PLUGIN_HEADER
#define CHAISCRIPTING_PLUGIN_HEADER

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

#include "ccStdPluginInterface.h"



class ChaiScriptingPlugin : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.ChaiScripting" FILE "info.json")
	
public:
	explicit ChaiScriptingPlugin( QObject *parent = nullptr );
	~ChaiScriptingPlugin();// override = default;

	QList<QAction *> getActions() override;
	
private:
	void setupChaiScriptEngine();

	void openScriptEditor();
	void dispToConsole(const std::string &str, const int lvl);
	void executionCalled(const std::string& evalStatement);
	void resetToDefaultChaiState();
	void saveChaiState();
	void destroyChai();
	void resetToSavedChaiState();
	std::string chaiSystemDump();

	//! Default action
	QAction* m_action;
	
	
};

#endif
