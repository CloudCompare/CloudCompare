#pragma once

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
#include <ccCommandLineInterface.h>
#include <chaiscript/chaiscript.hpp>

static const char COMMAND_CHAI[] = "CHAI";
static const char COMMAND_CHAIKILL[] = "KILLCHAI";
class ChaiScriptingPlugin : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccPluginInterface ccStdPluginInterface)
	
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qChaiScripting" FILE "info.json")
	
public:
	static ChaiScriptingPlugin* TheInstance();


	explicit ChaiScriptingPlugin( QObject *parent = nullptr );
	~ChaiScriptingPlugin();// override = default;

	QList<QAction *> getActions() override;
	virtual void stop() override;
	virtual void registerCommands(ccCommandLineInterface* cmd) override;

	struct CommandCHAI : public ccCommandLineInterface::Command
	{
		CommandCHAI() : ccCommandLineInterface::Command("ChaiScript file evaluator", COMMAND_CHAI) {}

		virtual bool process(ccCommandLineInterface& cmd) override
		{
			cmd.print(QString("[%1]").arg(COMMAND_CHAI));
			if (cmd.arguments().empty())
			{
				ChaiScriptingPlugin::TheInstance()->destroyChai();
				return cmd.error(QString("Missing parameter: script filename after \"-%1\"").arg(COMMAND_CHAI));
			}

			//open specified file
			QString paramFilename(cmd.arguments().takeFirst());
			cmd.print(QString("Script file: '%1'").arg(paramFilename));
			return ChaiScriptingPlugin::TheInstance()->loadChaiFile(paramFilename);
		}
	};


	struct CommandCHAIDerived : public ccCommandLineInterface::Command
	{
		CommandCHAIDerived(const QString& name, const QString& keyword) : ccCommandLineInterface::Command(name, keyword) {}

		CommandCHAIDerived(const QString& name, const QString& keyword, std::function<bool(ccCommandLineInterface&)> cp) : ccCommandLineInterface::Command(name, keyword) { chai_process = cp; }

		std::function<bool(ccCommandLineInterface&)> chai_process;

		virtual bool process(ccCommandLineInterface& cmd) override
		{
			if (chai_process)
			{
				cmd.print(QString("[%1]").arg(m_keyword));
				return chai_process(cmd);
			}
			else
			{
				cmd.error(QString("[%1] chai_process is not implemented in script").arg(m_keyword));
				ChaiScriptingPlugin::TheInstance()->destroyChai();
				return false;
			}
		}
	};


	void setupChaiScriptEngine();
	void destroyChai();
	bool loadChaiFile(const QString& file);
	void registerCommand(CommandCHAIDerived newCmd);
	void onNewSelection(const ccHObject::Container& selectedEntities) override;
	//std::function<void(const ccHObject::Container&)> chai_onNewSelection;
	std::function<void(std::vector<std::shared_ptr<ccHObject>>)> chai_onNewSelection;
	
private:

	void openScriptEditor();
	void dispToConsole(const std::string &str, const int lvl);

	void executionCalled(const std::string& evalFileName, const std::string& evalStatement);
	void resetToDefaultChaiState();
	void saveChaiState();
	void resetToSavedChaiState();
	std::string chaiSystemDump();
	std::map<std::string, chaiscript::Boxed_Value> getChaiObjects();
	ccCommandLineInterface* storedCMDLine;
	//! Default action
	QAction* m_action;
	

	std::unique_ptr<chaiscript::ChaiScript> chai;
	chaiscript::ModulePtr systemBootstrap = nullptr;// = chaiscript::cloudCompare::bootstrapSystem::bootstrap();

		/// \brief Represents the current state of the ChaiScript system. State and be saved and restored
		/// \warning State object does not contain the user defined type conversions of the engine. They
		///          are left out due to performance considerations involved in tracking the state
		/// \sa ChaiScript::get_state
		/// \sa ChaiScript::set_state
	chaiscript::ChaiScript::State chaiInitialState;
	chaiscript::ChaiScript::State chaiSaveState;
	
	
	
};


