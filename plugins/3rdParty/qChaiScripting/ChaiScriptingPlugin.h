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
#include <ccCommandLineInterface.h>
#include <chaiscript/chaiscript.hpp>

static const char COMMAND_CHAI[] = "CHAI";
static const char COMMAND_CHAIKILL[] = "KILLCHAI";
class ChaiScriptingPlugin : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.ChaiScripting" FILE "info.json")
	
public:
	static ChaiScriptingPlugin* TheInstance();
	template<typename T, typename... Args>
	std::unique_ptr<T> make_unique(Args&&... args)
	{
		return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
	}


	explicit ChaiScriptingPlugin( QObject *parent = nullptr );
	~ChaiScriptingPlugin();// override = default;

	QList<QAction *> getActions() override;
	virtual void registerCommands(ccCommandLineInterface* cmd) override;

	struct CommandCHAI : public ccCommandLineInterface::Command
	{
		CommandCHAI() : ccCommandLineInterface::Command(COMMAND_CHAI, COMMAND_CHAI) {}

		virtual bool process(ccCommandLineInterface& cmd) override
		{
			cmd.print(QString("[%1]").arg(COMMAND_CHAI));
			if (cmd.arguments().empty())
			{
				return cmd.error(QString("Missing parameter: parameters filename after \"-%1\"").arg(COMMAND_CHAI));
			}

			//open specified file
			QString paramFilename(cmd.arguments().takeFirst());
			cmd.print(QString("Parameters file: '%1'").arg(paramFilename));
			return ChaiScriptingPlugin::TheInstance()->loadChaiFile(paramFilename);
		}
	};

	struct CommandCHAIKILL : public ccCommandLineInterface::Command
	{
		CommandCHAIKILL() : ccCommandLineInterface::Command(COMMAND_CHAIKILL, COMMAND_CHAIKILL) {}

		virtual bool process(ccCommandLineInterface& cmd) override
		{
			cmd.print(QString("[%1]").arg(COMMAND_CHAIKILL));
			ChaiScriptingPlugin::TheInstance()->destroyChai();
			return true;
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
				cmd.print(QString("[%1] chai_process is not implemented in script").arg(m_keyword));
				return false;
			}
		}
	};

	void setupChaiScriptEngine();
	void destroyChai();
	bool loadChaiFile(const QString& file);
	void registerCommand(CommandCHAIDerived newCmd);

private:

	void openScriptEditor();
	void dispToConsole(const std::string &str, const int lvl);

	void executionCalled(const std::string& evalStatement);
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

#endif
