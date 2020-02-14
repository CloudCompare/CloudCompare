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


#include <QtGui>

#include "ChaiScriptingPlugin.h"

#include <chaiscript/chaiscript.hpp>
#include <chaiscript/chaiscript_stdlib.hpp>
#include <chaiscript/dispatchkit/bootstrap_stl.hpp>
#include <chaiscript/dispatchkit/function_call.hpp>


#include "bootstrap/systemBootstrap.hpp"

#include "chaiScriptCodeEditorMainWindow.h"



chaiScriptCodeEditorMainWindow* cseMW;
static ChaiScriptingPlugin* s_instance;
using namespace chaiscript;
ChaiScriptingPlugin::ChaiScriptingPlugin( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/ChaiScriptingPlugin/info.json" )
	, m_action( nullptr )
{
	if (!s_instance)
	{
		qAddPostRoutine([]() {ChaiScriptingPlugin::TheInstance()->destroyChai(); });
		s_instance = this;
	}
}

bool throws_exception(const std::function<void()>& f)
{
	try {
		f();
	}
	catch (...) {
		return true;
	}

	return false;
}

const QString stdStringToQString(const std::string &str)
{
	return QString::fromUtf8(str.c_str());
}

chaiscript::exception::eval_error get_eval_error(const std::function<void()>& f)
{
	try {
		f();
	}
	catch (const chaiscript::exception::eval_error & e) {
		return e;
	}

	throw std::runtime_error("no exception throw");
}

void ChaiScriptingPlugin::setupChaiScriptEngine()
{
	try 
	{
		if (!chai)
		{
			QDir  workingDir = QCoreApplication::applicationDirPath();
			
			std::vector<std::string> paths;
			paths.push_back(workingDir.absolutePath().toStdString());
			paths.push_back((workingDir.absolutePath() + QDir::separator() + "chaiScripts" + QDir::separator()).toLocal8Bit().constData());
			
			chai = std::make_unique<chaiscript::ChaiScript>(chaiscript::Std_Lib::library(), paths, paths);;
			if (systemBootstrap == nullptr)
			{
				systemBootstrap = chaiscript::cloudCompare::bootstrapSystem::bootstrap();
			}
			chai->add(systemBootstrap);
			if (m_app)
			{
				chai->add_global(var(m_app), "m_app");
			}
			chai->add(user_type<CommandCHAIDerived>(), "CommandCHAIDerived");
			chai->add(constructor< CommandCHAIDerived(const QString&, const QString&)>(), "CommandCHAIDerived");
			chai->add(fun(&CommandCHAIDerived::chai_process), "chai_process");
			chai->add_global(var(this), "chaiScriptingPlugin");
			chai->add(fun(&ChaiScriptingPlugin::registerCommand), "registerCommand");
			

			chai->add(fun(&ChaiScriptingPlugin::dispToConsole), "dispToConsole");
			chai->add(fun(&ChaiScriptingPlugin::chai_onNewSelection), "chai_onNewSelection");
			chai->add(chaiscript::fun(&throws_exception), "throws_exception");
			chai->add(chaiscript::fun(&get_eval_error), "get_eval_error");
			chai->eval(R"(global print = fun(x){chaiScriptingPlugin.dispToConsole(x.to_string(),0);})");
			chai->eval(R"(def method_missing(int i, string name, Vector v) {
					print("method_missing(${i}, ${name}), ${v.size()} params");
					})");
			chaiInitialState = chai->get_state();
		}
	}
	catch (const chaiscript::exception::eval_error & ee)
	{
		ccLog::Error(ee.what());
		if (ee.call_stack.size() > 0)
		{
			ccLog::Warning(QString("during evaluation at (%1, %2)").arg(ee.call_stack[0]->start().line).arg(ee.call_stack[0]->start().column));
		}
		std::cout << std::endl;
	}
	catch (const std::exception & e)
	{
		ccLog::Error(e.what());
	}

}


void ChaiScriptingPlugin::dispToConsole(const std::string &str,const int lvl)
{
	
	switch ((ccMainAppInterface::ConsoleMessageLevel)lvl)
	{
		case ccMainAppInterface::ConsoleMessageLevel::STD_CONSOLE_MESSAGE:
			ccLog::Print(QString::fromStdString(str));
			break;
		case ccMainAppInterface::ConsoleMessageLevel::WRN_CONSOLE_MESSAGE:
			ccLog::Warning(QString::fromStdString(str));
			break;
		case ccMainAppInterface::ConsoleMessageLevel::ERR_CONSOLE_MESSAGE:
			ccLog::Error(QString::fromStdString(str));
			break;
	default:
		ccLog::Print(QString::fromStdString(str));
		break;
	}
}



void ChaiScriptingPlugin::executionCalled(const std::string& evalFileName, const std::string& evalStatement)
{
	try
	{
		chai->eval(evalStatement, nullptr, evalFileName);
	}
	catch (const chaiscript::exception::eval_error & ee)
	{
		ccLog::Error(ee.what());
		if (ee.call_stack.size() > 0)
		{
			ccLog::Warning(QString("during evaluation at (%1, %2)").arg(ee.call_stack[0]->start().line).arg(ee.call_stack[0]->start().column));
		}
		std::cout << std::endl;
	}
	catch (const std::exception & e)
	{
		ccLog::Error(e.what());
	}
}

void ChaiScriptingPlugin::resetToDefaultChaiState()
{
	if (!chai)
	{
		setupChaiScriptEngine();
	}
	else
	{
		chai->set_state(chaiInitialState);
	}
}

void ChaiScriptingPlugin::saveChaiState()
{
	chaiSaveState = chai->get_state();
}

void ChaiScriptingPlugin::resetToSavedChaiState()
{
	if (!chai)
	{
		setupChaiScriptEngine();
	}
	else
	{
		chai->set_state(chaiSaveState);
	}
}

std::string ChaiScriptingPlugin::chaiSystemDump()
{
	std::function<void()> cs_dump_system = chai->eval<std::function<void()> >("dump_system");

	std::stringstream buffer;
	std::streambuf* old = std::cout.rdbuf(buffer.rdbuf());
	cs_dump_system(); // call the ChaiScript function dump_system, from C++
	std::string text = buffer.str();
	std::cout.rdbuf(old);
	return text;
}


std::map<std::string, chaiscript::Boxed_Value> ChaiScriptingPlugin::getChaiObjects()
{
	auto cs_get_objects = chai->eval<std::function<std::map<std::string, chaiscript::Boxed_Value>()> >("get_objects");
	auto rtrnd = cs_get_objects(); // call the ChaiScript function get_objects, from C++
	for (std::pair<std::string, chaiscript::Boxed_Value> i : rtrnd)
	{
		////testing output
		//m_app->dispToConsole(QString(";Object name;%1;Object Type Name;%2;").arg(i.first.c_str()).arg(i.second.get_type_info().name().c_str()), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
	}
	return rtrnd;
}


// Work around to allow multi threading on windows
// destroyChai must be called prior to its the class destructor
// This is because the Thread_Storage object is destructed before 
// the final ChaiScript object is destructed.
void ChaiScriptingPlugin::destroyChai() 
{
	if (chai)
	{
		try
		{
			chaiscript::ChaiScript* mp = chai.release();
			delete mp;
		}
		catch (...)
		{

		}
	}
}

bool ChaiScriptingPlugin::loadChaiFile(const QString& file)
{
	if (!chai)
	{
		setupChaiScriptEngine();
	}
	try 
	{
	chai->eval_file(file.toLocal8Bit().constData());

	}
	catch (const chaiscript::exception::eval_error & ee)
	{
		ccLog::Error(ee.what());
		if (ee.call_stack.size() > 0)
		{
			ccLog::Warning(QString("during evaluation at (%1, %2)").arg(ee.call_stack[0]->start().line).arg(ee.call_stack[0]->start().column));
		}
		std::cout << std::endl;
		return false;
	}
	catch (const std::exception & e)
	{
		ccLog::Error(e.what());
		return false;
	}
	return true;
}

QList<QAction *> ChaiScriptingPlugin::getActions()
{
	if ( !m_action )
	{
		m_action = new QAction( getName(), this );
		m_action->setToolTip( getDescription() );
		m_action->setIcon( getIcon() );
		
		// Connect appropriate signal
		connect( m_action, &QAction::triggered, this, &ChaiScriptingPlugin::openScriptEditor);
	}

	cseMW = chaiScriptCodeEditorMainWindow::TheInstance();
	connect(cseMW, &chaiScriptCodeEditorMainWindow::executionCalled, this, &ChaiScriptingPlugin::executionCalled);
	connect(cseMW, &chaiScriptCodeEditorMainWindow::reset_Chai_to_initial_state, this, &ChaiScriptingPlugin::resetToDefaultChaiState);
	connect(cseMW, &chaiScriptCodeEditorMainWindow::save_Chai_state, this, &ChaiScriptingPlugin::saveChaiState);
	connect(cseMW, &chaiScriptCodeEditorMainWindow::reset_chai_to_last_save, this, &ChaiScriptingPlugin::resetToSavedChaiState);
	connect(cseMW, &chaiScriptCodeEditorMainWindow::destroy_chai, this, &ChaiScriptingPlugin::destroyChai);

	return { m_action };
}

void ChaiScriptingPlugin::stop()
{
	destroyChai();
}

void ChaiScriptingPlugin::openScriptEditor()
{
	
	if (!cseMW)
	{
		cseMW = chaiScriptCodeEditorMainWindow::TheInstance();
	}
	if (!cseMW)
	{
		m_app->dispToConsole("Failed to open chaiScriptCodeEditorMainWindow", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	cseMW->show();
	cseMW->raise();
	cseMW->activateWindow();
	if (!chai)
	{
		setupChaiScriptEngine();
	}

}

void ChaiScriptingPlugin::registerCommands(ccCommandLineInterface* cmd)
{	
	if (!cmd)
	{
		assert(false);
		return;
	}
	if (!chai)
	{
		setupChaiScriptEngine();
	}
	storedCMDLine = cmd;
	chai->add_global(var(cmd), "m_cmd");
	chai->eval(R"(global print = fun(x){m_cmd.print(x.to_QString());})");
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new CommandCHAI()));	
}

void ChaiScriptingPlugin::registerCommand(CommandCHAIDerived newCmd)
{
	if (storedCMDLine)
	{
		ccLog::Print(QString("Attempting to register %1 : %2").arg(newCmd.m_keyword).arg(newCmd.m_name));
		if (storedCMDLine->registerCommand(ccCommandLineInterface::Command::Shared(new CommandCHAIDerived(newCmd.m_name, newCmd.m_keyword.toUpper(), newCmd.chai_process))))
		{
			ccLog::Print("registerCommand success!");
		}
		else
		{
			ccLog::Error("registerCommand Failed");
		}
	}
}

void ChaiScriptingPlugin::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (chai_onNewSelection)
	{
		std::vector<std::shared_ptr<ccHObject>> sel;
		try
		{
			for each (auto objectName in selectedEntities)
			{
				ccHObject test = ccHObject(*objectName);
				sel.push_back(std::shared_ptr<ccHObject>());

			}
			chai_onNewSelection(sel);
		}
		catch (const chaiscript::exception::eval_error & ee)
		{
			ccLog::Error(ee.what());
			if (ee.call_stack.size() > 0)
			{
				ccLog::Warning(QString("during evaluation at (%1, %2)").arg(ee.call_stack[0]->start().line).arg(ee.call_stack[0]->start().column));
			}
			std::cout << std::endl;
		}
		catch (const std::exception & e)
		{
			ccLog::Error(e.what());
		}
		
	}
}

ChaiScriptingPlugin* ChaiScriptingPlugin::TheInstance()
{
	if (!s_instance)
	{
		s_instance = new ChaiScriptingPlugin();
	}
	return s_instance;
}

ChaiScriptingPlugin::~ChaiScriptingPlugin()
{
	
}