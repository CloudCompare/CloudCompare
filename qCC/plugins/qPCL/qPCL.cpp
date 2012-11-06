//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#               COPYRIGHT: Luca Penasa                                   #
//#                                                                        #
//##########################################################################
//
#include "qPCL.h"

//qCC_db
#include <ccPointCloud.h>

//PclUtils
#include <BaseFilter.h>

//FILTERS
#include <LoadPCD.h>
#include <SavePCD.h>
#include <ExtractSIFT.h>
#include <NormalEstimation.h>
#include <MLSSmoothingUpsampling.h>
#include <StatisticalOutliersRemover.h>
//#include <ComputeSPINImages.h>


qPCL::qPCL()
	: m_toolbar(0)
	, m_menu(0)
{
}

qPCL::~qPCL()
{
	while (!m_filters.empty())
	{
		delete m_filters.back();
		m_filters.pop_back();
	}
}

void qPCL::getDescription(ccPluginDescription& desc)
{
	strcpy(desc.name,"Load PCL menu and toolbar");
	strcpy(desc.menuName,"qPCL");
	desc.hasAnIcon=true;
	desc.version=1;
}


void qPCL::handleNewEntity(ccHObject* entity)
{
	assert(entity && m_app);
	m_app->addToDB(entity);
}

void qPCL::handleEntityChange(ccHObject* entity)
{
	assert(entity && m_app);
	entity->prepareDisplayForRefresh_recursive();
	m_app->refreshAll();
	m_app->updateUI();
}

void qPCL::handleErrorMessage(QString message)
{
	assert(m_app);
	m_app->dispToConsole(qPrintable(message),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
}

int qPCL::doAction(ccHObject::Container& selectedEntities,
	unsigned& uiModificationFlags,
	ccProgressDialog* progressCb/*=NULL*/,
	QWidget* parent/*=NULL*/)
{
	//check if toolbar has been created, if not create it
	if (!m_toolbar)
	{

		//create toolbars etc
		assert(m_app);
		QMainWindow* win = m_app->getMainWindow();
		if (win)
		{
			m_toolbar = win->addToolBar("PCL Tools");
			m_menu = win->menuBar()->addMenu("PCL");
		}
		else
			m_toolbar = new QToolBar("PCL Tools");





		//ADD FILTERS
		addFilter( new LoadPCD() );
		addFilter( new SavePCD() );
		addFilter( new NormalEstimation());
		addFilter( new StatisticalOutliersRemover() );
		addFilter( new MLSSmoothingUpsampling() );
		//  addFilter( new ComputeSPINImages() );



	}
	else
	{
		//nothing, perhaps we could add an About info when pressing pcl button, also as action into the menu
	}

	if (m_toolbar)
		m_toolbar->show();


	onNewSelection(selectedEntities);


	return 1;
}

int qPCL::addFilter(BaseFilter* filter)
{
	assert(filter);
	filter->setMainAppInterface(m_app);

	QAction *action = filter->getAction();
	if (!action)
		return 0;

	//filter already inserted?
	if (std::find(m_filters.begin(),m_filters.end(),filter) != m_filters.end())
		return 0;

	//Add action to toolbar and menu
	if (m_toolbar)
		m_toolbar->addAction(action);
	if (m_menu)
		m_menu->addAction(action);

	m_filters.push_back(filter);

	//connect signals
	connect(filter, SIGNAL(newEntity(ccHObject*)),          this,   SLOT(handleNewEntity(ccHObject*)));
	connect(filter, SIGNAL(entityHasChanged(ccHObject*)),   this,   SLOT(handleEntityChange(ccHObject*)));
	connect(filter, SIGNAL(newErrorMessage(QString)),       this,   SLOT(handleErrorMessage(QString)));

	return 1;
}


bool qPCL::onNewSelection(const ccHObject::Container& selectedEntities)
{
	for (unsigned i=0;i<m_filters.size();++i)
		m_filters[i]->updateSelectedEntities(selectedEntities);


	return true;
}

//This is provided for compatibility with qCC, not used
//DGM: well... you should ;)
QString qPCL::getErrorMessage(int errorCode/*, LANGUAGE lang*/)
{
	return QString("Undefined error!");
}

QIcon qPCL::getIcon() const
{
	return QIcon(QString::fromUtf8(":/toolbar/pcl.png"));
}

//plugin export
Q_EXPORT_PLUGIN2(qPCL,qPCL);
