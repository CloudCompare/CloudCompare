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
//#                        COPYRIGHT: Luca Penasa                          #
//#                                                                        #
//##########################################################################
//
#include "qPCL.h"

//qCC_db
#include <ccPointCloud.h>

//PclUtils
#include <BaseFilter.h>

//FILTERS
//#include <LoadPCD.h>
//#include <SavePCD.h>
#include <ExtractSIFT.h>
#include <NormalEstimation.h>
#include <MLSSmoothingUpsampling.h>
//#include <StatisticalOutliersRemover.h>

qPCL::~qPCL()
{
	while (!m_filters.empty())
	{
		delete m_filters.back();
		m_filters.pop_back();
	}
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
	if (m_app)
		m_app->dispToConsole(message,ccMainAppInterface::ERR_CONSOLE_MESSAGE);
}

void qPCL::getActions(QActionGroup& group)
{
	if (m_filters.empty())
	{
		//ADD FILTERS
		//addFilter( new LoadPCD() ); //Now integrated in CC (qPCLIO plugin)
		//addFilter( new SavePCD() ); //Now integrated in CC (qPCLIO plugin)
		addFilter( new NormalEstimation());
		//addFilter( new StatisticalOutliersRemover() ); //Now integrated in CC ('Tools > Clean > SOR filter')
		addFilter( new MLSSmoothingUpsampling() );
	}

	for (std::vector<BaseFilter*>::const_iterator it = m_filters.begin(); it != m_filters.end(); ++it)
		group.addAction((*it)->getAction());
}

int qPCL::addFilter(BaseFilter* filter)
{
	assert(filter);
	filter->setMainAppInterface(m_app);

	QAction* action = filter->getAction();
	if (!action)
		return 0;

	//filter already inserted?
	if (std::find(m_filters.begin(),m_filters.end(),filter) != m_filters.end())
		return 0;

	m_filters.push_back(filter);

	//connect signals
	connect(filter, SIGNAL(newEntity(ccHObject*)),			this,	SLOT(handleNewEntity(ccHObject*)));
	connect(filter, SIGNAL(entityHasChanged(ccHObject*)),	this,	SLOT(handleEntityChange(ccHObject*)));
	connect(filter, SIGNAL(newErrorMessage(QString)),		this,	SLOT(handleErrorMessage(QString)));

	return 1;
}


void qPCL::onNewSelection(const ccHObject::Container& selectedEntities)
{
	for (size_t i=0; i<m_filters.size(); ++i)
		m_filters[i]->updateSelectedEntities(selectedEntities);
}

QIcon qPCL::getIcon() const
{
	return QIcon(QString::fromUtf8(":/toolbar/pcl.png"));
}

#ifndef CC_QT5
Q_EXPORT_PLUGIN2(qPCL,qPCL);
#endif
