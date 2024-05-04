//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#include "BaseFilter.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccHObjectCaster.h>

//CCCoreLib
#include <CCPlatform.h>

//qCC
#include <ccMainAppInterface.h>

//Qt
#include <QAction>
#include <QFuture>
#include <QApplication>
#include <QProgressDialog>
#include <qtconcurrentrun.h>

//system
#if defined(CC_WINDOWS)
#include "windows.h"
#else
#include <time.h>
#include <unistd.h>
#endif

BaseFilter::BaseFilter(FilterDescription desc, ccMainAppInterface* app)
	: m_desc(desc)
	, m_action(new QAction(desc.icon, desc.entryName, this))
	, m_app(app)
	, m_showProgress(true)
{
	m_action->setStatusTip(m_desc.statusTip);

	//connect this action
	connect(m_action, &QAction::triggered, this, &BaseFilter::performAction);
}

void BaseFilter::throwError(int errCode)
{
	QString errMsg = getErrorMessage(errCode);

	if (errCode == CancelledByUser)
	{
		// don't emit this particular 'error' message
		ccLog::Warning("[qPCL] " + errMsg);
	}
	else if (errCode < 0)
	{
		//DGM: as libraries shouldn't issue message themselves, it should be sent to the plugin via a signal
		Q_EMIT newErrorMessage(errMsg);
	}
}

void BaseFilter::updateSelectedEntities(const ccHObject::Container& selectedEntities)
{
	m_selectedEntities = selectedEntities;

	if (m_action)
	{
		m_action->setEnabled(checkSelected());
	}
}

void BaseFilter::performAction()
{
	//check if selected entities are good
	if (!checkSelected())
	{
		assert(false);
		throwError(InvalidInput);
		return;
	}

	//get parameters from the dialog (if any)
	int result = getParametersFromDialog();
	if (result != Success)
	{
		throwError(result);
		return;
	}

	//if so, go ahead with start()
	result = start();
	if (result != Success)
	{
		throwError(result);
		return;
	}
}

bool BaseFilter::checkSelected() const
{
	// default mode: only one cloud
	return (m_selectedEntities.size() == 1 && m_selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD));
}

static BaseFilter* s_filter = nullptr;
static int s_computeStatus = BaseFilter::ComputationError;

static void DoCompute()
{
	if (s_filter)
	{
		s_computeStatus = s_filter->compute();
	}
	else
	{
		s_computeStatus = BaseFilter::ComputationError;
	}
}

int BaseFilter::start()
{
	static bool s_computing = false;
	if (s_computing)
	{
		return ThreadAlreadyInUse;
	}

	QProgressDialog pDlg(tr("Operation in progress"), QString(), 0, 0);
	if (m_showProgress)
	{
		pDlg.setWindowTitle(m_desc.filterName);
		pDlg.show();
		QApplication::processEvents();
	}

	s_computeStatus = ComputationError;
	s_filter = this;
	s_computing = true;

	QFuture<void> future = QtConcurrent::run(DoCompute);
	int progress = 0;
	while (!future.isFinished())
	{
#if defined(CC_WINDOWS)
		::Sleep(500);
#else
		usleep(500 * 1000);
#endif
		if (m_showProgress)
		{
			pDlg.setValue(++progress);
		}
	}
	
	int result = s_computeStatus;
	s_filter = nullptr;
	s_computing = false;

	if (m_showProgress)
	{
		pDlg.close();
		QApplication::processEvents();
	}

	return result;
}

QString BaseFilter::getErrorMessage(int errorCode) const
{
	switch (errorCode)
	{
	case ComputationError:
		return tr("Errors while computing");
		break;

	case InvalidInput:
		return tr("Internal error: invalid input");
		break;

	case ThreadAlreadyInUse:
		return tr("Internal error: thread already in use");
		break;

	case CancelledByUser:
		return tr("Process cancelled by user");

	case InvalidParameters:
		return tr("Invalid parameters");

	case NotEnoughMemory:
		return tr("Not enough memory");

	case Success:
		return {};

	default:
		return tr("Undefined error in filter %1: %2").arg(m_desc.filterName).arg(errorCode);
	}

	return {};
}

ccPointCloud* BaseFilter::getFirstSelectedEntityAsCCPointCloud() const
{
	ccHObject* entity = getFirstSelectedEntity();
	if (entity && entity->isA(CC_TYPES::POINT_CLOUD))
	{
		return ccHObjectCaster::ToPointCloud(entity);
	}
	else
	{
		return nullptr;
	}
}

ccHObject* BaseFilter::getFirstSelectedEntity() const
{
	//do we have any selected entity?
	if (m_selectedEntities.empty())
		return nullptr;

	return m_selectedEntities.front();
}

void BaseFilter::getSelectedEntitiesThatAreCCPointCloud(ccHObject::Container& entities) const
{
	getSelectedEntitiesThatAre(CC_TYPES::POINT_CLOUD, entities);
}

void BaseFilter::getSelectedEntitiesThatAre(CC_CLASS_ENUM kind, ccHObject::Container& entities) const
{
	entities.reserve(m_selectedEntities.size());
	for (ccHObject* entity : m_selectedEntities)
	{
		if (entity && entity->isA(kind))
		{
			entities.push_back(entity);
		}
	}

	entities.shrink_to_fit();
}
