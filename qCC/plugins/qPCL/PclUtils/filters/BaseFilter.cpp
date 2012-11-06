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
#include "BaseFilter.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccHObjectCaster.h>

//#include <mainwindow.h>
//qCC
#include <ccMainAppInterface.h>

//Qt
#include <QAction>
#include <QFuture>
#include <QApplication>
#include <qtconcurrentrun.h>

//system
#if defined(_WIN32) || defined(WIN32)
#include "Windows.h"
#endif

BaseFilter::BaseFilter(FilterDescription desc)
	: m_action(0)
	, m_desc(desc)
{
	initAction();
}

void BaseFilter::initAction()
{
	if (m_action)
		return;

    m_action = new QAction(getIcon(), getEntryName(), this);
    m_action->setStatusTip(getStatusTip());
    //connect this action
    connect(m_action, SIGNAL(triggered()), this, SLOT(performAction()));
}

void BaseFilter::throwError(int errCode)
{
    QString errMsg = getErrorMessage(errCode);

	//we display message in a popup dialog
	//ccConsole::Error(errMsg.toStdString().c_str());
	//DGM: libraries shouldn't issue message this way! The information should be sent to the plugin!
	emit newErrorMessage(errMsg);
}


void BaseFilter::updateSelectedEntities(const ccHObject::Container& selectedEntities)
{
	m_selected = selectedEntities;

	if (m_action)
		m_action->setEnabled(checkSelected() == 1);
}

int BaseFilter::performAction()
{
    //check if selected entities are good
    int check_result = checkSelected();
    if (check_result != 1)
    {
        throwError(check_result);
        return check_result;
    }

    //if dialog is needed open the dialog
	int dialog_result = openDialog();
	if (dialog_result < 1)
	{
		if (dialog_result<0)
        throwError(dialog_result);
		else
			dialog_result = 1; //the operation is canceled by the user, no need to throw an error!
		return dialog_result;
	}

    //get the parameters from the dialog
    getParametersFromDialog();

    //are the given parameters ok?
    int par_status = checkParameters();
    if (par_status != 1)
    {
        throwError(par_status);
        return par_status;
    }

    //if so go ahead with start()
    int start_status = start();
    if (start_status != 1)
    {
        throwError(start_status);
        return start_status;
    }

	return 1;
}

int BaseFilter::checkSelected()
{
    //In most of the cases we just need 1 CC_POINT_CLOUD
    if (m_selected.empty())
        return -11;

    if (m_selected.size() != 1)
        return -12;

    if (!m_selected[0]->isA(CC_POINT_CLOUD) )
        return -13;

    return 1;
}

static BaseFilter* s_filter = 0;
static int s_computeStatus = 0;
static bool s_computing = false;
static void doCompute()
{
	if (!s_filter)
	{
		s_computeStatus=-1;
		return;
	}

	s_computeStatus = s_filter->compute();
}

int BaseFilter::start()
{   
	//old version
    //int is_ok = compute();

	//Version with a separate thread (and a progress callback)
	ccProgressDialog* progressCb = new ccProgressDialog();
	progressCb->setCancelButton(0);
	progressCb->setRange(0,0);
	progressCb->setInfo("Operation in progress");
	progressCb->setMethodTitle(qPrintable(getFilterName()));
	progressCb->start();
	QApplication::processEvents();

	if (s_computing)
	{
		throwError(-32);
		return -1;
	}

	s_computeStatus = -1;
	s_filter = this;
	s_computing = true;
	unsigned progress = 0;

	QFuture<void> future = QtConcurrent::run(doCompute);
	while (!future.isFinished())
	{
#if defined(_WIN32) || defined(WIN32)
		::Sleep(500);
#else
        sleep(1.0);
#endif
		progressCb->update(++progress);
	}
	int is_ok = s_computeStatus;
	s_filter = 0;
	s_computing = false;

	progressCb->stop();
	QApplication::processEvents();

    if (is_ok < 0)
    {
        throwError(is_ok);
        return -1;
    }

	return 1;
}

bool BaseFilter::hasDialog() const
{
	return m_desc.m_has_dialog;
}

QString BaseFilter::getFilterName() const
{
    return m_desc.m_filter_name;
}

QString BaseFilter::getEntryName() const
{
    return m_desc.m_entry_name;
}

QString BaseFilter::getStatusTip() const
{
    return m_desc.m_status_tip;
}

QIcon BaseFilter::getIcon() const
{
    return m_desc.m_icon;
}

QAction* BaseFilter::getAction()
{
    return m_action;
}

QString BaseFilter::getErrorMessage(int errorCode)
{
    QString errorMsg;
    switch(errorCode)
    {
        //ERRORS RELATED TO SELECTION
        case -11:
            errorMsg=QString("No entity selected in tree.");
            break;

        case -12:
            errorMsg=QString("Too many entities selected.");
            break;

        case -13:
            errorMsg=QString("Wrong type of entity selected");
            break;

        //ERRORS RELATED TO DIALOG
        case -21:
            errorMsg=QString("Dialog was not correctly filled in");
            break;

        //ERRORS RELATED TO COMPUTATION
        case -31:
            errorMsg=QString("Errors while computing");
            break;
        case -32:
			errorMsg=QString("Thread already in use!");
            break;

        // DEFAULT
        default:
            errorMsg=QString("Undefined error in " + getFilterName() + " filter");
            break;
    }

	return errorMsg;
}

ccPointCloud* BaseFilter::getSelectedEntityAsCCPointCloud()
{
    //does we have any selected entity?
    if (m_selected.size() == 0)
        return NULL;

	ccHObject* entity = m_selected.at(0);
	if (!entity->isA(CC_POINT_CLOUD))
		return NULL;



    return ccHObjectCaster::ToPointCloud(entity);
}

void BaseFilter::getAllEntitiesOfType(CC_CLASS_ENUM type, ccHObject::Container& entities)
{
	if (!m_app || !m_app->dbRoot())
		return;

	m_app->dbRoot()->filterChildren(entities,true,type);
}

int BaseFilter::hasSelectedScalarField(std::string field_name)
{
    ccPointCloud* cloud = getSelectedEntityAsCCPointCloud();
	if (!cloud)
		return -1;

    int result = cloud->getScalarFieldIndexByName(field_name.c_str());

	return (result >= 0 ? 1 : 0);
}

int BaseFilter::hasSelectedScalarField()
{
    if (isFirstSelectedCcPointCloud() != 1)
        return -1;

    ccPointCloud* cloud = getSelectedEntityAsCCPointCloud();
	if (!cloud)
		return -1;

	return (cloud->hasScalarFields() ? 1 : 0);
}

std::vector<std::string> BaseFilter::getSelectedAvailableScalarFields()
{
    std::vector<std::string> field_names;

	ccPointCloud* cloud = getSelectedEntityAsCCPointCloud();
	if (!cloud)
		return field_names;

    unsigned n_fields = cloud->getNumberOfScalarFields();
    field_names.reserve(n_fields);
    for (unsigned i = 0; i < n_fields; i++)
		field_names.push_back(cloud->getScalarFieldName(i));

	return field_names;
}

int BaseFilter::isFirstSelectedCcPointCloud()
{
    if (!m_selected.empty() && m_selected.at(0)->isA(CC_POINT_CLOUD))
        return 1;

	return -1;
}

int BaseFilter::hasSelectedRGB()
{
    if (isFirstSelectedCcPointCloud() != 1)
        return -1;
    //get the cloud

    ccPointCloud * cloud;
    cloud = getSelectedEntityAsCCPointCloud();

    return cloud->hasColors();
}
