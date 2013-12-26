//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccOverlayDialog.h"

//Local
#include "ccGLWindow.h"

//qCC_db
#include <ccLog.h>

//Qt
#include <QEvent>
#include <QKeyEvent>
#include <QApplication>

//system
#include <assert.h>

ccOverlayDialog::ccOverlayDialog(QWidget* parent/*=0*/)
	: QDialog(parent)
	, m_associatedWin(0)
	, m_processing(false)
{
}

ccOverlayDialog::~ccOverlayDialog()
{
	onLinkedWindowDeletion();
}

bool ccOverlayDialog::linkWith(ccGLWindow* win)
{
    if (m_processing)
    {
        ccLog::Warning("[ccOverlayDialog] Can't change associated window while running/displayed!");
        return false;
    }

	if (m_associatedWin)
	{
		//same dialog? nothing to do
		if (m_associatedWin == win)
			return false;
		
		//otherwise, we automatically detach it
		{
			QWidgetList topWidgets = QApplication::topLevelWidgets();
			foreach(QWidget* widget,topWidgets)
			{
				widget->removeEventFilter(this);
			}
		}
		disconnect(m_associatedWin, SIGNAL(destroyed(QObject*)), this, SLOT(onLinkedWindowDeletion(QObject*)));
		m_associatedWin = 0;
	}

	m_associatedWin = win;
	if (m_associatedWin)
	{
		QWidgetList topWidgets = QApplication::topLevelWidgets();
		foreach(QWidget* widget,topWidgets)
		{
			widget->installEventFilter(this);
		}
		connect(m_associatedWin, SIGNAL(destroyed(QObject*)), this, SLOT(onLinkedWindowDeletion(QObject*)));
	}

	return true;
}

void ccOverlayDialog::onLinkedWindowDeletion(QObject* object/*=0*/)
{
	if (m_processing)
		stop(false);

	linkWith(0);
}

bool ccOverlayDialog::start()
{
	if (m_processing)
		return false;

	m_processing = true;

	//auto-show
	show();

	return true;
}

void ccOverlayDialog::stop(bool accepted)
{
	m_processing = false;

	//auto-hide
	hide();

	linkWith(0);

	emit processFinished(accepted);
}

void ccOverlayDialog::reject()
{
	QDialog::reject();

	stop(false);
}

void ccOverlayDialog::addOverridenShortcut(Qt::Key key)
{
	m_overriddenKeys.push_back(key);
}

bool ccOverlayDialog::eventFilter(QObject *obj, QEvent *e)
{
	if (e->type() == QEvent::KeyPress)
	{
		QKeyEvent* keyEvent = static_cast<QKeyEvent*>(e);

		if (m_overriddenKeys.contains(keyEvent->key()))
		{
			emit shortcutTriggered(keyEvent->key());
			return true;
		}
		else
		{
			return QObject::eventFilter(obj, e);
		}
	}
	else
	{
		// standard event processing
		return QObject::eventFilter(obj, e);
	}
}