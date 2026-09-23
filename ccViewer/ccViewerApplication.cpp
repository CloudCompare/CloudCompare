// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: CloudCompare project                               #
// #                                                                        #
// ##########################################################################

#include <QtGlobal>

#ifdef Q_OS_MAC
#include <QFileOpenEvent>
#endif

#include "ccViewerApplication.h"
#include "ccviewer.h"

ccViewerApplication::ccViewerApplication(int& argc, char** argv, bool isCommandLine)
    : ccApplicationBase(argc, argv, isCommandLine, QString("1.42.beta (%1)").arg(__DATE__))
{
	setApplicationName("CloudCompareViewer");
}

void ccViewerApplication::setViewer(ccViewer* inViewer)
{
	mViewer = inViewer;
}

bool ccViewerApplication::event(QEvent* inEvent)
{
#ifdef Q_OS_MAC
	switch (inEvent->type())
	{
	case QEvent::FileOpen:
	{
		QString filename = static_cast<QFileOpenEvent*>(inEvent)->file();

		// the viewer may not be set yet when ccViewer is launched by opening a file
		if (mViewer == nullptr)
		{
			mPendingFiles << filename;
			return true;
		}

		mViewer->addToDB({filename});
		return true;
	}

	default:
		break;
	}
#endif

	return ccApplicationBase::event(inEvent);
}

void ccViewerApplication::openPendingFiles()
{
	if (mViewer == nullptr || mPendingFiles.isEmpty())
	{
		return;
	}

	QStringList filenames = mPendingFiles;
	mPendingFiles.clear();

	mViewer->addToDB(filenames);
}
