#pragma once

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

// Common
#include <ccApplicationBase.h>

// Qt
#include <QStringList>

class ccViewer;

class ccViewerApplication : public ccApplicationBase
{
	Q_OBJECT

  public:
	ccViewerApplication(int& argc, char** argv, bool isCommandLine);

	void setViewer(ccViewer* inViewer);

	//! Opens the files requested before the viewer was ready
	void openPendingFiles();

  protected:
	bool event(QEvent* inEvent) override;

  private:
	ccViewer* mViewer;
	//! Files the system asked to open before the viewer was ready
	QStringList mPendingFiles;
};
