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

#ifndef CC_ASCII_FILTER_HEADER
#define CC_ASCII_FILTER_HEADER

#include "FileIOFilter.h"

//dialogs
#include "AsciiOpenDlg.h"
#include "AsciiSaveDlg.h"

//Qt
#include <QSharedPointer>

//! ASCII point cloud I/O filter
class AsciiFilter : public FileIOFilter
{
public:

	//inherited from FileIOFilter
	virtual CC_FILE_ERROR loadFile(QString filename, ccHObject& container, bool alwaysDisplayLoadDialog = true, bool* coordinatesShiftEnabled = 0, CCVector3d* coordinatesShift = 0);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename);

	CC_FILE_ERROR loadCloudFromFormatedAsciiFile(	const QString& filename,
													ccHObject& container,
													const AsciiOpenDlg::Sequence& openSequence,
													char separator,
													unsigned approximateNumberOfLines,
													qint64 fileSize,
													unsigned maxCloudSize,
													unsigned skipLines=0,
													bool alwaysDisplayLoadDialog=true,
													bool* coordinatesShiftEnabled=0,
													CCVector3d* coordinatesShift=0);

	//! Returns associated dialog (creates it if necessary)
	static QSharedPointer<AsciiOpenDlg> GetOpenDialog();
	//! Returns associated dialog (creates it if necessary)
	static QSharedPointer<AsciiSaveDlg> GetSaveDialog();

protected:

	//! Internal use only
	CC_FILE_ERROR saveFile(ccHObject* entity, FILE *theFile);

	//! Associated (export) dialog
	static QSharedPointer<AsciiSaveDlg> s_saveDialog;
	//! Associated (import) dialog
	static QSharedPointer<AsciiOpenDlg> s_openDialog;
};

#endif
