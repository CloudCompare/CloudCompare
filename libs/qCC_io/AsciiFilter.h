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
class QCC_IO_LIB_API AsciiFilter : public FileIOFilter
{
public:

	//static accessors
	static inline QString GetFileFilter() { return "ASCII cloud (*.txt *.asc *.neu *.xyz *.pts *.csv)"; }
	static inline QString GetDefaultExtension() { return "asc"; }

	//inherited from FileIOFilter
	virtual bool importSupported() const { return true; }
	virtual bool exportSupported() const { return true; }
	virtual CC_FILE_ERROR loadFile(QString filename, ccHObject& container, LoadParameters& parameters);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename);
	virtual QStringList getFileFilters(bool onImport) const { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const { return GetDefaultExtension(); }
	virtual bool canLoadExtension(QString upperCaseExt) const;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const;

	//! Loads an ASCII file with a predefined format
	CC_FILE_ERROR loadCloudFromFormatedAsciiFile(	const QString& filename,
													ccHObject& container,
													const AsciiOpenDlg::Sequence& openSequence,
													char separator,
													unsigned approximateNumberOfLines,
													qint64 fileSize,
													unsigned maxCloudSize,
													unsigned skipLines,
													LoadParameters& parameters);

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

#endif //CC_ASCII_FILTER_HEADER
