//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_ASCII_FILTER_HEADER
#define CC_ASCII_FILTER_HEADER

#include "FileIOFilter.h"

//dialogs
#include "AsciiOpenDlg.h"
#include "AsciiSaveDlg.h"

template <class T> struct AutoDeletePtr
{
	AutoDeletePtr(T* _ptr = 0) : ptr(_ptr) {}
	~AutoDeletePtr() { release(); }
	inline void release() { if (ptr) { delete ptr; ptr = 0; } }
	T* ptr;
};

//! ASCII point cloud I/O filter
class QCC_IO_LIB_API AsciiFilter : public FileIOFilter
{
public:

	//static accessors
	static inline QString GetFileFilter() { return "ASCII cloud (*.txt *.asc *.neu *.xyz *.pts *.csv)"; }
	static inline QString GetDefaultExtension() { return "asc"; }

	//inherited from FileIOFilter
	virtual bool importSupported() const override { return true; }
	virtual bool exportSupported() const override { return true; }
	virtual CC_FILE_ERROR loadFile(QString filename, ccHObject& container, LoadParameters& parameters) override;
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename, SaveParameters& parameters) override;
	virtual QStringList getFileFilters(bool onImport) const override { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const override { return GetDefaultExtension(); }
	virtual bool canLoadExtension(QString upperCaseExt) const override;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;

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
	static AsciiOpenDlg* GetOpenDialog(QWidget* parentWidget = 0);
	//! Returns associated dialog (creates it if necessary)
	static AsciiSaveDlg* GetSaveDialog(QWidget* parentWidget = 0);

protected:

	//! Internal use only
	CC_FILE_ERROR saveFile(ccHObject* entity, FILE *theFile);

	//! Associated (export) dialog
	static AutoDeletePtr<AsciiSaveDlg> s_saveDialog;
	//! Associated (import) dialog
	static AutoDeletePtr<AsciiOpenDlg> s_openDialog;
};

#endif //CC_ASCII_FILTER_HEADER
