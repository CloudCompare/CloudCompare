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

//! ASCII point cloud I/O filter
class QCC_IO_LIB_API AsciiFilter : public FileIOFilter
{
public:
	AsciiFilter();
	
	//static accessors
	static inline QString GetFileFilter() { return "ASCII cloud (*.txt *.asc *.neu *.xyz *.pts *.csv)"; }

	//inherited from FileIOFilter
	CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters) override;

	bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;
	CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;

	//! Loads an ASCII file with a predefined format
	CC_FILE_ERROR loadCloudFromFormatedAsciiFile(	const QString& filename,
													ccHObject& container,
													const AsciiOpenDlg::Sequence& openSequence,
													char separator,
													bool commaAsDecimal,
													unsigned approximateNumberOfLines,
													qint64 fileSize,
													unsigned maxCloudSize,
													unsigned skipLines,
													LoadParameters& parameters,
													bool showLabelsIn2D = false);

	//! Returns associated dialog (creates it if necessary)
	static AsciiOpenDlg* GetOpenDialog(QWidget* parentWidget = nullptr);
	//! Returns associated dialog (creates it if necessary)
	static AsciiSaveDlg* GetSaveDialog(QWidget* parentWidget = nullptr);

private:
	//! Internal use only
	CC_FILE_ERROR saveFile(ccHObject* entity, FILE *theFile);
};

#endif //CC_ASCII_FILTER_HEADER
