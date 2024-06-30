#pragma once

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

#include "FileIOFilter.h"

//dialogs
#include "AsciiOpenDlg.h"
#include "AsciiSaveDlg.h"

//Qt
#include <QTextStream>
#include <QByteArray>

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

	//! Loads a cloud from a QByteArray
	CC_FILE_ERROR loadAsciiData(const QByteArray& data, QString sourceName, ccHObject& container, LoadParameters& parameters);

public: // Default / persistent settings

	//! Sets the default number of skipped lines (at loading time)
	static void SetDefaultSkippedLineCount(int count);
	//! Prevents the filter to create labels (at loading time)
	static void SetNoLabelCreated(bool state);

	//! Sets the default output coords precision (as saving time)
	static void SetOutputCoordsPrecision(int prec);
	//! Sets the default output scalar values precision (as saving time)
	static void SetOutputSFPrecision(int prec);
	//! Sets the default output separator (as saving time)
	/** index can be:
		- 0: space
		- 1: comma
		- 2: semicolon
		- 3: tab
	**/
	static void SetOutputSeparatorIndex(int separatorIndex);
	//! Sets whether color and SF should be swapped (default is color then SF)
	static void SaveSFBeforeColor(bool state);
	//! Sets whether fields names should be saved in a header line (default is false)
	static void SaveColumnsNamesHeader(bool state);
	//! Sets whether the number of points should be saved on the first line (default is false)
	static void SavePointCountHeader(bool state);

protected:
	//! Loads an ASCII stream
	CC_FILE_ERROR loadStream(	QTextStream& stream,
								QString filenameOrTitle,
								qint64 dataSize,
								ccHObject& container,
								LoadParameters& parameters);

	//! Loads an ASCII stream with a predefined format
	CC_FILE_ERROR loadCloudFromFormatedAsciiStream(	QTextStream& stream,
													QString filenameOrTitle,
													ccHObject& container,
													const AsciiOpenDlg::Sequence& openSequence,
													char separator,
													bool commaAsDecimal,
													unsigned approximateNumberOfLines,
													qint64 fileSize,
													unsigned maxCloudSize,
													unsigned skipLines,
													double quaternionScale,
													LoadParameters& parameters,
													bool showLabelsIn2D = false);
};
