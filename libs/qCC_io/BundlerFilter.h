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

#ifndef CC_BUNDLER_FILTER_HEADER
#define CC_BUNDLER_FILTER_HEADER

#include "FileIOFilter.h"

//! Noah Snavely's Bundler output file filter
/** See http://phototour.cs.washington.edu/
**/
class QCC_IO_LIB_API BundlerFilter : public FileIOFilter
{
public:

	//static accessors
	static inline QString GetFileFilter() { return "Snavely's Bundler output (*.out)"; }
	static inline QString GetDefaultExtension() { return "out"; }

	//inherited from FileIOFilter
	virtual bool importSupported() const override { return true; }
	virtual CC_FILE_ERROR loadFile(QString filename, ccHObject& container, LoadParameters& parameters) override;
	virtual QStringList getFileFilters(bool onImport) const override { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const override { return GetDefaultExtension(); }
	virtual bool canLoadExtension(QString upperCaseExt) const override;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;

	//! Specific load method
	CC_FILE_ERROR loadFileExtended(	const QString& filename,
									ccHObject& container,
									LoadParameters& parameters,
									const QString& altKeypointsFilename = QString(),
									bool undistortImages = false,
									bool generateColoredDTM = false,
									unsigned coloredDTMVerticesCount = 1000000,
									float scaleFactor = 1.0f);

};

#endif //CC_BUNDLER_FILTER_HEADER
