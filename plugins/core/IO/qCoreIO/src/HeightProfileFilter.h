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
//#                       COPYRIGHT: SAGE INGENIERIE                       #
//#                                                                        #
//##########################################################################

#ifndef CC_HEIGHT_PROFILE_HEADER
#define CC_HEIGHT_PROFILE_HEADER

#include "FileIOFilter.h"

//! Polyline height profile I/O filter
/** This file format contains a 2D series: (curvilinear absisca ; height)
**/
class HeightProfileFilter : public FileIOFilter
{
public:
	//static accessors
	static inline QString GetFileFilter() { return "Height profile (*.csv)"; }
	static inline QString GetDefaultExtension() { return QString(); }

	//inherited from FileIOFilter
	bool exportSupported() const override { return true; }
	CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;
	QStringList getFileFilters(bool onImport) const override { Q_UNUSED( onImport ); return { GetFileFilter() }; }
	QString getDefaultExtension() const override { return GetDefaultExtension(); }
	bool canLoadExtension(const QString& upperCaseExt) const override { Q_UNUSED( upperCaseExt ); return false; }
	bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;
};

#endif //CC_HEIGHT_PROFILE_HEADER
