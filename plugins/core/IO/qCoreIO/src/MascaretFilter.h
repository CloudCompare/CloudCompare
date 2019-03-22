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

#ifndef CC_MASCARET_FILTER_HEADER
#define CC_MASCARET_FILTER_HEADER

#include "FileIOFilter.h"

//! Mascaret profile I/O filter
/** See http://www.opentelemac.org/
**/
class MascaretFilter : public FileIOFilter
{
public:
	//static accessors
	static inline QString GetFileFilter() { return "(Geo-)Mascaret profile (*.georef)"; }
	static inline QString GetDefaultExtension() { return "georef"; }

	//inherited from FileIOFilter
	bool importSupported() const override { return false; }
	bool exportSupported() const override { return true; }
	CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;
	QStringList getFileFilters(bool onImport) const override { Q_UNUSED( onImport ); return { GetFileFilter() }; }
	QString getDefaultExtension() const override { return GetDefaultExtension(); }
	bool canLoadExtension(const QString& upperCaseExt) const override { Q_UNUSED( upperCaseExt ); return false; }
	bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;
};

#endif //CC_MASCARET_FILTER_HEADER
