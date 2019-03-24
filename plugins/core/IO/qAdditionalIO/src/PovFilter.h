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

#ifndef POV_FILTER_HEADER
#define POV_FILTER_HEADER

#include "FileIOFilter.h"

//! P.O.V. meta-file I/O filter
class PovFilter : public FileIOFilter
{
public:
	//static accessors
	static inline QString GetFileFilter() { return "Clouds + sensor info. [meta][ascii] (*.pov)"; }
	static inline QString GetDefaultExtension() { return "pov"; }

	//inherited from FileIOFilter
	bool importSupported() const override { return true; }
	bool exportSupported() const override { return true; }
	CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters) override;
	CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;
	QStringList getFileFilters(bool onImport) const override { Q_UNUSED( onImport ); return { GetFileFilter() }; }
	QString getDefaultExtension() const override { return GetDefaultExtension(); }
	bool canLoadExtension(const QString& upperCaseExt) const override;
	bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;
};

#endif //POV_FILTER_HEADER
