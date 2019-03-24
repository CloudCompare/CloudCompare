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

#ifndef CC_SINUSX_FILTER_HEADER
#define CC_SINUSX_FILTER_HEADER

#include "FileIOFilter.h"

//! Sinusx curve I/O filter
class SinusxFilter : public FileIOFilter
{
public:
	//static accessors
	static inline QString GetFileFilter() { return "Sinusx curve (*.sx)"; }
	static inline QString GetDefaultExtension() { return QString("sx"); }

	//inherited from FileIOFilter
	bool exportSupported() const override { return true; }
	bool importSupported() const override { return true; }
	CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters) override;
	CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;
	QStringList getFileFilters(bool onImport) const override { Q_UNUSED( onImport ); return { GetFileFilter() }; }
	QString getDefaultExtension() const override { return GetDefaultExtension(); }
	bool canLoadExtension(const QString& upperCaseExt) const override;
	bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;
};

#endif //CC_SINUSX_FILTER_HEADER
