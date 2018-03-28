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

#ifndef CC_IMAGE_FILE_FILTER_HEADER
#define CC_IMAGE_FILE_FILTER_HEADER

#include "FileIOFilter.h"

//! Filter to load or save an image (all types supported by Qt)
class QCC_IO_LIB_API ImageFileFilter : public FileIOFilter
{
public:

	//! Default constructor
	ImageFileFilter();

	//static accessors
	static inline QString GetDefaultExtension() { return "png"; }

	//inherited from FileIOFilter
	virtual bool importSupported() const override { return true; }
	virtual bool exportSupported() const override { return true; }
	virtual CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters) override;
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;
	virtual QStringList getFileFilters(bool onImport) const override;
	virtual QString getDefaultExtension() const override { return GetDefaultExtension(); }
	virtual bool canLoadExtension(const QString& upperCaseExt) const override;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;

	//! Helper: select an input image filename
	static QString GetLoadFilename(	const QString& dialogTitle,
									const QString& imageLoadPath,
									QWidget* parentWidget = 0);

	//! Helper: select an output image filename
	static QString GetSaveFilename(	const QString& dialogTitle,
									const QString& baseName,
									const QString& imageSavePath,
									QWidget* parentWidget = 0);

protected:

	//! Supported (output) filters
	QStringList m_outputFilters;
	//! Supported (input) filters
	QString m_inputFilter;

};

#endif //CC_IMAGE_FILE_FILTER_HEADER
