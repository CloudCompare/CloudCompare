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

#include "ImageFileFilter.h"
#include "FileIO.h"

//qCC_db
#include <ccHObjectCaster.h>
#include <ccImage.h>

//Qt
#include <QFileDialog>
#include <QFileInfo>
#include <QImage>
#include <QImageReader>
#include <QImageWriter>

//System
#include <cassert>

ImageFileFilter::ImageFileFilter()
	: FileIOFilter( {
					"_Image Filter",
					17.0f,	// priority
					QStringList(),	// set below
					"png",
					QStringList(),	// set below
					QStringList(),	// set below
					Import | Export | BuiltIn | DynamicInfo
					} )
{
	//output filters
	{
		//we grab the list of supported image file formats (for writing)
		QList<QByteArray> formats = QImageWriter::supportedImageFormats();
		
		QStringList exportFilters;

		//we convert this list into a proper "filters" string
		for (auto &format : formats)
		{
			exportFilters.append( QStringLiteral("%1 image (*.%2)")
									.arg( QString( format.data() ).toUpper(), format.data() ) );
		}

		setExportFileFilterStrings(exportFilters);
	}

	//input filters
	{
		//we grab the list of supported image file formats (for reading)
		QList<QByteArray> formats = QImageReader::supportedImageFormats();
		
		QStringList imageFilters;
		QStringList importExtensions;
		for (auto &format : formats)
		{
			imageFilters.append(QStringLiteral("*.%1").arg(format.data()));
			importExtensions.append(QStringLiteral("%1").arg(format.data()));
		}
		setImportExtensions(importExtensions);

		//we convert this list into a proper "filters" string
		if (!imageFilters.empty())
		{
			QString imageFilter = QString("Image (%1)").arg(imageFilters.join(" "));
			setImportFileFilterStrings({ imageFilter });
		}
	}
}

QString ImageFileFilter::GetSaveFilename(const QString& dialogTitle, const QString& baseName, const QString& imageSavePath, QWidget* parentWidget/*=0*/)
{
	//add images output file filters
	QString filters;

	//we grab the list of supported image file formats (writing)
	QList<QByteArray> formats = QImageWriter::supportedImageFormats();
	if (formats.empty())
	{
		ccLog::Error("No image format supported by your system?!\n(check that the 'imageformats' directory is alongside the application executable)");
		return QString();
	}

	//we convert this list into a proper "filters" string
	QString pngFilter;
	for (int i = 0; i < formats.size(); ++i)
	{
		QString ext = QString(formats[i].data()).toUpper();
		QString filter = QString("%1 image (*.%2)").arg(ext,formats[i].data());
		filters.append(filter + QString("\n"));

		//find PNG by default
		if (pngFilter.isEmpty() && ext == "PNG")
		{
			pngFilter = filter;
		}
	}

	QString outputFilename = QFileDialog::getSaveFileName(	parentWidget,
															dialogTitle,
															imageSavePath + QString("/%1.%2").arg(baseName, pngFilter.isEmpty() ? QString(formats[0].data()) : QString("png")),
															filters,
															pngFilter.isEmpty() ? nullptr : &pngFilter);

	return outputFilename;
}

QString ImageFileFilter::GetLoadFilename(const QString& dialogTitle, const QString& imageLoadPath, QWidget* parentWidget/*=0*/)
{
	//we grab the list of supported image file formats (for reading)
	QList<QByteArray> formats = QImageReader::supportedImageFormats();
	QStringList imageExts;
	for (int i = 0; i < formats.size(); ++i)
	{
		imageExts.append(QString("*.%1").arg(formats[i].data()));
	}
	//we convert this list into a proper "filters" string
	QString imageFilter = QString("Image (%1)").arg(imageExts.join(" "));

	return QFileDialog::getOpenFileName(	parentWidget,
											dialogTitle,
											imageLoadPath,
											imageFilter);
}

bool ImageFileFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::IMAGE)
	{
		multiple = false;
		exclusive = true;
		return true;
	}
	return false;
}

CC_FILE_ERROR ImageFileFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	if (!entity)
		return CC_FERR_BAD_ARGUMENT;

	ccImage* image = ccHObjectCaster::ToImage(entity);
	if (!image)
		return CC_FERR_BAD_ENTITY_TYPE;

	if (image->data().isNull() || image->getW() == 0 || image->getH() == 0)
	{
		ccLog::Warning(QString("[IMAGE] Image '%1' is empty!").arg(image->getName()));
		return CC_FERR_NO_SAVE;
	}
	
	QImageWriter writer(filename);
	
	writer.setText("Author", FileIO::writerInfo());
	
	if (!writer.write(image->data()))
	{
		ccLog::Warning(QString("[IMAGE] Failed to save image in '%1").arg(filename));
		return CC_FERR_CONSOLE_ERROR;
	}
	
	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR ImageFileFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	QImage qImage;
	if (!qImage.load(filename))
	{
		ccLog::Warning(QString("[IMAGE] Failed to load image '%1").arg(filename));
		return CC_FERR_CONSOLE_ERROR;
	}

	//create corresponding ccImage
	ccImage* image = new ccImage(qImage,QFileInfo(filename).baseName());

	container.addChild(image);

	return CC_FERR_NO_ERROR;
}
