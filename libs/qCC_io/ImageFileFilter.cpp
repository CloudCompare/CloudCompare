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

//qCC_db
#include <ccImage.h>

//Qt
#include <QImageReader>
#include <QImageWriter>
#include <QFileInfo>
#include <QImage>
#include <QFileDialog>

//System
#include <assert.h>

ImageFileFilter::ImageFileFilter()
	: FileIOFilter()
{
	//output filters
	{
		//we grab the list of supported image file formats (for writing)
		QList<QByteArray> formats = QImageWriter::supportedImageFormats();
		//we convert this list into a proper "filters" string
		for (int i = 0; i < formats.size(); ++i)
		{
			m_outputFilters.append(QString("%1 image (*.%2)").arg(QString(formats[i].data()).toUpper()).arg(formats[i].data()));
		}
	}

	//input filters
	{
		//we grab the list of supported image file formats (for reading)
		QList<QByteArray> formats = QImageReader::supportedImageFormats();
		QStringList imageExts;
		for (int i = 0; i < formats.size(); ++i)
		{
			imageExts.append(QString("*.%1").arg(formats[i].data()));
		}
		//we convert this list into a proper "filters" string
		if (!imageExts.empty())
		{
			m_inputFilter = QString("Image (%1)").arg(imageExts.join(" "));
		}
	}
}

QString ImageFileFilter::GetSaveFilename(QString dialogTitle, QString baseName, QString imageSavePath, QWidget* parentWidget/*=0*/)
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
		QString filter = QString("%1 image (*.%2)").arg(ext).arg(formats[i].data());
		filters.append(filter + QString("\n"));

		//find PNG by default
		if (pngFilter.isEmpty() && ext == "PNG")
		{
			pngFilter = filter;
		}
	}

	QString outputFilename = QFileDialog::getSaveFileName(	parentWidget,
															dialogTitle,
															imageSavePath + QString("/%1.%2").arg(baseName).arg(pngFilter.isEmpty() ? QString(formats[0].data()) : QString("png")),
															filters,
															pngFilter.isEmpty() ? static_cast<QString*>(0) : &pngFilter);

	return outputFilename;
}

QString ImageFileFilter::GetLoadFilename(QString dialogTitle, QString imageLoadPath, QWidget* parentWidget/*=0*/)
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

QStringList ImageFileFilter::getFileFilters(bool onImport) const
{
	return onImport ? QStringList(m_inputFilter) : m_outputFilters;
}

bool ImageFileFilter::canLoadExtension(QString upperCaseExt) const
{
	//we grab the list of supported image file formats (for reading)
	QList<QByteArray> formats = QImageReader::supportedImageFormats();
	//we convert this list into a proper "filters" string
	for (int i=0; i<formats.size(); ++i)
		if (QString(formats[i].data()).toUpper() == upperCaseExt)
			return true;

	return false;
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

CC_FILE_ERROR ImageFileFilter::saveToFile(ccHObject* entity, QString filename, SaveParameters& parameters)
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

	if (!image->data().save(filename))
	{
		ccLog::Warning(QString("[IMAGE] Failed to save image in '%1").arg(filename));
		return CC_FERR_CONSOLE_ERROR;
	}
	
	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR ImageFileFilter::loadFile(QString filename, ccHObject& container, LoadParameters& parameters)
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
