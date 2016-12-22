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
//#                         COPYRIGHT: CNRS / OSUR                         #
//#                                                                        #
//##########################################################################

#include "LASFWFFilter.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccWaveform.h>
#include <ccColorScalesManager.h>

//qCC_io
#include <LASFields.h>

//Qt
#include <QCoreApplication>
#include <QString>
#include <QFile>
#include <QFileInfo>

//LASLib
#include <lasreader.hpp>
#include <laspoint.hpp>

//system
#include <assert.h>
#include <string.h>

bool LASFWFFilter::canLoadExtension(QString upperCaseExt) const
{
	return (	upperCaseExt == "LAS"
			||	upperCaseExt == "LAZ" );
}

bool LASFWFFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	//TODO
	return false;
}

//CC_FILE_ERROR LASFWFFilter::saveToFile(ccHObject* entity, QString filename)
//{
//	return CC_FERR_NO_ERROR;
//}

bool PrepareLASField(ccScalarField*& field, LAS_FIELDS type, unsigned totalCount, unsigned currentCount, ScalarType defaultValue = 0)
{
	if (field)
	{
		assert(false);
		return true;
	}

	//try to reserve the memory to store the field values
	field = new ccScalarField(LAS_FIELD_NAMES[type]);
	if (!field->reserve(totalCount))
	{
		ccLog::Warning(QString("[LAS] Not enough memory to load a field: '%1'").arg(LAS_FIELD_NAMES[type]));
		field->release();
		field = 0;
		return false;
	}

	//fill the previous points values (if any)
	for (unsigned i = 0; i < currentCount; ++i)
	{
		//set the previous values!
		field->addElement(defaultValue);
	}

	return true;
}

struct LASField
{
	LASField() : data(0), ignore(false) {}
	ccScalarField* data;
	bool ignore;
};

CC_FILE_ERROR LASFWFFilter::loadFile(QString filename, ccHObject& container, LoadParameters& parameters)
{
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	
	try
	{
		LASreadOpener lasreadopener;
		lasreadopener.set_file_name(qPrintable(filename));
		assert(lasreadopener.active());

		LASreader* lasreader = lasreadopener.open();
		if (lasreader == 0)
		{
			ccLog::Warning("LASLib", "Failed to open 'lasreader'");
			return CC_FERR_THIRD_PARTY_LIB_FAILURE;
		}

		unsigned pointCount = static_cast<unsigned>(lasreader->npoints);
		ccLog::Print(QString("[LASLib] " + QObject::tr("Reading %1 points").arg(pointCount)));

		//progress dialog
		ccProgressDialog progressDialog(parameters.parentWidget);
		if (parameters.parentWidget)
		{
			progressDialog.setWindowTitle(QObject::tr("Import LAS file"));
			progressDialog.setLabelText(QObject::tr("Points: %1").arg(pointCount));
			progressDialog.setRange(0, 0/*static_cast<int>(pointCount)*/); //DGM FIXME: the progress doesn't update! Is it because it's in a separate DLL/plugin?
			progressDialog.show();
			QCoreApplication::processEvents();
		}
		CCLib::NormalizedProgress nProgress(&progressDialog, pointCount);

		//number of points read from the beginning of the current cloud part
		unsigned pointsRead = 0;
		CCVector3d Pshift(0, 0, 0);

		//create cloud
		ccPointCloud* cloud = new ccPointCloud("unnamed");
		if (!cloud->reserve(pointCount))
		{
			//not enough memory
			lasreader->close();
			delete lasreader;
			delete cloud;
			return CC_FERR_NOT_ENOUGH_MEMORY;
		}

		bool ignoreColors = false;
		bool hasColors = false;
		bool hasColorsAboveZero = false;
		int colorBitDec = 0;

		QMap<LAS_FIELDS, LASField> loadedFields;

		bool hasFWF = (lasreader->header.vlr_wave_packet_descr != 0);
		//QFile fwfFile;
		//bool hasFWFFile = false;
		LASwaveform13reader* fwfReader = 0;
		if (hasFWF)
		for (int fakeIteration = 0; fakeIteration < 1; ++fakeIteration)
		{
			try
			{
				cloud->waveforms().resize(pointCount);
			}
			catch (const std::bad_alloc&)
			{
				ccLog::Warning(QString("Not enough memory to import the waveform data"));
				hasFWF = false;
				break;
			}

			fwfReader = lasreadopener.open_waveform13(&(lasreader->header));
			if (!fwfReader)
			{
				ccLog::Warning(QString("Failed to open/read the associated waveform data packets"));
				hasFWF = false;
				break;
			}

			//determine the total size of the FWF data
			QFile fwfDataSource;
			uint64_t fwfDataCount = 0;
			if (lasreader->header.start_of_waveform_data_packet_record != 0)
			{
				//the FWF data is internal
				assert(lasreader->header.global_encoding & 2);
				//open the same file
				fwfDataSource.setFileName(filename);
				if (!fwfDataSource.open(QFile::ReadOnly))
				{
					ccLog::Warning(QString("Failed to read the associated waveform data packets"));
					hasFWF = false;
					break;
				}
				//seek for the waveform EVLR
				fwfDataSource.seek(lasreader->header.start_of_waveform_data_packet_record);
				QByteArray evlrHeader = fwfDataSource.read(60);
				if (evlrHeader.size() < 60)
				{
					ccLog::Warning(QString("Failed to read the associated waveform data packets"));
					hasFWF = false;
					break;
				}

				//get the number of bytes
				fwfDataCount = *reinterpret_cast<const uint64_t*>(evlrHeader.constData() + 20); //see LAS 1.4 EVLR header specifications
			}
			else
			{
				QString wdpFilename = filename;
				wdpFilename.replace(QFileInfo(filename).suffix(), "wdp");
				fwfDataSource.setFileName(wdpFilename);
				if (!fwfDataSource.open(QFile::ReadOnly))
				{
					ccLog::Warning(QString("Failed to read the associated waveform data packets file (looking for '%1')").arg(wdpFilename));
					hasFWF = false;
					break;
				}

				//the number of bytes is simply the file size
				fwfDataCount = fwfDataSource.size();
			}

			//load the FWF data
			if (fwfDataSource.isOpen() && fwfDataCount != 0)
			{
				ccPointCloud::FWFDataContainer* container = new ccPointCloud::FWFDataContainer;
				try
				{
					container->resize(fwfDataCount);
				}
				catch (const std::bad_alloc&)
				{
					ccLog::Warning(QString("Not enough memory to import the waveform data"));
					cloud->waveforms().clear();
					delete container;
					hasFWF = false;
					break;
				}

				fwfDataSource.read((char*)(&(container->front())), fwfDataCount);
				fwfDataSource.close();

				cloud->fwfData() = ccPointCloud::SharedFWFDataContainer(container);
			}
		}

		CCVector3d lasScale = CCVector3d(lasreader->header.x_scale_factor, lasreader->header.y_scale_factor, lasreader->header.z_scale_factor);
		CCVector3d lasShift = -CCVector3d(lasreader->header.x_offset, lasreader->header.y_offset, lasreader->header.z_offset);

		ccPointCloud::FWFDescriptorSet& descriptors = cloud->fwfDescriptors();

		//read the points
		for (size_t pointIndex = 0; lasreader->read_point(); ++pointIndex)
		{
			const LASpoint& point = lasreader->point;

			CCVector3d P(	point.quantizer->get_x(point.X),
							point.quantizer->get_y(point.Y),
							point.quantizer->get_z(point.Z));

			//Waveform
			if (hasFWF && point.have_wavepacket)
			{
				if (fwfReader->read_waveform(&point))
				{
					U8 packetIndex = point.wavepacket.getIndex();
					if (!descriptors.contains(packetIndex))
					{
						LASvlr_wave_packet_descr* descriptor = lasreader->header.vlr_wave_packet_descr[packetIndex];
						WaveformDescriptor wfd;
						if (descriptor)
						{
							wfd.numberOfSamples = descriptor->getNumberOfSamples();
							wfd.bitsPerSample = descriptor->getBitsPerSample();
							wfd.digitizerGain = descriptor->getDigitizerGain();
							if (wfd.digitizerGain == 0)
							{
								//shouldn't be 0 by default!
								wfd.digitizerGain = 1.0;
							}
							wfd.digitizerOffset = descriptor->getDigitizerOffset();
							wfd.samplingRate_ps = descriptor->getTemporalSpacing();
						}
						descriptors.insert(packetIndex, wfd);
					}

					ccWaveform& w = cloud->waveforms()[pointIndex];
					w.setDescriptorID(packetIndex);
					w.setDataDescription(point.wavepacket.getOffset(), point.wavepacket.getSize());
					w.setBeamDir(CCVector3f::fromArray(fwfReader->XYZt));
					w.setEchoTime_ps(fwfReader->location);
					w.setReturnIndex(point.return_number);
				}
			}

			if (cloud->size() == 0) //first point
			{
				//backup input global parameters
				ccGlobalShiftManager::Mode csModeBackup = parameters.shiftHandlingMode;
				bool useLasShift = false;
				//set the LAS shift as default shift (if none was provided)
				if (lasShift.norm2() != 0 && (!parameters.coordinatesShiftEnabled || !*parameters.coordinatesShiftEnabled))
				{
					useLasShift = true;
					Pshift = lasShift;
					if (	csModeBackup != ccGlobalShiftManager::NO_DIALOG
						&&	csModeBackup != ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT)
					{
						parameters.shiftHandlingMode = ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG;
					}
				}
				if (HandleGlobalShift(P, Pshift, parameters, useLasShift))
				{
					cloud->setGlobalShift(Pshift);
					ccLog::Warning("[LAS] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
				}

				//restore previous parameters
				parameters.shiftHandlingMode = csModeBackup;
			}

			//color
			if (!ignoreColors)
			{
				if (!hasColors)
				{
					U16 mergedColorComp = point.rgb[0] | point.rgb[1] | point.rgb[2];
					if (mergedColorComp != 0)
					{
						hasColors = cloud->reserveTheRGBTable();
						if (!hasColors)
						{
							//not enough memory!
							ccLog::Warning("[LAS] Not enough memory to load RGB colors!");
							ignoreColors = true;
						}
						else
						{
							for (unsigned i = 0; i < cloud->size(); ++i)
							{
								//set all previous colors!
								cloud->addRGBColor(ccColor::black.rgba);
							}
						}
					}
				}
					
				if (hasColors)
				{
					if (colorBitDec == 0)
					{
						U16 mergedColorComp = point.rgb[0] | point.rgb[1] | point.rgb[2];
						if (mergedColorComp > 255)
						{
							//by default we assume the colors are coded on 8 bits...
							//...while they are theoretically coded on 16 bits (but some
							//software wrongly export LAS files with colors on 8 bits).
							//As soon as we detect a value higher than 255, we shift to 16 bits mode!
							colorBitDec = 8;
							for (unsigned i = 0; i < cloud->size(); ++i)
							{
								//reset all previous colors!
								cloud->setPointColor(i, ccColor::black.rgba);
							}
						}
					}

					ccColor::Rgb color(	static_cast<unsigned char>((point.rgb[0] >> colorBitDec) & 255),
										static_cast<unsigned char>((point.rgb[1] >> colorBitDec) & 255),
										static_cast<unsigned char>((point.rgb[2] >> colorBitDec) & 255));

					cloud->addRGBColor(color.rgb);
				}
			}

			//intensity
			{
				LASField& intensity = loadedFields[LAS_INTENSITY];
				if (!intensity.ignore)
				{
					if (!intensity.data && point.intensity != 0)
					{
						//prepare the field if necessary (the first time we encounter a value <> 0)
						if (!PrepareLASField(intensity.data, LAS_INTENSITY, pointCount, cloud->size(), 0))
						{
							intensity.ignore = true;
						}
						else
						{
							//set default grey color scale
							intensity.data->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::GREY));
						}
					}

					if (intensity.data)
					{
						intensity.data->addElement(static_cast<ScalarType>(point.intensity));
					}
				}
			}
			//return number
			{
				LASField& returnNumber = loadedFields[LAS_RETURN_NUMBER];
				if (!returnNumber.ignore)
				{
					if (!returnNumber.data && point.return_number > 1)
					{
						//prepare the field if necessary (the first time we encounter a value <> 0)
						if (!PrepareLASField(returnNumber.data, LAS_RETURN_NUMBER, pointCount, cloud->size(), 1))
						{
							returnNumber.ignore = true;
						}
					}

					if (returnNumber.data)
					{
						returnNumber.data->addElement(static_cast<ScalarType>(point.return_number));
					}
				}
			}

			cloud->addPoint(CCVector3::fromArray((P + Pshift).u));

			if (parameters.parentWidget && !nProgress.oneStep())
			{
				result = CC_FERR_CANCELED_BY_USER;
				break;
			}
		}

		if (fwfReader)
		{
			delete fwfReader;
			fwfReader = 0;
		}

		lasreader->close();
		delete lasreader;
		lasreader = 0;

		if (cloud->size() == 0)
		{
			ccLog::Warning("LASLib", QObject::tr("No valid point in file"));

			//release scalar fields (if any)
			for (LASField& field : loadedFields)
			{
				if (field.data)
				{
					field.data->release();
				}
			}

			//release the cloud
			delete cloud;
			cloud = 0;
		}
		else
		{
			//associate the cloud with the various fields
			for (LASField& field : loadedFields)
			{
				if (field.data)
				{
					field.data->computeMinAndMax();
					int sfIdx = cloud->addScalarField(field.data);
					if (sfIdx == 0)
					{
						//enable the first one by default
						cloud->setCurrentDisplayedScalarField(sfIdx);
					}
					cloud->showSF(true);
					field.data = 0; //just in case
				}
			}

			if (hasColors)
			{
				cloud->showColors(true);
			}

			container.addChild(cloud);
		}
	}
	catch (const std::bad_alloc&)
	{
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}
	catch (...)
	{
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	return result;
}
