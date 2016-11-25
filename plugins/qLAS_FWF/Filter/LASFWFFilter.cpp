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

		//number of points read from the begining of the current cloud part
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

		bool ignoreIntensities = false;
		ccScalarField* intensities = 0;
		bool hasIntensitiesAboveZero = false;

		bool hasFWF = (lasreader->header.vlr_wave_packet_descr != 0);
		//QFile fwfFile;
		//bool hasFWFFile = false;
		LASwaveform13reader* fwfReader = 0;
		if (hasFWF)
		{
			try
			{
				cloud->fwfData().resize(pointCount);
			}
			catch (const std::bad_alloc&)
			{
				ccLog::Warning(QString("Not enough memory to import the waveform data"));
				hasFWF = false;
			}

			//check if the waveforms are stored in the same file or on the disk
			//if (hasFWF && lasreader->header.global_encoding & 4)
			//{
			//	QString wdpFilename = filename;
			//	wdpFilename.replace(QFileInfo(filename).suffix(), "wdp");
			//	fwfFile.setFileName(wdpFilename);
			//	if (fwfFile.open(QFile::ReadOnly))
			//	{
			//		hasFWFFile = true;
			//	}
			//	else
			//	{
			//		hasFWF = false;
			//		ccLog::Warning(QString("Failed to open the associated waveform data packets file (%1)").arg(wdpFilename));
			//	}
			//}

			fwfReader = lasreadopener.open_waveform13(&(lasreader->header));
			if (!fwfReader)
			{
				ccLog::Warning(QString("Failed to open/read the associated waveform data packets"));
				hasFWF = false;
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

					uint32_t bitCount = (fwfReader->nsamples * fwfReader->nbits);
					uint32_t byteCount = (bitCount >> 3); // = /8
					if (bitCount > (byteCount << 3))
					{
						++byteCount;
					}

					ccWaveform& w = cloud->fwfData()[pointIndex];
					w.setDescriptorID(packetIndex);
					w.setData(fwfReader->samples, byteCount);
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
					if (csModeBackup != ccGlobalShiftManager::NO_DIALOG
						&&	csModeBackup != ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT)
					{
						parameters.shiftHandlingMode = ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG;
					}
				}
				if (HandleGlobalShift(P, Pshift, parameters, useLasShift))
				{
					cloud->setGlobalShift(Pshift);
					ccLog::Warning("[LAS] Cloud has been recentered! Translation: (%.2f,%.2f,%.2f)", Pshift.x, Pshift.y, Pshift.z);
				}

				//restore previous parameters
				parameters.shiftHandlingMode = csModeBackup;
			}

			//color
			if (!ignoreColors)
			{
				U16 mergedColorComp = point.rgb[0] | point.rgb[1] | point.rgb[2];
				hasColorsAboveZero |= (mergedColorComp != 0);
				if (hasColorsAboveZero)
				{
					if (!hasColors)
					{
						hasColors = cloud->reserveTheRGBTable();
						if (!hasColors)
						{
							//not enough memory!
							ignoreColors = true;
						}
					}

					if (!ignoreColors)
					{
						if (colorBitDec == 0 && mergedColorComp > 255)
						{
							//by default we assume the colors are coded on 8 bits...
							//...while they are theoretically coded on 16 bits (but some
							//software wrongly export LAS files with colors on 8 bits).
							//As soon as we detect a value higher than 255, we shift to 16 bits mode!
							colorBitDec = 8;
							for (size_t i = 0; i < cloud->size(); ++i)
							{
								//reset all previous colors!
								cloud->addRGBColor(ccColor::black.rgba);
							}
						}

						ccColor::Rgb color(	static_cast<unsigned char>((point.rgb[0] >> colorBitDec) & 255),
											static_cast<unsigned char>((point.rgb[1] >> colorBitDec) & 255),
											static_cast<unsigned char>((point.rgb[2] >> colorBitDec) & 255));

						cloud->addRGBColor(color.rgb);
					}
				}
			}

			//intensity
			if (!ignoreIntensities)
			{
				hasIntensitiesAboveZero |= (point.intensity != 0);
				if (hasIntensitiesAboveZero)
				{
					if (!intensities)
					{
						intensities = new ccScalarField("Intensity");
						if (!intensities->reserve(pointCount))
						{
							intensities->release();
							intensities = 0;
							ignoreIntensities = true;
						}
						else
						{
							for (size_t i = 0; i < cloud->size(); ++i)
							{
								//set the previous values!
								intensities->addElement(0);
							}
						}
					}

					if (intensities)
					{
						intensities->addElement(static_cast<ScalarType>(point.intensity));
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
			delete cloud;
			cloud = 0;
			if (intensities)
			{
				intensities->release();
			}
		}
		else
		{
			if (intensities)
			{
				if (hasIntensitiesAboveZero)
				{
					intensities->computeMinAndMax();
					int sfIdx = cloud->addScalarField(intensities);
					cloud->setCurrentDisplayedScalarField(sfIdx);
					cloud->showSF(true);
				}
				else
				{
					intensities->release();
					intensities = 0;
				}
			}

			if (hasColors)
			{
				if (hasColorsAboveZero)
				{
					cloud->showColors(true);
				}
				else
				{
					cloud->unallocateColors();
					hasColors = false;
				}
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
