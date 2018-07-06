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
#include <laswriter.hpp>
#include <laspoint.hpp>

//Qt gui
#include <ui_saveLASFileDlg.h>

//system
#include <assert.h>
#include <string.h>

//! LAS Save dialog
class LASSaveDlg : public QDialog, public Ui::SaveLASFileDialog
{
public:
	explicit LASSaveDlg(QWidget* parent = 0)
		: QDialog(parent)
		, Ui::SaveLASFileDialog()
	{
		setupUi(this);
	}
};

//! Semi persistent save dialog
QSharedPointer<LASSaveDlg> s_saveDlg(0);

bool LASFWFFilter::canLoadExtension(const QString& upperCaseExt) const
{
	return (	upperCaseExt == "LAS"
			||	upperCaseExt == "LAZ" );
}

bool LASFWFFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type != CC_TYPES::POINT_CLOUD)
	{
		return false;
	}
	
	multiple = false;
	exclusive = true;
	return true;
}

CC_FILE_ERROR LASFWFFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	if (!entity || filename.isEmpty())
	{
		return CC_FERR_BAD_ARGUMENT;
	}

	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!cloud)
	{
		ccLog::Warning("[LAS_FWF] This filter can only save one cloud at a time!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	try
	{
		LASwriteOpener laswriteopener;
		laswriteopener.set_file_name(qPrintable(filename));
		assert(laswriteopener.active());

		if (QFileInfo(filename).suffix().endsWith('Z'))
		{
			laswriteopener.set_format(LAS_TOOLS_FORMAT_LAZ);
		}
		else
		{
			laswriteopener.set_format(LAS_TOOLS_FORMAT_LAS);
		}

		bool hasTime = (cloud->getScalarFieldIndexByName(LAS_FIELD_NAMES[LAS_TIME]) >= 0);
		bool hasFWF = cloud->hasFWF();
		bool hasColors = cloud->hasColors();
		bool hasIntensity = (cloud->getScalarFieldIndexByName(LAS_FIELD_NAMES[LAS_INTENSITY]) >= 0);
		bool isShifted = cloud->isShifted();

		if (hasFWF)
		{
			//try to compress the FWF data before creating the file
			cloud->compressFWFData();

			//save the FWF data before anything (in case it fails)
			//we save it in a separate file
			QFileInfo fi(filename);
			QString fwFilename = fi.absolutePath() + "/" + fi.completeBaseName() + ".wdp";
			QFile fwfFile(fwFilename);
			if (fwfFile.open(QFile::WriteOnly))
			{
				//write the	EVLR header first
				uint16_t reserved = 0;
				fwfFile.write((const char*)&reserved, 2);

				char userID[16] = { 0 };
				strcpy(userID, "LASF_Spec");
				fwfFile.write(userID, 16);

				uint16_t recordID = 65535;
				fwfFile.write((const char*)&recordID, 2);

				const ccPointCloud::SharedFWFDataContainer& data = cloud->fwfData();
				assert(data);
				uint64_t recordLength = data->size();
				fwfFile.write((const char*)&recordLength, 8);

				char description[32] = { 0 };
				strcpy(description, "WAVEFORM DATA PACKETS");
				fwfFile.write(description, 32);

				//eventually write the FWF data
				fwfFile.write((const char*)data->data(), data->size());
			}

			if (fwfFile.error() != QFile::NoError)
			{
				ccLog::Warning(QString("[LAS_FWF] An error occurred while writing the FWF data file!\n(%1)").arg(fwFilename));
				hasFWF = false;
			}
			else
			{
				ccLog::Print(QString("[LAS_FWF] FWF data file written: %1").arg(fwFilename));
			}
		}

		LASheader lasheader;
		{
			//header.SetDataFormatId(liblas::ePointFormat3);
			CCVector3d bbMin, bbMax;
			if (!cloud->getGlobalBB(bbMin, bbMax))
			{
				return CC_FERR_NO_SAVE;
			}

			if (isShifted)
			{
				lasheader.x_offset = -cloud->getGlobalShift().x;
				lasheader.y_offset = -cloud->getGlobalShift().y;
				lasheader.z_offset = -cloud->getGlobalShift().z;
			}
			else
			{
				lasheader.x_offset = bbMin.x;
				lasheader.y_offset = bbMin.y;
				lasheader.z_offset = bbMin.z;
			}

			//let the user choose between the original scale and the 'optimal' one (for accuracy, not for compression ;)
			bool hasScaleMetaData = false;
			CCVector3d lasScale(0, 0, 0);
			lasScale.x = cloud->getMetaData(LAS_SCALE_X_META_DATA).toDouble(&hasScaleMetaData);
			if (hasScaleMetaData)
			{
				lasScale.y = cloud->getMetaData(LAS_SCALE_Y_META_DATA).toDouble(&hasScaleMetaData);
				if (hasScaleMetaData)
				{
					lasScale.z = cloud->getMetaData(LAS_SCALE_Z_META_DATA).toDouble(&hasScaleMetaData);
				}
			}

			//optimal scale (for accuracy) --> 1e-9 because the maximum integer is roughly +/-2e+9
			CCVector3d diag = bbMax - bbMin;
			CCVector3d optimalScale(1.0e-9 * std::max<double>(diag.x, ZERO_TOLERANCE),
									1.0e-9 * std::max<double>(diag.y, ZERO_TOLERANCE),
									1.0e-9 * std::max<double>(diag.z, ZERO_TOLERANCE));

			if (parameters.alwaysDisplaySaveDialog)
			{
				if (!s_saveDlg)
					s_saveDlg.reset(new LASSaveDlg(0));
				s_saveDlg->bestAccuracyLabel->setText(QString("(%1, %2, %3)").arg(optimalScale.x).arg(optimalScale.y).arg(optimalScale.z));

				if (hasScaleMetaData)
				{
					s_saveDlg->origAccuracyLabel->setText(QString("(%1, %2, %3)").arg(lasScale.x).arg(lasScale.y).arg(lasScale.z));
				}
				else
				{
					s_saveDlg->origAccuracyLabel->setText("none");
					if (s_saveDlg->origRadioButton->isChecked())
						s_saveDlg->bestRadioButton->setChecked(true);
					s_saveDlg->origRadioButton->setEnabled(false);
				}

				s_saveDlg->exec();

				if (s_saveDlg->bestRadioButton->isChecked())
				{
					lasScale = optimalScale;
				}
				else if (s_saveDlg->customRadioButton->isChecked())
				{
					double s = s_saveDlg->customScaleDoubleSpinBox->value();
					lasScale = CCVector3d(s, s, s);
				}
				//else
				//{
				//	lasScale = lasScale;
				//}
			}
			else if (!hasScaleMetaData)
			{
				lasScale = optimalScale;
			}

			lasheader.x_scale_factor = lasScale.x;
			lasheader.y_scale_factor = lasScale.y;
			lasheader.z_scale_factor = lasScale.z;
			
#ifdef USE_LEGACY_FORMATS //LAS formats up to 5
			//LAS formats:
			//0 - base
			//1 - base + GPS
			//2 - base + RGB
			//3 - base + GPS + RGB
			//4 - base + GPS + FWF
			//5 - base + GPS + FWF + RGB
			lasheader.point_data_format = 0;
			lasheader.point_data_record_length = 20;
			//GPS time?
			if (hasTime)
			{
				lasheader.point_data_format = 1;
				lasheader.point_data_record_length = 20 + 8;
			}
			//FWF data?
			if (hasFWF)
			{
				//0, 1 --> 4
				lasheader.point_data_format = 4;
				lasheader.point_data_record_length = 20 + 8 + 29;
			}
			//Colors?
			if (hasColors)
			{
				if (lasheader.point_data_format == 4)
				{
					//4 --> 5
					lasheader.point_data_format = 5;
				}
				else
				{
					//0 --> 2
					//1 --> 3
					lasheader.point_data_format += 2;
				}
				lasheader.point_data_record_length += 6;
			}
#else
			//we use extended versions (up to 15 returns, up to 256 classes for classification, higher precision scan angle)
			//LAS formats:
			//6  - base (GPS included)
			//7  - base + RGB
			//8  - base + RGB + NIR (not used)
			//9  - base + FWF
			//10 - base + FWF + RGB + NIR
			lasheader.point_data_format = 6;
			lasheader.point_data_record_length = 30;
			//FWF data?
			if (hasFWF)
			{
				//6 --> 9
				lasheader.point_data_format = 9;
				lasheader.point_data_record_length = 30 + 29;
			}
			//Colors?
			if (hasColors)
			{
				lasheader.point_data_format += 1;
				if (lasheader.point_data_format == 6)
				{
					//6 --> 7
					lasheader.point_data_format = 7;
					lasheader.point_data_record_length += 6;
				}
				else if (lasheader.point_data_format == 9)
				{
					//9 --> 10
					lasheader.point_data_format = 10;
					lasheader.point_data_record_length += 6 + 2;
				}
				else
				{
					assert(false);
				}
			}
#endif

			//FWF descriptors and other parameters
			if (hasFWF)
			{
				//we always use an external file for FWF data
				lasheader.start_of_waveform_data_packet_record = 0;
				lasheader.global_encoding |= ((U16)4); // set external bit

				//if (!lasheader.vlr_wave_packet_descr)
				//{
				//	lasheader.vlr_wave_packet_descr = new LASvlr_wave_packet_descr*[256];
				//	for (int i = 0; i < 256; ++i)
				//	{
				//		lasheader.vlr_wave_packet_descr[i] = 0;
				//	}
				//}

				for (ccPointCloud::FWFDescriptorSet::const_iterator it = cloud->fwfDescriptors().begin(); it != cloud->fwfDescriptors().end(); ++it)
				{
					LASvlr_wave_packet_descr* d = new LASvlr_wave_packet_descr;
					{
						d->setBitsPerSample(static_cast<U8>(it.value().bitsPerSample));
						d->setCompressionType(static_cast<U8>(0));
						d->setDigitizerGain(static_cast<F64>(it.value().digitizerGain));
						d->setDigitizerOffset(static_cast<F64>(it.value().digitizerOffset));
						d->setNumberOfSamples(static_cast<F64>(it.value().numberOfSamples));
						d->setTemporalSpacing(static_cast<U32>(it.value().samplingRate_ps));
					}
					//lasheader.vlr_wave_packet_descr[it.key()] = d;

					lasheader.add_vlr(	"LASF_Spec",
										99 + static_cast<U16>(it.key()),
										26,
										(U8*)d,
										FALSE,
										"Waveform descriptor");
				}
			}
		}

		//init point 
		LASpoint laspoint;
		if (!laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0))
		{
			return CC_FERR_THIRD_PARTY_LIB_FAILURE;
		}

		// open laswriter
		LASwriter* laswriter = laswriteopener.open(&lasheader);
		if (laswriter == 0)
		{
			return CC_FERR_WRITING;
		}
		
		//LASwaveform13writer* fwfWriter = 0;
		//if (hasFWF)
		//{
		//	fwfWriter = laswriteopener.open_waveform13(&lasheader);
		//}
		
		//progress dialog
		QScopedPointer<ccProgressDialog> progressDialog(0);
		if (parameters.parentWidget)
		{
			progressDialog.reset(new ccProgressDialog(false, parameters.parentWidget));
			progressDialog->setWindowTitle(QObject::tr("Export LAS file"));
			progressDialog->setLabelText(QObject::tr("Points: %1").arg(cloud->size()));
			progressDialog->show();
			QCoreApplication::processEvents();
		}
		CCLib::NormalizedProgress nProgress(progressDialog.data(), cloud->size());

		//match cloud SFs with official LAS fields
		std::vector<LasField> fieldsToSave;
		LasField::GetLASFields(cloud, fieldsToSave);

		bool hasReturnNumberField = false;
		bool hasPlainClassificationField = false;
		for (const LasField& f : fieldsToSave)
		{
			if (f.type == LAS_RETURN_NUMBER)
			{
				hasReturnNumberField = true;
			}
			else if (f.type == LAS_CLASSIFICATION)
			{
				hasPlainClassificationField = true;
			}
		}

		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			nProgress.oneStep();

			const CCVector3* P = cloud->getPoint(i);

			//populate the point
			if (isShifted)
			{
				laspoint.set_X(static_cast<U32>(P->x / lasheader.x_scale_factor));
				laspoint.set_Y(static_cast<U32>(P->y / lasheader.y_scale_factor));
				laspoint.set_Z(static_cast<U32>(P->z / lasheader.z_scale_factor));
			}
			else
			{
				CCVector3d Pglobal = cloud->toGlobal3d<PointCoordinateType>(*P);
				laspoint.set_X(static_cast<U32>((Pglobal.x - lasheader.x_offset) / lasheader.x_scale_factor));
				laspoint.set_Y(static_cast<U32>((Pglobal.y - lasheader.y_offset) / lasheader.y_scale_factor));
				laspoint.set_Z(static_cast<U32>((Pglobal.z - lasheader.z_offset) / lasheader.z_scale_factor));
			}

			//additional fields
			U8 classification = 0;
			bool compositeClassif = false;
			for (std::vector<LasField>::const_iterator it = fieldsToSave.begin(); it != fieldsToSave.end(); ++it)
			{
				assert(it->sf);
				switch (it->type)
				{
				case LAS_X:
				case LAS_Y:
				case LAS_Z:
					assert(false);
					break;
				case LAS_INTENSITY:
					laspoint.set_intensity(static_cast<U16>(it->sf->getValue(i)));
					break;
				case LAS_RETURN_NUMBER:
					laspoint.set_return_number(static_cast<U8>(it->sf->getValue(i)));
					break;
				case LAS_NUMBER_OF_RETURNS:
					laspoint.set_number_of_returns(static_cast<U8>(it->sf->getValue(i)));
					break;
				case LAS_SCAN_DIRECTION:
					laspoint.set_scan_direction_flag(static_cast<U8>(it->sf->getValue(i)));
					break;
				case LAS_FLIGHT_LINE_EDGE:
					laspoint.set_edge_of_flight_line(static_cast<U8>(it->sf->getValue(i)));
					break;
				case LAS_CLASSIFICATION:
					laspoint.set_classification(static_cast<U8>(it->sf->getValue(i)));
					break;
				case LAS_SCAN_ANGLE_RANK:
					laspoint.set_scan_angle_rank(static_cast<U8>(it->sf->getValue(i)));
					break;
				case LAS_USER_DATA:
					laspoint.set_user_data(static_cast<U8>(it->sf->getValue(i)));
					break;
				case LAS_POINT_SOURCE_ID:
					laspoint.set_point_source_ID(static_cast<U16>(it->sf->getValue(i)));
					break;
				case LAS_RED:
				case LAS_GREEN:
				case LAS_BLUE:
					assert(false);
					break;
				case LAS_TIME:
					laspoint.set_gps_time(static_cast<F64>(it->sf->getValue(i)) + it->sf->getGlobalShift());
					break;
				case LAS_CLASSIF_VALUE:
					classification |= (static_cast<U8>(it->sf->getValue(i)) & 31); //5 first bits
					compositeClassif = true;
					break;
				case LAS_CLASSIF_SYNTHETIC:
					classification |= (static_cast<U8>(it->sf->getValue(i)) & 32); //6th bit
					compositeClassif = true;
					break;
				case LAS_CLASSIF_KEYPOINT:
					classification |= (static_cast<U8>(it->sf->getValue(i)) & 64); //7th bit
					compositeClassif = true;
					break;
				case LAS_CLASSIF_WITHHELD:
					classification |= (static_cast<U8>(it->sf->getValue(i)) & 128); //8th bit
					compositeClassif = true;
					break;
				case LAS_INVALID:
				default:
					assert(false);
					break;
				}
			}

			if (compositeClassif && !hasPlainClassificationField)
			{
				laspoint.set_classification(classification);
			}

			if (hasColors)
			{
				const ccColor::Rgb& rgb = cloud->getPointColor(i);
				//DGM: LAS colors are stored on 16 bits!
				laspoint.set_R(static_cast<U16>(rgb.r) << 8);
				laspoint.set_G(static_cast<U16>(rgb.g) << 8);
				laspoint.set_B(static_cast<U16>(rgb.b) << 8);
			}

			if (hasFWF)
			{
				ccWaveformProxy proxy = cloud->waveformProxy(i);
				if (proxy.isValid())
				{
					const ccWaveform& w = proxy.waveform();
					laspoint.wavepacket.setIndex(static_cast<U8>(proxy.descriptorID()));
					laspoint.wavepacket.setLocation(static_cast<F32>(w.echoTime_ps()));
					laspoint.wavepacket.setOffset(static_cast<U64>(w.dataOffset()) + 60); //EVLR header
					laspoint.wavepacket.setSize(static_cast<U32>(w.byteCount()));
					laspoint.wavepacket.setXt(static_cast<F32>(w.beamDir().x));
					laspoint.wavepacket.setYt(static_cast<F32>(w.beamDir().y));
					laspoint.wavepacket.setZt(static_cast<F32>(w.beamDir().z));

					if (!hasReturnNumberField)
					{
						//we know the return number
						laspoint.set_return_number(static_cast<U8>(w.returnIndex()));
					}
				}
				else
				{
					if (!hasReturnNumberField)
					{
						laspoint.set_return_number(0);
					}
				}
			}

			//write the point
			laswriter->write_point(&laspoint);

			//add it to the inventory
			laswriter->update_inventory(&laspoint);
		}

		//if (fwfWriter)
		//{
		//	delete fwfWriter;
		//	fwfWriter = 0;
		//}

		laswriter->close();
		delete laswriter;
		laswriter = 0;

		//if (lasheader.vlr_wave_packet_descr)
		//{
			//DGM: already handled by LASlib ('vlr' list)
			//for (int i = 0; i < 256; ++i)
			//{
			//	if (lasheader.vlr_wave_packet_descr[i])
			//		delete lasheader.vlr_wave_packet_descr[i];
			//}
		//	delete lasheader.vlr_wave_packet_descr;
		//}
	}
	catch (const std::bad_alloc&)
	{
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}
	catch (...)
	{
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	return CC_FERR_NO_ERROR;
}

bool PrepareLASField(ccScalarField*& field, LAS_FIELDS type, unsigned totalCount, unsigned currentCount, ScalarType defaultValue = 0)
{
	if (field)
	{
		assert(false);
		return true;
	}

	//try to reserve the memory to store the field values
	field = new ccScalarField(LAS_FIELD_NAMES[type]);
	if (!field->reserveSafe(totalCount))
	{
		ccLog::Warning(QString("[LAS] Not enough memory to load a field: '%1'").arg(LAS_FIELD_NAMES[type]));
		field->release();
		field = 0;
		return false;
	}
	field->link();

	//fill the previous points values (if any)
	for (unsigned i = 0; i < currentCount; ++i)
	{
		//set the previous values!
		field->addElement(defaultValue);
	}

	return true;
}

CC_FILE_ERROR LASFWFFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	
	//parameters
	bool ignoreDefaultFields = true;

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
		QScopedPointer<ccProgressDialog> progressDialog(0);
		if (parameters.parentWidget)
		{
			progressDialog.reset(new ccProgressDialog(true, parameters.parentWidget));
			progressDialog->setWindowTitle(QObject::tr("Import LAS file"));
			progressDialog->setLabelText(QObject::tr("Points: %1").arg(pointCount));
			progressDialog->setRange(0, 0/*static_cast<int>(pointCount)*/); //DGM FIXME: the progress doesn't update! Is it because it's in a separate DLL/plugin?
			progressDialog->show();
			QCoreApplication::processEvents();
		}
		CCLib::NormalizedProgress nProgress(progressDialog.data(), pointCount);

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
		uint64_t fwfDataOffset = 0;

		//DGM: from now on, we only enable scalar fields when we detect a valid value!
		std::vector< LasField::Shared > fieldsToLoad;
		{
			fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_CLASSIFICATION, 0, 0, 255))); //unsigned char: between 0 and 255
			//fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_CLASSIF_VALUE, 0, 0, 31))); //5 bits: between 0 and 31
			//fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_CLASSIF_SYNTHETIC, 0, 0, 1))); //1 bit: 0 or 1
			//fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_CLASSIF_KEYPOINT, 0, 0, 1))); //1 bit: 0 or 1
			//fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_CLASSIF_WITHHELD, 0, 0, 1))); //1 bit: 0 or 1
			fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_INTENSITY, 0, 0, 65535))); //16 bits: between 0 and 65536
			fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_TIME, 0, 0, -1.0))); //8 bytes (double) --> we use global shift!
			fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_RETURN_NUMBER, 1, 1, 7))); //3 bits: between 1 and 7
			fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_NUMBER_OF_RETURNS, 1, 1, 7))); //3 bits: between 1 and 7
			fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_SCAN_DIRECTION, 0, 0, 1))); //1 bit: 0 or 1
			fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_FLIGHT_LINE_EDGE, 0, 0, 1))); //1 bit: 0 or 1
			fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_SCAN_ANGLE_RANK, 0, -90, 90))); //signed char: between -90 and +90
			fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_USER_DATA, 0, 0, 255))); //unsigned char: between 0 and 255
			fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_POINT_SOURCE_ID, 0, 0, 65535))); //16 bits: between 0 and 65536
		}

		bool hasFWF = (lasreader->header.vlr_wave_packet_descr != 0);
		//QFile fwfFile;
		//bool hasFWFFile = false;
		LASwaveform13reader* fwfReader = 0;
		for (int fakeIteration = 0; hasFWF && fakeIteration < 1; ++fakeIteration)
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
				if (!fwfDataSource.seek(lasreader->header.start_of_waveform_data_packet_record))
				{
					ccLog::Warning(QString("Failed to find the associated waveform data packets header"));
					hasFWF = false;
					break;
				}
				QByteArray evlrHeader = fwfDataSource.read(60);
				if (evlrHeader.size() < 60)
				{
					ccLog::Warning(QString("Failed to read the associated waveform data packets"));
					hasFWF = false;
					break;
				}

				//get the number of bytes
				unsigned short reserved = *reinterpret_cast<const unsigned short*>(evlrHeader.constData() + 0);
				//char userID[16];
				//memcpy(userID, evlrHeader.constData() + 2, 16);
				unsigned short recordID = *reinterpret_cast<const unsigned short*>(evlrHeader.constData() + 18);
				assert(recordID == 65535);
				fwfDataCount = *reinterpret_cast<const uint64_t*>(evlrHeader.constData() + 20); //see LAS 1.4 EVLR header specifications
				if (fwfDataCount == 0)
				{
					ccLog::Warning(QString("Invalid waveform data packet size (0). We'll load all the remaining part of the file!"));
					fwfDataCount = fwfDataSource.size() - fwfDataSource.pos();
				}
				//char description[32];
				//memcpy(description, evlrHeader.constData() + 28, 32);
				fwfDataOffset = 60;
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

				if (fwfDataCount > 60)
				{
					QByteArray evlrHeader = fwfDataSource.read(60);
					const char* userID = reinterpret_cast<const char*>(evlrHeader.constData() + 2); //see LAS 1.4 EVLR header specifications
					if (strncmp(userID, "LASF_Spec", 9) == 0)
					{
						//this is a valid EVLR header, we can skip it
						fwfDataCount -= 60;
						fwfDataOffset += 60;
					}
					else
					{
						//this doesn't look like a valid EVLR
						fwfDataSource.seek(0);
					}
				}
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

				fwfDataSource.read((char*)container->data(), fwfDataCount);
				fwfDataSource.close();

				cloud->fwfData() = ccPointCloud::SharedFWFDataContainer(container);
			}
		}

		CCVector3d lasScale = CCVector3d(lasreader->header.x_scale_factor, lasreader->header.y_scale_factor, lasreader->header.z_scale_factor);
		CCVector3d lasShift = -CCVector3d(lasreader->header.x_offset, lasreader->header.y_offset, lasreader->header.z_offset);

		cloud->setMetaData(LAS_SCALE_X_META_DATA, QVariant(lasScale.x));
		cloud->setMetaData(LAS_SCALE_Y_META_DATA, QVariant(lasScale.y));
		cloud->setMetaData(LAS_SCALE_Z_META_DATA, QVariant(lasScale.z));

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
					w.setDataDescription(point.wavepacket.getOffset() - fwfDataOffset, point.wavepacket.getSize());
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
				bool preserveCoordinateShift = true;
				if (HandleGlobalShift(P, Pshift, preserveCoordinateShift, parameters, useLasShift))
				{
					if (preserveCoordinateShift)
					{
						cloud->setGlobalShift(Pshift);
					}
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
								cloud->addRGBColor(ccColor::black);
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
								cloud->setPointColor(i, ccColor::black);
							}
						}
					}

					ccColor::Rgb color(	static_cast<unsigned char>((point.rgb[0] >> colorBitDec) & 255),
										static_cast<unsigned char>((point.rgb[1] >> colorBitDec) & 255),
										static_cast<unsigned char>((point.rgb[2] >> colorBitDec) & 255));

					cloud->addRGBColor(color);
				}
			}

			//additional fields
			for (LasField::Shared& field : fieldsToLoad)
			{
				double value = 0.0;
				switch (field->type)
				{
				case LAS_INTENSITY:
					value = static_cast<double>(point.get_intensity());
					break;
				case LAS_RETURN_NUMBER:
					value = static_cast<double>(point.get_return_number());
					break;
				case LAS_NUMBER_OF_RETURNS:
					value = static_cast<double>(point.get_number_of_returns());
					break;
				case LAS_SCAN_DIRECTION:
					value = static_cast<double>(point.get_scan_direction_flag());
					break;
				case LAS_FLIGHT_LINE_EDGE:
					value = static_cast<double>(point.get_edge_of_flight_line());
					break;
				case LAS_CLASSIFICATION:
					value = static_cast<double>(point.get_classification());
					break;
				case LAS_SCAN_ANGLE_RANK:
					value = static_cast<double>(point.get_scan_angle_rank());
					break;
				case LAS_USER_DATA:
					value = static_cast<double>(point.get_user_data());
					break;
				case LAS_POINT_SOURCE_ID:
					value = static_cast<double>(point.get_point_source_ID());
					break;
				case LAS_TIME:
					value = point.get_gps_time();
					if (field->sf)
					{
						//shift time values (so as to avoid losing accuracy)
						value -= field->sf->getGlobalShift();
					}
					break;
				case LAS_CLASSIF_VALUE:
					value = static_cast<double>(point.get_classification() & 31); //5 bits
					break;
				case LAS_CLASSIF_SYNTHETIC:
					value = static_cast<double>(point.get_classification() & 32); //bit #6
					break;
				case LAS_CLASSIF_KEYPOINT:
					value = static_cast<double>(point.get_classification() & 64); //bit #7
					break;
				case LAS_CLASSIF_WITHHELD:
					value = static_cast<double>(point.get_classification() & 128); //bit #8
					break;
				default:
					//ignored
					continue;
				}

				if (field->sf)
				{
					ScalarType s = static_cast<ScalarType>(value);
					field->sf->addElement(s);
				}
				else
				{
					//first point? we track its value
					if (cloud->size() == 0)
					{
						field->firstValue = value;
					}

					if (	!ignoreDefaultFields
						||	value != field->firstValue
						||	(field->firstValue != field->defaultValue && field->firstValue >= field->minValue))
					{
						if (PrepareLASField(field->sf, field->type, pointCount, cloud->size(), field->firstValue))
						{
							if (field->type == LAS_TIME)
							{
								//we use the first value as 'global shift' (otherwise we will lose accuracy)
								field->sf->setGlobalShift(field->firstValue);
								value -= field->firstValue;
								ccLog::Warning("[LAS] Time SF has been shifted to prevent a loss of accuracy (%.2f)", field->firstValue);
								field->firstValue = 0;
							}

							ScalarType s = static_cast<ScalarType>(value);
							field->sf->addElement(s);
						}
						else
						{
							ccLog::Warning(QString("[LAS] Not enough memory: '%1' field will be ignored!").arg(LAS_FIELD_NAMES[field->type]));
							field->sf->release();
							field->sf = 0;
						}
					}
				}
			}

			cloud->addPoint(CCVector3::fromArray((P + Pshift).u));

			if (progressDialog && !nProgress.oneStep())
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
			for (LasField::Shared& field : fieldsToLoad)
			{
				if (field->sf)
				{
					field->sf->release();
				}
			}

			//release the cloud
			delete cloud;
			cloud = 0;
		}
		else
		{
			//associate the cloud with the various fields
			for (LasField::Shared& field : fieldsToLoad)
			{
				if (field->sf)
				{
					field->sf->computeMinAndMax();

					if (	field->type == LAS_CLASSIFICATION
						||	field->type == LAS_CLASSIF_VALUE
						||	field->type == LAS_CLASSIF_SYNTHETIC
						||	field->type == LAS_CLASSIF_KEYPOINT
						||	field->type == LAS_CLASSIF_WITHHELD
						||	field->type == LAS_RETURN_NUMBER
						||	field->type == LAS_NUMBER_OF_RETURNS)
					{
						int cMin = static_cast<int>(field->sf->getMin());
						int cMax = static_cast<int>(field->sf->getMax());
						field->sf->setColorRampSteps(std::min<int>(cMax - cMin + 1, 256));
						//classifSF->setMinSaturation(cMin);
					}
					else if (field->type == LAS_INTENSITY)
					{
						//set default grey color scale
						field->sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::GREY));
					}

					int sfIdx = cloud->addScalarField(field->sf);
					if (sfIdx == 0)
					{
						//enable the first one by default
						cloud->setCurrentDisplayedScalarField(sfIdx);
						cloud->showSF(true);
					}

					field->sf->release();
					field->sf = 0; //just in case
				}
				else
				{
					ccLog::Warning(QString("[LAS] All '%1' values were the same (%2)! We ignored them...").arg(field->type == LAS_EXTRA ? field->getName() : QString(LAS_FIELD_NAMES[field->type])).arg(field->firstValue));
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
