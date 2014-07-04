//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "PCDFilter.h"

//Local
#include "AsciiFilter.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>

//Qt
#include <QFile>
#include <QTextStream>

//system
#include <assert.h>
#include <stdint.h> //for uint fixed-sized types

CC_FILE_ERROR PCDFilter::saveToFile(ccHObject* entity, QString filename)
{
	ccLog::Error("Not available yet!");

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR PCDFilter::loadFile(QString filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, CCVector3d* coordinatesShift/*=0*/)
{
	//read header
	PCDHeader header;
	{
		CC_FILE_ERROR result = readFileHeader(filename, header);
		if (result != CC_FERR_NO_ERROR)
			return result;
	}

	if (header.data == "ascii")
	{
		//we can read it as a standard ASCII file!
		AsciiOpenDlg::Sequence openSequence;
		unsigned index=0;
		for (std::vector<QString>::const_iterator it = header.fields.begin(); it != header.fields.end(); ++it,++index)
		{
			AsciiOpenDlg::SequenceItem si;
			si.header = *it;
			if (*it == "x")
				si.type = ASCII_OPEN_DLG_X;
			else if (*it == "y")
				si.type = ASCII_OPEN_DLG_Y;
			else if (*it == "z")
				si.type = ASCII_OPEN_DLG_Z;
			else if (*it == "normal_x")
				si.type = ASCII_OPEN_DLG_NX;
			else if (*it == "normal_y")
				si.type = ASCII_OPEN_DLG_NY;
			else if (*it == "normal_z")
				si.type = ASCII_OPEN_DLG_NZ;
			else if (*it == "rgb")
			{
				if (header.size[index] == 4)
					si.type = (header.type[index] == "F" ? ASCII_OPEN_DLG_RGB32f : ASCII_OPEN_DLG_RGB32i);
				else
				{
					ccLog::Warning("[PCDFilter] RGB fields other than 32 bits are not handled!");
				}
			}
			else if (*it != "_") //paddig fields
				si.type = ASCII_OPEN_DLG_Scalar;
			else
				si.type = ASCII_OPEN_DLG_None; //skip it

			openSequence.push_back(si);
		}
		
		qint64 fileSize = QFile(filename).size();
		return AsciiFilter().loadCloudFromFormatedAsciiFile(filename,
															container,
															openSequence,
															' ',
															10, //will be evaluated again
															fileSize,
															CC_MAX_NUMBER_OF_POINTS_PER_CLOUD,
															static_cast<unsigned>(header.lineCount),
															false,
															coordinatesShiftEnabled,
															coordinatesShift);
	}
	else if (header.data != "binary")
	{
		ccLog::Error(QString("[PCDFilter] '%1' PCD files not handled! Try to use the qPCL plugin instead").arg(header.data));
		return CC_FERR_WRONG_FILE_TYPE;
	}

	return loadFileBinaryMemMap(filename, container, header);
}

CC_FILE_ERROR PCDFilter::readFileHeader(QString filename, PCDHeader &header)
{
	//we get the size of the file to open
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
		return CC_FERR_READING;
	QTextStream inFile(&file);

	header.data_position = 0;
	header.lineCount = 0;

	while (true) //read 11 lines
	{
		QString nextline = inFile.readLine();
		if (nextline.isEmpty())
			break;

		//split the line
		QStringList wlist = nextline.split(QRegExp("(\\ |\\t|\\r)"));
		assert(wlist.size()>0);

		if (wlist.at(0) == "VERSION")
		{
			header.version = wlist.at(1);
		}		
		else if  (wlist.at(0) == "FIELDS")
		{
			header.fields.clear();
			for (int i = 1; i < wlist.size(); ++i )
			{
				header.fields.push_back(wlist.at(i));
			}
		}
		else if (wlist.at(0) == "SIZE")
		{
			header.size.clear();
			for (int i = 1; i < wlist.size(); ++i )
			{
				size_t val = wlist.at(i).toInt();
				header.size.push_back(val);
			}
		}
		else if (wlist.at(0) == "TYPE") // Get the field types
		{
			header.type.clear();
			for (int i = 1; i < wlist.size(); ++i )
			{
				header.type.push_back(wlist.at(i));
			}
		}
		else if (wlist.at(0) == "COUNT") // Get the field counts
		{
			header.count.clear();
			for (int i = 1; i < wlist.size(); ++i )
			{
				header.count.push_back(wlist.at(i).toInt());
			}
		}
		else if (wlist.at(0) == "WIDTH")
		{
			header.width = wlist.at(1).toInt();
		}
		else if (wlist.at(0) == "HEIGHT")
		{
			header.height = wlist.at(1).toInt();
		}
		else if (wlist.at (0) == "VIEWPOINT")
		{
			header.viewpoint.clear();
			for (int i = 1; i < wlist.size(); ++i )
			{
				header.viewpoint.push_back(wlist.at(i).toFloat());
			}
		}
		else if (wlist.at (0) == "POINTS")
		{
			header.points = wlist.at(1).toInt();
		}
		else if (wlist.at (0) == "DATA")
		{
			header.data = wlist.at(1);
			header.data_position = inFile.pos();
		}
		else if (wlist.at(0) != "#" ) //end of header
		{
			break;
		}
		++header.lineCount;
	}

	if (header.data_position == 0)
		return CC_FERR_MALFORMED_FILE;

	//update point stride
	header.pointStride = 0;
	{
		for (size_t i = 0; i < header.size.size(); ++i)
			header.pointStride += header.size[i] * header.count[i];
	}

	return CC_FERR_NO_ERROR;
}

int PCDFilter::GetIDOfField(const QString& fieldName, const PCDHeader& header)
{
	for (size_t i = 0; i < header.fields.size(); ++i)
		if (header.fields.at(i) == fieldName)
			return (int)i;

	return -1;
}

size_t PCDFilter::GetSizeOfField(int fieldID, const PCDHeader& header)
{
	return (fieldID>=0 ? header.size[fieldID] * header.count[fieldID] : 0);
}

size_t PCDFilter::GetOffsetOfField(int fieldID, const PCDHeader& header)
{
	//compute offset
	if (fieldID == -1)
	{
		assert(false);
		return 0;
	}

	size_t offset = 0;
	for (int i = 0; i < fieldID; ++i) //fieldID equals the field position
		offset += header.size[i] * header.count[i];

	return offset;
}

int PCDFilter::ReadScalarFieldMemMap(const QString& fieldName,
									 const InputMemoryFile& mem_file,
									 const PCDHeader& header,
									 ccScalarField &field,
									 size_t count)
{
	int fieldID = GetIDOfField(fieldName, header);
	if (fieldID<0)
		return -1;
	size_t offset = GetOffsetOfField(fieldID, header);
	size_t field_element_size = header.size[fieldID];
	size_t multiplicity = header.count[fieldID];

	if (count > multiplicity)
		return 0;

	size_t pointCount = header.height * header.width;
	if (!field.reserve((unsigned)pointCount))
	{
		ccLog::Error("[PCDFilter] Not enough memory!");
		return -1;
	}

	size_t const_offset =  offset + count*field_element_size;

	for (size_t i = 0;  i < pointCount; ++i)
	{
		char buffer[8]; //maximum element size
		memcpy(buffer, mem_file.data() + header.data_position + i*header.pointStride + const_offset, field_element_size);

		switch(field_element_size)
		{
		case 1: //char
			field.addElement( static_cast<float> ( *((char*)buffer)) );
			break;
		case 2: //short
			field.addElement( static_cast<float> ( *((short*)buffer)) );
			break;
		case 4: //float
			field.addElement(  *((float*)buffer) );
			break;
		case 8: //double
			field.addElement( static_cast<float> ( *((double*) buffer)) );
			break;
		}
	}

	field.computeMinAndMax();
	if (multiplicity == 1)
		field.setName(qPrintable(fieldName));
	else
		field.setName(qPrintable(fieldName+QString("_%1").arg(count)));

	return 1;
}

int PCDFilter::ReadRGBMemMap(const InputMemoryFile & mem_file, const PCDHeader &header, ccPointCloud &cloud)
{
	int rgb_id = GetIDOfField("rgb", header);
	if (rgb_id < 0)
		return 0;

	size_t rgb_offset = GetOffsetOfField(rgb_id, header);
	size_t pointCount = header.width * header.height;

	if (!cloud.reserveTheRGBTable())
	{
		ccLog::Warning("[PCDFilter] Not enough memory: failed to read RGB info!");
		return -1;
	}

	for (size_t i =0; i < pointCount; ++i) //for each point of the cloud
	{
		uint32_t rgb = *(const uint32_t*)(mem_file.data() + header.data_position + i*header.pointStride + rgb_offset);

		//unpack colors
		uint8_t r = ((rgb >> 16)	& 0x0000ff);
		uint8_t g = ((rgb >> 8)		& 0x0000ff);
		uint8_t b = ((rgb)			& 0x0000ff);

		cloud.addRGBColor(r,g,b);
	}
	cloud.showColors(true);

	return 1;
}

int PCDFilter::ReadNormalsMemMap(const InputMemoryFile & mem_file, const PCDHeader &header, ccPointCloud &cloud)
{
	int normal_x = GetIDOfField("normal_x", header);
	int normal_y = GetIDOfField("normal_y", header);
	int normal_z = GetIDOfField("normal_z", header);

	if (normal_x < 0 || normal_y < 0 || normal_z < 0)
		return 0;

	size_t pointCount = header.width * header.height;
	assert(pointCount == cloud.capacity()); //cloud should already have been initialized

	if (!cloud.reserveTheNormsTable())
	{
		ccLog::Error("[PCDFilter] Not enough memory: failed to read normals info!");
		return -1;
	}

	size_t offsets[3] = {	GetOffsetOfField(normal_x, header),
							GetOffsetOfField(normal_y, header),
							GetOffsetOfField(normal_z, header)};

	size_t sizes[3] = {	GetSizeOfField(normal_x, header),
						GetSizeOfField(normal_y, header),
						GetSizeOfField(normal_z, header)};

	//for each point of the cloud
	for (size_t i=0; i < pointCount; ++i)
	{
		//unpack normals
		CCVector3 N;
		for (unsigned char j=0; j<3; ++j)
		{
			const char* tmp = mem_file.data() + header.data_position + i*header.pointStride + offsets[j];
			if (sizes[j] == 4) //is a float
				N.u[j] = (PointCoordinateType)(*((float*)tmp));
			else if (sizes[j] == 8) //is double
				N.u[j] = (PointCoordinateType)(*((double*)tmp));
		}

		cloud.addNorm(N);
	}
	cloud.showNormals(true);

	return 1;
}

CC_FILE_ERROR PCDFilter::loadFileBinaryMemMap(QString filename, ccHObject& container, const PCDHeader &header)
{
	//Should be ok if we are here (header has already been successfully read)
	//if (!QFile(filename).exists())
	//	return CC_FERR_READING;

	const int x = GetIDOfField("x", header);
	const int y = GetIDOfField("y", header);
	const int z = GetIDOfField("z", header);

	if (x < 0 || y < 0 || z < 0)
		return CC_FERR_MALFORMED_FILE;

	//load file in memory
	InputMemoryFile mem_file(qPrintable(filename));

	//allocate point cloud
	size_t pointCount = header.height * header.width;
	ccPointCloud * cloud = new ccPointCloud;
	if (!cloud->reserve((unsigned)pointCount))
	{
		ccLog::Error("[PCDFilter] Not enough memory!");
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	const size_t offsets[3] = {	GetOffsetOfField(x, header),
							GetOffsetOfField(y, header),
							GetOffsetOfField(z, header)};

	const size_t sizes[3] = {	GetSizeOfField(x, header),
						GetSizeOfField(y, header),
						GetSizeOfField(z, header)};

	const bool floatType[3] = {	header.type[x] == "F",
							header.type[y] == "F",
							header.type[z] == "F"};

	//read points
	{
		for (size_t i = 0; i < pointCount; ++i)
		{
			const char* buffer = mem_file.data() + header.data_position + i*header.pointStride;
			CCVector3 point;
			for (unsigned char j=0; j<3; ++j)
			{
				if (floatType[j])
				{
					if (sizes[j] == 4)
						point.u[j] = (PointCoordinateType)(*((float*)(buffer+offsets[j])));
					else if (sizes[j] == 8)
						point.u[j] = (PointCoordinateType)(*((double*)(buffer+offsets[j])));
				}
				else
				{
					if (sizes[j] == 4)
						point.u[j] = (PointCoordinateType)(*((int32_t*)(buffer+offsets[j])));
					else if (sizes[j] == 8)
						point.u[j] = (PointCoordinateType)(*((int64_t*)(buffer+offsets[j])));
				}
			}

			cloud->addPoint(point);
		}
	}

	//READ ALL FIELDS THAT ARE NOT RGB DATA OR NORMALS AS SCALAR FIELDS
	{
		for (size_t i = 0; i < header.fields.size(); ++i)
		{
			QString this_field_name = header.fields[i];

			if (this_field_name == "_" //paddig fields
				|| this_field_name == "x" || this_field_name == "y" || this_field_name == "z"
				|| this_field_name == "normal_x" || this_field_name == "normal_y" || this_field_name == "normal_z"
				|| this_field_name == "rgb")
				continue;

			size_t field_count = header.count[i];
			for (size_t j = 0; j < field_count; ++j)
			{
				ccScalarField* field = new ccScalarField();

				if (ReadScalarFieldMemMap(this_field_name, mem_file, header, *field, j) < 0)
				{
					field->release();
					ccLog::Warning(QString("[PCDFilter] Failed to read scalar field %1!").arg(j));
					continue;
				}

				if (field_count != 1)
					this_field_name.append(QString("_%1").arg(j));

				field->setName(qPrintable(this_field_name));
				field->computeMinAndMax();

				cloud->addScalarField(field);
				cloud->showSF(true);
			}
		}
	}

	//NOW ALSO READ RGB DATA (readRGB automatically check if the field exists, if not it simply does nothing)
	ReadRGBMemMap(mem_file, header, *cloud);
	//same for normals
	ReadNormalsMemMap(mem_file, header, *cloud);

	container.addChild(cloud);

	return CC_FERR_NO_ERROR;
}
