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
//
//*********************** Last revision of this file ***********************
//$Author:: lpenasa                                                        $
//$Rev:: 2266                                                              $
//$LastChangedDate:: 2012-10-15 00:07:12 +0200 (lun., 15 oct. 2012)        $
//**************************************************************************
//
#include "PCDFilter.h"

//qCC_db
#include <ccPointCloud.h>

#include <iostream>
#include <fstream>
#include <sstream>

//for uint fixed-sized types
#include <stdint.h>


std::ostream& operator<<(std::ostream& s, const PCDFilter::PCDHeader & v)
{

    s << "version: ";
    s << "  " << v.version << std::endl;

    s << "number of fields: ";
    s << "  " << v.fields.size() << std::endl;

    s << "fields:";
    for (int i = 0; i < v.fields.size(); ++i)
    {
        s << "  " << v.fields[i] ;
    }
    s << std::endl;
    s << "sizes: ";

    for (int i = 0; i < v.size.size(); ++i)
    {
        s << "  " << v.size[i] ;
    }
    s << std::endl;
    s << "count: ";
    for (int i = 0; i < v.count.size(); ++i)
    {
        s << "  " << v.count[i] ;
    }
    s << std::endl;
    s << "width: ";
    s << "  " << v.width << std::endl;
    s << "height: ";
    s << "  " << v.height << std::endl;
    s << "viewpoint: ";
    for (int i = 0; i < v.viewpoint.size(); ++i)
    {
        s << "  " << v.viewpoint[i] ;
    }
    s << std::endl;

    s << "points: ";
    s << "  " << v.points << std::endl;
    s << "data: ";
    s << "  " << v.data << std::endl;
    s << "data begins at: ";
    s << "  " << v.data_position << std::endl;
    return (s);
}

CC_FILE_ERROR
PCDFilter::readFileHeader(const char * filename, PCDHeader &header)
{
    std::ifstream fs;
    fs.open (filename, std::ios::binary);
    if (!fs.is_open () || fs.fail ())
    {
        fs.close ();
        return CC_FERR_READING;
    }

    // Seek at the given offset
    //fs.seekg (offset, std::ios::beg);

    std::string line;


    for (size_t i = 0; i < 11 ; ++i) //read 11 lines
    {
        std::getline(fs, line);
        QString qline = line.c_str();

        //split the line
        QRegExp rx("(\\ |\\t|\\r)");
        QStringList wlist = qline.split(rx);

        int n_fields;

        if (wlist.at(0) == "#" )
            continue;

        if (wlist.at(0) == "VERSION")
        {
            header.version = wlist.at(1).toStdString();
            continue;
        }

        if  (wlist.at(0) == "FIELDS")
        {
            header.fields.clear();
            for (size_t i = 1; i < wlist.size(); ++i )
            {
                header.fields.push_back(wlist.at(i).toStdString());
            }

            n_fields = wlist.size() - 1;
            continue;
        }

        if (wlist.at(0) == "SIZE")
        {
            header.size.clear();
            header.size.resize(wlist.size() - 1);
            for (size_t i = 1; i < wlist.size(); ++i )
            {
                size_t val = wlist.at(i).toInt();
                header.size[i-1] = val;
            }
            continue;
        }

        // Get the field types
        if (wlist.at(0) == "TYPE")
        {
            header.type.clear();
            for (size_t i = 1; i < wlist.size(); ++i )
            {
                header.type.push_back(wlist.at(i).toInt());
            }
            continue;
        }

        // Get the field counts
        if (wlist.at(0) == "COUNT")
        {
            header.count.clear();
            for (size_t i = 1; i < wlist.size(); ++i )
            {
                header.count.push_back(wlist.at(i).toInt());
            }
        }

        if (wlist.at(0) == "WIDTH")
        {
            header.width = wlist.at(1).toInt();
            continue;
        }

        if (wlist.at(0) == "HEIGHT")
        {
            header.height = wlist.at(1).toInt();
            continue;
        }

        if (wlist.at (0) == "VIEWPOINT")
        {
            header.viewpoint.clear();
            for (size_t i = 1; i < wlist.size(); ++i )
            {
                header.viewpoint.push_back(wlist.at(i).toFloat());
            }
            continue;
        }

        if (wlist.at (0) == "POINTS")
        {
            header.points = wlist.at(1).toInt();
            continue;
        }

        if (wlist.at (0) == "DATA")
        {
            header.data = wlist.at(1).toStdString();
            header.data_position = static_cast<int> (fs.tellg ());
            continue;
        }
    }


    return CC_FERR_NO_ERROR;
}


CC_FILE_ERROR
PCDFilter::saveToFile(ccHObject* entity, const char* filename)
{
    return CC_FERR_NO_ERROR;
}

int
PCDFilter::getIDOfField(const std::string fieldname, const PCDHeader header)
{

    for (int i = 0; i < header.fields.size(); ++i)
    {
        if (header.fields.at(i) == fieldname)
        {
            return i;
            break;
        }
    }
    return -1;
}

int
PCDFilter::getSizeOfField(const std::string fieldname, const PCDHeader header)
{
    int id = getIDOfField(fieldname, header);
    if (id == -1)
        return -1;
    else
        return header.size.at(id) * header.count.at(id);
}

int
PCDFilter::getOffsetOfField(const std::string fieldname, const PCDHeader header)
{
    int field_position = getIDOfField(fieldname, header);

    //compute offset
    if (field_position == -1)
        return -1;

    int offset = 0;
    for (int i = 0; i < field_position; ++i)
    {
        offset += header.size.at(i) * header.count.at(i);
    }



    return offset;
}

int
PCDFilter::readScalarFieldMemMap(const std::string fieldname, const InputMemoryFile & mem_file, const PCDHeader header, ccScalarField &field , const int count)
{


    int fieldID = getIDOfField(fieldname, header);
    size_t offset = getOffsetOfField(fieldname, header);
    size_t full_field_size = getSizeOfField(fieldname, header);
    size_t field_element_size = header.size.at(fieldID);
    size_t multiplicity = header.count.at(fieldID);

    size_t point_step = 0;
    for (int i = 0; i < header.size.size(); ++i)
    {
        point_step += header.size[i] * header.count[i];
    }

    if (count > multiplicity)
        return -1;

    size_t n_points = header.height * header.width;
    field.reserve(n_points);

    size_t const_offset =  offset + count*field_element_size;

    void * value = malloc(field_element_size);


    for (int i = 0;  i < n_points; ++i)
    {

        memcpy(value, mem_file.data() + header.data_position + i*point_step + const_offset, field_element_size);

        if (field_element_size == 1) //char
            field.addElement( static_cast<float> ( *((char*) value)) );

        else if (field_element_size == 2) //short
            field.addElement( static_cast<float> ( *((short*) value)) );

        else if (field_element_size == 4) //float
            field.addElement(  *((float*) value) ) ;

        else if (field_element_size == 8) //double
            field.addElement( static_cast<float> ( *((double*) value)) );

    }

    field.computeMinAndMax();
    if (multiplicity == 1)
        field.setName(fieldname.c_str());

    else
    {
        std::stringstream newname;
        newname << fieldname << "_" << count;

        field.setName(newname.str().c_str());
    }




    return 1;
}

CC_FILE_ERROR
PCDFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{

    //we get the size of the file to open
    QFile file(filename);
    if (!file.exists())
        return CC_FERR_READING;

    PCDHeader header;
    readFileHeader(filename, header);

//    std::cout << header << std::endl;
    if (header.data == "binary")
    {
        loadFileBinaryMemMap(filename, container, header);
    }
    else if (header.data == "binary_compressed")
    {
        return CC_FERR_MALFORMED_FILE;
    }
    else if (header.data == "ascii")
    {
        return CC_FERR_MALFORMED_FILE;
    }

    return CC_FERR_NO_ERROR;
}





int
PCDFilter::readRGBMemMap(const InputMemoryFile & mem_file, const PCDHeader &header, ccPointCloud &cloud)
{
    int rgb_id = getIDOfField("rgb", header);
    if (rgb_id < 0)
        return -1;

    //point step
    size_t point_step = 0;
    for (int i = 0; i < header.size.size(); ++i)
    {
        point_step += header.size[i] * header.count[i];
    }

    size_t rgb_offset = getOffsetOfField("rgb", header);

    size_t n_points = header.width * header.height;

    cloud.reserveTheRGBTable();
    //    colorType color[3];
    uint32_t rgb;
    uint8_t r,g,b;

    for (size_t i =0; i < n_points; ++i) //for each point of the cloud
    {
        memcpy(&rgb, mem_file.data() + header.data_position + point_step * i + rgb_offset, sizeof(uint32_t));

        //unpack colors
        r = (rgb >> 16) & 0x0000ff;
        g = (rgb >> 8)  & 0x0000ff;
        b = (rgb)     & 0x0000ff;

        cloud.addRGBColor(r,g,b);
    }

    return 1;



}

int
PCDFilter::readNormalsMemMap(const InputMemoryFile & mem_file, const PCDHeader &header, ccPointCloud &cloud)
{
    int normal_x = getIDOfField("normal_x", header);
    int normal_y = getIDOfField("normal_y", header);
    int normal_z = getIDOfField("normal_z", header);

    if ((normal_x < 0)|| (normal_x < 0)||(normal_x < 0))
        return -1;

    //point step
    size_t point_step = 0;
    for (int i = 0; i < header.size.size(); ++i)
    {
        point_step += header.size[i] * header.count[i];
    }

    size_t x_offset = getOffsetOfField("normal_x", header);
    size_t y_offset = getOffsetOfField("normal_y", header);
    size_t z_offset = getOffsetOfField("normal_z", header);

    size_t x_size = getSizeOfField("normal_x", header);
    size_t y_size = getSizeOfField("normal_y", header);
    size_t z_size = getSizeOfField("normal_z", header);

    size_t n_points = header.width * header.height;

    cloud.reserveTheNormsTable();

    float n_x=0.0, n_y=0.0, n_z = 0.0;

    void *x_tmp = malloc(x_size);
    void *y_tmp = malloc(y_size);
    void *z_tmp = malloc(z_size);

    for (size_t i =0; i < n_points; ++i) //for each point of the cloud
    {
        memcpy(x_tmp, mem_file.data() + header.data_position + point_step * i + x_offset, x_size);
        memcpy(y_tmp, mem_file.data() + header.data_position + point_step * i + y_offset, y_size);
        memcpy(z_tmp, mem_file.data() + header.data_position + point_step * i + z_offset, z_size);


        //unpack normals
        if (x_size == 4) //is a float
            n_x = *(float *) x_tmp;

        else if (x_size == 8) //is double
            n_x = static_cast<float>(*(double *) x_tmp);

        if (y_size == 4) //is a float
            n_y = *(float *) y_tmp;

        else if (y_size == 8) //is double
            n_y = static_cast<float>(*(double *) y_tmp);

        if (z_size == 4) //is a float
            n_z = *(float *) z_tmp;

        else if (z_size == 8) //is double
            n_z = static_cast<float>(*(double *) z_tmp);

        cloud.addNorm(n_x, n_y, n_z);
    }

    return 1;



}

CC_FILE_ERROR
PCDFilter::loadFileBinaryMemMap(const char* filename, ccHObject& container, PCDHeader &header)
{
    std::ifstream fs;
    fs.open (filename, std::ios::binary);
    if (!fs.is_open () || fs.fail ())
    {
        fs.close ();
        return CC_FERR_READING;
    }
    fs.close();

    InputMemoryFile mem_file(filename);

    //point step
    size_t point_step = 0;
    for (int i = 0; i < header.size.size(); ++i)
    {
        point_step += header.size[i] * header.count[i];
    }

    //n_points
    size_t n_points  = header.height * header.width;

    ccPointCloud * cloud = new ccPointCloud;
    cloud->reserve(n_points);

    int x_pos = getOffsetOfField("x", header);
    int y_pos = getOffsetOfField("y", header);
    int z_pos = getOffsetOfField("z", header);

    if ((x_pos < 0 ) || (y_pos < 0) || (z_pos < 0))
        return CC_FERR_MALFORMED_FILE;

    //should be all the same size normally, but...
    int x_size = getSizeOfField("x", header);
    int y_size = getSizeOfField("y", header);
    int z_size = getSizeOfField("z", header);

    //try to print something from file
    CCVector3 point;

    //what I am going to do there probably is SHIT!! Be aware!
    //PLEASE CHECK HOW I AM CASTING TYPES IN THE FOR BELOW
    void * x = malloc( x_size);
    void * y = malloc( y_size);
    void * z = malloc( z_size);

    //READ GEOMETRY
    for (int i = 0; i < n_points; ++i)
    {
        memcpy(x, mem_file.data() + header.data_position + i*point_step + x_pos, x_size);
        memcpy(y, mem_file.data() + header.data_position + i*point_step + y_pos, y_size);
        memcpy(z, mem_file.data() + header.data_position + i*point_step + z_pos, z_size);

        //in the case someone save xyz as int this will be a problem ...

        if (x_size == 4)
            point[0] = *(float * ) x;
        else if (x_size == 8)
            point[0] = static_cast<float>(*(double * ) x);

        if (y_size == 4)
            point[1] = *(float * ) y;
        else if (y_size == 8)
            point[1] = static_cast<float>(*(double * ) y);

        if (z_size == 4)
            point[2] = *(float * ) z;
        else if (z_size == 8)
            point[2] = static_cast<float>(*(double * ) z);


        cloud->addPoint(point);
    }

    //READ ALL FIELDS THAT ARE NOT RGB DATA OR NORMALS
    for (int i = 0; i < header.fields.size(); ++i)
    {
        std::string this_field_name = header.fields.at(i);

        if ((this_field_name == "_") || (this_field_name == "x") || (this_field_name == "y") || (this_field_name == "z")
                || (this_field_name == "normal_x") || (this_field_name == "normal_y") || (this_field_name == "normal_z") ||(this_field_name == "rgb"))
            continue;

        int field_count = header.count.at(i);
        for (int j = 0; j < field_count; ++j)
        {
            ccScalarField  * field = new ccScalarField;

            readScalarFieldMemMap(this_field_name, mem_file, header, *field, j);

            if (field_count != 1)
            {   std::stringstream ss;
                ss << this_field_name << "_" << j;
                this_field_name = ss.str();
            }
            field->setName(this_field_name.c_str());

            field->computeMinAndMax();

            cloud->addScalarField(field);
        }
    }

    //NOW ALSO READ RGB DATA (readRGB automatically check if the field exists, if not it simply dos nothing)
    readRGBMemMap(mem_file, header, *cloud);

    //same for normals
    readNormalsMemMap(mem_file, header, *cloud);

    container.addChild(cloud);


    return CC_FERR_NO_ERROR;
}
