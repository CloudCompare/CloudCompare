/*
===============================================================================

  FILE:  LASdefinitions.h
  
  CONTENTS:
  
    Contains the Header and Point classes for reading and writing LIDAR points
    in the LAS format

      Version 1.2, April 29, 2008.
      Version 1.1, March 07, 2005.
      Version 1.0,   May 09, 2003

  PROGRAMMERS:
  
    martin isenburg@cs.unc.edu
  
  COPYRIGHT:
  
    copyright (C) 2007  martin isenburg@cs.unc.edu
    
    This software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  
  CHANGE HISTORY:
  
    17 January 2010 -- added arithmetic coder  
    7 September 2008 -- updated to support LAS format 1.2 
    11 June 2007 -- number of return / scan direction bitfield order was wrong
    18 February 2007 -- created after repairing 2 vacuum cleaners in the garden
  
===============================================================================
*/
#ifndef LAS_DEFINITIONS_H
#define LAS_DEFINITIONS_H

#define ENABLE_LAS_COMPRESSION
#undef ENABLE_LAS_COMPRESSION

#define LAS_COMPRESSION_RANGE 1
#define LAS_COMPRESSION_ARITHMETIC 2

#include <vector>
#include <string.h>

class LASpoint
{
public:
  int x;
  int y;
  int z;
  unsigned short intensity;
  unsigned char return_number : 3;
  unsigned char number_of_returns_of_given_pulse : 3;
  unsigned char scan_direction_flag : 1;
  unsigned char edge_of_flight_line : 1;
  unsigned char classification;
  char scan_angle_rank;
  unsigned char user_data;
  unsigned short point_source_ID;

  LASpoint()
  {
    x=0;
    y=0;
    z=0;
    intensity=0;
    edge_of_flight_line=0;
    scan_direction_flag=0;
    number_of_returns_of_given_pulse = 0;
    return_number = 0;
    classification = 0;
    scan_angle_rank = 0;
    user_data = 0;
    point_source_ID = 0;
  };
};

class LASvlr
{
public:
  unsigned short reserved;
  char user_id[16]; 
  unsigned short record_id;
  unsigned short record_length_after_header;
  char description[32];
  char* data;
};

class LASvlr_geo_keys
{
public:
  unsigned short key_directory_version;
  unsigned short key_revision;
  unsigned short minor_revision;
  unsigned short number_of_keys;
};

class LASvlr_key_entry
{
public:
  unsigned short key_id;
  unsigned short tiff_tag_location;
  unsigned short count;
  unsigned short value_offset;
};

class LASheader
{
public:
  char file_signature[4];
  unsigned short file_source_id;
  unsigned short global_encoding;
  unsigned int project_ID_GUID_data_1;
  unsigned short project_ID_GUID_data_2;
  unsigned short project_ID_GUID_data_3;
  char project_ID_GUID_data_4[8];
  char version_major;
  char version_minor;
  char system_identifier[32];
  char generating_software[32];
  unsigned short file_creation_day;
  unsigned short file_creation_year;
  unsigned short header_size;
  unsigned int offset_to_point_data;
  unsigned int number_of_variable_length_records;
  unsigned char point_data_format;
  unsigned short point_data_record_length;
  unsigned int number_of_point_records;
  unsigned int number_of_points_by_return[5];
  double x_scale_factor;
  double y_scale_factor;
  double z_scale_factor;
  double x_offset;
  double y_offset;
  double z_offset;
  double max_x;
  double min_x;
  double max_y;
  double min_y;
  double max_z;
  double min_z;

  int user_data_in_header_size;
  char* user_data_in_header;

  ::LASvlr* vlrs;
  ::LASvlr_geo_keys* vlr_geo_keys;
  ::LASvlr_key_entry* vlr_geo_key_entries;
  double* vlr_geo_double_params;
  char* vlr_geo_ascii_params;

  int user_data_after_header_size;
  char* user_data_after_header;

  bool points_have_gps_time; //added by xf
  bool points_have_rgb;

  LASheader()
  {
    for (unsigned int i = 0; i < sizeof(LASheader); i++) ((char*)this)[i] = 0;
    file_signature[0] = 'L'; file_signature[1] = 'A'; file_signature[2] = 'S'; file_signature[3] = 'F';
    version_major = 1;
    version_minor = 1;
    header_size = 227;
    offset_to_point_data = 227;
    point_data_record_length = 20;
    x_scale_factor = 0.01;
    y_scale_factor = 0.01;
    z_scale_factor = 0.01;

	points_have_gps_time = false;
	points_have_rgb = false;
  };

  LASheader(const LASheader& pheader)
  {
	  strcpy(file_signature, pheader.file_signature);
	  file_source_id = pheader.file_source_id;
	  global_encoding = pheader.global_encoding;
	  project_ID_GUID_data_1 = pheader.project_ID_GUID_data_1;
	  project_ID_GUID_data_2 = pheader.project_ID_GUID_data_2;
	  project_ID_GUID_data_3 = pheader.project_ID_GUID_data_3;
	  strcpy(project_ID_GUID_data_4, pheader.project_ID_GUID_data_4);

	  version_major = pheader.version_major;
	  version_minor = pheader.version_minor;
	  strcpy(system_identifier, pheader.system_identifier);
	  strcpy(generating_software, pheader.generating_software);

	  file_creation_day = pheader.file_creation_day;
	  file_creation_year = pheader.file_creation_year;
	  header_size = pheader.header_size;
	  offset_to_point_data = pheader.offset_to_point_data;
	  number_of_variable_length_records = pheader.number_of_variable_length_records;
	  point_data_format = pheader.point_data_format;
	  point_data_record_length = pheader.point_data_record_length;
	  number_of_point_records = pheader.number_of_point_records;

	  for (int i=0; i<5; ++i){
		  number_of_points_by_return[i] = pheader.number_of_points_by_return[i];
	  }
	  x_scale_factor = pheader.x_scale_factor;
	  y_scale_factor = pheader.y_scale_factor;
	  z_scale_factor = pheader.z_scale_factor;
	  x_offset = pheader.x_offset;
	  y_offset = pheader.y_offset;
	  z_offset = pheader.z_offset;
	  max_x = pheader.max_x;
	  min_x = pheader.min_x;
	  max_y = pheader.max_y;
	  min_y = pheader.min_y;
	  max_z = pheader.max_z;
	  min_z = pheader.min_z;

	  user_data_in_header_size = pheader.user_data_in_header_size;
	  user_data_in_header = pheader.user_data_in_header;

	  vlrs = pheader.vlrs;
	  vlr_geo_keys = pheader.vlr_geo_keys;
	  vlr_geo_key_entries = pheader.vlr_geo_key_entries;
	  vlr_geo_double_params = pheader.vlr_geo_double_params;
	  vlr_geo_ascii_params = pheader.vlr_geo_ascii_params;

	  user_data_after_header_size = pheader.user_data_after_header_size;
	  user_data_after_header = pheader.user_data_after_header;

	  points_have_gps_time = pheader.points_have_gps_time;
	  points_have_rgb = pheader.points_have_rgb;
  }

  void clean_user_data_in_header()
  {
    if (user_data_in_header)
    {
      header_size -= user_data_in_header_size;
      delete [] user_data_in_header;
      user_data_in_header = 0;
      user_data_in_header_size = 0;
    }
  };

  void clean_vlrs()
  {
    if (vlrs)
    {
      unsigned i;
      for (i = 0; i < number_of_variable_length_records; i++)
      {
        offset_to_point_data -= (54 + vlrs[i].record_length_after_header);
        delete [] vlrs[i].data;
      }
      delete [] vlrs;
      vlrs = 0;
      vlr_geo_keys = 0;
      vlr_geo_key_entries = 0;
      vlr_geo_double_params = 0;
      vlr_geo_ascii_params = 0;
      number_of_variable_length_records = 0;
    }
  };

  void clean_vlrs(int i)
  {
    if (i < (int)number_of_variable_length_records)
    {
      offset_to_point_data -= (54 + vlrs[i].record_length_after_header);
      delete [] vlrs[i].data;
      number_of_variable_length_records--;
      if (number_of_variable_length_records)
      {
        vlrs[i] = vlrs[number_of_variable_length_records];
      }
    }
  };

  void clean_user_data_after_header()
  {
    if (user_data_after_header)
    {
      offset_to_point_data -= user_data_after_header_size;
      delete [] user_data_after_header;
      user_data_after_header = 0;
      user_data_after_header_size = 0;
    }
  };

  void clean()
  {
    clean_user_data_in_header();
    clean_vlrs();
    clean_user_data_after_header();
  };

  LASheader* simple_copy()
  {
	  LASheader* ph = new LASheader;
	  ph->number_of_point_records = this->number_of_point_records;

	  ph->min_x = this->min_x;
	  ph->max_x = this->max_x;
	  ph->min_y = this->min_y;
	  ph->max_y = this->max_y;
	  ph->max_z = this->max_z;
	  ph->min_z = this->min_z;

	  return ph;
  }

  ~LASheader()
  {
    //clean();
  };
};

#endif
