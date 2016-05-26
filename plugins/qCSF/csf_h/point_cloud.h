#ifndef WL_PCL_POINT_CLOUD_H_
#define WL_PCL_POINT_CLOUD_H_

#include "point_types.h"
#include "lasdefinitions.h"

namespace wl
{
  class PointCloud
  {
    public:
      /** \brief Default constructor. Sets \ref is_dense to true, \ref width
        * and \ref height to 0, and the \ref sensor_origin_ and \ref
        * sensor_orientation_ to identity.
        */
      PointCloud () : 
        _pheader (), _points ()
      {}

      /** \brief Copy constructor from point cloud subset
        * \param[in] pc the cloud to copy into this
        * \param[in] indices the subset to copy
        */
      PointCloud (const PointCloud  &pc, 
                  const std::vector<unsigned int> &indices) :
        _pheader (pc._pheader), _points (indices.size ())
      {
        // Copy the obvious
        //assert (indices.size () <= pc.size ());
        for (size_t i = 0; i < indices.size (); i++)
          _points[i] = pc._points[indices[i]];
      }

		PointCloud (const std::vector<unsigned int> &invert_indices, 
			const PointCloud  &pc) :
		_pheader (pc._pheader)/*, _points (pc.size()-invert_indices.size ())*/
		{
			// Copy the obvious
			//assert (indices.size () <= pc.size ());
			if (invert_indices.empty())
			{
				_points = pc._points;
			}

			else
			{
				int k = 0;
				int j = invert_indices[k];
				for (size_t i = 0; i < pc.size (); i++)
				{
					if (i != j)
					{
						_points.push_back(pc._points[i]);
					}

					else if (k<invert_indices.size()-1)
					{
						j = invert_indices[++k];
					}
				}	
			}
		}

      /** \brief Destructor. */
      virtual ~PointCloud () {}

      /** \brief Add a point cloud to the current cloud.
        * \param[in] rhs the cloud to add to the current cloud
        * \return the new cloud as a concatenation of the current cloud and the new given cloud
        */ 
      inline PointCloud&
      operator += (const PointCloud& rhs)
      {
        size_t nr__points = _points.size ();
        _points.resize (nr__points + rhs._points.size ());
        for (size_t i = nr__points; i < _points.size (); ++i)
          _points[i] = rhs._points[i - nr__points];

        return (*this);
      }

      /** \brief Add a point cloud to another cloud.
        * \param[in] rhs the cloud to add to the current cloud
        * \return the new cloud as a concatenation of the current cloud and the new given cloud
        */ 
      inline const PointCloud
      operator + (const PointCloud& rhs)
      {
        return (PointCloud (*this) += rhs);
      }

      inline std::vector< LASPoint >::iterator begin () { return (_points.begin ()); }
      inline std::vector< LASPoint >::iterator end ()   { return (_points.end ()); }
      inline std::vector< LASPoint >::const_iterator begin () const { return (_points.begin ()); }
      inline std::vector< LASPoint >::const_iterator end () const  { return (_points.end ()); }

      //capacity
      inline size_t size () const { return (_points.size ()); }
      inline bool empty () const { return _points.empty (); }

      /** \brief Resize the cloud
        * \param[in] n the new cloud size
        */
      inline void resize (size_t n) 
      { 
        _points.resize (n);
      }

      /** \brief Reserve the cloud
        * \param[in] n the new cloud capacity
        */
      inline void reserve (size_t n) 
      { 
        _points.reserve (n);
      }

      //element access
      inline const LASPoint& operator[] (size_t n) const { return (_points[n]); }
      inline LASPoint& operator[] (size_t n) { return (_points[n]); }
      inline const LASPoint& at (size_t n) const { return (_points.at (n)); }
      inline LASPoint& at (size_t n) { return (_points.at (n)); }
      inline const LASPoint& front () const { return (_points.front ()); }
      inline LASPoint& front () { return (_points.front ()); }
      inline const LASPoint& back () const { return (_points.back ()); }
      inline LASPoint& back () { return (_points.back ()); }

      /** \brief Insert a new point in the cloud, at the end of the container.
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] pt the point to insert
        */
      inline void 
      push_back (const LASPoint& pt)
      {
        _points.push_back (pt);
      }

      /** \brief Insert a new point in the cloud, given an iterator.
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] position where to insert the point
        * \param[in] pt the point to insert
        * \return returns the new position iterator
        */
      inline std::vector< LASPoint >::iterator 
      insert (std::vector< LASPoint >::iterator position, const LASPoint& pt)
      {
        std::vector< LASPoint >::iterator it = _points.insert (position, pt);
        return (it);
      }

      /** \brief Insert a new point in the cloud N times, given an iterator.
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] position where to insert the point
        * \param[in] n the number of times to insert the point
        * \param[in] pt the point to insert
        */
      inline void 
      insert (std::vector< LASPoint >::iterator position, size_t n, const LASPoint& pt)
      {
        _points.insert (position, n, pt);
      }

      /** \brief Insert a new range of _points in the cloud, at a certain position.
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] position where to insert the data
        * \param[in] first where to start inserting the _points from
        * \param[in] last where to stop inserting the _points from
        */
      inline void 
      insert (std::vector< LASPoint >::iterator position, 
	  std::vector< LASPoint >::iterator first, std::vector< LASPoint >::iterator last)
      {
        _points.insert (position, first, last);
      }

      /** \brief Erase a point in the cloud. 
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] position what data point to erase
        * \return returns the new position iterator
        */
      inline std::vector< LASPoint >::iterator 
      erase (std::vector< LASPoint >::iterator position)
      {
        std::vector< LASPoint >::iterator it = _points.erase (position); 
        return (it);
      }

      /** \brief Erase a set of _points given by a (first, last) iterator pair
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] first where to start erasing _points from
        * \param[in] last where to stop erasing _points from
        * \return returns the new position iterator
        */
      inline std::vector< LASPoint >::iterator 
      erase (std::vector< LASPoint >::iterator first, std::vector< LASPoint >::iterator last)
      {
        std::vector< LASPoint >::iterator it = _points.erase (first, last);
        return (it);
      }

      /** \brief Swap a point cloud with another cloud.
        * \param[in,out] rhs point cloud to swap this with
        */ 
      inline void 
      swap (PointCloud  &rhs)
      {
        this->_points.swap (rhs._points);
       
		LASheader ptemp = this->_pheader;
		this->_pheader = rhs._pheader;
		rhs._pheader = ptemp;

      }

      /** \brief Removes all _points in a cloud and sets the width and height to 0. */
      inline void 
      clear ()
      {
        _points.clear ();
      }


	  /** \brief The point cloud header. It contains information about the acquisition time. */
	  LASheader _pheader;

	  std::string _name;
	  /** \brief The point data. */
	  std::vector< LASPoint > _points;
  };

  //class LASvlrW
  //{
  //public:
	 // unsigned short reserved;
	 // char user_id[16]; 
	 // unsigned short record_id;
	 // unsigned short record_length_after_header;
	 // char description[32];
	 // char* data;
  //};

  //class LASvlr_geo_keysW
  //{
  //public:
	 // unsigned short key_directory_version;
	 // unsigned short key_revision;
	 // unsigned short minor_revision;
	 // unsigned short number_of_keys;
  //};

  //class LASvlr_key_entryW
  //{
  //public:
	 // unsigned short key_id;
	 // unsigned short tiff_tag_location;
	 // unsigned short count;
	 // unsigned short value_offset;
  //};

  //class LASheaderW
  //{
  //public:
	 // char file_signature[4];
	 // unsigned short file_source_id;
	 // unsigned short global_encoding;
	 // unsigned int project_ID_GUID_data_1;
	 // unsigned short project_ID_GUID_data_2;
	 // unsigned short project_ID_GUID_data_3;
	 // char project_ID_GUID_data_4[8];
	 // char version_major;
	 // char version_minor;
	 // char system_identifier[32];
	 // char generating_software[32];
	 // unsigned short file_creation_day;
	 // unsigned short file_creation_year;
	 // unsigned short header_size;
	 // unsigned int offset_to_point_data;
	 // unsigned int number_of_variable_length_records;
	 // unsigned char point_data_format;
	 // unsigned short point_data_record_length;
	 // unsigned int number_of_point_records;
	 // unsigned int number_of__points_by_return[5];
	 // double x_scale_factor;
	 // double y_scale_factor;
	 // double z_scale_factor;
	 // double x_offset;
	 // double y_offset;
	 // double z_offset;
	 // double max_x;
	 // double min_x;
	 // double max_y;
	 // double min_y;
	 // double max_z;
	 // double min_z;

	 // int user_data_in_header_size;
	 // char* user_data_in_header;

	 // LASvlrW* vlrs;
	 // LASvlr_geo_keysW* vlr_geo_keys;
	 // LASvlr_key_entryW* vlr_geo_key_entries;
	 // double* vlr_geo_double_params;
	 // char* vlr_geo_ascii_params;

	 // int user_data_after_header_size;
	 // char* user_data_after_header;

	 // LASheaderW()
	 // {
		//  for (unsigned int i = 0; i < sizeof(LASheaderW); i++) ((char*)this)[i] = 0;
		//  file_signature[0] = 'L'; file_signature[1] = 'A'; file_signature[2] = 'S'; file_signature[3] = 'F';
		//  version_major = 1;
		//  version_minor = 1;
		//  header_size = 227;
		//  offset_to_point_data = 227;
		//  point_data_record_length = 20;
		//  x_scale_factor = 0.01;
		//  y_scale_factor = 0.01;
		//  z_scale_factor = 0.01;
	 // };

	 // LASheaderW(const LASheaderW *pheader)
	 // {
		//  if (pheader)
		//  {
		//	  strcpy(file_signature, pheader->file_signature);
		//	  file_source_id = pheader->file_source_id;
		//	  global_encoding = pheader->global_encoding;
		//	  project_ID_GUID_data_1 = pheader->project_ID_GUID_data_1;
		//	  project_ID_GUID_data_2 = pheader->project_ID_GUID_data_2;
		//	  project_ID_GUID_data_3 = pheader->project_ID_GUID_data_3;
		//	  strcpy(project_ID_GUID_data_4, pheader->project_ID_GUID_data_4);

		//	  version_major = pheader->version_major;
		//	  version_minor = pheader->version_minor;
		//	  strcpy(system_identifier, pheader->system_identifier);
		//	  strcpy(generating_software, pheader->generating_software);

		//	  file_creation_day = pheader->file_creation_day;
		//	  file_creation_year = pheader->file_creation_year;
		//	  header_size = pheader->header_size;
		//	  offset_to_point_data = pheader->offset_to_point_data;
		//	  number_of_variable_length_records = pheader->number_of_variable_length_records;
		//	  point_data_format = pheader->point_data_format;
		//	  point_data_record_length = pheader->point_data_record_length;
		//	  number_of_point_records = pheader->number_of_point_records;

		//	  for (int i=0; ++i; i<5){
		//		  number_of__points_by_return[i] = pheader->number_of__points_by_return[i];
		//	  }
		//	  x_scale_factor = pheader->x_scale_factor;
		//	  y_scale_factor = pheader->y_scale_factor;
		//	  z_scale_factor = pheader->z_scale_factor;
		//	  x_offset = pheader->x_offset;
		//	  y_offset = pheader->y_offset;
		//	  z_offset = pheader->z_offset;
		//	  max_x = pheader->max_x;
		//	  min_x = pheader->min_x;
		//	  max_y = pheader->max_y;
		//	  min_y = pheader->min_y;
		//	  max_z = pheader->max_z;
		//	  min_z = pheader->min_z;

		//	  user_data_in_header_size = pheader->user_data_in_header_size;
		//	  user_data_in_header = pheader->user_data_in_header;

		//	  vlrs = pheader->vlrs;
		//	  vlr_geo_keys = pheader->vlr_geo_keys;
		//	  vlr_geo_key_entries = pheader->vlr_geo_key_entries;
		//	  vlr_geo_double_params = pheader->vlr_geo_double_params;
		//	  vlr_geo_ascii_params = pheader->vlr_geo_ascii_params;

		//	  user_data_after_header_size = pheader->user_data_after_header_size;
		//	  user_data_after_header = pheader->user_data_after_header;
		//  }
	 // }

	 // void clean_user_data_in_header()
	 // {
		//  if (user_data_in_header)
		//  {
		//	  header_size -= user_data_in_header_size;
		//	  delete [] user_data_in_header;
		//	  user_data_in_header = 0;
		//	  user_data_in_header_size = 0;
		//  }
	 // };

	 // void clean_vlrs()
	 // {
		//  if (vlrs)
		//  {
		//	  unsigned i;
		//	  for (i = 0; i < number_of_variable_length_records; i++)
		//	  {
		//		  offset_to_point_data -= (54 + vlrs[i].record_length_after_header);
		//		  delete [] vlrs[i].data;
		//	  }
		//	  delete [] vlrs;
		//	  vlrs = 0;
		//	  vlr_geo_keys = 0;
		//	  vlr_geo_key_entries = 0;
		//	  vlr_geo_double_params = 0;
		//	  vlr_geo_ascii_params = 0;
		//	  number_of_variable_length_records = 0;
		//  }
	 // };

	 // void clean_vlrs(int i)
	 // {
		//  if (i < (int)number_of_variable_length_records)
		//  {
		//	  offset_to_point_data -= (54 + vlrs[i].record_length_after_header);
		//	  delete [] vlrs[i].data;
		//	  number_of_variable_length_records--;
		//	  if (number_of_variable_length_records)
		//	  {
		//		  vlrs[i] = vlrs[number_of_variable_length_records];
		//	  }
		//  }
	 // };

	 // void clean_user_data_after_header()
	 // {
		//  if (user_data_after_header)
		//  {
		//	  offset_to_point_data -= user_data_after_header_size;
		//	  delete [] user_data_after_header;
		//	  user_data_after_header = 0;
		//	  user_data_after_header_size = 0;
		//  }
	 // };

	 // void clean()
	 // {
		//  clean_user_data_in_header();
		//  clean_vlrs();
		//  clean_user_data_after_header();
	 // };

	 // LASheaderW* simple_copy()
	 // {
		//  LASheaderW* ph = new LASheaderW;
		//  ph->number_of_point_records = this->number_of_point_records;

		//  ph->min_x = this->min_x;
		//  ph->max_x = this->max_x;
		//  ph->min_y = this->min_y;
		//  ph->max_y = this->max_y;
		//  ph->max_z = this->max_z;
		//  ph->min_z = this->min_z;

		//  return ph;
	 // }

	 // ~LASheaderW()
	 // {
		//  clean();
	 // };
  //};

}

#endif  //#ifndef WL_POINT_CLOUD_H_
