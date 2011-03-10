/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: conversions.h 33238 2010-03-11 00:46:58Z rusu $
 *
 */
/**

\author Patrick Mihelich

Point Cloud conversions to/from sensor_msgs::PointCloud2.

**/

#ifndef PCL_ROS_CONVERSIONS_H_ 
#define PCL_ROS_CONVERSIONS_H_

#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "pcl/point_cloud.h"
#include "pcl/ros/point_traits.h"
#include "pcl/ros/for_each_type.h"
#include "pcl/exceptions.h"
#include <boost/foreach.hpp>

namespace pcl
{
  namespace detail
  {
    // For converting template point cloud to message.
    template<typename PointT>
    struct FieldAdder
    {
      FieldAdder (std::vector<sensor_msgs::PointField>& fields) : fields_ (fields) {};
      
      template<typename U> void operator() ()
      {
        sensor_msgs::PointField f;
        f.name = traits::name<PointT, U>::value;
        f.offset = traits::offset<PointT, U>::value;
        f.datatype = traits::datatype<PointT, U>::value;
        f.count = traits::datatype<PointT, U>::size;
        fields_.push_back (f);
      }

      std::vector<sensor_msgs::PointField>& fields_;
    };

    // For converting message to template point cloud.
    template<typename PointT>
    struct FieldMapper
    {
      FieldMapper (const std::vector<sensor_msgs::PointField>& fields, std::vector<FieldMapping>& map) : fields_ (fields), map_ (map) {};
      
      template<typename Tag> void operator() ()
      {
        const char* name = traits::name<PointT, Tag>::value;
        BOOST_FOREACH(const sensor_msgs::PointField& field, fields_)
        {
          if (field.name == name)
          {
            typedef traits::datatype<PointT, Tag> Data;
            assert (Data::value == field.datatype);
            /// @todo Verify size when common_msgs released again
            //assert(Data::size  == field.size);
            
            FieldMapping mapping;
            mapping.serialized_offset = field.offset;
            mapping.struct_offset = traits::offset<PointT, Tag>::value;
            mapping.size = sizeof (typename Data::type);
            map_.push_back (mapping);
            return;
          }
        }
        // didn't find it...
        std::stringstream ss;
        ss << "Failed to find a field named: '" << name << "'. Cannot convert message to PCL type.";
        ROS_ERROR ("%s", ss.str().c_str());
        throw pcl::InvalidConversionException(ss.str());
      }

      const std::vector<sensor_msgs::PointField>& fields_;
      std::vector<FieldMapping>& map_;
    };

  } //namespace detail

  template<typename PointT>
  void createMapping (const sensor_msgs::PointCloud2& msg, MsgFieldMap& field_map)
  {
    detail::FieldMapper<PointT> mapper (msg.fields, field_map);
    for_each_type< typename traits::fieldList<PointT>::type > (mapper);
  }

  template<typename PointT>  
  void fromROSMsg (const sensor_msgs::PointCloud2& msg, pcl::PointCloud<PointT>& cloud,
                   const MsgFieldMap& field_map)
  {
    // Copy info fields
    cloud.header   = msg.header;
    cloud.width    = msg.width;
    cloud.height   = msg.height;
    cloud.is_dense = msg.is_dense;

    // Copy point data
    uint32_t num_points = msg.width * msg.height;
    cloud.points.resize (num_points);
    uint8_t* cloud_data = (uint8_t*)&cloud.points[0];
    const uint8_t *row_data, *msg_data;

    for (uint32_t row = 0; row < msg.height; ++row)
    {
      row_data = &msg.data[row * msg.row_step];
      for (uint32_t col = 0; col < msg.width; ++col)
      {
        msg_data = row_data + col * msg.point_step;
        BOOST_FOREACH (const detail::FieldMapping& mapping, field_map)
        {
          memcpy (cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
        }
        cloud_data += sizeof (PointT);
      }
    }
  }

  template<typename PointT>  
  void fromROSMsg (const sensor_msgs::PointCloud2& msg, pcl::PointCloud<PointT>& cloud)
  {
    MsgFieldMap field_map;
    createMapping<PointT> (msg, field_map);
    fromROSMsg (msg, cloud, field_map);
  }

  template<typename PointT>
  void toROSMsg (const pcl::PointCloud<PointT>& cloud, sensor_msgs::PointCloud2& msg)
  {
    // Ease the user's burden on specifying width/height for unorganized datasets
    if (cloud.width == 0 && cloud.height == 0)
    {
      msg.width  = cloud.points.size ();
      msg.height = 1;
    }
    else
    {
      assert (cloud.points.size () == cloud.width * cloud.height);
      msg.height = cloud.height;
      msg.width  = cloud.width;
    }

    // Get the fields and create a field map
    std::vector<sensor_msgs::PointField> fields;
    for_each_type<typename traits::fieldList<PointT>::type> (detail::FieldAdder<PointT>(fields));
    MsgFieldMap field_map;
    detail::FieldMapper<PointT> mapper (fields, field_map);
    for_each_type< typename traits::fieldList<PointT>::type > (mapper);

    // Get the true size of the fields
    size_t fields_size = 0;
    // fields_size needs to contain the exact size of the fields, without the extra SSE padding
    // Example: PointXYZW is actually PointXYZ + 4 (1 float) + W + 3x4 (3 floats). 
    // * sizeof (PointT) = 32
    // * fields_size = 16
    int i = 0;
    assert (fields.size () == field_map.size ());
    BOOST_FOREACH (const detail::FieldMapping& mapping, field_map)
    {
      fields[i++].offset = fields_size;
      fields_size += mapping.size;
    }
    
    //fields_size = sizeof (PointT);
    //size_t data_size = sizeof (PointT) * cloud.points.size ();
    size_t data_size = fields_size * cloud.points.size ();

    // Fill point cloud binary data
    msg.data.resize (data_size);
    //memcpy (&msg.data[0], &cloud.points[0], data_size);

    msg.header     = cloud.header;
    //msg.point_step = sizeof (PointT);
    msg.point_step = fields_size;
    msg.row_step   = msg.point_step * msg.width;
    msg.is_dense   = cloud.is_dense;
    /// @todo msg.is_bigendian = ?;

    uint8_t *msg_data, *row_data;
    const uint8_t *cloud_data = (const uint8_t*)&cloud.points[0];

    for (uint32_t row = 0; row < msg.height; ++row)
    {
      row_data = &msg.data[row * msg.row_step];
      for (uint32_t col = 0; col < msg.width; ++col)
      {
        msg_data = row_data + col * msg.point_step;
        int i = 0;
        BOOST_FOREACH (const detail::FieldMapping& mapping, field_map)
        {
          // Copy from cloud_data to msg_data
          memcpy (msg_data + fields[i++].offset, cloud_data + mapping.struct_offset, mapping.size);
        }
        cloud_data += sizeof (PointT);
      }
    }

    // Fill fields metadata
    /// @todo Better to do this once somewhere?
    msg.fields.clear ();
    // Because of padding, the offset of each field might be displaced. We need to reconstruct the offsets.
    msg.fields = fields;
    //for_each_type< typename traits::fieldList<PointT>::type > (detail::FieldAdder<PointT>(msg.fields));
  }

   /** \brief Copy the RGB fields of a PointCloud into sensor_msgs::Image format
     * \param cloud the point cloud message
     * \param msg the resultant sensor_msgs::Image
     * CloudT cloud type, CloudT should be akin to pcl::PointCloud<pcl::PointXYZRGB>
     *  will throw std::runtime_error if there is a problem
     */
  template<typename CloudT> void
  toROSMsg(const CloudT& cloud, sensor_msgs::Image& msg) throw (std::runtime_error)
  {
    // Ease the user's burden on specifying width/height for unorganized datasets
    if (cloud.width == 0 && cloud.height == 0)
      throw std::runtime_error("Needs to be a dense like cloud!!");
    else
    {
      if(cloud.points.size () != cloud.width * cloud.height){
        throw std::runtime_error("The width and height do not match the cloud size!");
      }
      msg.height = cloud.height;
      msg.width = cloud.width;
    }

    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.step = msg.width * sizeof(uint8_t) * 3;
    msg.data.resize(msg.step * msg.height);
    for (size_t y = 0; y < cloud.height; y++)
    {
      for (size_t x = 0; x < cloud.width; x++)
      {
        uint8_t * pixel = &(msg.data[y * msg.step + x * 3]);
        memcpy(pixel, &cloud.at(x, y).rgb, 3 * sizeof(uint8_t));
      }
    }
  }
  /** \brief Copy the RGB fields of a PointCloud2 msg into sensor_msgs::Image format
   * \param cloud the point cloud message
   * \param msg the resultant sensor_msgs::Image
   * will throw std::runtime_error if there is a problem
   */
  inline void
  toROSMsg (const sensor_msgs::PointCloud2& cloud, sensor_msgs::Image& msg) throw (std::runtime_error)
  {
    int rgb_index = -1;
    // Get the index we need
    for (size_t d = 0; d < cloud.fields.size (); ++d)
      if (cloud.fields[d].name == "rgb")
      {
        rgb_index = d;
        break;
      }
    if(rgb_index == -1){
      throw std::runtime_error ("No rgb field!!");
    }
    if (cloud.width == 0 && cloud.height == 0)
      throw std::runtime_error ("Needs to be a dense like cloud!!");
    else
    {
      msg.height = cloud.height;
      msg.width = cloud.width;
    }
    int rgb_offset = cloud.fields[rgb_index].offset;
    int point_step = cloud.point_step;

    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.step = msg.width * sizeof(uint8_t) * 3;
    msg.data.resize (msg.step * msg.height);

    for (size_t y = 0; y < cloud.height; y++)
    {
      for (size_t x = 0; x < cloud.width; x++, rgb_offset += point_step)
      {
        uint8_t * pixel = &(msg.data[y * msg.step + x * 3]);
        memcpy (pixel, &(cloud.data[rgb_offset]), 3 * sizeof(uint8_t));
      }
    }
  }
}

#endif  //#ifndef PCL_ROS_CONVERSIONS_H_
