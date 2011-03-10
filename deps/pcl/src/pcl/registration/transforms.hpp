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
 *
 */

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in, const Eigen::Vector4f &centroid,
                         pcl::PointCloud<PointT> &cloud_out)
{
  cloud_out = cloud_in;

  // Subtract the centroid from cloud_in
  for (size_t i = 0; i < cloud_in.points.size (); ++i)
  {
    pcl::Vector4fMap p = cloud_out.points[i].getVector4fMap ();
    p -= centroid;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in, const std::vector<int> &indices,
                       const Eigen::Vector4f &centroid, pcl::PointCloud<PointT> &cloud_out)
{
  cloud_out.header = cloud_in.header;
  cloud_out.is_dense = cloud_in.is_dense;
  if (indices.size () == cloud_in.points.size ())
  {
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
  }
  else
  {
    cloud_out.width    = indices.size ();
    cloud_out.height   = 1;
  }
  cloud_out.points.resize (indices.size ());

  // Subtract the centroid from cloud_in
  for (size_t i = 0; i < cloud_in.points.size (); ++i)
  {
    pcl::Vector4fMapConst p_in  = cloud_in.points[indices[i]].getVector4fMap ();
    pcl::Vector4fMap p_out = cloud_out.points[i].getVector4fMap ();
    p_out = p_in - centroid;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in, const Eigen::Vector4f &centroid,
                       Eigen::MatrixXf &cloud_out)
{
  size_t npts = cloud_in.points.size ();

  cloud_out = Eigen::MatrixXf::Zero (4, npts);        // keep the data aligned

  for (size_t i = 0; i < npts; ++i)
  {
    pcl::Vector4fMapConst p = cloud_in.points[i].getVector4fMap ();
    cloud_out.block<4, 1> (0, i) = p - centroid;       // One column at a time
  }
  // Make sure we zero the 4th dimension out (1 row, N columns)
  cloud_out.block (3, 0, 1, npts).setZero ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::demeanPointCloud (const pcl::PointCloud<PointT> &cloud_in, const std::vector<int> &indices,
                       const Eigen::Vector4f &centroid, Eigen::MatrixXf &cloud_out)
{
  size_t npts = indices.size ();

  cloud_out = Eigen::MatrixXf::Zero (4, npts);        // keep the data aligned

  for (size_t i = 0; i < npts; ++i)
  {
    pcl::Vector4fMapConst p = cloud_in.points[indices[i]].getVector4fMap ();
    cloud_out.block<4, 1> (0, i) = p - centroid;       // One column at a time
  }
  // Make sure we zero the 4th dimension out (1 row, N columns)
  cloud_out.block (3, 0, 1, npts).setZero ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out,
                          const Eigen::Affine3f &transform)
{
  // In order to transform the data, we need to remove NaNs
  cloud_out.is_dense = true;
  if (&cloud_in != &cloud_out)
  {
    // Note: could be replaced by cloud_out = cloud_in
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.points.reserve (cloud_out.points.size ());
    cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
  }

  if (cloud_in.is_dense)
  {
    // If the dataset is dense, simply transform it!
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      Eigen::Map<Eigen::Vector3f> p_in (&cloud_in.points[i].x, 3, 1);
      Eigen::Map<Eigen::Vector3f> p_out (&cloud_out.points[i].x, 3, 1);
      p_out = transform * p_in;
    }
  }
  else
  {
    // If the data is dense, we don't need to check for NaN
    if (cloud_in.is_dense)
    {
      for (size_t i = 0; i < cloud_out.points.size (); ++i)
      {
        Eigen::Map<Eigen::Vector3f> p_in (&cloud_in.points[i].x, 3, 1);
        Eigen::Map<Eigen::Vector3f> p_out (&cloud_out.points[i].x, 3, 1);
        p_out = transform * p_in;
      }
    }
    else
    {
      // Dataset might contain NaNs and Infs, so check for them first,
      // otherwise we get errors during the multiplication (?)
      for (size_t i = 0; i < cloud_out.points.size (); ++i)
      {
        if (!pcl_isfinite (cloud_in.points[i].x) || 
            !pcl_isfinite (cloud_in.points[i].y) || 
            !pcl_isfinite (cloud_in.points[i].z))
          continue;
        Eigen::Map<Eigen::Vector3f> p_in (&cloud_in.points[i].x, 3, 1);
        Eigen::Map<Eigen::Vector3f> p_out (&cloud_out.points[i].x, 3, 1);
        p_out = transform * p_in;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out,
                                     const Eigen::Affine3f &transform)
{
  if (&cloud_in != &cloud_out)
  {
    // Note: could be replaced by cloud_out = cloud_in
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.points.reserve (cloud_out.points.size ());
    cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
  }

  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      Eigen::Map<Eigen::Vector3f> p_in (&cloud_in.points[i].x, 3, 1);
      Eigen::Map<Eigen::Vector3f> p_out (&cloud_out.points[i].x, 3, 1);
      p_out = transform * p_in;

      // Rotate normals
      Eigen::Map<Eigen::Vector3f> n_in (&cloud_in.points[i].normal[0], 3, 1);
      Eigen::Map<Eigen::Vector3f> n_out (&cloud_out.points[i].normal[0], 3, 1);
      n_out = transform.rotation() * n_in;
    }
  }
  // Dataset might contain NaNs and Infs, so check for them first.
  else
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      if (!pcl_isfinite (cloud_in.points[i].x) || 
          !pcl_isfinite (cloud_in.points[i].y) || 
          !pcl_isfinite (cloud_in.points[i].z))
        continue;
      Eigen::Map<Eigen::Vector3f> p_in (&cloud_in.points[i].x, 3, 1);
      Eigen::Map<Eigen::Vector3f> p_out (&cloud_out.points[i].x, 3, 1);
      p_out = transform * p_in;

      // Rotate normals
      Eigen::Map<Eigen::Vector3f> n_in (&cloud_in.points[i].normal[0], 3, 1);
      Eigen::Map<Eigen::Vector3f> n_out (&cloud_out.points[i].normal[0], 3, 1);
      n_out = transform.rotation() * n_in;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out,
                          const Eigen::Matrix4f &transform)
{
  if (&cloud_in != &cloud_out)
  {
    // Note: could be replaced by cloud_out = cloud_in
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.points.reserve (cloud_out.points.size ());
    cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
  }

  Eigen::Matrix3f rot   = transform.block<3, 3> (0, 0);
  Eigen::Vector3f trans = transform.block<3, 1> (0, 3);
  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      //Eigen::Map<Eigen::Vector3f> p_in (&cloud_in.points[i].x, 3, 1);
      //Eigen::Map<Eigen::Vector3f> p_out (&cloud_out.points[i].x, 3, 1);
      //p_out = rot * p_in + trans;
      cloud_out.points[i].getVector3fMap() = rot * cloud_in.points[i].getVector3fMap() + trans;
    }
  }
  // Dataset might contain NaNs and Infs, so check for them first.
  else
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      if (!pcl_isfinite (cloud_in.points[i].x) || 
          !pcl_isfinite (cloud_in.points[i].y) || 
          !pcl_isfinite (cloud_in.points[i].z))
        continue;
      //Eigen::Map<Eigen::Vector3f> p_in (&cloud_in.points[i].x, 3, 1);
      //Eigen::Map<Eigen::Vector3f> p_out (&cloud_out.points[i].x, 3, 1);
      //p_out = rot * p_in + trans;
      cloud_out.points[i].getVector3fMap() = rot * cloud_in.points[i].getVector3fMap() + trans;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out,
                                     const Eigen::Matrix4f &transform)
{
  if (&cloud_in != &cloud_out)
  {
    // Note: could be replaced by cloud_out = cloud_in
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.points.reserve (cloud_out.points.size ());
    cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
  }

  Eigen::Matrix3f rot   = transform.block<3, 3> (0, 0);
  Eigen::Vector3f trans = transform.block<3, 1> (0, 3);

  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      Eigen::Map<Eigen::Vector3f> p_in (&cloud_in.points[i].x, 3, 1);
      Eigen::Map<Eigen::Vector3f> p_out (&cloud_out.points[i].x, 3, 1);
      p_out = rot * p_in + trans;

      // Rotate normals
      Eigen::Map<Eigen::Vector3f> n_in (&cloud_in.points[i].normal[0], 3, 1);
      Eigen::Map<Eigen::Vector3f> n_out (&cloud_out.points[i].normal[0], 3, 1);
      n_out = rot * n_in;
    }
  }
  // Dataset might contain NaNs and Infs, so check for them first.
  else
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      if (!pcl_isfinite (cloud_in.points[i].x) || 
          !pcl_isfinite (cloud_in.points[i].y) || 
          !pcl_isfinite (cloud_in.points[i].z))
        continue;
      Eigen::Map<Eigen::Vector3f> p_in (&cloud_in.points[i].x, 3, 1);
      Eigen::Map<Eigen::Vector3f> p_out (&cloud_out.points[i].x, 3, 1);
      p_out = rot * p_in + trans;

      // Rotate normals
      Eigen::Map<Eigen::Vector3f> n_in (&cloud_in.points[i].normal[0], 3, 1);
      Eigen::Map<Eigen::Vector3f> n_out (&cloud_out.points[i].normal[0], 3, 1);
      n_out = rot * n_in;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out,
                          const Eigen::Vector3f &offset, const Eigen::Quaternionf &rotation)
{
  Eigen::Translation3f translation (offset);
  // Assemble an Eigen Transform
  Eigen::Affine3f t;
  t = translation * rotation;
  transformPointCloud (cloud_in, cloud_out, t);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
pcl::transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out,
                                     const Eigen::Vector3f &offset, const Eigen::Quaternionf &rotation)
{
  Eigen::Translation3f translation (offset);
  // Assemble an Eigen Transform
  Eigen::Affine3f t;
  t = translation * rotation;
  transformPointCloudWithNormals (cloud_in, cloud_out, t);
}
