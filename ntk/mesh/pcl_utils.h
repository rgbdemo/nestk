/**
 * This file is part of the nestk library.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#ifndef NESTK_USE_PCL
# error "NESTK_USE_PCL should be defined!"
# define NESTK_USE_PCL
#endif

#include <ntk/core.h>

#include <ntk/mesh/mesh.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#include <vector>

namespace ntk
{

typedef pcl::PointXYZRGBA PointXYZIndex;

class RGBDImage;
class Pose3D;

inline pcl::PointXYZ toPcl(const cv::Point3f& p)
{ return pcl::PointXYZ(p.x, p.y, p.z); }

inline pcl::PointNormal toPcl(const cv::Point3f& p, const cv::Point3f& n)
{
    pcl::PointNormal r;
    r.x = p.x;
    r.y = p.y;
    r.z = p.z;
    r.normal_x = n.x;
    r.normal_y = n.y;
    r.normal_z = n.z;
    return r;
}

inline cv::Point3f toOpencv(const pcl::PointXYZ& p)
{ return cv::Point3f(p.x, p.y, p.z); }

Eigen::Affine3f toPclCameraTransform(const Pose3D& pose);
Eigen::Affine3f toPclInvCameraTransform(const Pose3D& pose);

template <class PointT>
void vectorToPointCloud(pcl::PointCloud<PointT>& cloud,
                        const std::vector<cv::Point3f>& points,
                        const std::vector<int>& indices = std::vector<int>());

template <class PointT>
void rgbdImageToPointCloud(pcl::PointCloud<PointT>& cloud, const RGBDImage& image, bool keep_dense = false);

template <class PointT>
void rgbdImageToPointCloud(pcl::PointCloud<PointT>& cloud,
                           const RGBDImage& image,
                           const Pose3D& pose,
                           int subsampling_factor = 1,
                           bool keep_dense = false);

template <class PointT>
void pointCloudToMesh(ntk::Mesh& mesh,
                      const pcl::PointCloud<PointT>& cloud);

void meshToPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                      const ntk::Mesh& mesh);

void meshToPointCloud(pcl::PointCloud<pcl::PointNormal>& cloud,
                      const ntk::Mesh& mesh);

void polygonMeshToMesh(ntk::Mesh& mesh, pcl::PolygonMesh& polygon);

template <class PointT>
void sampledRgbdImageToPointCloud(pcl::PointCloud<PointT>& cloud,
                                  const RGBDImage& image,
                                  const Pose3D& pose,
                                  int n_samples);

template <class PointT>
class CloudSampler
{
public:
    CloudSampler() {}

    virtual void subsample(const pcl::PointCloud<PointT>& cloud,
                           pcl::PointCloud<PointT>& output,
                           int n_samples)
    {
        if (cloud.points.size() < (n_samples*1.5f))
        {
            output = cloud;
            return;
        }

        output.points.resize(n_samples);
        output.width = n_samples;
        output.height = 1;
        for (int i = 0; i < n_samples; ++i)
        {
            int k = m_rng(cloud.points.size());
            output.points[i] = cloud.points[k];
        }
    }

private:
    cv::RNG m_rng;
};

template <class PointT>
class NormalCloudSampler : public CloudSampler<PointT>
{
public:
    NormalCloudSampler() : m_bin_size_in_rad(M_PI/4.0) {}

    void setBinSizeInRadians(const float size) { m_bin_size_in_rad = size; }

    virtual void subsample(const pcl::PointCloud<PointT>& cloud,
                           pcl::PointCloud<PointT>& output,
                           int n_samples)
    {
        if (cloud.points.size() < (n_samples*1.5f))
        {
            output = cloud;
            return;
        }

        computeBinnedIndices(cloud);
        output.points.resize(n_samples);
        output.width = n_samples;
        output.height = 1;
        // FIXME: ensure that the same sample does not get selected twice?
        for (int i = 0; i < n_samples; ++i)
        {
            int bin = m_rng(m_binned_indices.size());
            int k = m_rng(m_binned_indices[bin].size());
            int index = m_binned_indices[bin][k];
            output.points[i] = cloud.points[index];
        }
    }

    int computeBin(const Eigen::Vector3f& normal) const
    {
        int n_bins_per_dim = std::ceil(M_PI / m_bin_size_in_rad);
        ntk_dbg_print(n_bins_per_dim, 2);
        float roll = acos(std::abs(normal.dot(Eigen::Vector3f(1,0,0)))) / m_bin_size_in_rad;
        float pitch = acos(std::abs(normal.dot(Eigen::Vector3f(0,1,0)))) / m_bin_size_in_rad;
        float yaw = acos(std::abs(normal.dot(Eigen::Vector3f(0,0,1)))) / m_bin_size_in_rad;
        int bin = roll + pitch*n_bins_per_dim + yaw*n_bins_per_dim*n_bins_per_dim;
        ntk_dbg(2) << cv::format("[%.2f %.2f %.2f] => %d", roll, pitch, yaw, bin);
        ntk_dbg(2) << cv::format("Normal [%.2f %.2f %.2f]", normal[0], normal[1], normal[2]);
        return bin;
    }

    void computeBinnedIndices(const pcl::PointCloud<PointT>& cloud)
    {
        std::map< int, std::vector<int> > histogram;
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            Eigen::Vector3f normal = cloud.points[i].getNormalVector3fMap();
            normal.normalize();
            int bin = computeBin(normal);
            histogram[bin].push_back(i);
        }
        ntk_dbg_print(histogram.size(), 1);

        m_binned_indices.resize(histogram.size());

        typedef std::map< int, std::vector<int> >::const_iterator const_iterator_type;
        int cur_index = 0;
        for (const_iterator_type it = histogram.begin(); it != histogram.end(); ++it)
        {
            m_binned_indices[cur_index] = it->second;
            ++cur_index;
        }
    }

protected:
    std::vector< std::vector<int> > m_binned_indices;
    float m_bin_size_in_rad;

private:
    cv::RNG m_rng;
};

}

#endif // PCL_UTILS_H
