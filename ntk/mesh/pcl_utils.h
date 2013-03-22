/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
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
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
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
#include <pcl/TextureMesh.h>

#ifdef HAVE_PCL_GREATER_THAN_1_5_1
#include <pcl/segmentation/planar_region.h>
#endif

#include <vector>

namespace ntk
{

typedef pcl::PointXYZRGBA PointXYZIndex;

class RGBDImage;
class Pose3D;

void removeExtrapoledTriangles(ntk::Mesh& surface, const ntk::Mesh& ground_cloud, float radius);
void removeExtrapoledBoundaries(ntk::Mesh& surface, const ntk::Mesh& ground_cloud, float radius, float max_dist_from_boundary);

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

inline pcl::PointXYZRGB toPcl(const cv::Point3f& p, const cv::Vec3b& c)
{
    pcl::PointXYZRGB r;
    r.x = p.x;
    r.y = p.y;
    r.z = p.z;
    r.r = c[0];
    r.g = c[1];
    r.b = c[2];
    return r;
}

inline cv::Point3f toOpencv(const pcl::PointXYZ& p)
{ return cv::Point3f(p.x, p.y, p.z); }

inline cv::Point3f toOpencv(const pcl::PointNormal& p)
{ return cv::Point3f(p.x, p.y, p.z); }

Eigen::Affine3f toPclCameraTransform(const Pose3D& pose);
Eigen::Affine3f toPclInvCameraTransform(const Pose3D& pose);

template <class PointT>
void vectorToPointCloud(pcl::PointCloud<PointT>& cloud,
                        const std::vector<cv::Point3f>& points,
                        const std::vector<int>& indices = std::vector<int>());

template <class PointT>
void removeNan(pcl::PointCloud<PointT>& clean_cloud, typename pcl::PointCloud<PointT>::ConstPtr source_cloud);

template <class PointT>
void rgbdImageToPointCloud(pcl::PointCloud<PointT>& cloud, const RGBDImage& image, bool keep_dense = false);

template <class PointT>
void rgbdImageToPointCloud(pcl::PointCloud<PointT>& cloud,
                           const RGBDImage& image,
                           const Pose3D& pose,
                           int subsampling_factor = 1,
                           bool keep_dense = false);

#ifdef HAVE_PCL_GREATER_THAN_1_5_1
void planarRegionToMesh(ntk::Mesh& mesh, const pcl::PlanarRegion<pcl::PointNormal> &region);
#endif

template <class PointT>
void pointCloudToMesh(ntk::Mesh& mesh,
                      const pcl::PointCloud<PointT>& cloud);

void meshToPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                      const ntk::Mesh& mesh);

void meshToPointCloud(pcl::PointCloud<pcl::PointNormal>& cloud,
                      const ntk::Mesh& mesh);

void meshToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                      const ntk::Mesh& mesh);

void polygonMeshToMesh(ntk::Mesh& mesh, const pcl::PolygonMesh &polygon);

void meshToPolygonMesh(pcl::PolygonMesh& polygon, const ntk::Mesh& mesh);
void meshToPolygonMeshWithNormals(pcl::PolygonMesh& polygon, const ntk::Mesh& mesh);
void meshToTextureMesh(pcl::TextureMesh& texture_mesh, const ntk::Mesh& mesh);

void setupPclTexMaterial(pcl::TexMaterial& mesh_material, const std::string& name);

void copyVertexColors (const ntk::Mesh& fromPoints, ntk::Mesh& toSurface, float max_dist);

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

        output.points = cloud.points;
        output.width = n_samples;
        output.height = 1;
        for (int i = 0; i < n_samples; ++i)
        {
            // int k = m_rng(cloud.points.size());
            // output.points[i] = cloud.points[k];
            int k = m_rng(output.points.size());
            std::swap(output.points[i], output.points[k]);
        }
        output.points.resize(n_samples);
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
