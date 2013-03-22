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

#include "sift_object_detector_lowe.h"
#include "sift_hough.h"
#include "sift_parameters.h"
#include "feature_indexer.h"
#include "sift_point_match.h"
#include "located_feature.h"
#include "sift_object_match_lowe.h"
#include "sift_object_pose_estimator.h"

#include "object_database.h"
#include "object_pose.h"
#include "task_pool.h"

#include <ntk/ntk.h>
#ifdef NESTK_USE_GSL
# include <gsl/gsl_cdf.h>
#endif

using namespace ntk;
using namespace cv;

namespace ntk
{

SiftObjectDetectorLowe::SiftObjectDetectorLowe(const SiftParameters& params)
    : SiftObjectDetector(params)
{
}

SiftObjectDetectorLowe::~SiftObjectDetectorLowe()
{
}

const SiftObjectMatchLowe& SiftObjectDetectorLowe::objectMatch(unsigned idx) const
{
    ntk_assert(idx < m_objects.size(), "Index too large");
    return *m_objects[idx];
}

SiftObjectMatchLowe& SiftObjectDetectorLowe::objectMatch(unsigned idx)
{
    ntk_assert(idx < m_objects.size(), "Index too large");
    return *m_objects[idx];
}

void SiftObjectDetectorLowe :: keepOnlyBestMatch()
{
    if (m_objects.size() < 1)
        return;

    int best_index = 0;
    double best_score = 0;
    foreach_idx(i, m_objects)
    {
        if (m_objects[i]->score() > best_score)
        {
            best_score = m_objects[i]->score();
            best_index = i;
        }
    }
    SiftObjectMatchLowePtr best_match = m_objects[best_index];
    m_objects.clear();
    m_objects.push_back(best_match);
}

void SiftObjectDetectorLowe::fillXmlElement(XMLNode& element) const
{
    foreach_const_it(it, m_objects, std::vector<SiftObjectMatchLowePtr>)
    {
        addXmlChild(element, "object_match", **it);
    }
}

void SiftObjectDetectorLowe::loadFromXmlElement(const XMLNode& element)
{
    ntk_assert(0, "not implemented");
}

void SiftObjectDetectorLowe :: initializeFindObjects()
{
    super::initializeFindObjects();
    m_objects.clear();
}

static int global_nb_points = 0;

struct FindClosestMatchTask : public Task
{
    FindClosestMatchTask(const ObjectDetectorData& data,
                         const FeatureIndexer& indexer,
                         const LocatedFeature& image_sift_point,
                         SiftHough& hough,
                         const SiftParameters& params)
        : Task("FindClosestMatch"),
          m_finder_data(data),
          m_indexer(indexer),
          m_image_sift_point(image_sift_point),
          m_hough(hough),
          m_params(params)
    {
    }

    virtual TaskStatus runTask()
    {
        FeatureIndexer::MatchResults result = m_indexer.findMatches(m_image_sift_point);

        if (result.matches.size() == 0) return taskHasFinishedAndShouldBeDeleted;
        foreach_idx(i, result.matches)
        {
            ntk_dbg_print(result.matches[i].score, 2);
            if (result.matches[i].score < (m_params.max_dist_ratio*m_params.max_dist_ratio))
            {
                ++global_nb_points;
                const double eps = 1e-10;
                SiftPointMatchPtr match (new SiftPointMatch(m_finder_data, m_image_sift_point, *result.matches[i].point,
                                                            1.0 / (result.matches[i].score + eps), -1 /*id*/));
                match->distance_ratio = result.matches[i].score;
                m_hough.vote(match);
            }
        }
        return taskHasFinishedAndShouldBeDeleted;
    }

private:
    const ObjectDetectorData& m_finder_data;
    const FeatureIndexer& m_indexer;
    const LocatedFeature& m_image_sift_point;
    SiftHough& m_hough;
    const SiftParameters& m_params;
};

void SiftObjectDetectorLowe ::filterMultipleInstanceClusters(SiftHough::clusters_type& clusters)
{
    SiftHough::clusters_type filtered_clusters;
    typedef std::map<int,SiftHough::clusters_type::const_iterator> best_clusters_type;
    best_clusters_type best_clusters;
    foreach_const_it(it, clusters, SiftHough::clusters_type)
    {
        int object_id = it->first.object_id.object_id;
        best_clusters_type::const_iterator best_it;
        best_it = best_clusters.find(object_id);
        if (best_it != best_clusters.end())
        {
            // Use a functor instead of bare number of features.
            if (best_it->second->second.size() > it->second.size())
                continue;
        }
        best_clusters[object_id] = it;
    }

    foreach_const_it(it, best_clusters, best_clusters_type)
    {
        SiftHough::clusters_type::const_iterator cluster_it = it->second;
        filtered_clusters[cluster_it->first] = cluster_it->second;
    }
    clusters = filtered_clusters;
}

namespace {

struct ClusterComparator
{
    ClusterComparator(const SiftHough::clusters_type& clusters)
        : clusters(clusters)
    {}

    const SiftHough::clusters_type& clusters;
    bool operator()(const SiftHough::clusters_type::const_iterator& lhs,
                    const SiftHough::clusters_type::const_iterator& rhs)
    {
        return lhs->second.size() > rhs->second.size();
    }
};

}

void SiftObjectDetectorLowe::findObjects()
{
    {
        TimeCount pcinit("init lowe-find-objects");
        initializeFindObjects();
        pcinit.stop();
    }

    if (m_image_sift_points.size() == 0)
    {
        finalizeFindObjects();
        return;
    }

    TimeCount pc("lowe-find-objects");

    int start_timestamp = ntk::Time::getMillisecondCounter();

    SiftHough hough(m_params.hough);

    {
        TaskPool taskpool(4);
        foreach_const_it(it, m_image_sift_points, std::vector<LocatedFeature*>)
        {
            taskpool.addTask(new FindClosestMatchTask(data(), siftDatabase(), **it, hough, m_params));
        }
        taskpool.waitForTasksToFinish();
    }
    pc.elapsedMsecs("find closest matches");

    ntk_dbg_print(global_nb_points, 1);

    SiftObjectPoseEstimator pose_estimator(siftParameters());

    ntk_dbg_print((int)hough.clusters().size(), 1);
    if (false && m_no_multiple_instances)
        filterMultipleInstanceClusters(hough.clusters());
    const SiftHough::clusters_type& clusters = hough.clusters();
    ntk_dbg_print((int)clusters.size(), 1);

    pc.elapsedMsecs("filter clusters");

    const int min_matches = 5; // FIXME: determine automatically.

    std::vector<SiftHough::clusters_type::const_iterator> sorted_iterators;
    sorted_iterators.reserve(clusters.size());
    foreach_const_it(it, clusters, SiftHough::clusters_type)
    {
        sorted_iterators.push_back(it);
    }
    std::sort(sorted_iterators.begin(), sorted_iterators.end(), ClusterComparator(clusters));

    int nb_processed_clusters = 0;
    foreach_const_it(sorted_it, sorted_iterators, std::vector<SiftHough::clusters_type::const_iterator>)
    {
        if (nb_processed_clusters > 10)
            break;
        ++nb_processed_clusters;

        const SiftHough::clusters_type::const_iterator& it = *sorted_it;
        ntk_dbg(2) << "[" << it->first << "] => " << it->second.size();
        if ((int)it->second.size() < min_matches) continue;

        const VisualObjectView& model = (*it->second.begin())->modelView();

        SiftPointMatchConstPtrSet compatible_point_matches;
        std::copy(stl_bounds(it->second),
                  std::inserter(compatible_point_matches, compatible_point_matches.begin()));

        SiftPointMatchConstPtrSet valid_point_matches = compatible_point_matches;

        if (valid_point_matches.size() < 1) continue;

        ntk::AffineTransform global_transform;

        const bool show_all = false; // FIXME: temp

        if (valid_point_matches.size() < min_matches)
            continue;

        ntk_dbg_print(std::string("========================") + model.name(), 1);
        ObjectPosePtr pose (new ObjectPose(m_data, model, global_transform));
        if (!pose_estimator.optimize(*pose, valid_point_matches, m_data))
        {
            ntk_dbg(1) << "Could not estimate last transform, aborting.";
            continue;
        }
        pose->finalize();

        ntk_dbg_print(valid_point_matches.size(), 2);

        if (valid_point_matches.size() < min_matches)
            continue;

        SiftObjectMatchLowePtr match(new SiftObjectMatchLowe(m_data, pose));
        match->nbvotes = valid_point_matches.size();
        match->pfa = computePFA(*match);
        match->hough_point = it->first;
        int end_timestamp = ntk::Time::getMillisecondCounter();
        match->setTimestamp(end_timestamp - start_timestamp);
        std::copy(stl_bounds(valid_point_matches),
                  std::inserter(match->point_matches, match->point_matches.begin()));

        std::vector<cv::Point3f> points_3d;
        foreach_const_it(it, valid_point_matches, SiftPointMatchConstPtrSet)
        {
            points_3d.push_back((*it)->obsPoint().location().p_image);
        }
        match->setMatchedPoints(points_3d);

        ntk_dbg_print(match->pfa, 2);
        ntk_dbg_print(m_params.pfa_threshold, 2);

        if (show_all || (match->pfa < m_params.pfa_threshold))
        {
            m_objects.push_back(match);
            if (m_no_multiple_instances)
                break;
#if 0
            std::copy(stl_bounds(valid_point_matches),
                      std::inserter(m_point_matches, m_point_matches.begin()));
#endif
        }
        ntk_dbg_print(bounding_box(match->projectedBoundingRect()), 2);
    }
    pc.elapsedMsecs("pose refinement");

    ntk_dbg_print(m_objects.size(), 1);
    filterIdenticalMatches();
    ntk_dbg_print(m_objects.size(), 1);

#if 0
    foreach_const_it(it_obj, m_objects, std::vector<SiftObjectMatchLowePtr>)
    {
        const SiftPointMatchConstPtrSet& valid_point_matches = (*it_obj)->point_matches;
        cv::Mat3b debug_img;
        data().image.rgb().copyTo(debug_img);
        int width = data().image.rgb().cols;
        int height = data().image.rgb().rows;

        cv::Mat3b debug_all_matches(height, width*2);
        {
            cv::Mat3b model_roi = debug_all_matches(Rect(width,0,width,height));
            cv::Mat3b obs_roi = debug_all_matches(Rect(0,0,width,height));
            data().image.rgb().copyTo(obs_roi);
            (*it_obj)->modelView().colorImage().copyTo(model_roi);
        }

        foreach_const_it(it, valid_point_matches, SiftPointMatchConstPtrSet)
        {
            const LocatedFeature& obs_p = (*it)->obsPoint();
            const LocatedFeature& model_p = (*it)->modelPoint();

            line(debug_all_matches, Point(obs_p.location().p_image.x, obs_p.location().p_image.y),
                 Point(model_p.location().p_image.x+width, model_p.location().p_image.y), Scalar(255,0,0));

            cv::Point3f p3d = (*it)->modelPoint().location().p_world;
            if (flt_eq(p3d.z, 0, 1e-5))
                continue;
            cv::Point3d p2d = (*it_obj)->pose()->pose3d().projectToImage(p3d);
            cv::circle(debug_img, Point(obs_p.location().p_image.x, obs_p.location().p_image.y),
                       5, Scalar(255,0,0));
            line(debug_img, Point(obs_p.location().p_image.x, obs_p.location().p_image.y),
                 Point(p2d.x, p2d.y), Scalar(0,0,255));
            double error = 0;
            error += ntk::math::sqr((p2d.x - obs_p.location().p_image.x));
            error += ntk::math::sqr((p2d.y - obs_p.location().p_image.y));
            error = sqrt(error);
            ntk_dbg_print(error, 1);
        }
        imwrite("/tmp/debug_matches.png", debug_img);
        imwrite("/tmp/debug_all_matches.png", debug_all_matches);
    }
#endif

    std::sort(stl_bounds(m_objects), SiftObjectMatchVoteLt());

#if 0
    foreach_const_it(it, m_objects, std::vector<SiftObjectMatchLowePtr>)
    {
        std::copy(stl_bounds((*it)->point_matches),
                  std::inserter(m_point_matches, m_point_matches.begin()));
        ntk_dbg_print(**it, 1);
    }
#endif

    finalizeFindObjects();
    pc.stop();
}

static
unsigned nbPointsInRegion(const std::vector<LocatedFeature*>& points,
                          const ntk::Polygon2d& region)
{
    const cv::Rect_<float> bbox = bounding_box(region);
    unsigned i = 0;
    foreach_const_it(it, points, std::vector<LocatedFeature*>)
    {
        cv::Point2f p((*it)->location().p_image.x, (*it)->location().p_image.y);
        if (bbox.contains(p)) ++i;
    }
    return i;
}

double computeLowePfa(const SiftParameters& params,
                      double nb_points_in_object_view,
                      double nb_points_in_db,
                      double nb_points_in_region,
                      double nb_matches)
{
    ntk_dbg(2) << "===============";
    ntk_dbg_print(nb_points_in_object_view, 2);
    ntk_dbg_print(nb_points_in_db, 2);
    ntk_dbg_print(nb_points_in_region, 2);
    ntk_dbg_print(nb_matches, 2);

    double d = nb_points_in_object_view / nb_points_in_db;
    const double l = params.hough.delta_location * params.hough.delta_location;
    const double s = 0.5;
    const double r = params.hough.delta_orientation / (2.0 * M_PI);
    double p = d * l * s * r;

    unsigned n = nb_points_in_region;
    unsigned k = nb_matches;

#ifdef NESTK_USE_GSL
    double bnk = 1.0 - gsl_cdf_binomial_P(k - 1, p, n);    
    // double bnk = gsl_cdf_binomial_Q(k - 1, p, n);
#else
// FIXME: Implement, extract, and expose something similar to:
// http://stackoverflow.com/questions/471935/user-warnings-on-msvc-and-gcc
#ifdef __GNUC__
#warning Compiling object detector without GSL, will give wrong results!
#endif
    double bnk = exp(binomial_logicdf_hoeffding_estimate(k - 1, p, n));
#endif
    if (k == 1) // special case.
        bnk = 1.0;

    const double pm = params.p_good_match;
    double pfa = pm / (pm + bnk);

    pfa = 1.0 - pfa;
    double logpfa = 0;
    if (pfa < 1e-20)
    {
        logpfa = log(1e-20);
    }
    else
    {
        logpfa = log(pfa);
    }
    // this is to keep some rought ordering between matches when the
    // minimal precision is reached.
    logpfa -= 1e-5 * log(double(k));

    return logpfa;
}

double SiftObjectDetectorLowe::computePFA(const SiftObjectMatchLowe& match) const
{
    const FeatureIndexer& sift_indexer = siftDatabase();
    const VisualObjectView& model_object = match.modelView();

    return computeLowePfa(m_params,
                          sift_indexer.nbPointsInObjectView(model_object),
                          sift_indexer.nbPoints(),
                          nbPointsInRegion(m_image_sift_points, match.projectedBoundingRect()),
                          match.nbvotes);

#if 0
    double d = double(sift_indexer.nbPointsInObjectView(model_object)) / sift_indexer.nbPoints();
    const double l = m_params.hough.delta_location * m_params.hough.delta_location;
    const double s = 0.5;
    const double r = m_params.hough.delta_orientation / (2.0 * M_PI);
    double p = d * l * s * r;

    ntk_dbg_print(d, 2);
    ntk_dbg_print(p, 2);


    // unsigned n = nbPointsInRegion(m_image_sift_points, match.projectedBoundingRect3D());
    // FIXME: use 3d projection, but use some geometrical features before.
    unsigned n = nbPointsInRegion(m_image_sift_points, match.projectedBoundingRect());
    unsigned k = match.nbvotes;

#ifdef NESTK_USE_GSL
    double bnk = 1.0 - gsl_cdf_binomial_P(k - 1, p, n);
    if (k == 1) // special case.
        bnk = 1.0;
    // double bnk = gsl_cdf_binomial_Q(k - 1, p, n);
#else
    double bnk = 1.0;
#endif

    const double pm = m_params.p_good_match;
    double pfa = pm / (pm + bnk);

    pfa = 1.0 - pfa;
    double logpfa = 0;
    if (pfa < 1e-20)
    {
        logpfa = log(1e-20);
        // this is to keep some rought ordering between matches when the
        // minimal precision is reached.
        logpfa -= log(k);
    }
    else
    {
        logpfa = log(pfa);
    }
    return logpfa;
#endif
}

namespace
{

struct FilterPredicate
{

    bool operator()(SiftObjectMatchLoweConstPtr lhs, SiftObjectMatchLoweConstPtr rhs) const
    {
        cv::Rect_<float> r1 = bounding_box(lhs->projectedBoundingRect());
        cv::Rect_<float> r2 = bounding_box(rhs->projectedBoundingRect());
        if (ntk::overlap_ratio(r1, r2) > 0.3 && &lhs->model() == &rhs->model())
            return true;
        // Very similar objects, take the closest one.
        // if (ntk::overlap_ratio(r1, r2) > 0.7)
        //  return true;
        return false;
    }

};

struct MatchLt
{
    bool operator()(SiftObjectMatchLoweConstPtr lhs, SiftObjectMatchLoweConstPtr rhs) const
    {
        return (lhs->score() < rhs->score());
    }
};

}

void SiftObjectDetectorLowe::filterIdenticalMatches()
{
    ntk_dbg_print(m_objects.size(), 1);
    std::sort(stl_bounds(m_objects), MatchLt());
    std::vector<SiftObjectMatchLowePtr> output;
#if 1
    ntk::keep_maximal_among_sorted_objects(std::back_inserter(output),
                                           m_objects.begin(), m_objects.end(),
                                           FilterPredicate());
#else
    std::copy(stl_bounds(m_objects), std::back_inserter(output));
#endif
    ntk_dbg_print(output.size(), 1);
    m_objects.swap(output);
}

} // end of avs
