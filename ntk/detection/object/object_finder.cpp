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

#include "object_finder.h"
#include "object_detector.h"
#include "object_database.h"
#include "visual_object.h"
#include "sift_object_detector_lowe.h"
#include "vfh_object_detector.h"
#include "feature_indexer.h"
#include "sift_parameters.h"

using namespace ntk;

namespace ntk
{

ObjectFinderParams :: ObjectFinderParams()
{
    use_tracking = false;
    object_detector = "sift";

    feature.sift_database_type = "kdt";
    feature.feature_type = LocatedFeature :: featureTypeFromName("sift");

    detection_threshold = -10;

    // avs specific
    time_limit_in_msecs = 1500;
    use_event_collector = false;

    keep_only_best_match = false;
}

SiftObjectDetectorPtr ObjectFinder :: createSiftObjectDetector()
{
    m_params.feature.pfa_threshold = m_params.detection_threshold;
    return toPtr(new SiftObjectDetectorLowe(m_params.feature));
}

#ifdef NESTK_DISABLED_ADS_NOT_YET_IMPORTED
AdsObjectDetectorPtr ObjectFinder :: createAdsObjectDetector()
{
    ntk_ensure(!m_params.ads_behavior.empty(), "ads-behavior required.");

    AdsObjectDetectorPtr finder
            = toPtr(new AdsObjectDetector(m_params.ads_behavior, m_params.feature));

    finder->setThresholdFactor(m_params.detection_threshold);
    finder->setUserTimeLimitInMsecs(m_params.time_limit_in_msecs);
    if (!m_params.use_event_collector)
        finder->disableEventCollector();
    return finder;
}
#endif

void ObjectFinder::initialize(const ObjectFinderParams& params)
{
    m_params = params;

    if (params.object_detector == "sift")
        m_detector = createSiftObjectDetector();
#ifdef NESTK_DISABLED_ADS_NOT_YET_IMPORTED
    else if (params.object_detector == "ads")
        m_detector = createAdsObjectDetector();
#endif
    else if (params.object_detector == "fpfh")
    {
        m_detector = toPtr(new VFHObjectDetector());
    }
    else
        ntk_throw_exception("Detector not supported: " + params.object_detector);

    m_database = ObjectDatabasePtr(new ObjectDatabase(m_params.object_database));
    if (m_database->nbVisualObjects() == 0) fatal_error("Database directory is empty.");
    m_detector->setObjectDatabase(m_database);
}

void ObjectFinder :: resetTrackers()
{
    m_trackers.clear();
}

void ObjectFinder :: processNewImage(const ntk::RGBDImage& image)
{
    const int max_frames_without_detect = 2;

    m_detector->setAnalyzedImage(image);
    m_detector->findObjects();
    if (m_params.keep_only_best_match)
        m_detector->keepOnlyBestMatch();

    if (!m_params.use_tracking)
        return;

    foreach_idx(tracker_i, m_trackers)
    {
        m_trackers[tracker_i]->prepareForNewFrame();
    }

    // Update existing trackers.
    std::vector<bool> tracked_matches(m_detector->nbObjectMatches(), false);
    foreach_idx(tracker_i, m_trackers)
    {
        double best_mesure = 0;
        int best_match = -1;
        for (int match_i = 0; match_i < m_detector->nbObjectMatches(); ++match_i)
        {
            const ObjectMatch& match = m_detector->objectMatch(match_i);
            double compatibility = m_trackers[tracker_i]->compatibilityMeasure(match);
            if (compatibility > best_mesure)
            {
                best_mesure = compatibility;
                best_match = match_i;
            }
        }
        if (best_match >= 0)
        {
            m_trackers[tracker_i]->addNewDetection(m_detector->objectMatch(best_match));
            tracked_matches[best_match] = true;
        }
    }

    // Create trackers for untracked objects.
    std::vector<ObjectTrackerPtr> new_trackers;
    foreach_idx(i, tracked_matches)
    {
        if (tracked_matches[i])
            continue;
        ObjectTrackerPtr tracker (new ObjectTracker(m_detector->objectMatch(i)));
        new_trackers.push_back(tracker);
    }

    // Delete obsolete trackers.
    foreach_idx(i, m_trackers)
    {
        if (m_trackers[i]->numFramesWithoutDetection() < max_frames_without_detect)
            new_trackers.push_back(m_trackers[i]);
    }
    m_trackers = new_trackers;
}

} // avs
