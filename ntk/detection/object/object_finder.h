//
// Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2009
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef ADSOBJECTFINDER_H
#define ADSOBJECTFINDER_H

# include "object_tracker.h"
# include "object_detector.h"
# include "sift_parameters.h"

# include <ntk/camera/calibration.h>
# include <vector>

namespace ntk
{

  class ObjectDetector; ntk_ptr_typedefs(ObjectDetector);
  class SiftObjectDetector; ntk_ptr_typedefs(SiftObjectDetector);
  class AdsObjectDetector; ntk_ptr_typedefs(AdsObjectDetector);
  class VFHObjectDetector; ntk_ptr_typedefs(VFHObjectDetector);
  class SiftParameters;

  struct ObjectFinderParams
  {
    ObjectFinderParams();

    SiftParameters feature;

    bool use_tracking;
    std::string object_database;
    std::string object_detector; // sift ads
    double detection_threshold;

    // Ads specific
    std::string ads_behavior;
    int time_limit_in_msecs;
    bool use_event_collector;

    bool keep_only_best_match;
  };

  class ObjectFinder
  {
  public:
    ObjectFinder() {}
    void initialize(const ObjectFinderParams& params);
    void resetTrackers();

    void processNewImage(const ntk::RGBDImage& image);

    int numTrackers() const { return m_trackers.size(); }
    ObjectTrackerPtr getTracker(int index) const { return m_trackers[index]; }
    const ObjectDetectorPtr& objectDetector() const { return m_detector; }
    ObjectDetectorPtr objectDetector() { return m_detector; }

  private:
    SiftObjectDetectorPtr createSiftObjectDetector();
#ifdef NESTK_DISABLED_ADS_NOT_YET_IMPORTED
    AdsObjectDetectorPtr createAdsObjectDetector();
#endif

  private:
    ObjectFinderParams m_params;
    ObjectDetectorPtr m_detector;
    std::vector<ObjectTrackerPtr> m_trackers;
    ObjectDatabasePtr m_database;
  };
  ntk_ptr_typedefs(ObjectFinder)

} // avs

#endif // ADSOBJECTFINDER_H
