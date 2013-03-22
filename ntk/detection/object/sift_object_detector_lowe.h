#ifndef FROL_sift_object_finder_lowe_H
# define FROL_sift_object_finder_lowe_H

# include "sift_object_detector.h"
# include "sift_object_match_lowe.h"

namespace ntk
{
  
  double computeLowePfa(const SiftParameters& params,
                        double nb_points_in_object_view,
                        double nb_points_in_db,
                        double nb_points_in_region,
                        double nb_matches);

  class SiftObjectDetectorLowe : public SiftObjectDetector
  {
    typedef SiftObjectDetector super;

    public:
      SiftObjectDetectorLowe(const SiftParameters& params);
      virtual ~SiftObjectDetectorLowe();

    public:
      virtual void fillXmlElement(XMLNode& element) const;
      virtual void loadFromXmlElement(const XMLNode& element);

    public:
      virtual unsigned nbObjectMatches() const { return m_objects.size(); }
      virtual const SiftObjectMatchLowe& objectMatch(unsigned idx) const;
      virtual SiftObjectMatchLowe& objectMatch(unsigned idx);
      virtual void keepOnlyBestMatch();

    public:
      virtual void initializeFindObjects();
      virtual void findObjects();

    protected:
      double computePFA(const SiftObjectMatchLowe& match) const;
      void filterIdenticalMatches();
      void filterMultipleInstanceClusters(SiftHough::clusters_type& clusters);

    private:
      std::vector<SiftObjectMatchLowePtr> m_objects;
  };  

} // end of avs

#endif // ndef FROL_sift_object_finder_lowe_H
