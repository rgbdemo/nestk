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
 */

/* \author Bastian Steder */

#include <iostream>
using std::cout;
using std::cerr;
#include <vector>
using std::vector;
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/pcl_macros.h>
#include <pcl/range_image/range_image.h>
//#include <pcl/common/vector_average.h>


#define USE_OMP 1

namespace pcl {


NarfKeypoint::NarfKeypoint(RangeImageBorderExtractor* range_image_border_extractor, float support_size) : BaseClass(),
   range_image_border_extractor_(range_image_border_extractor),
   interest_image_(NULL), interest_points_(NULL)
{
  name_ = "NarfKeypoint";
  clearData();
  if (support_size > 0.0f)
    parameters_.support_size = support_size;
}

NarfKeypoint::~NarfKeypoint()
{
  clearData();
}

void NarfKeypoint::clearData()
{
  //cerr << __PRETTY_FUNCTION__<<" called.\n";
  
  delete[] interest_image_; interest_image_=NULL;
  delete interest_points_;  interest_points_=NULL;
  is_interest_point_image_.clear();
}

void NarfKeypoint::setRangeImageBorderExtractor(RangeImageBorderExtractor* range_image_border_extractor)
{
  clearData();
  range_image_border_extractor_ = range_image_border_extractor;
}

void NarfKeypoint::setRangeImage(const RangeImage* range_image)
{
  clearData();
  range_image_border_extractor_->setRangeImage(range_image);
}

#define USE_BEAM_AVERAGE 1

namespace {  // Anonymous namespace - only available in this file
  inline void nkdGetScores(float distance_factor, int pixel_distance, float surface_change_score, float optimal_distance,
                           float& negative_score, float& positive_score)
  {
    //float negative_score_regarding_pixel_distance = 0.0f;//min(0.0f, 0.5f*pixel_distance-1.0f);
    negative_score = 1.0f - surface_change_score * (std::max)(1.0f - distance_factor/optimal_distance, 0.0f);
    negative_score = powf(negative_score, 2);
    //cout << PVARC(surface_change_score)<<PVARC(distance_factor)<<PVARC(optimal_distance)<<PVARN(negative_score);
    
    //if (negative_score < 1.0f)
      //cout << PVARC(surface_change_score)<<PVARC(distance_factor)<<PVARN(negative_score);
    positive_score = surface_change_score * (1.0f-fabsf(distance_factor-optimal_distance));
    //positive_score = surface_change_score;
  }
  void translateDirection180To360(Eigen::Vector2f& direction_vector)
  {
    //// The following code does the equivalent to this:
    //// Get the angle of the 2D direction (with positive x) alpha, and return the direction of 2*alpha
    //// We do this to create a normal angular wrap-around at -180,180 instead of the one at -90,90, 
    //// enabling us to calculate the average angle as the direction of the sum of all unit vectors.
    //// We use sin(2a)=2*sin(a)*cos(a) and cos(2a)=2cos^2(a)-1 so that we needn't actually compute the angles, which would be expensive
    float cos_a = direction_vector[0],
          cos_2a = 2*cos_a*cos_a - 1.0f,
          sin_a = direction_vector[1],
          sin_2a = 2.0f*sin_a*cos_a;
    direction_vector[0] = cos_2a;
    direction_vector[1] = sin_2a;
  }
  void translateDirection360To180(Eigen::Vector2f& direction_vector)
  {
    //// Inverse of the above
    float cos_2a = direction_vector[0],
          cos_a = sqrtf(0.5f*(cos_2a+1.0f)),
          sin_2a = direction_vector[1],
          sin_a = sin_2a / (2.0f*cos_a);
    direction_vector[0] = cos_a;
    direction_vector[1] = sin_a;
  }
  inline Eigen::Vector2f nkdGetDirectionVector(const Eigen::Vector3f& direction, const Eigen::Affine3f& rotation)
  {
    //if (fabsf(direction.norm()-1.0f) > 0.001)
      //cerr << direction[0]<<","<<direction[1]<<","<<direction[2]<<" has norm "<<direction.norm()<<"\n";
    //else
      //cerr<<"OK";
    Eigen::Vector3f rotated_direction = rotation*direction;
    Eigen::Vector2f direction_vector(rotated_direction[0], rotated_direction[1]);
    direction_vector.normalize();
    if (direction_vector[0]<0.0f)
      direction_vector *= -1.0f;
    

#   if USE_BEAM_AVERAGE
      translateDirection180To360(direction_vector);
#   endif
    ////cout << PVARN(direction_vector);
    
    return direction_vector;
  }
  inline float nkdGetDirectionAngle(const Eigen::Vector3f& direction, const Eigen::Affine3f& rotation)
  {
    Eigen::Vector3f rotated_direction = rotation*direction;
    Eigen::Vector2f direction_vector(rotated_direction[0], rotated_direction[1]);
    direction_vector.normalize();
    float angle = 0.5f*normAngle(2.0f*acosf(direction_vector[0]));
    //std::cout << PVARN(direction_vector)<<PVARAN(angle);
    return angle;
  }

}  // end anonymous namespace

//int my_point_x=506, my_point_y=76;  // For debugging purposes

void NarfKeypoint::calculateInterestImage()
{
  if (interest_image_!=NULL)  // Already done
    return;
  
  if (parameters_.support_size <= 0.0f)
  {
    cerr << __PRETTY_FUNCTION__<<": parameters_.support_size is not set!\n";
    return;
  }
  if (range_image_border_extractor_==NULL)
  {
    cerr << __PRETTY_FUNCTION__<<": range_image_border_extractor_ is not set!\n";
    return;
  }
  
  const RangeImage& range_image = range_image_border_extractor_->getRangeImage();
  const ::pcl::PointCloud<BorderDescription> border_descriptions = range_image_border_extractor_->getBorderDescriptions();
  const float* surface_change_scores = range_image_border_extractor_->getSurfaceChangeScores();
  const Eigen::Vector3f* surface_change_directions = range_image_border_extractor_->getSurfaceChangeDirections();
  
  //MEASURE_FUNCTION_TIME;
  
  int width  = range_image.width,
      height = range_image.height,
      array_size = width*height;
  float search_radius = 0.5*parameters_.support_size,
        radius_squared = search_radius*search_radius,
        radius_reciprocal = 1.0f/search_radius;
  interest_image_ = new float[array_size];
  //Eigen::Vector3f sensor_pos = range_image.getTransformationToWorldSystem()*Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  
  //TODO: Turn process around - go thorugh points with high change and update rest accordingly
  //      Save histogram per cell - should be faster overall... or at least we can loose the 9 beam approximation
  int angle_histogram_size = 18;
  
# if USE_OMP
//#   pragma omp parallel for default(shared) schedule(dynamic, 10)
//#   pragma omp parallel for default(shared)
# endif
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      //cout << __PRETTY_FUNCTION__ <<": "<<x<<","<<y<<" ("<<width<<","<<height<<")\n";
      int index = y*width + x;
      float& interest_value = interest_image_[y*width + x];
      interest_value = 0.0f;
      if (!range_image.isValid(x, y))
        continue;
      const BorderTraits& border_traits = border_descriptions.points[index].traits;
      bool is_border = border_traits[BORDER_TRAIT__OBSTACLE_BORDER] ||
                                    border_traits[BORDER_TRAIT__SHADOW_BORDER]   ||
                                    border_traits[BORDER_TRAIT__VEIL_POINT];
      if (is_border)
        continue;
      
      const PointWithRange& point = range_image.getPoint(index);
      Eigen::Affine3f rotation_to_viewer_coordinate_system;
      range_image.getRotationToViewerCoordinateFrame(point.getVector3fMap(), rotation_to_viewer_coordinate_system);
      
      //VectorAverage<float, 2> vector_average;
      
      std::vector<float> angle_histogram (angle_histogram_size, 0);
      //ERASE_ARRAY(angle_histogram, angle_histogram_size);
      
      float overall_negative_score = 1.0f;
      for (int offset_y=-1; offset_y<=1; ++offset_y)
      {
        for (int offset_x=-1; offset_x<=1; ++offset_x)
        {
          if (offset_x == 0 && offset_y == 0)
            continue;
          
          int step = 0;
          while (true)
          {
            int x2=x+step*offset_x, y2=y+step*offset_y;
            
            const PointWithRange& neighbor = range_image.getPoint(x2, y2);
            if (!pcl_isfinite(neighbor.range))
              break;
            float distance_squared = squaredEuclideanDistance(point, neighbor);
            if (distance_squared > radius_squared)
              break;
            int index2 = y2*width+x2;
            const BorderTraits& border_traits_neighbor = border_descriptions.points[index2].traits;
            if (border_traits_neighbor[BORDER_TRAIT__VEIL_POINT])
              break;
            
            float surface_change_score_neighbor = surface_change_scores[index2];
            
            if (surface_change_score_neighbor > parameters_.min_interest_value)
            {
              const Eigen::Vector3f& surface_change_direction_neighbor = surface_change_directions[index2];
              float distance_factor = radius_reciprocal*sqrtf(distance_squared);
              float negative_score, positive_score;
              nkdGetScores(distance_factor, step, surface_change_score_neighbor,
                           2.0f*parameters_.optimal_distance_to_high_surface_change,
                           negative_score, positive_score);
              float angle = nkdGetDirectionAngle(surface_change_direction_neighbor, rotation_to_viewer_coordinate_system);
              int histogram_cell = (std::min)(angle_histogram_size-1,
                                            (int)lrint (floorf((angle+deg2rad(90.0f))/deg2rad(180.0f) * angle_histogram_size)));
              angle_histogram[histogram_cell] = (std::max)(angle_histogram[histogram_cell], positive_score);
              overall_negative_score = (std::min)(overall_negative_score, negative_score);
            }
            
            if (border_traits_neighbor[BORDER_TRAIT__SHADOW_BORDER] || border_traits_neighbor[BORDER_TRAIT__OBSTACLE_BORDER])
              break;
            ++step;
          }
        }
      }
      
      for (int histogram_cell1=0; histogram_cell1<angle_histogram_size-1; ++histogram_cell1)
      {
        for (int histogram_cell2=histogram_cell1+1; histogram_cell2<angle_histogram_size; ++histogram_cell2)
        {
          if (angle_histogram[histogram_cell1]==0.0f || angle_histogram[histogram_cell2]==0.0f)
            continue;
          float normalized_angle_diff = 2.0f*float(histogram_cell2-histogram_cell1)/float(angle_histogram_size);
          normalized_angle_diff = (normalized_angle_diff <= 1.0f ? normalized_angle_diff : 2.0f-normalized_angle_diff);
          //cout << PVARC(histogram_cell1)<<PVARC(histogram_cell2)<<PVARN(normalized_angle_diff);
          interest_value = (std::max)(angle_histogram[histogram_cell1]*angle_histogram[histogram_cell2]*normalized_angle_diff, interest_value);
        }
      }
      interest_value = sqrtf(interest_value);
      
      if (parameters_.add_points_on_straight_edges)
      {
        float max_histogram_cell_value = 0.0f;
        for (int histogram_cell=0; histogram_cell<angle_histogram_size; ++histogram_cell)
          max_histogram_cell_value = (std::max)(max_histogram_cell_value, angle_histogram[histogram_cell]);
        interest_value = (std::min)(interest_value+max_histogram_cell_value, 1.0f);
      }
      
      //cout << PVARN(overall_negative_score);
      //overall_negative_score = -powf(-overall_negative_score, 2);  // Some scaling
      interest_value *= overall_negative_score;  // Give punishment according to how much the immediate neighborhood changes
      
      if (!pcl_isfinite(interest_value))
        interest_value = 0.0f;
      
      //if (interest_value<0.0f || interest_value>1.0f)
        //cout << PVARC(overall_negative_score)<<PVARN(intereobject_creation_testst_value);
      
      //Eigen::Vector2f eigenvalues;
      //vector_average.doPCA(eigenvalues);
      //interest_value = vector_average.getAccumulatedWeight()*eigenvalues[1]; // * (1.0f+overall_negative_score);
      //if (x==my_point_x && y==my_point_y)
        //cout << PVARC(overall_negative_score)<<PVARC(vector_average.getAccumulatedWeight())<<PVARC(eigenvalues[1])<<PVARN(interest_value);
      //* vector_average.getAccumulatedWeight();
      
    }
  }
  blurInterestImage();
}

//void NarfKeypoint::calculateInterestImage()
//{
  //if (interest_image_!=NULL)  // Already done
    //return;
  
  //if (parameters_.support_size <= 0.0f)
  //{
    //cerr << __PRETTY_FUNCTION__<<": parameters_.support_size is not set!\n";
    //return;
  //}
  //if (range_image_border_extractor_==NULL)
  //{
    //cerr << __PRETTY_FUNCTION__<<": range_image_border_extractor_ is not set!\n";
    //return;
  //}
  
  //const RangeImage& range_image = range_image_border_extractor_->getRangeImage();
  //const ::pcl::PointCloud<BorderDescription> border_descriptions = range_image_border_extractor_->getBorderDescriptions();
  //const float* surface_change_scores = range_image_border_extractor_->getSurfaceChangeScores();
  //const Eigen::Vector3f* surface_change_directions = range_image_border_extractor_->getSurfaceChangeDirections();
  
  ////MEASURE_FUNCTION_TIME;
  
  //int width  = range_image.width,
      //height = range_image.height,
      //array_size = width*height;
  //float search_radius = 0.5*parameters_.support_size,
        //radius_squared = search_radius*search_radius,
        //radius_reciprocal = 1.0f/search_radius;
  //interest_image_ = new float[array_size];
  ////Eigen::Vector3f sensor_pos = range_image.getTransformationToWorldSystem()*Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  
  //float interest_value_average_normalization_factor = 1.0f / float(parameters_.no_of_direction_pairs_to_average);
  
//# if USE_OMP
////#   pragma omp parallel for default(shared) schedule(dynamic, 10)
//#   pragma omp parallel for default(shared)
//# endif
  //for (int y=0; y<height; ++y)
  //{
    //for (int x=0; x<width; ++x)
    //{
      ////cout << __PRETTY_FUNCTION__ <<": "<<x<<","<<y<<" ("<<width<<","<<height<<")\n";
      //int index = y*width + x;
      //float& interest_value = interest_image_[y*width + x];
      //interest_value = 0.0f;
      //if (!range_image.isValid(x, y))
        //continue;
      //const BorderTraits& border_traits = border_descriptions.points[index].traits;
      //bool is_border = border_traits[BORDER_TRAIT__OBSTACLE_BORDER] ||
                                    //border_traits[BORDER_TRAIT__SHADOW_BORDER]   ||
                                    //border_traits[BORDER_TRAIT__VEIL_POINT];
      //if (is_border)
        //continue;
      
      //const PointWithRange& point = range_image.getPoint(index);
      //Eigen::Affine3f rotation_to_viewer_coordinate_system;
      //range_image.getRotationToViewerCoordinateFrame(point.getVector3fMap(), rotation_to_viewer_coordinate_system);
      
      ////VectorAverage<float, 2> vector_average;
      
      //float beam_weights[8];
      //Eigen::Vector2f beam_directions[8];
      
      //float overall_negative_score = 1.0f;
      //int beam_idx = 0;
      //float results[28];
      //int results_idx=0;
      //for (int offset_y=-1; offset_y<=1; ++offset_y)
      //{
        //for (int offset_x=-1; offset_x<=1; ++offset_x)
        //{
          //if (offset_x == 0 && offset_y == 0)
            //continue;
          
          //Eigen::Vector2f& beam_direction = beam_directions[beam_idx];
          //beam_direction.setZero();
          
          //float weight_sum = 0.0f; (void)weight_sum;
          //float& beam_weight = beam_weights[beam_idx];
          //beam_weight = 0.0f;
          
          
          //int step = (beam_idx==0 ? 0 : 1);
          //while (true)
          //{
            //int x2=x+step*offset_x, y2=y+step*offset_y;
            
            //const PointWithRange& neighbor = range_image.getPoint(x2, y2);
            //if (!pcl_isfinite(neighbor.range))
            //{
              ////if (x==my_point_x && y==my_point_y)
                ////cout << "Stopping because of "<<neighbor.range<<".\n";
              //break;
            //}
            //float distance_squared = squaredEuclideanDistance(point, neighbor);
            //if (distance_squared > radius_squared)
            //{
              ////if (x==my_point_x && y==my_point_y)
                ////cout << "Stopping because of distance.\n";
              //break;
            //}
            
            //int index2 = y2*width+x2;
            
            //const BorderTraits& border_traits_neighbor = border_descriptions.points[index2].traits;
            //if (border_traits_neighbor[BORDER_TRAIT__VEIL_POINT])
            //{
              ////if (x==my_point_x && y==my_point_y)
                ////cout << "Stopping because of veil.\n";
              //break;
            //}
            
            //float surface_change_score_neighbor = surface_change_scores[index2];
            ////if (surface_change_score_neighbor > 1e-3)
            //if (surface_change_score_neighbor > 0.1)  // Ignoring small scores improves the speed
            //{
              //const Eigen::Vector3f& surface_change_direction_neighbor = surface_change_directions[index2];
              ////cerr <<"("<<surface_change_score_neighbor<<")";
              //float distance_factor = radius_reciprocal*sqrtf(distance_squared);
              //float negative_score, positive_score;
              //nkdGetScores(distance_factor, step, surface_change_score_neighbor,
                           //2.0f*parameters_.optimal_distance_to_high_surface_change,
                           //negative_score, positive_score);
              //Eigen::Vector2f direction_vector = nkdGetDirectionVector(surface_change_direction_neighbor, rotation_to_viewer_coordinate_system);
              
              //overall_negative_score = (std::min)(overall_negative_score, negative_score);
              ////if (positive_score > 0.3f)
                ////cout << step*offset_x<<","<<step*offset_y<<": ("<<direction_vector[0]<<","<<direction_vector[1]<<") has score "<<positive_score<<"\n";
              
//#             if USE_BEAM_AVERAGE  
                //weight_sum += positive_score;
                //beam_weight += positive_score*positive_score;
                //beam_direction += positive_score * direction_vector;
//#             else
                //if (positive_score > beam_weight)  // Find maximum
                //{
                  //beam_weight = positive_score;
                  //beam_direction = direction_vector;
                //}
//#             endif
            //}
            
            //if (border_traits_neighbor[BORDER_TRAIT__SHADOW_BORDER] || border_traits_neighbor[BORDER_TRAIT__OBSTACLE_BORDER])
            //{
              ////if (x==my_point_x && y==my_point_y)
                ////cout << "Stopping because of border.\n";
              //break;
            //}
            //++step;
          //}
          ////cerr << beam_weight<<", ";
          
//#         if USE_BEAM_AVERAGE
            //if (weight_sum > 0.0f) {
              //float normalization_factor = 1.0f / weight_sum;
              //beam_weight *= normalization_factor;
              //beam_direction.normalize();
              //translateDirection360To180(beam_direction);
            //}
//#         endif
          
          //beam_weight += beam_weight - beam_weight*beam_weight; // Some scaling to prevent small numbers in later step - cheaper than sqrtf
          
          ////if (x==my_point_x && y==my_point_y)
            ////cout << "Offset "<<offset_x<<","<<offset_y<<": direction "<<beam_direction[0]<<","<<beam_direction[1]<<" ("
                 ////<< rad2deg((atan2f(beam_direction[1],beam_direction[0]))) <<"deg) with weight "<<beam_weight<<".\n";
          
          //// Do pairwise comparison
          //for (int beam_idx2=0; beam_idx2<beam_idx; ++beam_idx2)
          //{
            //float interest_value_for_pair = beam_weight*beam_weights[beam_idx2]*(1.0f-fabsf(beam_direction.dot(beam_directions[beam_idx2])));
            //results[results_idx++] = interest_value_for_pair;
            ////cout << PVARN(results_idx);
            ////if (interest_value_for_pair > interest_value)
            ////{
              //////if (x==my_point_x && y==my_point_y)
              //////cout << "Pair "<<beam_idx<<","<<beam_idx2<<" has value "<<interest_value_for_pair<<" ("
                   //////<< beam_directions[beam_idx2][0]<<","<<beam_directions[beam_idx2][1]<<"("<<beam_weights[beam_idx2]<<") - "
                   //////<< beam_direction[0]<<","<<beam_direction[1]<<"("<<beam_weight<<")).\n";
              ////interest_value = interest_value_for_pair;
            ////}
          //}
          
          //++beam_idx;
        //}
      //}
      
      //std::sort(results, results+28);
      //for (int i=27; i>=28-parameters_.no_of_direction_pairs_to_average; --i)
        //interest_value += results[i];
      //interest_value *= interest_value_average_normalization_factor;
      
      //if (parameters_.add_points_on_straight_edges)
      //{
        //float max_beam_weight = 0.0f;
        //for (int i=0; i<8; ++i)
        //{
          //max_beam_weight = max(max_beam_weight, beam_weights[i]);
        //}
        //interest_value += powf(max_beam_weight, 1);
      //}
      
      ////cout << PVARN(overall_negative_score);
      ////overall_negative_score = -powf(-overall_negative_score, 2);  // Some scaling
      //interest_value *= overall_negative_score;  // Give punishment according to how much the immediate neighborhood changes
      
      //if (!pcl_isfinite(interest_value))
        //interest_value = 0;
      
      ////if (interest_value<0.0f || interest_value>1.0f)
        ////cout << PVARC(overall_negative_score)<<PVARN(interest_value);
      
      ////Eigen::Vector2f eigenvalues;
      ////vector_average.doPCA(eigenvalues);
      ////interest_value = vector_average.getAccumulatedWeight()*eigenvalues[1]; // * (1.0f+overall_negative_score);
      ////if (x==my_point_x && y==my_point_y)
        ////cout << PVARC(overall_negative_score)<<PVARC(vector_average.getAccumulatedWeight())<<PVARC(eigenvalues[1])<<PVARN(interest_value);
      ///[> vector_average.getAccumulatedWeight();
      
    //}
  //}
  ////blurInterestImage();
//}

void NarfKeypoint::blurInterestImage()
{
  //MEASURE_FUNCTION_TIME;
  
  int blur_radius = parameters_.interest_image_blur_size;
  //int blur_radius = 1;
  if (blur_radius==0)
    return;
  
  const RangeImage& range_image = range_image_border_extractor_->getRangeImage();
  float* blurred_image = new float[range_image.width*range_image.height];
  
  for (int y=0; y<int(range_image.height); ++y)
  {
    for (int x=0; x<int(range_image.width); ++x)
    {
      float& new_point = blurred_image[y*range_image.width + x];
      new_point = 0.0f;
      float weight_sum = 0.0f;
      for (int y2=y-blur_radius; y2<y+blur_radius; ++y2)
      {
        for (int x2=x-blur_radius; x2<x+blur_radius; ++x2)
        {
          if (!range_image.isInImage(x2,y2))
            continue;
          new_point += interest_image_[y2*range_image.width + x2];
          weight_sum += 1.0f;
        }
      }
      new_point /= weight_sum;
    }
  }
  delete[] interest_image_;
  interest_image_ = blurred_image;
}

void NarfKeypoint::calculateInterestPoints()
{
  //TODO: bivariate polynomials to get exact point position
  if (interest_points_ != NULL)
  {
    //cout << "Interest points member is not NULL => Doing nothing.\n";
    return;
  }
  calculateInterestImage();

  interest_points_ = new ::pcl::PointCloud<InterestPoint>;
  
  //cout << PVARN(range_image_border_extractor_->getParameters());
  //cout << PVARN(this->getParameters());
  
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  //MEASURE_FUNCTION_TIME;
  
  const RangeImage& range_image = range_image_border_extractor_->getRangeImage();
  const ::pcl::PointCloud<BorderDescription> border_descriptions = range_image_border_extractor_->getBorderDescriptions();
  int width  = range_image.width,
      height = range_image.height,
      size = width*height;
  float min_distance_squared = powf(parameters_.min_distance_between_interest_points*parameters_.support_size, 2),
        distance_for_additional_points = parameters_.distance_for_additional_points*parameters_.support_size,
        distance_for_additional_points_squared = distance_for_additional_points*distance_for_additional_points;
  is_interest_point_image_.clear();
  is_interest_point_image_.resize(size, true);
  
  std::multimap<float, Eigen::Vector2i> ordered_points;
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      float interest_value = interest_image_[index];
      if (interest_value <= parameters_.min_interest_value || !range_image.isValid(index))
      {
        is_interest_point_image_[index] = false;
        continue;
      }
      ordered_points.insert(std::make_pair(interest_value, Eigen::Vector2i(x,y)));
    }
  }
  
  vector<int> neighbor_indices;
  vector<int> interest_point_indices;
  for (std::multimap<float, Eigen::Vector2i>::const_reverse_iterator it=ordered_points.rbegin(); it!=ordered_points.rend(); ++it)
  {
    int x=it->second[0], y=it->second[1], index = y*width + x;
    if (!is_interest_point_image_[index])
      continue;
    const PointWithRange& point = range_image.getPoint(index);
    InterestPoint interest_point;
    interest_point.x=point.x;  interest_point.y=point.y;  interest_point.z=point.z;
    interest_point.strength = interest_image_[index];
    
    bool is_maxmimum = true;
    bool stop = false;
    neighbor_indices.clear();
    for (int radius=1;  !stop;  ++radius) 
    {
      int x2=x-radius-1, y2=y-radius;  // Top left - 1
      stop = true;
      for (int i=0; i<8*radius; ++i)
      {
        if (i<=2*radius) ++x2; else if (i<=4*radius) ++y2; else if (i<=6*radius) --x2; else --y2;
        int neighbor_index = y2*width+x2;
        if (!range_image.isValid(x2, y2))
          continue;
        const PointWithRange& neighbor = range_image.getPoint(neighbor_index);
        if (radius>=parameters_.min_pixel_distance_between_interest_points && squaredEuclideanDistance(point, neighbor)>min_distance_squared)
          continue;
        stop = false; // There is a point in range -> Have to check further distances
        neighbor_indices.push_back(neighbor_index);
        if (interest_image_[neighbor_index] > interest_point.strength)
          is_maxmimum = false;
      }
    }
    if (!parameters_.do_non_maximum_suppression || is_maxmimum)
    {
      interest_point_indices.push_back(index);
      for (unsigned int i=0; i<neighbor_indices.size(); ++i)
        is_interest_point_image_[neighbor_indices[i]] = false;
    }
    else
    {
      is_interest_point_image_[index] = false;
    }
  }
  
  for (unsigned int i=0; i<interest_point_indices.size(); ++i)
  {
    int index = interest_point_indices[i];
    const PointWithRange& point = range_image.getPoint(index);
    interest_points_->points.push_back(InterestPoint());
    interest_points_->points.back().getVector3fMap() = point.getVector3fMap();
    interest_points_->points.back().strength = interest_image_[index];
    if (distance_for_additional_points_squared > 0.0f)
    {
      float y=index/range_image.width, x=index-y*range_image.width;
      bool still_in_range = true;
      for (int radius=1;  still_in_range;  ++radius) 
      {
        int x2=x-radius-1, y2=y-radius;  // Top left - 1
        still_in_range = false;
        for (int i=0; i<8*radius; ++i)
        {
          if (i<=2*radius) ++x2; else if (i<=4*radius) ++y2; else if (i<=6*radius) --x2; else --y2;
          if (!range_image.isValid(x2, y2))
            continue;
          int neighbor_index = y2*width+x2;
          const PointWithRange& neighbor = range_image.getPoint(neighbor_index);
          if (squaredEuclideanDistance(point, neighbor) > distance_for_additional_points_squared)
            continue;
          still_in_range = true;
          float neighbor_interest_value = interest_image_[neighbor_index];
          if (neighbor_interest_value > 0.5f*parameters_.min_interest_value)
          {
            //cout << "Adding "<<x2<<","<<y2<<" as neighbor of "<<x<<","<<y<<".\n";
            is_interest_point_image_[neighbor_index] = true;
            interest_points_->points.push_back(InterestPoint());
            interest_points_->points.back().getVector3fMap() = neighbor.getVector3fMap();
            interest_points_->points.back().strength = interest_image_[neighbor_index];
          }
        }
      }
    }
  }
  

//# if USE_OMP
////#   pragma omp parallel for default(shared) schedule(dynamic, 10)
////#   pragma omp parallel for default(shared)
//# endif
  //for (int y=0; y<height; ++y)
  //{
    //for (int x=0; x<width; ++x)
    //{
      //int index = y*width + x;
      //float interest_value = interest_image_[index];
      //if (interest_value <= parameters_.min_interest_value)
        //continue;

      ////cout << x<<","<<y<<": "<<interest_value<<"\n";
      //const PointWithRange& point = range_image.getPoint(index);
      
      //bool is_maximum = true;
      //for (int offset_y=-1; offset_y<=1 && is_maximum; ++offset_y)
      //{
        //for (int offset_x=-1; offset_x<=1 && is_maximum; ++offset_x)
        //{
          //if (offset_x == 0 && offset_y == 0)
            //continue;
          //for (int step=1; ; ++step)
          //{
            //int x2=x+step*offset_x, y2=y+step*offset_y;
            
            //const PointWithRange& neighbor = range_image.getPoint(x2, y2);
            //if (!pcl_isfinite(neighbor.range))
              //break;
            //if (step>min_pixel_distance && squaredEuclideanDistance(point, neighbor) > min_distance_squared)
              //break;
            //int index2 = y2*width+x2;
            //float interest_value_neighbor = interest_image_[index2];
            //if (interest_value < interest_value_neighbor)
            //{
              //is_maximum = false;
              //break;
            //}
            //const BorderTraits& border_traits_neighbor = border_descriptions.points[index2].traits;
            //if (border_traits_neighbor[BORDER_TRAIT__SHADOW_BORDER] ||
                //border_traits_neighbor[BORDER_TRAIT__OBSTACLE_BORDER] ||
                //border_traits_neighbor[BORDER_TRAIT__VEIL_POINT])
              //break;
          //}
        //}
      //}
      
      ////bool is_maximum = true, still_in_range = true;
      ////for (int radius=1;  is_maximum && still_in_range;  ++radius) 
      ////{
        ////int x2=x-radius-1, y2=y-radius;  // Top left - 1
        ////still_in_range = false;
        ////for (int i=0; i<8*radius; ++i)
        ////{
          ////if (i<=2*radius) ++x2; else if (i<=4*radius) ++y2; else if (i<=6*radius) --x2; else --y2;
          ////if (!range_image.isInImage(x2, y2))
            ////continue;
          ////int neighbor_index = y2*width+x2;
          ////if (radius > 1)
          ////{
            ////const PointWithRange& neighbor = range_image.getPoint(neighbor_index);
            ////if (squaredEuclideanDistance(point, neighbor) > max_distance_squared)
              ////continue;
          ////}
          ////still_in_range = true;
          ////float neighbor_interest_value = interest_image[neighbor_index];
          ////if (neighbor_interest_value > interest_value)
          ////{
            ////is_maximum = false;
            ////break;
          ////}
        ////}
      ////}
      
      
      ////cout << interest_point.x<<","<<interest_point.y<<","<<interest_point.z<<" : "<<interest_point.strength<<"\n";
      //is_interest_point_image_[index] = is_maximum;
      
      //if (!is_maximum || distance_for_additional_points <= 0.0f)
        //continue;
      
      //bool still_in_range = true;
      //for (int radius=1;  still_in_range;  ++radius) 
      //{
        //int x2=x-radius-1, y2=y-radius;  // Top left - 1
        //still_in_range = false;
        //for (int i=0; i<8*radius; ++i)
        //{
          //if (i<=2*radius) ++x2; else if (i<=4*radius) ++y2; else if (i<=6*radius) --x2; else --y2;
          //if (!range_image.isValid(x2, y2))
            //continue;
          //int neighbor_index = y2*width+x2;
          //const PointWithRange& neighbor = range_image.getPoint(neighbor_index);
          //if (squaredEuclideanDistance(point, neighbor) > distance_for_additional_points_squared)
            //continue;
          //still_in_range = true;
          //float neighbor_interest_value = interest_image_[neighbor_index];
          //if (neighbor_interest_value > 0.5f*parameters_.min_interest_value)
          //{
            //is_interest_point_image_[neighbor_index] = true;
            ////cout << "Adding "<<x2<<","<<y2<<" as neighbor of "<<x<<","<<y<<".\n";
          //}
        //}
      //}
    //}
  //}
  
  //for (int index=0; index<size; ++index)
  //{
    //if (!is_interest_point_image_[index])
      //continue;
    //const PointWithRange& point = range_image.getPoint(index);
    //InterestPoint interest_point;
    //interest_point.x=point.x;  interest_point.y=point.y;  interest_point.z=point.z;
    //interest_point.strength = interest_image_[index];
    //interest_points_->points.push_back(interest_point);
  //}
  ////cout << PVARN(interest_points_->points.size());
}

const RangeImage& NarfKeypoint::getRangeImage()
{
  return range_image_border_extractor_->getRangeImage();
}

void NarfKeypoint::detectKeypoints (NarfKeypoint::PointCloudOut& output)
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";
  
  output.points.clear ();
  
  if (indices_)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": Sorry, usage of indices for the extraction is not supported for NARF interest points (yet).\n\n";
    return;
  }
  
  if (range_image_border_extractor_ == NULL)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": RangeImageBorderExtractor member is not set. Sorry, this is need for the NARF keypoint extraction.\n\n";
    return;
  }
  
  if (!range_image_border_extractor_->hasRangeImage())
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": RangeImage is not set. Sorry, the NARF keypoint extraction works on range images, not on normal point clouds.\n\n"
              << " Use setRangeImage(...).\n\n";
    return;
  }
  
  calculateInterestPoints ();
  
  int size = getRangeImage ().width * getRangeImage ().height;
  
  for (int index=0; index<size; ++index)
  {
    if (!is_interest_point_image_[index])
      continue;
    output.points.push_back (index);
  }
}

void NarfKeypoint::compute(NarfKeypoint::PointCloudOut& output)
{
  detectKeypoints(output);
}











  //cout << __PRETTY_FUNCTION__<<" called.\n";
  //MEASURE_FUNCTION_TIME;
  
  //int width  = range_image.width,
      //height = range_image.height,
      //array_size = width*height;
  //float search_radius = 0.5*support_size,
        //max_distance_squared = search_radius*search_radius;
  
  //CloseByBorderInformation* close_by_border_informations = new CloseByBorderInformation[array_size];
  //Eigen::Vector3f sensor_pos = range_image.getTransformationToWorldSystem()*Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  
  ////#pragma omp parallel for default(shared) schedule(dynamic, 10)
  //for (int y=0; y<height; ++y)
  //{
    //for (int x=0; x<width; ++x)
    //{
      ////cout << __PRETTY_FUNCTION__ <<": "<<x<<","<<y<<" ("<<width<<","<<height<<")\n";
      //int index = y*width + x;
      //float surface_change_score = surface_change_scores[index];
      //if (surface_change_score<0.1f)
        //continue;
      //const Eigen::Vector3f& surface_change_direction = surface_change_directions[index];
      //cout << PVARN(surface_change_score);
      
      //const PointWithRange& point = range_image.getPoint(index);
      
      //bool still_in_range = true;
      //for (int radius=1;  still_in_range;  ++radius) 
      //{
        //int x2=x-radius-1, y2=y-radius;  // Top left - 1
        //still_in_range = false;
        //for (int i=0; i<8*radius; ++i)
        //{
          //if (i<=2*radius) ++x2; else if (i<=4*radius) ++y2; else if (i<=6*radius) --x2; else --y2;
          //if (!range_image.isInImage(x2, y2))
            //continue;
          //int neighbor_index = y2*width+x2;
          //const PointWithRange& neighbor = range_image.getPoint(neighbor_index);
          //if (!isfinite(neighbor.range))
            //continue;
          //float distance_squared = squaredEuclideanDistance(point, neighbor);
          //if (distance_squared > max_distance_squared)
            //continue;
          //still_in_range = true;
          //float distance = sqrtf(distance_squared);
          
          //CloseByBorderInformation& close_by_border_information = close_by_border_informations[neighbor_index];

          ////#pragma omp critical  // Might create problems in rare cases without critical
          //{
            //close_by_border_information.vector_average.add(surface_change_direction, surface_change_score * (1.0f - distance/search_radius));
            //close_by_border_information.distance_to_border = (std::min)(close_by_border_information.distance_to_border, distance);
          //}
        //}
      //}
    //}
  //}
  
  //float* interest_image = new float[array_size];
  
  ////#pragma omp parallel for default(shared) schedule(dynamic, 50)
  //for (int y=0; y<height; ++y)
  //{
    //for (int x=0; x<width; ++x)
    //{
      ////cout << __PRETTY_FUNCTION__ <<": "<<x<<","<<y<<" ("<<width<<","<<height<<")\n";
      //int point_index = y*width + x;
      //float& interest_value = interest_image[point_index];
      //interest_value = 0.0f;
      //CloseByBorderInformation& border_information = close_by_border_informations[point_index];
      //if (border_information.vector_average.getNoOfSamples() < 3)
        //continue;
      //Eigen::Vector3f eigen_values;
      //border_information.vector_average.doPCA(eigen_values);
      
      //// Get score, that punishes being very close to a border or close to the maximum distance
      //float normalized_border_distance = border_information.distance_to_border/search_radius;
      //float optimal_normalized_distance = 0.5f,
            //optimal_normalized_distance_factor = 1.0f / (optimal_normalized_distance*optimal_normalized_distance - optimal_normalized_distance);
      //// The following function goes through (0, 0), (optimal_normalized_distance, 1), and (1, 0)
      //float border_distance_weight = optimal_normalized_distance_factor*(normalized_border_distance*normalized_border_distance - normalized_border_distance);
      //border_distance_weight = pow(border_distance_weight, 2.0f);
      ////cout << "Distance is "<<normalized_border_distance<<" => weighut is "<<border_distance_weight<<"\n";
      
      //float unique_direction_weight;
      //unique_direction_weight = border_information.vector_average.getMean().norm();
      //unique_direction_weight = pow(unique_direction_weight, 0.5f);
      
      //float direction_change_weight = eigen_values.norm();
      //interest_value = direction_change_weight * unique_direction_weight * border_distance_weight;
    //}
  //}
  //delete[] close_by_border_informations;
  
//# if SHOW_DEBUG_IMAGES
    //stringstream ss;
    //ss << " for radius "<<search_radius<<"m";
    //std::string for_radius = ss.str();
    
    ////static ImageWidgetWX* interest_image_widget = new ImageWidgetWX;
    ////interest_image_widget->setFloatImage(interest_image, width, height, ("interest image"+for_radius).c_str(), 0.0f, 0.5f, false);
//#endif  // #if SHOW_DEBUG_IMAGES


//float* NarfKeypoint::getBorderInterestPoints(const RangeImage& range_image, const ::pcl::PointCloud<BorderDescription>& border_descriptions,
                                                                //float support_size, ::pcl::PointCloud<InterestPoint>& interest_points)
//{
  ////MEASURE_FUNCTION_TIME;
  
  //Eigen::Vector3f sensor_pos = range_image.getTransformationToWorldSystem()*Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  
  //int width  = range_image.width,
      //height = range_image.height,
      //array_size = width*height;
  //float max_distance_for_border_directions = 0.3f*support_size,
        //max_distance_for_direction_change = 0.5f*support_size,
        //max_distance_for_direction_change_squared = max_distance_for_direction_change*max_distance_for_direction_change;
  
  //float* angle_change_image = new float[array_size];
  //SET_ARRAY(angle_change_image, 0, array_size);
  
  //Eigen::Vector3f** border_direction_image = getBorderDirections(range_image, border_descriptions, max_distance_for_border_directions);
  
  ////#pragma omp parallel for default(shared) private(checked_points, unchecked_points) schedule(dynamic, 10)
  //for (int y=0; y<height; ++y)
  //{
    //for (int x=0; x<width; ++x)
    //{
      //int point_index = y*width + x;
      ////float& interest_value = angle_change_image[point_index];
      ////interest_value = 0.0f;
      
      //const Eigen::Vector3f* point_border_direction_ptr = border_direction_image[point_index];
      //if (point_border_direction_ptr == NULL) continue;
      ////const Eigen::Vector3f& point_border_direction = *point_border_direction_ptr;
      
      ////angle_change_image[point_index] = std::numeric_limits<float>::infinity ();
      
      //const PointWithRange& point = range_image.getPoint(x, y);
      
      //if (point.range > 2.0)  continue;
      
      ////----------------------------------------------------------------------------------
      //// Region Growing to get border points for the calculation of the dominant direction
      //set<int> checked_points;
      //set<RegionGrowingElement> unchecked_points;
      //unchecked_points.insert(RegionGrowingElement(0.0f, point_index));
      //VectorAverage3f vector_average_border_directions;
      //VectorAverage3f vector_average_border_points;
      ////cout << "----------------\n";
      ////float border_distance_squared = std::numeric_limits<float>::infinity ();
      //while (!unchecked_points.empty()) {
        //int neighbor_index = unchecked_points.begin()->index;
        //const BorderDescription& neighbor_border_description = border_descriptions.points[neighbor_index];
        //checked_points.insert(neighbor_index);
        //unchecked_points.erase(unchecked_points.begin());
        
        //const Eigen::Vector3f* neighbor_border_direction_ptr = border_direction_image[neighbor_index];
        //if (neighbor_border_direction_ptr == NULL) continue;
        //const Eigen::Vector3f& neighbor_border_direction = *neighbor_border_direction_ptr;

        //const PointWithRange& neighbor = range_image.getPoint(neighbor_index);
        
        //float distance_squared = squaredEuclideanDistance(point, neighbor);
        //if (distance_squared > max_distance_for_direction_change_squared)
          //continue;
        
        //for (int i=0; i<(int)neighbor_border_description.neighbors.size(); ++i)
        //{
          //const BorderDescription* border_description3 = neighbor_border_description.neighbors[i];
          //int x3 = border_description3->x,
              //y3 = border_description3->y,
              //index3 = y3*width + x3;
          //if (checked_points.find(index3)==checked_points.end())
            //unchecked_points.insert(RegionGrowingElement(distance_squared, index3));
        //}
        
        //vector_average_border_directions.add(neighbor_border_direction, 1.0f);
        //vector_average_border_points.add(Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z), 1.0f);
      //}
      ////cout << PVARN(vector_average_border_directions.getNoOfSamples());
      //if (vector_average_border_directions.getNoOfSamples() < 3)
        //continue;
      //Eigen::Vector3f eigen_values, eigen_vector1, eigen_vector2, eigen_vector3;
      //vector_average_border_directions.doPCA(eigen_values, eigen_vector1, eigen_vector2, eigen_vector3);

      //Eigen::Vector3f eigen_values_2, eigen_vector1_2, eigen_vector2_2, eigen_vector3_2;
      //vector_average_border_points.doPCA(eigen_values_2, eigen_vector1_2, eigen_vector2_2, eigen_vector3_2);
      
      ////float border_distance_weight = (sqrtf(border_distance_squared)-0.5*search_radius)/(0.5*search_radius);  // [-1,1]
      ////border_distance_weight = -fabs(border_distance_weight) + 1;
      ////cout << "Distance is "<<sqrtf(border_distance_squared)<<"/"<<search_radius<<" => weight is "<<border_distance_weight<<"\n";
      
      ////float unique_direction_weight = vector_average_border_directions.getMean().norm();
      ////unique_direction_weight = pow(vector_average_border_directions.getMean().norm(), 2);
      
      //float direction_change_weight = eigen_values.norm();
      
      ////interest_value = direction_change_weight;
      //int mean_x, mean_y;
      //const Eigen::Vector3f& mean = vector_average_border_points.getMean();
      //range_image.getImagePoint(mean[0], mean[1], mean[2], mean_x, mean_y);
      //float interest_value = direction_change_weight;
      
      ////angle_change_image[mean_y*width + mean_x] = interest_value;
      ////angle_change_image[point_index] = eigen_values_2[1];
      //angle_change_image[point_index] = direction_change_weight;
      ////cout << point_index<<" => "<<angle_change_image[point_index]<<"\n";

      //if (interest_value > 0.1)
      //{
        //InterestPoint interest_point;
        //interest_point.x=mean[0];  interest_point.y=mean[1];  interest_point.z=mean[2];
        //interest_point.strength = interest_value;
        //interest_points.points.push_back(interest_point);
      //}
    //}
  //}
  //for (int i=0; i<array_size; ++i)
    //delete border_direction_image[i];
  //delete[] border_direction_image;
  
//# if SHOW_DEBUG_IMAGES
    //stringstream ss;
    //ss << " for feature size "<<support_size<<"m";
    //std::string for_radius = ss.str();
    
    ////static ImageWidgetWX* angle_change_image_widget = new ImageWidgetWX;
    //ImageWidgetWX* angle_change_image_widget = new ImageWidgetWX;
    //angle_change_image_widget->setFloatImage(angle_change_image, width, height, ("angle change image"+for_radius).c_str());
//#endif  // #if SHOW_DEBUG_IMAGES

  //interest_points.width = interest_points.points.size();
  //interest_points.height = 1;
  
  //return angle_change_image;
//}

//struct CloseByBorderInformation {
  //CloseByBorderInformation() : distance_to_border(std::numeric_limits<float>::infinity ()) {}
  //VectorAverage3f vector_average;
  //float distance_to_border;
//};



//float* NarfKeypoint::getBorderInterestPoints(const RangeImage& range_image, const ::pcl::PointCloud<BorderDescription>& border_descriptions,
                                                                //float support_size)
//{
  //MEASURE_FUNCTION_TIME;
  
  //Eigen::Vector3f sensor_pos = range_image.getTransformationToWorldSystem()*Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  
  //int width  = range_image.width,
      //height = range_image.height,
      //array_size = width*height;
  //float search_radius = 0.5*support_size,
        //max_distance_squared = search_radius*search_radius;
  
  //float* interest_image = new float[array_size];
  
  //RangeImageBorderExtractor range_image_border_extractor;
  //Eigen::Vector3f** border_direction_image = range_image_border_extractor.getBorderDirections(range_image, border_descriptions, search_radius);
  
  //set<int> checked_points, unchecked_points;
  //#pragma omp parallel for default(shared) private(checked_points, unchecked_points) schedule(dynamic, 10)
  //for (int y=0; y<height; ++y)
  //{
    //for (int x=0; x<width; ++x)
    //{
      //int point_index = y*width + x;
      //float& interest_value = interest_image[point_index];
      //interest_value = 0.0f;
      //const PointWithRange& point = range_image.getPoint(point_index);
      //if (pcl_isinf(point.range))
        //continue;
      
      //// Now do some region growing to extract the part of the surface and the border, that we are interested in
      //checked_points.clear();
      //unchecked_points.clear();
      //unchecked_points.insert(point_index);
      //VectorAverage3f vector_average_border_directions;
      ////cout << "----------------\n";
      //float border_distance_squared = std::numeric_limits<float>::infinity ();
      //while (!unchecked_points.empty()) {
        //int neighbor_index = *unchecked_points.begin();
        //checked_points.insert(neighbor_index);
        //unchecked_points.erase(unchecked_points.begin());
        //const PointWithRange& neighbor = range_image.getPoint(neighbor_index);
        //if (pcl_isinf(neighbor.range))
          //continue;
        //float distance_squared = squaredEuclideanDistance(point, neighbor);
        //if (distance_squared > max_distance_squared)
          //continue;
        //const BorderTraits& neighbor_border_traits = border_descriptions.points[neighbor_index].traits;
        //if (neighbor_border_traits[BORDER_TRAIT__SHADOW_BORDER])
          //continue;
        
        //if (neighbor_border_traits[BORDER_TRAIT__OBSTACLE_BORDER])
          //border_distance_squared = (std::min)(border_distance_squared, distance_squared);
        
        //const Eigen::Vector3f* neighbor_border_direction_ptr = border_direction_image[neighbor_index];
        //if (neighbor_border_direction_ptr != NULL) {
          //const Eigen::Vector3f& neighbor_border_direction = *neighbor_border_direction_ptr;
          //vector_average_border_directions.add(neighbor_border_direction, 1.01f - sqrtf(distance_squared)/search_radius);
        //}
        
        //int y2=neighbor_index/width, x2=neighbor_index - y2*width;
        //for (int i=0; i<4; ++i)
        //{
          //int x3, y3;
          //switch (i) {
            //case 0 : x3=x2-1; y3=y2;   break;  // left
            //case 1 : x3=x2+1; y3=y2;   break;  //right
            //case 2 : x3=x2;   y3=y2-1; break;  // top
            //case 3 : x3=x2;   y3=y2+1; break;  // bottom
          //}
          //int index3 = y3*width+x3;
          //if (range_image.isInImage(x3, y3) && checked_points.find(index3)==checked_points.end())
            //unchecked_points.insert(index3);
        //}
      //}
      //if (vector_average_border_directions.getNoOfSamples() < 2)
        //continue;
      //Eigen::Vector3f eigen_values, eigen_vector1, eigen_vector2, eigen_vector3;
      //vector_average_border_directions.doPCA(eigen_values, eigen_vector1, eigen_vector2, eigen_vector3);
      
      //// Get that punishes being very close to a border or close to the maximum distance
      //float normalized_border_distance = sqrtf(border_distance_squared)/search_radius;
      //float optimal_normalized_distance = 0.5f,
            //optimal_normalized_distance_factor = 1.0f / (optimal_normalized_distance*optimal_normalized_distance - optimal_normalized_distance);
      //// The following function goes through (0, 0), (optimal_normalized_distance, 1), and (1, 0)
      //float border_distance_weight = optimal_normalized_distance_factor*(normalized_border_distance*normalized_border_distance - normalized_border_distance);
      //border_distance_weight = pow(border_distance_weight, 2.0f);
      ////cout << "Distance is "<<normalized_border_distance<<" => weighut is "<<border_distance_weight<<"\n";
      
      //float unique_direction_weight = 1.0f;
      //unique_direction_weight = vector_average_border_directions.getMean().norm();
      //unique_direction_weight = pow(vector_average_border_directions.getMean().norm(), 0.5f);
      
      //float direction_change_weight = eigen_values.norm();
      //interest_value = direction_change_weight * unique_direction_weight * border_distance_weight;
    //}
  //}
  //for (int i=0; i<array_size; ++i)
    //delete border_direction_image[i];
  
//# if SHOW_DEBUG_IMAGES
    //stringstream ss;
    //ss << " for radius "<<search_radius<<"m";
    //std::string for_radius = ss.str();
    
    ////static ImageWidgetWX* angle_change_image_widget = new ImageWidgetWX;
    ////ImageWidgetWX* angle_change_image_widget = new ImageWidgetWX;
    ////angle_change_image_widget->setFloatImage(interest_image, width, height, ("angle change image"+for_radius).c_str(), 0.0f, 0.5f, false);
//#endif  // #if SHOW_DEBUG_IMAGES
  
  //return interest_image;
//}





}  // end namespace pcl


