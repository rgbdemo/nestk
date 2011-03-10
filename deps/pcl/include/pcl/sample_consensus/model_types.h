/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: model_types.h 34248 2010-11-25 04:33:06Z rusu $
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_MODEL_TYPES_H_
#define PCL_SAMPLE_CONSENSUS_MODEL_TYPES_H_

namespace pcl
{
  const static int SACMODEL_PLANE                 = 0;
  const static int SACMODEL_LINE                  = 1;
  const static int SACMODEL_CIRCLE2D              = 2;
  const static int SACMODEL_CIRCLE3D              = 3;
  const static int SACMODEL_SPHERE                = 4;
  const static int SACMODEL_CYLINDER              = 5;
  const static int SACMODEL_CONE                  = 6;
  const static int SACMODEL_TORUS                 = 7;
  const static int SACMODEL_PARALLEL_LINE         = 8;
  const static int SACMODEL_PERPENDICULAR_PLANE   = 9;
  const static int SACMODEL_PARALLEL_LINES        = 10;
  const static int SACMODEL_NORMAL_PLANE          = 11;
  const static int SACMODEL_REGISTRATION          = 12;
  const static int SACMODEL_PARALLEL_PLANE        = 13;
  const static int SACMODEL_NORMAL_PARALLEL_PLANE = 14;
}

#endif  //#ifndef PCL_SAMPLE_CONSENSUS_MODEL_TYPES_H_
