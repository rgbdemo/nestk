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

#ifndef   	NTK_CORE_H_
# define   	NTK_CORE_H_

# include <limits>
# include <math.h>

# include "opencv2/core/core_c.h"
# include "opencv2/core/core.hpp"
# include "opencv2/imgproc/imgproc_c.h"
# include "opencv2/imgproc/imgproc.hpp"
# include "opencv2/calib3d/calib3d.hpp"
# include "opencv2/objdetect/objdetect.hpp"
# include "opencv2/legacy/compat.hpp"
# include <opencv2/highgui/highgui.hpp>

#ifndef FLT_MAX
# define FLT_MAX std::numeric_limits<double>::max()
#endif

#ifndef M_PI
# define M_PI 3.141592653589793238462643
#endif

# include <ntk/utils/common.h>

#endif	    /* !NTK_CORE_H_ */
