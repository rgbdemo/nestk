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
