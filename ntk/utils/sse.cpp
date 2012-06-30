#include "sse.h"

#include <ntk/geometry/pose_3d.h>

namespace ntk
{

VectorialProjector::VectorialProjector(const Pose3D &pose)
{
    sse_proj = toSSE(pose.cvProjectionMatrix());
    sse_unproj = toSSE(pose.cvInvProjectionMatrix());
}

}
