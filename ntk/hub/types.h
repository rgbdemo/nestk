#pragma once

#include "ntk/mesh/meshfwd.h"
#include <QtGlobal>

class QString;
class QStringList;
class QImage;

namespace cv { class Mat; }

//------------------------------------------------------------------------------

namespace ntk { namespace hub {

typedef     QString Name;
typedef     QString Line;
typedef QStringList Lines;
typedef       qreal Percentage;
typedef      QImage Image;
typedef     cv::Mat Matrix;
typedef        Mesh Mesh;

//------------------------------------------------------------------------------

void registerTypes ();

} }
