#pragma once

#include <QtGlobal>

class QString;
class QStringList;
class QImage;

namespace cv { class Mat; }

namespace ntk { class Mesh; }

//------------------------------------------------------------------------------

namespace ntk { namespace hub {

typedef     QString Name;
typedef     QString Line;
typedef QStringList Lines;
typedef       qreal Percentage;
typedef      QImage Image;
typedef     cv::Mat Matrix;
typedef        Mesh Mesh;

} }
