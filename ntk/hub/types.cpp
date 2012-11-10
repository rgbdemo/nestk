#include "types.h"
#include <QMetaType>
#include <QString>
#include <QStringList>
#include <QtGlobal>
#include <QImage>
#include <mesh/mesh.h>
#include <opencv/cv.h>

namespace ntk { namespace hub {

void registerTypes ()
{
    qRegisterMetaType<ntk::hub::Name >("Name");
    qRegisterMetaType<ntk::hub::Line >("Line");
    qRegisterMetaType<ntk::hub::Lines>("Lines");
    qRegisterMetaType<ntk::hub::Image>("Percentage");
    qRegisterMetaType<ntk::hub::Image>("Image");
    qRegisterMetaType<ntk::hub::Mesh >("Mesh");
}

} }
