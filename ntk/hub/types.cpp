#include "types.h"
#include <QMetaType>
#include <QString>
#include <QStringList>
#include <QtGlobal>
#include <QImage>
#include <mesh/mesh.h>
#include <opencv/cv.h>

namespace ntk { namespace hub {

#define HUB_TYPE(Type, type, Arg, Ret, Val)  \
const char* TypeTraits<Type>::Name = #Type;  \
const char* TypeTraits<Type>::name = #type;
        HUB_TYPES()
#undef  HUB_TYPE

//------------------------------------------------------------------------------

void registerTypes ()
{
#define HUB_TYPE(Type, type, Arg, Ret, Val) \
    qRegisterMetaType<Type>(#Type);
        HUB_TYPES()
#undef  HUB_TYPE
}

} }
