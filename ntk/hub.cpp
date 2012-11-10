#include "hub.h"
#include "hub/hub.h"
#include "mesh/mesh.h"

namespace {

inline QString prefixed (const char* prefix, const QString& name)
{
    return QString(prefix) + "." + name;
}

}

//------------------------------------------------------------------------------

namespace ntk { namespace hub {

void report (const String& name, const String& status, Real progress)
{
    setStatus(  name, status);
    setProgress(name, progress);
}

void finish (const String& name)
{
    resetStatus(  name);
    resetProgress(name);
}

//------------------------------------------------------------------------------

#define HUB_SET(Function, Type, Prefix)                          \
void                                                             \
Function (const String& name, HUB_TYPE_ARG(Type) arg)            \
{                                                                \
    Hub::getInstance()->set##Type(prefixed(#Prefix, name), arg); \
}

#define HUB_RESET(Function, Type, Prefix)                          \
void Function (const String& name)                                 \
{                                                                  \
    Hub::getInstance()->reset##Type(prefixed(#Prefix, name));      \
}

#define HUB_CLASS(Name, Type, Prefix) \
HUB_SET(     set##Name, Type, Prefix) \
HUB_RESET( reset##Name, Type, Prefix)

HUB_CLASS(Status  , String , statuses  )
HUB_CLASS(Progress, Real   , progresses)
HUB_CLASS(Log     , Strings, logs      )
HUB_CLASS(Image   , Image  , images    )
HUB_CLASS(Mesh    , Mesh   , meshes    )

//------------------------------------------------------------------------------

#define HUB_FWD_0(Function, Method, Prefix)                \
    void                                                   \
Function (const String& name)                              \
{                                                          \
    Hub::getInstance()->Function(prefixed(#Prefix, name)); \
}

#define HUB_FWD_1(Function, Method, Prefix, Arg0)              \
void                                                           \
Function (const String& name, Arg0 arg0)                       \
{                                                              \
    Hub::getInstance()->Method(prefixed(#Prefix, name), arg0); \
}

HUB_FWD_1(appendLog , appendToStrings, logs  , const String&);
HUB_FWD_1(setImage  , setImageMatrix , images, const Matrix&);
HUB_FWD_1(setMesh   , setMesh        , meshes, const Mesh&);

} }
