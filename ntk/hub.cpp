#include "hub.h"
#include "hub/hub.h"
#include "mesh/mesh.h"

/*
    FIXME:

Thread 0 Crashed:  Dispatch queue: com.apple.main-thread
0   QtGui                         	0x000000011c2d9689 QImagePixmapCleanupHooks::executeImageHooks(long long) + 57
1   QtGui                         	0x000000011c2a9ef4 QImageData::~QImageData() + 196
2   QtGui                         	0x000000011c2aa041 QImage::~QImage() + 65
3   skanpad                       	0x00000001001d5b8c QHash<QString, QImage>::deleteNode2(QHashData::Node*) + 28
4   QtCore                        	0x000000011d730cba QHashData::free_helper(void (*)(QHashData::Node*)) + 138
5   skanpad                       	0x00000001001d543e ntk::hub::Hub::Impl::~Impl() + 254
6   skanpad                       	0x00000001001d45d6 ntk::hub::Hub::~Hub() + 86
7   libSystem.B.dylib             	0x00007fff87800374 __cxa_finalize + 203
8   libSystem.B.dylib             	0x00007fff8780028c exit + 18
9   skanpad                       	0x000000010001b99b start + 59

*/

namespace {

inline QString prefixed (const char* prefix, const QString& name)
{
    return QString(prefix) + "." + name;
}

}

//------------------------------------------------------------------------------

namespace ntk { namespace hub {

void setStatus (const String& status)
{
    Hub::getInstance()->setString("status", status);
}

void   resetStatus ()
{
    Hub::getInstance()->resetString("status");
}

void   setProgress (Real progress)
{
    Hub::getInstance()->setReal("progress", progress);
}

void resetProgress ()
{
    Hub::getInstance()->resetReal("progress");
}

void report (const String& status, Real progress)
{
    setStatus(status);
    setProgress(progress);
}

void finish ()
{
    resetStatus();
    resetProgress();
}

//------------------------------------------------------------------------------

#define HUB_SET(Function, Type)                        \
void                                                   \
Function (const String& name, HUB_TYPE_ARG(Type) arg)  \
{                                                      \
    Hub::getInstance()->set##Type(name, arg);          \
}

#define HUB_RESET(Function, Type)          \
void Function (const String& name)         \
{                                          \
    Hub::getInstance()->reset##Type(name); \
}

#define HUB_CLASS(Name, Type) \
HUB_SET(     set##Name, Type) \
HUB_RESET( reset##Name, Type)

HUB_CLASS(Real    , Real   )
HUB_CLASS(Status  , String )
HUB_CLASS(Log     , Strings)
HUB_CLASS(Image   , Image  )
HUB_CLASS(Mesh    , Mesh   )

//------------------------------------------------------------------------------

#define HUB_FWD_0(Function, Method, Prefix)  \
    void                                     \
Function (const String& name)                \
{                                            \
    Hub::getInstance()->Function(name);      \
}

#define HUB_FWD_1(Function, Method, Prefix, Arg0) \
void                                              \
Function (const String& name, Arg0 arg0)          \
{                                                 \
    Hub::getInstance()->Method(name, arg0);       \
}

HUB_FWD_1(appendLog , appendToStrings, stringlists , const String&);
HUB_FWD_1(setImage  , setImageMatrix , images      , const Matrix&);
HUB_FWD_1(setMesh   , setMesh        , meshes      , const Mesh&  );

} }
