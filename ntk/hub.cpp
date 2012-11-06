#include "hub.h"
#include "hub/hub.h"

namespace ntk { namespace hub {

#define HUB_FWD_0(Fun)             \
void Fun (const Name& name)        \
{                                  \
    Hub::getInstance()->Fun(name); \
}

#define HUB_FWD_1(Fun, Arg0)              \
void Fun (const Name& name, Arg0 arg0)    \
{                                         \
    Hub::getInstance()->Fun(name, arg0);  \
}

HUB_FWD_1(   setStatus,   const Line&)
HUB_FWD_0( clearStatus)
HUB_FWD_1(   setProgress, Percentage)
HUB_FWD_0( clearProgress)
HUB_FWD_1(   setLog,      const Lines&)
HUB_FWD_1(appendLog,      const Line&)
HUB_FWD_0( clearLog)
HUB_FWD_1(   setImage,    const Image&)
HUB_FWD_1(   setImage,    const Matrix&)
HUB_FWD_0( clearImage)
HUB_FWD_1(   setMesh,     const Mesh&)
HUB_FWD_0( clearMesh)

void report (const Name& name, const Line& status, Percentage progress)
{
    setStatus(  name, status);
    setProgress(name, progress);
}

void finish (const Name& name)
{
    clearStatus(  name);
    clearProgress(name);
}

} }
