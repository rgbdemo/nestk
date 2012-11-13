#pragma once

#include "ntk/thread/event.h"
#include "types.h"

namespace ntk { namespace hub {

struct Outlet;

//------------------------------------------------------------------------------

class Hub
: public  ntk::EventBroadcaster
, private ntk::AsyncEventListener
{
public:
    static Hub* getInstance ();

public:
     Hub ();
    ~Hub ();

public:
     void enable  ();
     void disable ();

#define HUB_TYPE(Type, type, Arg, Ret, Val)                      \
public:                                                          \
     Ret       get##Type (HUB_TYPE_ARG(String) name) const;      \
     void      set##Type (HUB_TYPE_ARG(String) name, Arg arg);   \
     void    reset##Type (HUB_TYPE_ARG(String) name);
        HUB_TYPES()
#undef  HUB_TYPE

public:
    void appendToStrings (const String& name, const String& string);
    void  setImageMatrix (const String& name, const Matrix& matrix);
    void  setMesh        (const String& name, const Mesh&     mesh);

protected:
    virtual void handleAsyncEvent (Event event);

public:
    class               Update;
    class        SetRealUpdate;
    class      SetStringUpdate;
    class        StringsUpdate;
    class     SetStringsUpdate;
    class  AppendStringsUpdate;
    class   ClearStringsUpdate;
    class          ImageUpdate;
    class       SetImageUpdate;
    class SetMatrixImageUpdate;
    class     ClearImageUpdate;
    class           MeshUpdate;
    class        SetMeshUpdate;
    class      ClearMeshUpdate;

public:
    void      attachOutlet (Outlet* outlet);
    void      detachOutlet (Outlet* outlet);
    void   subscribeOutlet (Outlet* outlet, const String& name);
    void unsubscribeOutlet (Outlet* outlet, const String& name);
    void       startOutlet (Outlet* outlet);
    void        stopOutlet (Outlet* outlet);

private:
    void postUpdate (Update* update);
    void quit ();

private:
    struct Impl;
    Impl*  impl;
};

} }
