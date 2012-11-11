#include "hub.h"
#include "hub-impl.h"
#include "outlet.h"
#include "updates.h"
#include "mesh/mesh.h"
#include "impl.h"
#include <QHash>
#include <QMutex>
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub*
Hub::getInstance()
{
    static Hub instance;

    return &instance;
}

//------------------------------------------------------------------------------

Hub::Hub ()
    : impl(new Impl(this))
{

}

Hub::~Hub ()
{
    delete impl;
}

//------------------------------------------------------------------------------

void
Hub::handleAsyncEvent (EventListener::Event event)
{
    HubUpdatePtr update = dynamic_Ptr_cast<HubUpdate>(event.data);

    ntk_assert(update, "Invalid hub update. Something is very wrong.");

    update->updateHub(*this);

    broadcastEvent(update);
}

void
Hub::postUpdate (Update* update)
{
    assert(0 != update);

    if (impl->maybeAddName(update->name))
    {
        // FIXME: Newborn value.
    }

    // FIXME: Trick the event system into believing we have one sender per update.
    // FIXME: This only takes into account the update target :-/
    EventBroadcaster* sender = reinterpret_cast<EventBroadcaster*>(qHash(update->name.toAscii()));

    newEvent(sender, HubUpdatePtr(update));
}

//------------------------------------------------------------------------------

#define HUB_TYPE(Type, type, Arg, Ret, Val)          \
Ret                                                  \
Hub::get##Type (HUB_TYPE_ARG(String) name) const     \
{                                                    \
    QMutexLocker _(&impl->type##ValuesMutex);        \
                                                     \
    return impl->type##Values[name];                 \
}                                                    \
                                                     \
void                                                 \
Hub::set##Type (HUB_TYPE_ARG(String) name, Arg arg)  \
{                                                    \
    if (!impl->isActive(name))                       \
        return;                                      \
                                                     \
    postUpdate(new Set##Type##Update(name, arg));    \
}                                                    \
                                                     \
void                                                 \
Hub::reset##Type (HUB_TYPE_ARG(String) name)         \
{                                                    \
    if (!impl->isActive(name))                       \
        return;                                      \
                                                     \
    postUpdate(new Set##Type##Update(name, Val()));  \
}

HUB_TYPES()

#undef HUB_TYPE

//------------------------------------------------------------------------------

void
Hub::appendToStrings (const String& name, const String& string)
{
    if (!impl->isActive(name))
        return;

    postUpdate(new AppendStringsUpdate(name, string));
}

void
Hub::setImageMatrix (const String& name, const Matrix& matrix)
{
    if (!impl->isActive(name))
        return;

    postUpdate(new SetMatrixImageUpdate(name, matrix));
}

void
Hub::setMesh (const String& name, const Mesh& mesh)
{
    if (!impl->isActive(name))
        return;

    postUpdate(new SetMeshUpdate(name, MeshConstPtr(new Mesh(mesh))));
}

//------------------------------------------------------------------------------

FWD_IMPL_0(void, Hub, enable )
FWD_IMPL_0(void, Hub, disable)

FWD_IMPL_1(void, Hub,      attachOutlet, Outlet*)
FWD_IMPL_1(void, Hub,      detachOutlet, Outlet*)
FWD_IMPL_2(void, Hub,   subscribeOutlet, Outlet*, String)
FWD_IMPL_2(void, Hub, unsubscribeOutlet, Outlet*, String)
FWD_IMPL_1(void, Hub,       startOutlet, Outlet*)
FWD_IMPL_1(void, Hub,        stopOutlet, Outlet*)

} }
