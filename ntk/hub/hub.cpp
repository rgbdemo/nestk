#include "hub.h"
#include "hub-impl.h"
#include "outlet.h"
#include "updates.h"
#include "../mesh/mesh.h"
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
    : impl(new Impl)
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

    // FIXME: Trick the event system into believing we have one sender per update.
    // FIXME: This only takes into account the update target :-/
    EventBroadcaster* sender = reinterpret_cast<EventBroadcaster*>(qHash(update->name.toAscii()));

    newEvent(sender, HubUpdatePtr(update));
}

//------------------------------------------------------------------------------

QString
Hub::getStatus (const Name& name) const
{
    QMutexLocker _(&impl->statusesMutex);

    return impl->statuses[name];
}

void
Hub::setStatus (const Name& name, const Line& status)
{
    postUpdate(new StatusUpdate(name, status));
}

void
Hub::clearStatus (const Name& name)
{
    postUpdate(new StatusUpdate(name, QString()));
}

//------------------------------------------------------------------------------

qreal
Hub::getProgress (const Name& name) const
{
    QMutexLocker _(&impl->progressesMutex);

    return impl->progresses[name];
}

void
Hub::setProgress (const Name& name, qreal progress)
{
    postUpdate(new ProgressUpdate(name, progress));
}

void
Hub::clearProgress (const Name &name)
{
    postUpdate(new ProgressUpdate(name, 0.));
}

//------------------------------------------------------------------------------

Lines
Hub::getLog (const Name& name) const
{
    QMutexLocker _(&impl->logsMutex);

    return impl->logs[name];
}

void
Hub::setLog (const Name& name, const Lines& log)
{
    postUpdate(new SetLogUpdate(name, log));
}

void
Hub::appendLog (const Name& name, const Line& line)
{
    postUpdate(new AppendLogUpdate(name, line));
}

void
Hub::clearLog (const Name& name)
{
    postUpdate(new ClearLogUpdate(name));
}

//------------------------------------------------------------------------------

QImage
Hub::getImage (const Name& name) const
{
    QMutexLocker _(&impl->imagesMutex);

    return impl->images[name];
}

void
Hub::setImage (const Name& name, const Image& image)
{
    postUpdate(new SetImageUpdate(name, image));
}

void
Hub::setImage (const Name& name, const Matrix& matrix)
{
    postUpdate(new SetImageMatrixUpdate(name, matrix));
}

void
Hub::clearImage (const Name& name)
{
    postUpdate(new ClearImageUpdate(name));
}

//------------------------------------------------------------------------------

MeshConstPtr
Hub::getMesh (const Name& name) const
{
    QMutexLocker _(&impl->meshesMutex);

    return impl->meshes[name];
}

void
Hub::setMesh (const Name& name, MeshConstPtr mesh)
{
    postUpdate(new SetMeshUpdate(name, mesh));
}

void
Hub::setMesh (const Name& name, const Mesh& mesh)
{
    postUpdate(new SetMeshUpdate(name, MeshConstPtr(new Mesh(mesh))));
}

void
Hub::clearMesh (const Name& name)
{
    postUpdate(new ClearMeshUpdate(name));
}

} }
