#include "outlet.h"
#include "hub.h"
#include "update.h"
#include "thread/event.h"

namespace ntk { namespace hub {

struct Outlet::Impl : public ntk::EventListener
{
    Impl (Outlet* that) : that(that)
    {
        Hub::getInstance()->addEventListener(this);
    }

    ~Impl ()
    {
        Hub::getInstance()->removeEventListener(this);
    }

    virtual void newEvent (ntk::EventBroadcaster* sender, ntk::EventDataPtr eventData)
    {
        HubUpdatePtr update = ntk::dynamic_Ptr_cast<HubUpdate>(eventData);

        ntk_assert(update, "Invalid hub update. Something is very wrong.");

        update->updateOutlet(*that);
    }

    Outlet* that;
};

//------------------------------------------------------------------------------

Outlet::Outlet ()
: impl(new Impl(this))
{

}

Outlet::~Outlet ()
{
    delete impl;
}

//------------------------------------------------------------------------------

QOutlet::QOutlet (QObject* parent)
: QObject(parent)
{

}

QOutlet::~QOutlet ()
{

}

void
QOutlet::changeStatus (QString name, QString status)
{
    emit statusChanged(name, status);
}

void
QOutlet::changeProgress (QString name, qreal progress)
{
    emit progressChanged(name, progress);
}

void
QOutlet::changeLog (QString name, QStringList log)
{
    emit logChanged(name, log);
}

void
QOutlet::changeImage (QString name, QImage image)
{
    emit imageChanged(name, image);
}

void
QOutlet::changeMesh (QString name, const Mesh* mesh)
{
    emit meshChanged(name, mesh);
}

//------------------------------------------------------------------------------

OutletListener::~OutletListener ()
{

}

void
OutletListener::changeStatus (QString name, QString status)
{
    onStatusChanged(name, status);
}

void
OutletListener::changeProgress (QString name, qreal progress)
{
    onProgressChanged(name, progress);
}

void
OutletListener::changeLog (QString name, QStringList log)
{
    onLogchanged(name, log);
}

void
OutletListener::changeImage (QString name, QImage image)
{
    onImageChanged(name, image);
}

void
OutletListener::changeMesh (QString name, const Mesh* mesh)
{
    onMeshChanged(name, mesh);
}

} }
