#include "outlet.h"
#include "hub.h"
#include "update.h"
#include "thread/event.h"
#include <QStringList>
#include <QImage>
#include <cassert>

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
QOutlet::onStringChanged (String name, const String& string)
{
    emit stringChanged(name, string);
}

void
QOutlet::onRealChanged (String name, Real real)
{
    emit realChanged(name, real);
}

void
QOutlet::onStringsChanged (String name, const Strings& strings)
{
    emit stringsChanged(name, strings);
}

void
QOutlet::onImageChanged (String name, const Image& image)
{
    emit imageChanged(name, image);
}

void
QOutlet::onMeshChanged (String name, const Mesh& mesh)
{
    emit meshChanged(name, mesh);
}

} }
