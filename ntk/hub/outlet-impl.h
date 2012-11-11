#pragma once

#include "outlet.h"
#include "hub.h"
#include "update.h"
#include "thread/event.h"

namespace ntk { namespace hub {

struct Outlet::Impl : public ntk::EventListener
{
    Impl (Outlet* that)
        : that(that)
    {
        Hub::getInstance()->attachOutlet(that);
    }

    ~Impl ()
    {
        Hub::getInstance()->detachOutlet(that);
    }

    void subscribe (String name)
    {
        Hub::getInstance()->subscribeOutlet(that, name);
    }

    void unsubscribe (String name)
    {
        Hub::getInstance()->unsubscribeOutlet(that, name);
    }

    void start ()
    {
        Hub::getInstance()->startOutlet(that);
    }

    void stop ()
    {
        Hub::getInstance()->stopOutlet(that);
    }

    virtual void newEvent (ntk::EventBroadcaster* sender, ntk::EventDataPtr eventData)
    {
        HubUpdatePtr update = ntk::dynamic_Ptr_cast<HubUpdate>(eventData);

        ntk_assert(update, "Invalid hub update. Something is very wrong.");

        update->updateOutlet(*that);
    }

    Outlet* that;
};

} }
