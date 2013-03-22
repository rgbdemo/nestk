/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Tisserand <nicolas.tisserand@manctl.com>
 */

#ifndef NTK_HUB_OUTLET_IMPL_H
# define NTK_HUB_OUTLET_IMPL_H

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

    void subscribe (const String& name)
    {
        Hub::getInstance()->subscribeOutlet(that, name);
    }

    void unsubscribe (const String& name)
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

#endif // !NTK_HUB_OUTLET_IMPL_H
