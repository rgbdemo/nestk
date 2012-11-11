#include "hub-impl.h"
#include "outlet-impl.h"
#include <QMutexLocker>
#include <cassert>

namespace ntk { namespace hub {

Hub::Impl::Impl (Hub* that)
    : that(that)
{
    assert(0 != that);
}

void
Hub::Impl::attachOutlet (Outlet* outlet)
{
    QMutexLocker _(&outletInfosMutex);

    outletInfos[outlet].running = false;
    outletInfos[outlet].subscriptions = OutletInfo::Subscriptions();
}

void
Hub::Impl::detachOutlet (Outlet* outlet)
{
    stopOutlet(outlet);

    QMutexLocker _(&outletInfosMutex);

    outletInfos.remove(outlet);
}

void
Hub::Impl::subscribeOutlet (Outlet* outlet, String name)
{
    QMutexLocker _(&outletInfosMutex);

    OutletInfo& outletInfo = outletInfos[outlet];

    outletInfo.subscriptions.insert(name);
}


void
Hub::Impl::unsubscribeOutlet (Outlet* outlet, String name)
{
    QMutexLocker _(&outletInfosMutex);

    OutletInfo& outletInfo = outletInfos[outlet];

    outletInfo.subscriptions.remove(name);
}


void
Hub::Impl::startOutlet (Outlet* outlet)
{
    QMutexLocker _(&outletInfosMutex);

    OutletInfo& outletInfo = outletInfos[outlet];

    if (outletInfo.running)
        return;

    {
        QMutexLocker _(&activeSubscriptionsMutex);

        foreach (const String& name, outletInfo.subscriptions)
            ++activeSubscriptions[name];
    }

    that->addEventListener(outlet->impl);

    outletInfo.running = true;
}

void
Hub::Impl::stopOutlet (Outlet* outlet)
{
    QMutexLocker _(&outletInfosMutex);

    OutletInfo& outletInfo = outletInfos[outlet];

    if (!outletInfo.running)
        return;

    {
        QMutexLocker _(&activeSubscriptionsMutex);

        foreach (const String& name, outletInfo.subscriptions)
            --activeSubscriptions[name];
    }

    that->removeEventListener(outlet->impl);

    outletInfo.running = false;
}

} }
