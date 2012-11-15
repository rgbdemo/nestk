#include "hub-impl.h"
#include "outlet-impl.h"
#include <QMutexLocker>
#include <cassert>

namespace ntk { namespace hub {

Hub::Impl::Impl (Hub* that)
    : enabled(true)
    , that(that)
{
    assert(0 != that);
}

Hub::Impl::~Impl ()
{

}

void
Hub::Impl::quit ()
{
    that->quit();
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
Hub::Impl::subscribeOutlet (Outlet* outlet, const String& name)
{
    QMutexLocker _(&outletInfosMutex);

    OutletInfo& outletInfo = outletInfos[outlet];

    outletInfo.subscriptions.insert(name);
}

void
Hub::Impl::unsubscribeOutlet (Outlet* outlet, const String& name)
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

void
Hub::Impl::setEnabled (bool enabled_)
{
    HUB_IMPL_LOCKED(enabled)

    enabled = enabled_;
}

void
Hub::Impl::enable ()
{
    setEnabled(true);
}

void
Hub::Impl::disable ()
{
    setEnabled(false);
}

bool
Hub::Impl::isActive (const QString& name)
{
    {
        HUB_IMPL_LOCKED(enabled)

        if (!enabled)
            return false;
    }

    {
        HUB_IMPL_LOCKED(activeSubscriptions);

        const ActiveSubscriptions::ConstIterator i = activeSubscriptions.find(name);

        if (i == activeSubscriptions.end())
        {
            return false;
        }

        return 0 < i.value();
    }
}

} }
