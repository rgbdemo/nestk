#include "string-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include "hub/hub-impl.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::SetStringUpdate::SetStringUpdate (String name, String newValue)
: Hub::Update(name)
, newValue(newValue)
{

}

void
Hub::SetStringUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.impl->stringValuesMutex);

    hub.impl->stringValues[name] = newValue;
}

void
Hub::SetStringUpdate::updateOutlet (Outlet& outlet)
{
    outlet.onStringChanged(name, newValue);
}

} }
