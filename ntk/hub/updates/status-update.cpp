#include "status-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include "hub/hub-impl.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::StatusUpdate::StatusUpdate (Name name, Line status)
: Hub::Update(name)
, status(status)
{

}

void
Hub::StatusUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.impl->statusesMutex);

    hub.impl->statuses[name] = status;
}

void
Hub::StatusUpdate::updateOutlet (Outlet& outlet)
{
    outlet.onStatusChanged(name, status);
}

} }
