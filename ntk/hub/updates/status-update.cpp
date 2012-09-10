#include "status-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::StatusUpdate::StatusUpdate (QString name, QString status)
: Hub::Update(name)
, status(status)
{

}

void
Hub::StatusUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.statusesMutex);

    hub.statuses[name] = status;
}

void
Hub::StatusUpdate::updateOutlet (Outlet& outlet)
{
    outlet.changeStatus(name, status);
}

} }
