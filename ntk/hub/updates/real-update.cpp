#include "real-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include "hub/hub-impl.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::SetRealUpdate::SetRealUpdate (String name, Real value)
: Hub::Update(name)
, value(value)
{

}

void
Hub::SetRealUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.impl->realValuesMutex);

    hub.impl->realValues[name] = value;
}

void
Hub::SetRealUpdate::updateOutlet (Outlet& outlet)
{
    outlet.onRealChanged(name, value);
}

} }
