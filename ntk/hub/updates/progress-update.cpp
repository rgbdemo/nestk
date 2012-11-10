#include "progress-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include "hub/hub-impl.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::ProgressUpdate::ProgressUpdate (Name name, Percentage progress)
: Hub::Update(name)
, progress(progress)
{

}

void
Hub::ProgressUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.impl->progressesMutex);

    hub.impl->progresses[name] = progress;
}

void
Hub::ProgressUpdate::updateOutlet (Outlet& outlet)
{
    outlet.onProgressChanged(name, progress);
}

} }
