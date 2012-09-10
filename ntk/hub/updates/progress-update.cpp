#include "progress-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::ProgressUpdate::ProgressUpdate (QString name, qreal progress)
: Hub::Update(name)
, progress(progress)
{

}

void
Hub::ProgressUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.progressesMutex);

    hub.progresses[name] = progress;
}

void
Hub::ProgressUpdate::updateOutlet (Outlet& outlet)
{
    outlet.changeProgress(name, progress);
}

} }
