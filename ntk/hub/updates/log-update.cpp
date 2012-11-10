#include "log-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include "hub/hub-impl.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::LogUpdate::LogUpdate (Name name)
: Hub::Update(name)
{

}

void
Hub::LogUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.impl->logsMutex);

    log = hub.impl->logs[name];

    _.unlock();

    updateLog(log);

    _.relock();

    hub.impl->logs[name] = log;
}

void
Hub::LogUpdate::updateOutlet (Outlet& outlet)
{
    outlet.onLogChanged(name, log);
}

//------------------------------------------------------------------------------

Hub::SetLogUpdate::SetLogUpdate (Name name, const QStringList& newLog)
: Hub::LogUpdate(name)
, newLog(newLog)
{

}

void
Hub::SetLogUpdate::updateLog (QStringList& log)
{
    log = newLog;
}

//------------------------------------------------------------------------------

Hub::AppendLogUpdate::AppendLogUpdate (Name name, QString line)
: Hub::LogUpdate(name)
, line(line)
{

}

void
Hub::AppendLogUpdate::updateLog(QStringList& log)
{
    log << line;
}

//------------------------------------------------------------------------------

Hub::ClearLogUpdate::ClearLogUpdate (Name name)
: Hub::LogUpdate(name)
{

}

void
Hub::ClearLogUpdate::updateLog (QStringList& log)
{
    log = QStringList();
}


} }
