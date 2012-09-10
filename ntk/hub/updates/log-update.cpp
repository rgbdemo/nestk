#include "log-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::LogUpdate::LogUpdate (QString name)
: Hub::Update(name)
{

}

void
Hub::LogUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.logsMutex);

    hubLog = hub.logs[name];

    _.unlock();

    updateHubLog(hubLog);

    _.relock();

    hub.logs[name] = hubLog;
}

void
Hub::LogUpdate::updateOutlet (Outlet& outlet)
{
    outlet.changeLog(name, hubLog);
}

//------------------------------------------------------------------------------

Hub::SetLogUpdate::SetLogUpdate (QString name, QStringList log)
: Hub::LogUpdate(name)
, log(log)
{

}

void
Hub::SetLogUpdate::updateHubLog (QStringList& hubLog)
{
    hubLog = log;
}

//------------------------------------------------------------------------------

Hub::AppendLogUpdate::AppendLogUpdate (QString name, QString line)
: Hub::LogUpdate(name)
, line(line)
{

}

void
Hub::AppendLogUpdate::updateHubLog(QStringList& hubLog)
{
    hubLog << line;
}

//------------------------------------------------------------------------------

Hub::ClearLogUpdate::ClearLogUpdate (QString name)
: Hub::LogUpdate(name)
{

}

void
Hub::ClearLogUpdate::updateHubLog (QStringList& hubLog)
{
    hubLog = QStringList();
}


} }
