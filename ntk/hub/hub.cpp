#include "outlet.h"
#include "updates.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub Hub::instance;

Hub*
Hub::getInstance()
{
    return &instance;
}

void
Hub::handleAsyncEvent (EventListener::Event event)
{
    HubUpdatePtr update = dynamic_Ptr_cast<HubUpdate>(event.data);

    ntk_assert(update, "Invalid hub update. Something is very wrong.");

    update->updateHub(*this);

    broadcastEvent(update);
}

void
Hub::postUpdate (Update* update)
{
    assert(0 != update);

    // FIXME: Trick the event system into believing we have one sender per update.
    // FIXME: This only takes into account the update target :-/
    EventBroadcaster* sender = reinterpret_cast<EventBroadcaster*>(qHash(update->name.toAscii()));

    newEvent(sender, HubUpdatePtr(update));
}

//------------------------------------------------------------------------------

QString
Hub::getStatus (QString name) const
{
    QMutexLocker _(&statusesMutex);

    return statuses[name];
}

void
Hub::setStatus (QString name, QString status)
{
    postUpdate(new StatusUpdate(name, status));
}

void
Hub::clearStatus (QString name)
{
    postUpdate(new StatusUpdate(name, QString()));
}

//------------------------------------------------------------------------------

qreal
Hub::getProgress (QString name) const
{
    QMutexLocker _(&progressesMutex);

    return progresses[name];
}

void
Hub::setProgress (QString name, qreal progress)
{
    postUpdate(new ProgressUpdate(name, progress));
}

//------------------------------------------------------------------------------

QStringList
Hub::getLog (QString name) const
{
    QMutexLocker _(&logsMutex);

    return logs[name];
}

void
Hub::setLog (QString name, QStringList log)
{
    postUpdate(new SetLogUpdate(name, log));
}

void
Hub::appendLog (QString name, QString line)
{
    postUpdate(new AppendLogUpdate(name, line));
}

void
Hub::clearLog (QString name)
{
    postUpdate(new ClearLogUpdate(name));
}

//------------------------------------------------------------------------------

QImage
Hub::getImage (QString name) const
{
    QMutexLocker _(&imagesMutex);

    return images[name];
}

void
Hub::setImage (QString name, QImage image)
{
    postUpdate(new SetImageUpdate(name, image));
}

void
Hub::setImage (QString name, const cv::Mat& mat)
{
    postUpdate(new SetOpenCVImageUpdate(name, mat));
}

void
Hub::clearImage (QString name)
{
    postUpdate(new ClearImageUpdate(name));
}

} }
