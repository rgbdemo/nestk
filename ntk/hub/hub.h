#pragma once

#include <ntk/thread/event.h>
#include <QImage>
#include <QHash>
#include <QStringList>
#include <QString>
#include <QMutex>
#include <QtGlobal>

namespace ntk { namespace hub {

class Hub
: public ntk::AsyncEventListener
, public ntk::EventBroadcaster
{
public:
    static Hub* getInstance ();

public:
    QString getStatus (QString name) const;
    void    setStatus (QString name, QString status);
    void  clearStatus (QString name);

private:
    typedef QHash<QString, QString> Statuses;
    Statuses statuses;
    mutable QMutex statusesMutex;

public:
    qreal getProgress (QString name) const;
    void  setProgress (QString name, qreal progress);

private:
    typedef QHash<QString, qreal> Progresses;
    Progresses progresses;
    mutable QMutex progressesMutex;

public:
    QStringList getLog (QString name) const;
    void        setLog (QString name, QStringList log);
    void     appendLog (QString name, QString line);
    void      clearLog (QString name);

private:
    typedef QHash<QString, QStringList> Logs;
    Logs   logs;
    mutable QMutex logsMutex;

public:
    QImage getImage (QString name) const;
    void   setImage (QString name, QImage image);
    void   setImage (QString name, const cv::Mat& mat);
    void clearImage (QString name);

private:
    typedef QHash<QString, QImage> Images;
    Images images;
    mutable QMutex imagesMutex;

protected:
    virtual void handleAsyncEvent (Event event);

public:
    class               Update;
    class       ProgressUpdate;
    class         StatusUpdate;
    class            LogUpdate;
    class         SetLogUpdate;
    class      AppendLogUpdate;
    class       ClearLogUpdate;
    class          ImageUpdate;
    class       SetImageUpdate;
    class SetOpenCVImageUpdate;
    class     ClearImageUpdate;

private:
    void postUpdate (Update* update);

private:
    static Hub instance;
};

} }
