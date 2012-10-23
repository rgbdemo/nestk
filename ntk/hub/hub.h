#pragma once

#include "ntk/thread/event.h"
#include "ntk/mesh/meshfwd.h"
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

public: // Logs
    QStringList getLog (QString name) const;
    void        setLog (QString name, QStringList log);
    void     appendLog (QString name, QString line);
    void      clearLog (QString name);

private: // Logs
    typedef QHash<QString, QStringList> Logs;
    Logs   logs;
    mutable QMutex logsMutex;

public: // Images
    QImage getImage (QString name) const;
    void   setImage (QString name, QImage image);
    void   setImage (QString name, const cv::Mat& mat);
    void clearImage (QString name);

private: // Images
    typedef QHash<QString, QImage> Images;
    Images images;
    mutable QMutex imagesMutex;

public: // Meshes
    MeshConstPtr getMesh (QString name) const;
    void         setMesh (QString name, MeshConstPtr mesh);
    void         setMesh (QString name, const Mesh& mesh);
    void       clearMesh (QString name);

private: // Meshes
    typedef QHash<QString, MeshConstPtr> Meshes;
    Meshes meshes;
    mutable QMutex meshesMutex;

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
    class           MeshUpdate;
    class        SetMeshUpdate;
    class      ClearMeshUpdate;

private:
    void postUpdate (Update* update);

private:
    static Hub instance;
};

} }
