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

typedef     QString Name;
typedef     QString Line;
typedef QStringList Lines;
typedef       qreal Percentage;
typedef      QImage Image;
typedef     cv::Mat Matrix;

//------------------------------------------------------------------------------

class Hub
: public ntk::AsyncEventListener
, public ntk::EventBroadcaster
{
public:
    static Hub* getInstance ();

public:
    Line    getStatus (const Name& name) const;
    void    setStatus (const Name& name, const Line& status);
    void  clearStatus (const Name& name);

private:
    typedef QHash<Name, Line> Statuses;
    Statuses statuses;
    mutable QMutex statusesMutex;

public:
    qreal  getProgress (const Name& name) const;
    void   setProgress (const Name& name, qreal progress);
    void clearProgress (const Name &name);

private:
    typedef QHash<Name, qreal> Progresses;
    Progresses progresses;
    mutable QMutex progressesMutex;

public: // Logs
    Lines   getLog (const Name& name) const;
    void    setLog (const Name& name, const Lines& log);
    void appendLog (const Name& name, const Line& line);
    void  clearLog (const Name& name);

private: // Logs
    typedef QHash<Name, Lines> Logs;
    Logs   logs;
    mutable QMutex logsMutex;

public: // Images
    Image  getImage (const Name& name) const;
    void   setImage (const Name& name, const Image& image);
    void   setImage (const Name& name, const Matrix& matrix);
    void clearImage (const Name& name);

private: // Images
    typedef QHash<Name, QImage> Images;
    Images images;
    mutable QMutex imagesMutex;

public: // Meshes
    MeshConstPtr getMesh (const Name& name) const;
    void         setMesh (const Name& name, MeshConstPtr mesh);
    void         setMesh (const Name& name, const Mesh& mesh);
    void       clearMesh (const Name& name);

private: // Meshes
    typedef QHash<Name, MeshConstPtr> Meshes;
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
    class SetImageMatrixUpdate;
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

//------------------------------------------------------------------------------

namespace ntk { namespace hub {

inline void      setStatus (const Name& name, Line status         ) { Hub::getInstance()->  setStatus(name, status); }
inline void    clearStatus (const Name& name)                       { Hub::getInstance()->clearStatus(name        ); }

inline void    setProgress (const Name& name, Percentage progress ) { Hub::getInstance()->  setProgress(name, progress); }
inline void  clearProgress (const Name& name)                       { Hub::getInstance()->clearProgress(name          ); }

inline void         setLog (const Name& name, Lines log           ) { Hub::getInstance()->     setLog(name, log ); }
inline void      appendLog (const Name& name, Line line           ) { Hub::getInstance()->  appendLog(name, line); }
inline void       clearLog (const Name& name                      ) { Hub::getInstance()->   clearLog(name      ); }

inline void       setImage (const Name& name, const Image& image  ) { Hub::getInstance()->   setImage(name, image ); }
inline void       setImage (const Name& name, const Matrix& matrix) { Hub::getInstance()->   setImage(name, matrix); }
inline void     clearImage (const Name& name                      ) { Hub::getInstance()-> clearImage(name        ); }

inline void        setMesh (const Name& name, const Mesh& mesh    ) { Hub::getInstance()->    setMesh(name, mesh); }
inline void      clearMesh (const Name& name                      ) { Hub::getInstance()->  clearMesh(name      ); }

inline void report (const Name& name, const Line& status, Percentage progress)
{
    setStatus(  name, status);
    setProgress(name, progress);
}

inline void finish (const Name& name)
{
    clearStatus(  name);
    clearProgress(name);
}

} }
