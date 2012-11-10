#pragma once

#include "ntk/thread/event.h"
#include "types.h"

namespace ntk { namespace hub {

class Hub
: public ntk::AsyncEventListener
, public ntk::EventBroadcaster
{
    Q_OBJECT

public:
    static Hub* getInstance ();

public:
     Hub ();
    ~Hub ();

public:
    Line    getStatus (const Name& name) const;
    void    setStatus (const Name& name, const Line& status);
    void  clearStatus (const Name& name);

public:
    qreal  getProgress (const Name& name) const;
    void   setProgress (const Name& name, Percentage progress);
    void clearProgress (const Name &name);

public: // Logs
    Lines   getLog (const Name& name) const;
    void    setLog (const Name& name, const Lines& log);
    void appendLog (const Name& name, const Line& line);
    void  clearLog (const Name& name);

public: // Images
    Image  getImage (const Name& name) const;
    void   setImage (const Name& name, const Image& image);
    void   setImage (const Name& name, const Matrix& matrix);
    void clearImage (const Name& name);

public: // Meshes
    MeshConstPtr getMesh (const Name& name) const;
    void         setMesh (const Name& name, MeshConstPtr mesh);
    void         setMesh (const Name& name, const Mesh& mesh);
    void       clearMesh (const Name& name);

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
    struct Impl;
    Impl*  impl;
};

} }
