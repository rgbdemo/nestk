#pragma once

#include "types.h"
#include <QObject>
#include <QString>
#include <QMetaType>

namespace ntk { namespace hub {

class Outlet
{    
public:
     Outlet ();
    ~Outlet ();

public:
    virtual void onStatusChanged   (Name name, const Line& status) = 0;
    virtual void onProgressChanged (Name name, Percentage progress) = 0;
    virtual void onLogChanged      (Name name, const Lines& log) = 0;
    virtual void onImageChanged    (Name name, const Image& image) = 0;
    virtual void onMeshChanged     (Name name, const Mesh& mesh) = 0;

private:
    struct Impl;
    Impl* impl;
};

//------------------------------------------------------------------------------

class QOutlet
    : public QObject
    , public Outlet
{
    Q_OBJECT

public:
    QOutlet (QObject* parent = 0);
    virtual ~QOutlet ();

public: // Outlet
    virtual void onStatusChanged   (Name name, const Line& status);
    virtual void onProgressChanged (Name name, Percentage progress);
    virtual void onLogChanged      (Name name, const Lines& log);
    virtual void onImageChanged    (Name name, const Image& image);
    virtual void onMeshChanged     (Name name, const Mesh& mesh);

signals:
    void   statusChanged (Name name, const Line& status);
    void progressChanged (Name name, Percentage progress);
    void      logChanged (Name name, const Lines& log);
    void    imageChanged (Name name, const Image& image);
    void     meshChanged (Name name, const Mesh& mesh);
};

} }
