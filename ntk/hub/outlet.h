#pragma once

#include "ntk/mesh/meshfwd.h"
#include <QImage>
#include <QStringList>
#include <QString>
#include <QObject>

namespace ntk { namespace hub {

class Outlet
{
public:
     Outlet ();
    ~Outlet ();

public:
    virtual void changeStatus   (QString name, QString    status) = 0;
    virtual void changeProgress (QString name, qreal    progress) = 0;
    virtual void changeLog      (QString name, QStringList   log) = 0;
    virtual void changeImage    (QString name, QImage      image) = 0;
    virtual void changeMesh     (QString name, const Mesh*  mesh) = 0;

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
    virtual void changeStatus   (QString name, QString   status);
    virtual void changeProgress (QString name, qreal   progress);
    virtual void changeLog      (QString name, QStringList  log);
    virtual void changeImage    (QString name, QImage     image);
    virtual void changeMesh     (QString name, const Mesh* mesh);

signals:
    void   statusChanged (QString name, QString   status);
    void progressChanged (QString name, qreal   progress);
    void      logChanged (QString name, QStringList  log);
    void    imageChanged (QString name, QImage     image);
    void     meshChanged (QString name, const Mesh* mesh);
};

//------------------------------------------------------------------------------

class OutletListener : public Outlet
{
public:
    virtual ~OutletListener ();

public: // Outlet
    virtual void changeStatus   (QString name, QString   status);
    virtual void changeProgress (QString name, qreal   progress);
    virtual void changeLog      (QString name, QStringList  log);
    virtual void changeImage    (QString name, QImage     image);
    virtual void changeMesh     (QString name, const Mesh* mesh);

protected:
    virtual void   onStatusChanged (QString name, QString   status) {}
    virtual void onProgressChanged (QString name, qreal   progress) {}
    virtual void      onLogchanged (QString name, QStringList  log) {}
    virtual void    onImageChanged (QString name, QImage     image) {}
    virtual void     onMeshChanged (QString name, const Mesh* mesh) {}
};

} }
