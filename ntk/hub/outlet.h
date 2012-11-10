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
    virtual void onRealChanged    (String name,       Real     real   ) = 0;
    virtual void onStringChanged  (String name, const String&  string ) = 0;
    virtual void onStringsChanged (String name, const Strings& strings) = 0;
    virtual void onImageChanged   (String name, const Image&   image  ) = 0;
    virtual void onMeshChanged    (String name, const Mesh&    mesh   ) = 0;

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
    virtual void onRealChanged     (String name,       Real     real  );
    virtual void onStringChanged   (String name, const String&  string);
    virtual void onStringsChanged  (String name, const Strings& strings);
    virtual void onImageChanged    (String name, const Image&   image  );
    virtual void onMeshChanged     (String name, const Mesh&    mesh   );

signals:
    void    realChanged (String name,       Real     real   );
    void  stringChanged (String name, const String&  string );
    void stringsChanged (String name, const Strings& strings);
    void   imageChanged (String name, const Image&   image  );
    void    meshChanged (String name, const Mesh&    mesh   );
};

} }
