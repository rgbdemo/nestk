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
    void start ();
    void stop  ();

public:
    void subscribe   (String name);
    void unsubscribe (String name);

public:
#define HUB_TYPE(Type, type, Arg, Ret, Val)                     \
    virtual void on##Type##Changed (String name, Arg type) = 0;
        HUB_TYPES()
#undef  HUB_TYPE

private:
    friend class Hub;
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
#define HUB_TYPE(Type, type, Arg, Ret, Val)                 \
    virtual void on##Type##Changed (String name, Arg type);
        HUB_TYPES()
#undef  HUB_TYPE

// FIXME: Qt's signal declarations cannot be preprocessor-expanded.

signals:
    void    realChanged (String name,       Real          real   );
    void  stringChanged (String name, const String&       string );
    void stringsChanged (String name, const Strings&      strings);
    void   imageChanged (String name, const Image&        image  );
    void    meshChanged (String name, const MeshConstPtr& mesh   );
};

} }
