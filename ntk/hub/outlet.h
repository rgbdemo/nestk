/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Tisserand <nicolas.tisserand@manctl.com>
 */

#ifndef NTK_HUB_OUTLET_H
# define NTK_HUB_OUTLET_H

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
    void   subscribe (const String& name);
    void unsubscribe (const String& name);

public:
#define HUB_TYPE(Type, type, Arg, Ret, Val)                            \
    virtual void on##Type##Changed (const String& name, Arg type) = 0;
        HUB_TYPES()
#undef  HUB_TYPE

private:
    friend class Hub;
    struct Impl;
    Impl* impl;
};

//------------------------------------------------------------------------------

#define NO_REAL    virtual void onRealChanged    (const String& name,       Real          real   ) {}
#define NO_STRING  virtual void onStringChanged  (const String& name, const String&       string ) {}
#define NO_STRINGS virtual void onStringsChanged (const String& name, const Strings&      strings) {}
#define NO_IMAGE   virtual void onImageChanged   (const String& name, const Image&        image  ) {}
#define NO_MESH    virtual void onMeshChanged    (const String& name, const MeshConstPtr& mesh   ) {}

struct    RealOutlet : Outlet {         NO_STRING NO_STRINGS NO_IMAGE NO_MESH };
struct  StringOutlet : Outlet { NO_REAL           NO_STRINGS NO_IMAGE NO_MESH };
struct StringsOutlet : Outlet { NO_REAL NO_STRING            NO_IMAGE NO_MESH };
struct   ImageOutlet : Outlet { NO_REAL NO_STRING NO_STRINGS          NO_MESH };
struct    MeshOutlet : Outlet { NO_REAL NO_STRING NO_STRINGS NO_IMAGE         };

#undef NO_REAL
#undef NO_STRING
#undef NO_STRINGS
#undef NO_IMAGE
#undef NO_MESH

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
#define HUB_TYPE(Type, type, Arg, Ret, Val)                        \
    virtual void on##Type##Changed (const String& name, Arg type);
        HUB_TYPES()
#undef  HUB_TYPE

// FIXME: Qt's signal declarations cannot be preprocessor-expanded.

signals:
    void    realChanged (const String& name,       Real          value);
    void  stringChanged (const String& name, const String&       value);
    void stringsChanged (const String& name, const Strings&      value);
    void   imageChanged (const String& name, const Image&        value);
    void    meshChanged (const String& name, const MeshConstPtr& value);
};

} }

#endif // !NTK_HUB_OUTLET_H
