// This file was intentionally written using some wide text lines.
// Please keep it this way.

#include "outlet.h"
#include "outlet-impl.h"
#include "hub.h"
#include "impl.h"
#include <QStringList>
#include <QImage>
#include <cassert>

namespace ntk { namespace hub {

Outlet::Outlet ()
: impl(new Impl(this))
{

}

Outlet::~Outlet ()
{
    delete impl;
}

FWD_IMPL_0(void, Outlet, start)
FWD_IMPL_0(void, Outlet, stop )
FWD_IMPL_1(void, Outlet,   subscribe, const String&)
FWD_IMPL_1(void, Outlet, unsubscribe, const String&)

//------------------------------------------------------------------------------

QOutlet::QOutlet (QObject* parent)
    : QObject(parent)
{

}

QOutlet::~QOutlet ()
{

}

// FIXME: Qt's signal emissions cannot be preprocessor-expanded.

void QOutlet::onRealChanged    (const String& name, Real                value) { emit    realChanged(name, value); }
void QOutlet::onStringChanged  (const String& name, const String&       value) { emit  stringChanged(name, value); }
void QOutlet::onStringsChanged (const String& name, const Strings&      value) { emit stringsChanged(name, value); }
void QOutlet::onImageChanged   (const String& name, const Image&        value) { emit   imageChanged(name, value); }
void QOutlet::onMeshChanged    (const String& name, const MeshConstPtr& value) { emit    meshChanged(name, value); }

} }
