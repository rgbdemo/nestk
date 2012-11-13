// This file was intentionally written using some very wide text lines.
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

void QOutlet::onRealChanged    (const String& name, Real real               ) { emit    realChanged(name, real   ); }
void QOutlet::onStringChanged  (const String& name, const String&     string) { emit  stringChanged(name, string ); }
void QOutlet::onStringsChanged (const String& name, const Strings&   strings) { emit stringsChanged(name, strings); }
void QOutlet::onImageChanged   (const String& name, const Image&       image) { emit   imageChanged(name, image  ); }
void QOutlet::onMeshChanged    (const String& name, const MeshConstPtr& mesh) { emit    meshChanged(name, mesh   ); }

} }
