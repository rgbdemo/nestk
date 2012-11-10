#include "strings-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include "hub/hub-impl.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::StringsUpdate::StringsUpdate (String name)
: Hub::Update(name)
{

}

void
Hub::StringsUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.impl->stringsValuesMutex);

    strings = hub.impl->stringsValues[name];

    _.unlock();

    updateStrings(strings);

    _.relock();

    hub.impl->stringsValues[name] = strings;
}

void
Hub::StringsUpdate::updateOutlet (Outlet& outlet)
{
    outlet.onStringsChanged(name, strings);
}

//------------------------------------------------------------------------------

Hub::SetStringsUpdate::SetStringsUpdate (String name, const Strings& newStrings)
: Hub::StringsUpdate(name)
, newStrings(newStrings)
{

}

void
Hub::SetStringsUpdate::updateStrings (Strings& strings)
{
    strings = newStrings;
}

//------------------------------------------------------------------------------

Hub::AppendStringsUpdate::AppendStringsUpdate (String name, QString chunk)
: Hub::StringsUpdate(name)
, chunk(chunk)
{

}

void
Hub::AppendStringsUpdate::updateStrings (Strings& strings)
{
    strings << chunk;
}

//------------------------------------------------------------------------------

Hub::ClearStringsUpdate::ClearStringsUpdate (String name)
: Hub::StringsUpdate(name)
{

}

void
Hub::ClearStringsUpdate::updateStrings (Strings& strings)
{
    strings = Strings();
}


} }
