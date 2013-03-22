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
