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

#include "string-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include "hub/hub-impl.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::SetStringUpdate::SetStringUpdate (String name, String newValue)
: Hub::Update(name)
, newValue(newValue)
{

}

void
Hub::SetStringUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.impl->stringValuesMutex);

    hub.impl->stringValues[name] = newValue;
}

void
Hub::SetStringUpdate::updateOutlet (Outlet& outlet)
{
    outlet.onStringChanged(name, newValue);
}

} }
