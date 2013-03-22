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

#ifndef NTK_HUB_UPDATES_STRING_UPDATE_H
# define NTK_HUB_UPDATES_STRING_UPDATE_H

#include "hub/update.h"
#include <QString>

namespace ntk { namespace hub {

class Hub::SetStringUpdate : public Hub::Update
{
public:
    SetStringUpdate (String name, String newValue);

public:
    virtual void updateHub (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

public:
    QString newValue;
};

} }

#endif // !NTK_HUB_UPDATES_STRING_UPDATE_H
