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

#ifndef NTK_HUB_UPDATES_STRINGS_UPDATE_H
# define NTK_HUB_UPDATES_STRINGS_UPDATE_H

#include "hub/update.h"
#include <QStringList>

namespace ntk { namespace hub {

class Hub::StringsUpdate : public Hub::Update
{
public:
    StringsUpdate (String name);

public:
    virtual void updateHub (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

protected:
    virtual void updateStrings (Strings& strings) = 0;

private:
    Strings strings;
};

//------------------------------------------------------------------------------

class Hub::SetStringsUpdate : public Hub::StringsUpdate
{
public:
    SetStringsUpdate (String name, const Strings& newStrings);

private:
    virtual void updateStrings (Strings& strings);

private:
    Strings newStrings;
};

//------------------------------------------------------------------------------

class Hub::AppendStringsUpdate : public Hub::StringsUpdate
{
public:
    AppendStringsUpdate (String name, String chunk);

private:
    virtual void updateStrings (Strings& strings);

private:
    const String chunk;
};

//------------------------------------------------------------------------------

class Hub::ClearStringsUpdate : public Hub::StringsUpdate
{
public:
    ClearStringsUpdate (String name);

private:
    virtual void updateStrings (Strings& strings);
};

} }

#endif // !NTK_HUB_UPDATES_STRINGS_UPDATE_H
