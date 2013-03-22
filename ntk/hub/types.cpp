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

#include "types.h"
#include <QMetaType>
#include <QString>
#include <QStringList>
#include <QtGlobal>
#include <QImage>
#include <mesh/mesh.h>
#include <opencv2/core/core.hpp>

namespace ntk { namespace hub {

#define HUB_TYPE(Type, type, Arg, Ret, Val)  \
const char* TypeTraits<Type>::Name = #Type;  \
const char* TypeTraits<Type>::name = #type;
        HUB_TYPES()
#undef  HUB_TYPE

//------------------------------------------------------------------------------

void registerTypes ()
{
#define HUB_TYPE(Type, type, Arg, Ret, Val) \
    qRegisterMetaType<Type>(#Type);
        HUB_TYPES()
#undef  HUB_TYPE
}

} }
