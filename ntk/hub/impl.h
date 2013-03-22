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

// This file is intentionally written using some very wide text lines.
// Please keep it this way.

#ifndef NTK_HUB_IMPL_H
# define NTK_HUB_IMPL_H

#include "fwd.h"

#define IMPL(Class)      \
private:                 \
    typedef Class That;  \
    struct Impl;         \
    Impl* impl;

#define IMPL_PUBLIC(Class) \
private:                   \
    typedef Class That;    \
public:                    \
    struct Impl;           \
    Impl* impl;

#define IMPL_ENTER(That) \
struct That::Impl        \
{                        \
    That* const that;    \

#define IMPL_SUPER(That, Super) \
struct That::Impl : Super       \
{                               \
    That* const that;           \

#define IMPL_EXIT() \
};

#define IMPL_CONSTRUCTOR(That) \
Impl (That* that)              \
    : that(that)               \
{                              \
    assert(0 != that);         \
                               \
}

#define FWD_IMPL_0(      Ret, Class, Method                        ) FWD_OBJ_0(   Class::Method, impl, Ret, Method                               )
#define FWD_IMPL_1(      Ret, Class, Method, Arg0                  ) FWD_OBJ_1(   Class::Method, impl, Ret, Method, Arg0                         )
#define FWD_IMPL_2(      Ret, Class, Method, Arg0, Arg1            ) FWD_OBJ_2(   Class::Method, impl, Ret, Method, Arg0, Arg1                   )
#define FWD_IMPL_3(      Ret, Class, Method, Arg0, Arg1, Arg2      ) FWD_OBJ_3(   Class::Method, impl, Ret, Method, Arg0, Arg1, Arg2             )
#define FWD_IMPL_4(      Ret, Class, Method, Arg0, Arg1, Arg2, Arg3) FWD_OBJ_4(   Class::Method, impl, Ret, Method, Arg0, Arg1, Arg2, Arg3       )
#define FWD_IMPL_0_CONST(Ret, Class, Method                        ) FWD_OBJ_0_CV(Class::Method, impl, Ret, Method                        , const)
#define FWD_IMPL_1_CONST(Ret, Class, Method, Arg0                  ) FWD_OBJ_1_CV(Class::Method, impl, Ret, Method, Arg0                  , const)
#define FWD_IMPL_2_CONST(Ret, Class, Method, Arg0, Arg1            ) FWD_OBJ_2_CV(Class::Method, impl, Ret, Method, Arg0, Arg1            , const)
#define FWD_IMPL_3_CONST(Ret, Class, Method, Arg0, Arg1, Arg2      ) FWD_OBJ_3_CV(Class::Method, impl, Ret, Method, Arg0, Arg1, Arg2      , const)
#define FWD_IMPL_4_CONST(Ret, Class, Method, Arg0, Arg1, Arg2, Arg3) FWD_OBJ_4_CV(Class::Method, impl, Ret, Method, Arg0, Arg1, Arg2, Arg3, const)

#endif // !NTK_HUB_IMPL_H
