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

#include "hub.h"
#include "hub/hub.h"
#include "mesh/mesh.h"

namespace {

inline QString prefixed (const char* prefix, const QString& name)
{
    return QString(prefix) + "." + name;
}

}

//------------------------------------------------------------------------------

namespace ntk { namespace hub {

void setStatus (const String& status)
{
    Hub::getInstance()->setString("status", status);
}

void   resetStatus ()
{
    Hub::getInstance()->resetString("status");
}

void setError (const String& error)
{
    Hub::getInstance()->setString("error", error);
}

void setMessage (const String& message)
{
    Hub::getInstance()->setString("message", message);
}

void   setProgress (Real progress)
{
    Hub::getInstance()->setReal("progress", progress);
}

void resetProgress ()
{
    Hub::getInstance()->resetReal("progress");
}

void report (const String& status, Real progress)
{
    setStatus(status);
    setProgress(progress);
}

void finish ()
{
    resetStatus();
    resetProgress();
}

//------------------------------------------------------------------------------

#define HUB_SET(Function, Type)                        \
void                                                   \
Function (const String& name, HUB_TYPE_ARG(Type) arg)  \
{                                                      \
    Hub::getInstance()->set##Type(name, arg);          \
}

#define HUB_RESET(Function, Type)          \
void Function (const String& name)         \
{                                          \
    Hub::getInstance()->reset##Type(name); \
}

#define HUB_CLASS(Name, Type) \
HUB_SET(     set##Name, Type) \
HUB_RESET( reset##Name, Type)

HUB_CLASS(Real    , Real   )
HUB_CLASS(Status  , String )
HUB_CLASS(Log     , Strings)
HUB_CLASS(Image   , Image  )
HUB_CLASS(Mesh    , Mesh   )

//------------------------------------------------------------------------------

#define HUB_FWD_0(Function, Method, Prefix)  \
    void                                     \
Function (const String& name)                \
{                                            \
    Hub::getInstance()->Function(name);      \
}

#define HUB_FWD_1(Function, Method, Prefix, Arg0) \
void                                              \
Function (const String& name, Arg0 arg0)          \
{                                                 \
    Hub::getInstance()->Method(name, arg0);       \
}

HUB_FWD_1(appendLog , appendToStrings, stringlists , const String&);
HUB_FWD_1(setImage  , setImageMatrix , images      , const Matrix&);
HUB_FWD_1(setMesh   , setMesh        , meshes      , const Mesh&  );

} }
