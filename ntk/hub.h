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

#ifndef NTK_HUB_H
# define NTK_HUB_H

#include "hub/types.h"

namespace ntk { namespace hub {

// hub.status
void     setStatus (const String& status);
void   resetStatus ();

// hub.error
void setError   (const String& error);

// hub.message
void setMessage (const String& message);

// hub.progress
void   setProgress (Real progress);
void resetProgress ();

// hub.{status,progress}
void report (const String& status, Real progress);
void finish ();

// hub.log
void    setLog (const Strings& log);
void appendLog (const String& line);
void  resetLog ();

// hub.reals.*
// hub.strings.*
// hub.meshes.*
// hub.images.*
// hub.meshes.*

#define HUB_TYPE(Type, type, Arg, Ret, Val)        \
void      set##Type (const String& name, Arg arg); \
void    reset##Type (const String& name);
        HUB_TYPES()
#undef  HUB_TYPE

void setImage (const String& name, const Matrix& matrix);
void setMesh  (const String& name, const Mesh&  mesh);

} }

#endif // !NTK_HUB_H
