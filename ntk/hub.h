#pragma once

#include "hub/types.h"

namespace ntk { namespace hub {

// hub.status
void     setStatus (const String& status);
void   resetStatus ();

// hub.error
void      setError (const String& status);

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
