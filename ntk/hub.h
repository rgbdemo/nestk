#pragma once

#include "hub/types.h"

namespace ntk { namespace hub {

// statuses.*
void     setStatus (const String& name, const String& status);
void   resetStatus (const String& name);

// progresses.*
void   setProgress (const String& name, Real progress);
void resetProgress (const String& name);

// {statuses,progresses}.*
void report (const String& name, const String& status, Real progress);
void finish (const String& name);

// logs.*
void        setLog (const String& name, const Strings& log);
void     appendLog (const String& name, const String& line);
void      resetLog (const String& name);

// images.*
void      setImage (const String& name, const Image& image);
void      setImage (const String& name, const Matrix& matrix);
void    resetImage (const String& name);

// meshes.*
void       setMesh (const String& name, MeshConstPtr mesh);
void       setMesh (const String& name, const Mesh&  mesh);
void     resetMesh (const String& name);

} }
