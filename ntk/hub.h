#pragma once

#include "hub/types.h"

namespace ntk { namespace hub {

void     setStatus (const Name& name, const Line& status);
void   clearStatus (const Name& name);

void   setProgress (const Name& name, Percentage progress);
void clearProgress (const Name& name);

void        setLog (const Name& name, const Lines& log);
void     appendLog (const Name& name, const Line& line);
void      clearLog (const Name& name);

void      setImage (const Name& name, const Image& image);
void      setImage (const Name& name, const Matrix& matrix);
void    clearImage (const Name& name);

void       setMesh (const Name& name, const Mesh& mesh);
void     clearMesh (const Name& name);

void report (const Name& name, const Line& status, Percentage progress);
void finish (const Name& name);

} }
