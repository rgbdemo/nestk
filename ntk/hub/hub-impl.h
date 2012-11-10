#pragma once

#include "hub.h"
#include "../mesh/mesh.h"
#include <QImage>
#include <QStringList>
#include <QString>
#include <QHash>
#include <QMutex>

namespace ntk { namespace hub {

struct Hub::Impl
{
    // Real Values
    typedef QHash<String, Real> RealValues;
    RealValues     realValues;
    mutable QMutex realValuesMutex;

    // String Values
    typedef QHash<String, String> StringValues;
    StringValues   stringValues;
    mutable QMutex stringValuesMutex;

    // Strings Values
    typedef QHash<String, Strings> StringsValues;
    StringsValues  stringsValues;
    mutable QMutex stringsValuesMutex;

    // Matrix Values
//    typedef QHash<String, Matrix> MatrixValues;
//    MatrixValues   matrixValues;
//    mutable QMutex matrixValuesMutex;

    // Images
    typedef QHash<String, Image> ImageValues;
    ImageValues    imageValues;
    mutable QMutex imageValuesMutex;

    // Meshes
    typedef QHash<String, MeshConstPtr> MeshValues;
    MeshValues     meshValues;
    mutable QMutex meshValuesMutex;
};

} }
