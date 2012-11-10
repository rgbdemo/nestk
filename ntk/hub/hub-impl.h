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
    // Statuses
    typedef QHash<Name, Line> Statuses;
    Statuses statuses;
    mutable QMutex statusesMutex;

    // Progresses
    typedef QHash<Name, Percentage> Progresses;
    Progresses progresses;
    mutable QMutex progressesMutex;

    // Logs
    typedef QHash<Name, Lines> Logs;
    Logs logs;
    mutable QMutex logsMutex;

    // Images
    typedef QHash<Name, Image> Images;
    Images images;
    mutable QMutex imagesMutex;

    // Meshes
    typedef QHash<Name, MeshConstPtr> Meshes;
    Meshes meshes;
    mutable QMutex meshesMutex;
};

} }
