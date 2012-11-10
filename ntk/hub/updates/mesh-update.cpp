#include "mesh-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include "mesh/mesh.h"
#include "hub/hub-impl.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::MeshUpdate::MeshUpdate (Name name)
: Hub::Update(name)
{

}

void
Hub::MeshUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.impl->meshesMutex);

    mesh = hub.impl->meshes[name];

    _.unlock();

    updateMesh(mesh);

    _.relock();

    hub.impl->meshes[name] = mesh;
}

void
Hub::MeshUpdate::updateOutlet (Outlet& outlet)
{
    outlet.onMeshChanged(name, *mesh);
}

//------------------------------------------------------------------------------

Hub::SetMeshUpdate::SetMeshUpdate (Name name, MeshConstPtr newMesh)
: MeshUpdate(name)
, newMesh(newMesh)
{

}

void
Hub::SetMeshUpdate::updateMesh (MeshConstPtr& mesh)
{
    mesh = newMesh;
}

//------------------------------------------------------------------------------

Hub::ClearMeshUpdate::ClearMeshUpdate (Name name)
: Hub::MeshUpdate(name)
{

}

void
Hub::ClearMeshUpdate::updateMesh (MeshConstPtr& mesh)
{
    mesh = MeshConstPtr();
}

} }
