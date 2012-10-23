#include "mesh-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include "mesh/mesh.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::MeshUpdate::MeshUpdate (QString name)
: Hub::Update(name)
{

}

void
Hub::MeshUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.meshesMutex);

    hubMesh = hub.meshes[name];

    _.unlock();

    updateHubMesh(hubMesh);

    _.relock();

    hub.meshes[name] = hubMesh;
}

void
Hub::MeshUpdate::updateOutlet (Outlet& outlet)
{
    outlet.changeMesh(name, hubMesh);
}

//------------------------------------------------------------------------------

Hub::SetMeshUpdate::SetMeshUpdate (QString name, MeshConstPtr mesh)
: MeshUpdate(name)
, mesh(mesh)
{

}

void
Hub::SetMeshUpdate::updateHubMesh (MeshConstPtr& hubMesh)
{
    hubMesh = mesh;
}

//------------------------------------------------------------------------------

Hub::ClearMeshUpdate::ClearMeshUpdate (QString name)
: Hub::MeshUpdate(name)
{

}

void
Hub::ClearMeshUpdate::updateHubMesh (MeshConstPtr& hubMesh)
{
    hubMesh = MeshConstPtr();
}

} }
