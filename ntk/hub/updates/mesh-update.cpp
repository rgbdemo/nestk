#include "mesh-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include "mesh/mesh.h"
#include "hub/hub-impl.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::MeshUpdate::MeshUpdate (String name)
: Hub::Update(name)
{

}

void
Hub::MeshUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.impl->meshValuesMutex);

    mesh = hub.impl->meshValues[name];

    _.unlock();

    updateMesh(mesh);

    _.relock();

    hub.impl->meshValues[name] = mesh;
}

void
Hub::MeshUpdate::updateOutlet (Outlet& outlet)
{
    outlet.onMeshChanged(name, mesh);
}

//------------------------------------------------------------------------------

Hub::SetMeshUpdate::SetMeshUpdate (String name, MeshConstPtr newMesh)
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

Hub::ClearMeshUpdate::ClearMeshUpdate (String name)
: Hub::MeshUpdate(name)
{

}

void
Hub::ClearMeshUpdate::updateMesh (MeshConstPtr& mesh)
{
    mesh = MeshConstPtr();
}

} }
