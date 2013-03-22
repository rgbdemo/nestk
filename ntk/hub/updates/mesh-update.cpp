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
