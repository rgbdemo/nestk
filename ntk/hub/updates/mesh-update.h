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

#ifndef NTK_HUB_UPDATES_MESH_UPDATE_H
# define NTK_HUB_UPDATES_MESH_UPDATE_H

#include "hub/update.h"
#include "mesh/meshfwd.h"

namespace ntk { namespace hub {

class Hub::MeshUpdate : public Hub::Update
{
public:
    MeshUpdate (String name);

public:
    virtual void updateHub    (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

private:
    virtual void updateMesh (MeshConstPtr& mesh) = 0;

private:
    MeshConstPtr mesh;
};

//------------------------------------------------------------------------------

class Hub::SetMeshUpdate : public Hub::MeshUpdate
{
public:
    SetMeshUpdate (String name, MeshConstPtr newMesh);

private:
    virtual void updateMesh (MeshConstPtr& mesh);

private:
    const MeshConstPtr newMesh;
};

//------------------------------------------------------------------------------

class Hub::ClearMeshUpdate : public Hub::MeshUpdate
{
public:
    ClearMeshUpdate (String name);

private:
    virtual void updateMesh (MeshConstPtr& mesh);
};

} }

#endif // !NTK_HUB_UPDATES_MESH_UPDATE_H
