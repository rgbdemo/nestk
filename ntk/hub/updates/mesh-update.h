#pragma once

#include "hub/update.h"
#include "mesh/meshfwd.h"

namespace ntk { namespace hub {

class Hub::MeshUpdate : public Hub::Update
{
public:
    MeshUpdate (QString name);

public:
    virtual void updateHub    (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

private:
    virtual void updateHubMesh (MeshConstPtr& hubMesh) = 0;

private:
    MeshConstPtr hubMesh;
};

//------------------------------------------------------------------------------

class Hub::SetMeshUpdate : public Hub::MeshUpdate
{
public:
    SetMeshUpdate (QString name, MeshConstPtr mesh);

private:
    virtual void updateHubMesh (MeshConstPtr& hubMesh);

private:
    const MeshConstPtr mesh;
};

//------------------------------------------------------------------------------

class Hub::ClearMeshUpdate : public Hub::MeshUpdate
{
public:
    ClearMeshUpdate (QString name);

private:
    virtual void updateHubMesh (MeshConstPtr& hubMesh);
};

} }
