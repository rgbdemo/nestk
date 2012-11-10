#pragma once

#include "hub/update.h"
#include "mesh/meshfwd.h"

namespace ntk { namespace hub {

class Hub::MeshUpdate : public Hub::Update
{
public:
    MeshUpdate (Name name);

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
    SetMeshUpdate (Name name, MeshConstPtr newMesh);

private:
    virtual void updateMesh (MeshConstPtr& mesh);

private:
    const MeshConstPtr newMesh;
};

//------------------------------------------------------------------------------

class Hub::ClearMeshUpdate : public Hub::MeshUpdate
{
public:
    ClearMeshUpdate (Name name);

private:
    virtual void updateMesh (MeshConstPtr& mesh);
};

} }
