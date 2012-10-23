#pragma once

namespace ntk {
    template < typename Instance > class Ptr;
    class Mesh;
    typedef ntk::Ptr<      Mesh> MeshPtr;
    typedef ntk::Ptr<const Mesh> MeshConstPtr;
}
