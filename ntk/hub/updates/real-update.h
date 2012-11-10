#pragma once

#include "hub/update.h"
#include <QtGlobal>

namespace ntk { namespace hub {

class Hub::SetRealUpdate : public Hub::Update
{
public:
    SetRealUpdate (String name, Real progress);

public:
    virtual void updateHub (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

public:
    Real value;
};

} }
