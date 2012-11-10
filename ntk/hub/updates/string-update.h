#pragma once

#include "hub/update.h"
#include <QString>

namespace ntk { namespace hub {

class Hub::SetStringUpdate : public Hub::Update
{
public:
    SetStringUpdate (String name, String newValue);

public:
    virtual void updateHub (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

public:
    QString newValue;
};

} }
